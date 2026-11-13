#!/usr/bin/env python3
#
# HOW TO USE
# ----------
#
# This script emulates the KrakenSDR HTTP endpoint. It does not write a Kraken
# JSONL file and it does not update mission_state.json. The real kraken_logger.py
# remains the HTTP client, so this test exercises its actual HTTP request,
# CSV parsing, field mapping, VFO handling, deduplication, and spectrum metrics.
#
# Start the endpoint simulator in one terminal from the project root:
#
#   python src/loggers/SimKrakenEndpoint.py --mode coherent --seed 42
#
# The default endpoint is:
#
#   http://127.0.0.1:8081/DOA_value.html
#
# Verify the response in another PowerShell terminal:
#
#   (Invoke-WebRequest -UseBasicParsing `
#       -Uri "http://127.0.0.1:8081/DOA_value.html").Content
#
# A complete Kraken App record should contain 377 comma-separated fields:
# 13 metadata fields, 4 reserved fields, and 360 spectrum bins.
#
# Start the real Kraken logger in a second terminal:
#
#   python src/loggers/kraken_logger.py
#
# For a complete fused-log test without real UAV telemetry, start the existing
# telemetry file simulator and fusion logger in additional terminals:
#
#   python src/loggers/SimTelemData.py
#   python src/loggers/fusion_logger.py
#
# Then inspect the newest files under logs/kraken/ and logs/fusion/. The Kraken
# JSONL should contain doa_papr_db, spectrum_parse_valid, spectrum_bin_count,
# peak_width_deg, spectrum_entropy_0_1, spectrum_peakiness_0_1, and the other
# endpoint fields. The fusion JSONL should contain the Kraken summaries together
# with the simulated telemetry fields.
#
# Modes:
#
#   --mode coherent
#       Produces a slowly moving, narrow dominant bearing. This is the normal
#       first test and should produce strong temporal stability.
#
#   --mode random
#       Produces randomly changing bearings and competing peaks. This should
#       produce weaker temporal stability and less reliable spectrum metrics.
#
# Useful options:
#
#   --vfo-count 2
#       Return two CSV lines per HTTP response, one for each simulated VFO.
#
#   --dropout-rate 0.05
#   --malformed-rate 0.02
#   --incomplete-spectrum-rate 0.02
#       Inject endpoint failures to test logger resilience.
#
#   --port 8082
#       Run on another port. If the logger is also changed to use that port,
#       set KRAKEN_DOA_URL before starting it, for example in PowerShell:
#
#       $env:KRAKEN_DOA_URL = "http://127.0.0.1:8082/DOA_value.html"
#
#   --save behavior
#       The simulator itself never writes the full spectrum to disk. The real
#       logger also discards it by default. KRAKEN_SAVE_SPECTRUM=1 is available
#       only when deliberately debugging the logger's raw spectrum handling.
#
# Stop every process with Ctrl+C. Do not run the simulator on the same port as a
# real KrakenSDR endpoint. The older SimKrakenData.py and SimImprovedKrakenData.py
# write JSONL directly and therefore bypass this HTTP endpoint test.
#

from __future__ import annotations

import argparse
import math
import random
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse


DOA_PATH = "/DOA_value.html"
SPECTRUM_BIN_COUNT = 360
SPECTRUM_START_INDEX = 17
FULL_RECORD_FIELD_COUNT = SPECTRUM_START_INDEX + SPECTRUM_BIN_COUNT


def wrap_angle_deg(angle_deg: float) -> float:
    return angle_deg % 360.0


def circular_distance_deg(first: float, second: float) -> float:
    distance = abs(first - second) % 360.0
    return min(distance, 360.0 - distance)


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def probability(value: str) -> float:
    parsed = float(value)
    if not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be between 0 and 1")
    return parsed


@dataclass(frozen=True)
class SimulatorConfig:
    mode: str
    vfo_count: int
    rate_hz: float
    seed: int
    base_bearing_deg: float
    base_frequency_hz: int
    latitude_deg: float
    longitude_deg: float
    dropout_rate: float
    malformed_rate: float
    incomplete_spectrum_rate: float
    verbose: bool


class KrakenEndpointSimulator:
    """Generate and cache Kraken App CSV responses for the HTTP endpoint."""

    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.random = random.Random(config.seed)
        self.frame_period_ms = max(1, round(1000.0 / config.rate_hz))
        self.lock = threading.Lock()
        self.last_frame_timestamp_ms: int | None = None
        self.last_payload = ""
        self.frame_index = 0

    def get_payload(self) -> str:
        now_ms = time.time_ns() // 1_000_000
        frame_timestamp_ms = (now_ms // self.frame_period_ms) * self.frame_period_ms

        with self.lock:
            if frame_timestamp_ms != self.last_frame_timestamp_ms:
                self.last_frame_timestamp_ms = frame_timestamp_ms
                self.last_payload = self._generate_payload(
                    frame_timestamp_ms,
                    self.frame_index,
                )
                self.frame_index += 1

            return self.last_payload

    def _generate_payload(self, source_timestamp_ms: int, frame_index: int) -> str:
        roll = self.random.random()
        if roll < self.config.dropout_rate:
            return ""
        if roll < self.config.dropout_rate + self.config.malformed_rate:
            return "not,a,valid,kraken,record\n"

        lines = [
            self._generate_vfo_line(
                source_timestamp_ms=source_timestamp_ms,
                frame_index=frame_index,
                vfo_index=vfo_index,
            )
            for vfo_index in range(self.config.vfo_count)
        ]

        if self.random.random() < self.config.incomplete_spectrum_rate:
            incomplete_lines = []
            for line in lines:
                fields = line.split(",")
                # Keep metadata and only a small number of spectrum bins.
                incomplete_lines.append(",".join(fields[: SPECTRUM_START_INDEX + 12]))
            lines = incomplete_lines

        return "\n".join(lines) + "\n"

    def _generate_vfo_line(
        self,
        *,
        source_timestamp_ms: int,
        frame_index: int,
        vfo_index: int,
    ) -> str:
        compass_bearing_deg, power_spectrum = self._generate_power_spectrum(
            frame_index=frame_index,
            vfo_index=vfo_index,
        )

        maximum_power = max(power_spectrum)
        mean_power = sum(power_spectrum) / len(power_spectrum)
        papr_db = 10.0 * math.log10(maximum_power / mean_power)

        # Kraken's full spectrum is a log-scale output. Shift it upward so the
        # lowest value is zero, matching the style emitted by the real service.
        spectrum_db = [
            10.0 * math.log10(max(power, 1e-12) / maximum_power)
            for power in power_spectrum
        ]
        minimum_spectrum_db = min(spectrum_db)
        spectrum_output = [
            round(value - minimum_spectrum_db, 2)
            for value in spectrum_db
        ]

        frequency_hz = self.config.base_frequency_hz + (vfo_index * 12_500)
        gps_heading_deg = wrap_angle_deg(
            compass_bearing_deg + self.random.uniform(-2.0, 2.0)
        )
        compass_heading_deg = wrap_angle_deg(
            gps_heading_deg + self.random.uniform(-1.0, 1.0)
        )
        rssi_power_db = round(-self.random.uniform(12.0, 55.0), 2)
        latency_ms = round(self.random.uniform(60.0, 180.0), 2)

        fields: list[str] = [
            str(source_timestamp_ms),
            f"{compass_bearing_deg:.4f}",
            f"{papr_db:.6f}",
            f"{rssi_power_db:.2f}",
            str(frequency_hz),
            "UCA",
            f"{latency_ms:.2f}",
            "SIM_KRAKEN",
            f"{self.config.latitude_deg:.7f}",
            f"{self.config.longitude_deg:.7f}",
            f"{gps_heading_deg:.4f}",
            f"{compass_heading_deg:.4f}",
            "GPS",
            "R",
            "R",
            "R",
            "R",
        ]
        fields.extend(f"{value:.2f}" for value in spectrum_output)

        if len(fields) != FULL_RECORD_FIELD_COUNT:
            raise RuntimeError(
                f"generated {len(fields)} fields; expected {FULL_RECORD_FIELD_COUNT}"
            )

        return ",".join(fields)

    def _generate_power_spectrum(
        self,
        *,
        frame_index: int,
        vfo_index: int,
    ) -> tuple[float, list[float]]:
        if self.config.mode == "coherent":
            slow_drift = 12.0 * math.sin((frame_index / 80.0) + vfo_index)
            compass_bearing_deg = wrap_angle_deg(
                self.config.base_bearing_deg + slow_drift
            )
            primary_strength = self.random.uniform(8.0, 16.0)
            primary_width_deg = self.random.uniform(2.5, 5.0)
            noise_level = 0.08
            secondary_peaks: list[tuple[float, float, float]] = []
        else:
            compass_bearing_deg = self.random.uniform(0.0, 360.0)
            primary_strength = self.random.uniform(2.0, 8.0)
            primary_width_deg = self.random.uniform(5.0, 18.0)
            noise_level = 0.35
            secondary_peaks = [
                (
                    self.random.uniform(0.0, 360.0),
                    self.random.uniform(1.0, primary_strength),
                    self.random.uniform(4.0, 20.0),
                )
            ]

        # The endpoint's full spectrum uses unit-circle angles:
        # 0=East, 90=North. Convert the desired compass bearing accordingly.
        primary_unit_angle = wrap_angle_deg(90.0 - compass_bearing_deg)
        power_spectrum: list[float] = []

        for bin_index in range(SPECTRUM_BIN_COUNT):
            distance = circular_distance_deg(float(bin_index), primary_unit_angle)
            primary_peak = primary_strength * math.exp(
                -0.5 * (distance / primary_width_deg) ** 2
            )

            secondary_power = 0.0
            for secondary_angle, strength, width in secondary_peaks:
                secondary_distance = circular_distance_deg(
                    float(bin_index),
                    secondary_angle,
                )
                secondary_power += strength * math.exp(
                    -0.5 * (secondary_distance / width) ** 2
                )

            noise = self.random.uniform(0.0, noise_level)
            power_spectrum.append(max(1.0 + primary_peak + secondary_power + noise, 1e-9))

        return compass_bearing_deg, power_spectrum


class EndpointHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        request_handler: type[BaseHTTPRequestHandler],
        simulator: KrakenEndpointSimulator,
    ) -> None:
        super().__init__(server_address, request_handler)
        self.simulator = simulator


class EndpointRequestHandler(BaseHTTPRequestHandler):
    server: EndpointHTTPServer

    def do_GET(self) -> None:  # noqa: N802 - required by BaseHTTPRequestHandler
        path = urlparse(self.path).path
        if path != DOA_PATH:
            self.send_error(404, f"Expected {DOA_PATH}")
            return

        body = self.server.simulator.get_payload().encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_string: str, *args: Any) -> None:
        if self.server.simulator.config.verbose:
            super().log_message(format_string, *args)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="HTTP test double for KrakenSDR DOA_value.html."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--mode", choices=("coherent", "random"), default="coherent")
    parser.add_argument("--vfo-count", type=positive_int, default=1)
    parser.add_argument("--rate-hz", type=positive_float, default=10.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--base-bearing-deg", type=float, default=105.0)
    parser.add_argument("--base-frequency-hz", type=int, default=416_588_000)
    parser.add_argument("--latitude-deg", type=float, default=34.0400)
    parser.add_argument("--longitude-deg", type=float, default=-117.8100)
    parser.add_argument("--dropout-rate", type=probability, default=0.0)
    parser.add_argument("--malformed-rate", type=probability, default=0.0)
    parser.add_argument("--incomplete-spectrum-rate", type=probability, default=0.0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    config = SimulatorConfig(
        mode=args.mode,
        vfo_count=args.vfo_count,
        rate_hz=args.rate_hz,
        seed=args.seed,
        base_bearing_deg=wrap_angle_deg(args.base_bearing_deg),
        base_frequency_hz=args.base_frequency_hz,
        latitude_deg=args.latitude_deg,
        longitude_deg=args.longitude_deg,
        dropout_rate=args.dropout_rate,
        malformed_rate=args.malformed_rate,
        incomplete_spectrum_rate=args.incomplete_spectrum_rate,
        verbose=args.verbose,
    )
    simulator = KrakenEndpointSimulator(config)
    server = EndpointHTTPServer((args.host, args.port), EndpointRequestHandler, simulator)

    print(f"Serving simulated Kraken endpoint at http://{args.host}:{args.port}{DOA_PATH}")
    print(
        f"mode={args.mode} | vfo_count={args.vfo_count} | "
        f"rate_hz={args.rate_hz} | seed={args.seed}"
    )
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        print("\nStopping simulated Kraken endpoint.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
