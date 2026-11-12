#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import requests

sys.path.append(str(Path(__file__).resolve().parents[1]))

from state.mission_state_utils import update_state  # For mission_state.json


# The endpoint is served by KrakenSDR's "Kraken App" CSV output mode.
DOA_URL = os.environ.get("KRAKEN_DOA_URL", "http://127.0.0.1:8081/DOA_value.html")

# Polling rate (seconds)
UPDATE_RATE = 0.1  # 10 Hz or match Kraken update rate

BASE_DIR = Path(__file__).resolve().parents[2]
LOG_DIR = BASE_DIR / "logs" / "kraken"
LOG_DIR.mkdir(parents=True, exist_ok=True)

START_STRUCT = time.localtime()
RUN_ID = time.strftime("%Y%m%d_%H%M%S", START_STRUCT)
OUT_FILE = LOG_DIR / f"doa_{RUN_ID}.jsonl"

# Kraken App CSV layout, using zero-based Python indexes.
MIN_METADATA_FIELDS = 13
SPECTRUM_START_INDEX = 17
SPECTRUM_BIN_COUNT = 360
FULL_RECORD_FIELD_COUNT = SPECTRUM_START_INDEX + SPECTRUM_BIN_COUNT

# The full spectrum is retained only in memory by default. Enable this only for
# endpoint debugging because it substantially increases log size.
SAVE_SPECTRUM = os.environ.get("KRAKEN_SAVE_SPECTRUM", "0").lower() in {
    "1",
    "true",
    "yes",
    "on",
}
TEMPORAL_WINDOW_SIZE = max(2, int(os.environ.get("KRAKEN_TEMPORAL_WINDOW_SIZE", "10")))

seq: int = 0
last_source_timestamp_ms: int | None = None
last_measurement_keys: set[tuple[int, int | None, int]] = set()
temporal_histories: dict[tuple[int | None, int], deque[float]] = {}


def finite_float(value: object) -> float | None:
    """Parse a finite float, returning None for blank or invalid values."""

    if value is None:
        return None

    try:
        parsed = float(value)
    except (AttributeError, TypeError, ValueError):
        return None

    return parsed if math.isfinite(parsed) else None


def integer_field(fields: list[str], index: int) -> int | None:
    if index >= len(fields):
        return None

    value = finite_float(fields[index])
    return int(value) if value is not None else None


def text_field(fields: list[str], index: int) -> str | None:
    if index >= len(fields):
        return None

    value = fields[index].strip()
    return value or None


def normalize_angle_deg(angle_deg: float | None) -> float | None:
    if angle_deg is None or not math.isfinite(angle_deg):
        return None
    return round(angle_deg % 360.0, 4)


def unit_circle_to_compass_deg(angle_deg: float) -> float:
    """Convert endpoint spectrum angles: 0=East, 90=North -> compass bearing."""

    return round((90.0 - angle_deg) % 360.0, 4)


def circular_distance_deg(first: int, second: int, modulo: int) -> int:
    distance = abs(first - second) % modulo
    return min(distance, modulo - distance)


def calculate_spectrum_metrics(spectrum: list[float]) -> dict[str, Any]:
    """Calculate compact metrics and release the full spectrum after the call.

    The endpoint's full spectrum is represented as one value per degree. The
    values are used here only to calculate summaries; they are not returned
    unless KRAKEN_SAVE_SPECTRUM is enabled by the caller.
    """

    metrics: dict[str, Any] = {
        "spectrum_parse_valid": len(spectrum) == SPECTRUM_BIN_COUNT,
        "spectrum_bin_count": len(spectrum),
    }

    if len(spectrum) != SPECTRUM_BIN_COUNT or not all(math.isfinite(value) for value in spectrum):
        return metrics

    peak_index = max(range(SPECTRUM_BIN_COUNT), key=spectrum.__getitem__)
    peak_value = spectrum[peak_index]
    minimum_value = min(spectrum)

    # Shift the spectrum so entropy is calculated from non-negative weights.
    weights = [max(value - minimum_value, 0.0) for value in spectrum]
    total_weight = sum(weights)

    if total_weight <= 0.0:
        entropy_0_1 = 1.0
    else:
        probabilities = [weight / total_weight for weight in weights]
        entropy = -sum(
            probability * math.log(probability)
            for probability in probabilities
            if probability > 0.0
        )
        entropy_0_1 = entropy / math.log(SPECTRUM_BIN_COUNT)

    # A -3 dB width is a compact approximation of the dominant peak width.
    threshold = peak_value - 3.0
    if all(value >= threshold for value in spectrum):
        peak_width_deg = float(SPECTRUM_BIN_COUNT)
    else:
        width = 1
        for offset in range(1, SPECTRUM_BIN_COUNT // 2):
            left = (peak_index - offset) % SPECTRUM_BIN_COUNT
            right = (peak_index + offset) % SPECTRUM_BIN_COUNT

            if spectrum[left] < threshold or spectrum[right] < threshold:
                break
            width += 2
        peak_width_deg = float(width)

    # Exclude the dominant lobe when looking for a competing peak.
    exclusion_radius = max(1, int(math.ceil(peak_width_deg / 2.0)))
    secondary_values = [
        value
        for index, value in enumerate(spectrum)
        if circular_distance_deg(index, peak_index, SPECTRUM_BIN_COUNT) > exclusion_radius
    ]
    peak_to_second_peak_db = (
        round(peak_value - max(secondary_values), 4)
        if secondary_values
        else None
    )

    metrics.update(
        {
            "spectrum_peak_unit_deg": float(peak_index),
            "spectrum_peak_compass_deg": unit_circle_to_compass_deg(float(peak_index)),
            "peak_width_deg": peak_width_deg,
            "peak_to_second_peak_db": peak_to_second_peak_db,
            "spectrum_entropy_0_1": round(entropy_0_1, 6),
            "spectrum_peakiness_0_1": round(1.0 - entropy_0_1, 6),
        }
    )
    return metrics


def update_temporal_metrics(record: dict[str, Any]) -> dict[str, Any]:
    """Add short-term circular DoA stability metrics using bounded memory."""

    frequency_hz = record.get("frequency_hz")
    vfo_index = int(record.get("vfo_index", 0))
    history_key = (frequency_hz, vfo_index)
    history = temporal_histories.setdefault(
        history_key,
        deque(maxlen=TEMPORAL_WINDOW_SIZE),
    )

    doa_deg = finite_float(record.get("doa_deg"))
    if doa_deg is None:
        record["temporal_sample_count"] = len(history)
        record["temporal_stability_0_1"] = None
        record["temporal_jitter_deg"] = None
        return record

    history.append(math.radians(doa_deg))
    sum_sin = sum(math.sin(angle) for angle in history)
    sum_cos = sum(math.cos(angle) for angle in history)
    concentration = math.hypot(sum_sin, sum_cos) / len(history)

    record["temporal_sample_count"] = len(history)
    if len(history) < 2:
        record["temporal_stability_0_1"] = None
        record["temporal_jitter_deg"] = None
    else:
        record["temporal_stability_0_1"] = round(concentration, 6)
        # Circular standard deviation in degrees. A concentration of zero is
        # treated as maximally dispersed rather than taking log(0).
        safe_concentration = max(concentration, 1e-12)
        circular_std_rad = math.sqrt(max(0.0, -2.0 * math.log(safe_concentration)))
        record["temporal_jitter_deg"] = round(math.degrees(circular_std_rad), 4)

    return record


def parse_endpoint_line(
    raw_line: str,
    *,
    t_rx_ms: int,
    run_id: str,
    seq_value: int,
    vfo_index: int,
    save_spectrum: bool = False,
) -> dict[str, Any] | None:
    """Parse one Kraken App CSV line into a compact JSON-ready record."""

    fields = [field.strip() for field in raw_line.split(",")]
    if len(fields) < MIN_METADATA_FIELDS:
        return None

    source_timestamp_ms = integer_field(fields, 0)
    doa_deg = normalize_angle_deg(finite_float(fields[1]))
    if source_timestamp_ms is None or doa_deg is None:
        return None

    spectrum = [
        value
        for value in (
            finite_float(field)
            for field in fields[SPECTRUM_START_INDEX:FULL_RECORD_FIELD_COUNT]
        )
        if value is not None
    ]
    spectrum_metrics = calculate_spectrum_metrics(spectrum)

    entry: dict[str, Any] = {
        "t_rx_ms": t_rx_ms,
        "source_timestamp_ms": source_timestamp_ms,
        "run_id": run_id,
        "seq": seq_value,
        "vfo_index": vfo_index,
        "raw_field_count": len(fields),
        "doa_angle_convention": "compass",
        "spectrum_angle_convention": "unit_circle",
        "doa_deg": doa_deg,
        "doa_papr_db": finite_float(fields[2]),
        "rssi_power_db": finite_float(fields[3]),
        "frequency_hz": integer_field(fields, 4),
        "antenna_array_arrangement": text_field(fields, 5),
        "latency_ms": finite_float(fields[6]),
        "station_id": text_field(fields, 7),
        "lat_deg": finite_float(fields[8]),
        "lon_deg": finite_float(fields[9]),
        "gps_heading_deg": normalize_angle_deg(finite_float(fields[10])),
        "compass_heading_deg": normalize_angle_deg(finite_float(fields[11])),
        "heading_sensor": text_field(fields, 12),
        **spectrum_metrics,
    }

    if save_spectrum:
        # Debug-only escape hatch. Normal operation never writes this field.
        entry["doa_spectrum"] = spectrum

    return entry


def parse_endpoint_response(
    response_text: str,
    *,
    t_rx_ms: int,
    run_id: str,
    starting_seq: int = 0,
    save_spectrum: bool = False,
) -> list[dict[str, Any]]:
    """Parse all non-empty VFO lines returned by DOA_value.html."""

    records: list[dict[str, Any]] = []
    for vfo_index, raw_line in enumerate(response_text.splitlines()):
        line = raw_line.strip()
        if not line:
            continue

        record = parse_endpoint_line(
            line,
            t_rx_ms=t_rx_ms,
            run_id=run_id,
            seq_value=starting_seq + len(records),
            vfo_index=vfo_index,
            save_spectrum=save_spectrum,
        )
        if record is not None:
            records.append(record)

    return records


def is_new_measurement(record: dict[str, Any]) -> bool:
    """Deduplicate repeated endpoint frames while preserving multiple VFOs."""

    global last_source_timestamp_ms, last_measurement_keys

    source_timestamp_ms = record.get("source_timestamp_ms")
    if not isinstance(source_timestamp_ms, int):
        return True

    if (
        last_source_timestamp_ms is None
        or source_timestamp_ms != last_source_timestamp_ms
    ):
        last_source_timestamp_ms = source_timestamp_ms
        last_measurement_keys = set()

    measurement_key = (
        source_timestamp_ms,
        record.get("frequency_hz"),
        int(record.get("vfo_index", 0)),
    )
    if measurement_key in last_measurement_keys:
        return False

    last_measurement_keys.add(measurement_key)
    return True


def log_once(file_handle: Any) -> None:
    global seq

    try:
        try:
            response = requests.get(DOA_URL, timeout=1)
            response.raise_for_status()
        except requests.exceptions.RequestException:
            return  # Endpoint not ready or no signal yet.

        # This is the actual local receipt time, after the HTTP response arrives.
        t_rx_ms = time.time_ns() // 1_000_000
        records = parse_endpoint_response(
            response.text,
            t_rx_ms=t_rx_ms,
            run_id=RUN_ID,
            starting_seq=seq,
            save_spectrum=SAVE_SPECTRUM,
        )

        for record in records:
            if not is_new_measurement(record):
                continue

            update_temporal_metrics(record)
            record["seq"] = seq
            line_out = json.dumps(record, allow_nan=False)
            file_handle.write(line_out + "\n")
            file_handle.flush()
            print(line_out)
            seq += 1

    except Exception as error:
        # Keep the long-running logger alive, but expose parser/runtime errors.
        print("Error:", error)


def main() -> None:
    print(f"Logging in {OUT_FILE}  (run_id={RUN_ID})")

    with open(OUT_FILE, "a", encoding="utf-8") as file_handle:
        update_state("kraken_log", str(OUT_FILE))
        while True:
            loop_start = time.time()
            log_once(file_handle)
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, UPDATE_RATE - elapsed))


if __name__ == "__main__":
    main()
