import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC_DIR = ROOT / "src"
sys.path.insert(0, str(SRC_DIR))

from loggers.kraken_logger import (  # noqa: E402
    FULL_RECORD_FIELD_COUNT,
    SPECTRUM_START_INDEX,
    SPECTRUM_BIN_COUNT,
    calculate_spectrum_metrics,
    parse_endpoint_response,
    temporal_histories,
    update_temporal_metrics,
)
from loggers.fusion_logger import fuse  # noqa: E402


def make_endpoint_line(
    *,
    source_timestamp_ms: int = 1_700_000_000_000,
    frequency_hz: int = 416_588_000,
    peak_index: int = 90,
) -> str:
    fields = [""] * FULL_RECORD_FIELD_COUNT
    fields[0] = str(source_timestamp_ms)
    fields[1] = "360"
    fields[2] = "7.78"
    fields[3] = "-34.2"
    fields[4] = str(frequency_hz)
    fields[5] = "UCA"
    fields[6] = "122"
    fields[7] = "TEST_STATION"
    fields[8] = "34.0400"
    fields[9] = "-117.8100"
    fields[10] = "180.0"
    fields[11] = "179.5"
    fields[12] = "GPS"

    for index in range(SPECTRUM_BIN_COUNT):
        fields[SPECTRUM_START_INDEX + index] = "-20.0"
    fields[SPECTRUM_START_INDEX + peak_index] = "0.0"
    fields[SPECTRUM_START_INDEX + ((peak_index + 30) % 360)] = "-6.0"
    return ",".join(fields)


class KrakenLoggerParserTests(unittest.TestCase):
    def test_maps_endpoint_fields_and_discards_spectrum_by_default(self) -> None:
        record = parse_endpoint_response(
            make_endpoint_line(),
            t_rx_ms=1_700_000_000_100,
            run_id="TEST_RUN",
        )[0]

        self.assertEqual(record["source_timestamp_ms"], 1_700_000_000_000)
        self.assertEqual(record["doa_deg"], 0.0)
        self.assertEqual(record["doa_papr_db"], 7.78)
        self.assertEqual(record["rssi_power_db"], -34.2)
        self.assertEqual(record["frequency_hz"], 416_588_000)
        self.assertEqual(record["antenna_array_arrangement"], "UCA")
        self.assertEqual(record["latency_ms"], 122.0)
        self.assertEqual(record["station_id"], "TEST_STATION")
        self.assertEqual(record["lat_deg"], 34.04)
        self.assertEqual(record["lon_deg"], -117.81)
        self.assertEqual(record["gps_heading_deg"], 180.0)
        self.assertEqual(record["compass_heading_deg"], 179.5)
        self.assertEqual(record["heading_sensor"], "GPS")
        self.assertNotIn("doa_spectrum", record)
        self.assertTrue(record["spectrum_parse_valid"])
        self.assertEqual(record["spectrum_bin_count"], 360)

    def test_full_spectrum_is_available_only_in_debug_mode(self) -> None:
        record = parse_endpoint_response(
            make_endpoint_line(),
            t_rx_ms=1_700_000_000_100,
            run_id="TEST_RUN",
            save_spectrum=True,
        )[0]

        self.assertEqual(len(record["doa_spectrum"]), 360)
        self.assertEqual(record["spectrum_peak_unit_deg"], 90.0)
        self.assertEqual(record["spectrum_peak_compass_deg"], 0.0)
        self.assertGreater(record["spectrum_peakiness_0_1"], 0.0)
        self.assertLess(record["spectrum_entropy_0_1"], 1.0)

    def test_multiple_vfo_lines_are_parsed(self) -> None:
        response = "\n".join(
            [
                make_endpoint_line(frequency_hz=416_588_000),
                make_endpoint_line(frequency_hz=416_600_000),
            ]
        )

        records = parse_endpoint_response(
            response,
            t_rx_ms=1_700_000_000_100,
            run_id="TEST_RUN",
        )

        self.assertEqual(len(records), 2)
        self.assertEqual(records[0]["vfo_index"], 0)
        self.assertEqual(records[1]["vfo_index"], 1)
        self.assertEqual(records[0]["frequency_hz"], 416_588_000)
        self.assertEqual(records[1]["frequency_hz"], 416_600_000)

    def test_incomplete_spectrum_keeps_metadata_but_marks_metrics_invalid(self) -> None:
        line = make_endpoint_line().split(",")[:SPECTRUM_START_INDEX + 12]
        record = parse_endpoint_response(
            ",".join(line),
            t_rx_ms=1_700_000_000_100,
            run_id="TEST_RUN",
        )[0]

        self.assertFalse(record["spectrum_parse_valid"])
        self.assertEqual(record["spectrum_bin_count"], 12)
        self.assertEqual(record["doa_papr_db"], 7.78)

    def test_invalid_short_line_is_skipped(self) -> None:
        records = parse_endpoint_response(
            "1,2,3",
            t_rx_ms=1_700_000_000_100,
            run_id="TEST_RUN",
        )
        self.assertEqual(records, [])

    def test_flat_spectrum_has_zero_peakiness(self) -> None:
        metrics = calculate_spectrum_metrics([1.0] * 360)

        self.assertTrue(metrics["spectrum_parse_valid"])
        self.assertEqual(metrics["spectrum_entropy_0_1"], 1.0)
        self.assertEqual(metrics["spectrum_peakiness_0_1"], 0.0)
        self.assertEqual(metrics["peak_width_deg"], 360.0)

    def test_temporal_metrics_use_bounded_circular_history(self) -> None:
        temporal_histories.clear()
        first = update_temporal_metrics(
            {"frequency_hz": 416_588_000, "vfo_index": 0, "doa_deg": 90.0}
        )
        second = update_temporal_metrics(
            {"frequency_hz": 416_588_000, "vfo_index": 0, "doa_deg": 91.0}
        )

        self.assertEqual(first["temporal_sample_count"], 1)
        self.assertIsNone(first["temporal_stability_0_1"])
        self.assertEqual(second["temporal_sample_count"], 2)
        self.assertGreater(second["temporal_stability_0_1"], 0.99)
        self.assertLess(second["temporal_jitter_deg"], 2.0)

    def test_fusion_uses_papr_field_and_propagates_endpoint_metrics(self) -> None:
        fused = fuse(
            [
                {
                    "t_rx_ms": 1_000,
                    "seq": 4,
                    "doa_deg": 90.0,
                    "doa_papr_db": 4.0,
                    "rssi_power_db": -30.0,
                    "frequency_hz": 416_588_000,
                    "peak_width_deg": 12.0,
                }
            ],
            [
                {
                    "t_rx_ms": 1_010,
                    "seq": 8,
                    "lat_deg": 34.04,
                    "lon_deg": -117.81,
                    "roll_deg": 0.0,
                    "ground_speed_ft_s": 10.0,
                }
            ],
            max_time_diff_ms=250,
            min_doa_papr_db=3.0,
            max_roll_deg=60.0,
            min_ground_speed_ft_s=0.0,
        )

        self.assertEqual(len(fused), 1)
        self.assertTrue(fused[0]["usable_for_triangulation"])
        self.assertEqual(fused[0]["doa_papr_db"], 4.0)
        self.assertEqual(fused[0]["rssi_power_db"], -30.0)
        self.assertEqual(fused[0]["peak_width_deg"], 12.0)

    def test_fusion_rejects_missing_position(self) -> None:
        fused = fuse(
            [
                {
                    "t_rx_ms": 1_000,
                    "seq": 4,
                    "doa_deg": 90.0,
                    "doa_papr_db": 4.0,
                }
            ],
            [
                {
                    "t_rx_ms": 1_010,
                    "seq": 8,
                    "lat_deg": None,
                    "lon_deg": None,
                    "roll_deg": 0.0,
                    "ground_speed_ft_s": 10.0,
                }
            ],
            max_time_diff_ms=250,
            min_doa_papr_db=3.0,
            max_roll_deg=60.0,
            min_ground_speed_ft_s=0.0,
        )

        self.assertFalse(fused[0]["usable_for_triangulation"])


if __name__ == "__main__":
    unittest.main()
