# Kraken Logger and Fusion Output

The updated pipeline now has three layers:

1. Raw Kraken endpoint data.
2. Temporary spectrum-derived metrics.
3. Fused Kraken + telemetry output.

## 1. New Kraken logger fields

The full spectrum is parsed in memory and discarded by default. It is only written if `KRAKEN_SAVE_SPECTRUM=1` is enabled in [kraken_logger.py](</src/loggers/kraken_logger.py:40>).

### Identification and timing

| Field | Meaning |
|---|---|
| `t_rx_ms` | Local time when the HTTP response was received. |
| `source_timestamp_ms` | Timestamp supplied by Kraken itself. |
| `run_id` | Logger session identifier. |
| `seq` | Local sequential record number. |
| `vfo_index` | Which VFO/channel produced the line. |
| `raw_field_count` | Number of CSV fields received. |

### Direct Kraken measurements

| Field | Meaning | How to interpret |
|---|---|---|
| `doa_deg` | Kraken’s strongest bearing | Compass convention: `0° = North`, `90° = East`. |
| `doa_papr_db` | Kraken peak-sharpness metric | Higher usually means a sharper DoA peak. It is not a probability. |
| `rssi_power_db` | Relative received power | Closer to `0 dB` generally means stronger signal. |
| `frequency_hz` | Active frequency/VFO | Identifies which RF channel produced the reading. |
| `antenna_array_arrangement` | `UCA`, `ULA`, or `Custom` | Describes the antenna geometry. |
| `latency_ms` | Kraken processing latency | Lower is generally better, but this excludes network latency. |
| `station_id` | Kraken station name | Metadata only. |
| `lat_deg`, `lon_deg` | Kraken station position | This is separate from the UAV telemetry position. |
| `gps_heading_deg` | GPS-derived Kraken heading | May be unavailable. |
| `compass_heading_deg` | Compass-derived Kraken heading | May be unavailable. |
| `heading_sensor` | Heading source used | Usually GPS or Compass. |

These fields are parsed in [parse_endpoint_line](</src/loggers/kraken_logger.py:215>).

## 2. Temporary spectrum-derived fields

The logger uses the 360 spectrum to calculate compact summaries.

| Field | Meaning | Prefer |
|---|---|---|
| `spectrum_parse_valid` | Whether all 360 spectrum values were valid | Must be `true`. |
| `spectrum_bin_count` | Number of valid spectrum bins | Should be `360`. |
| `spectrum_peak_unit_deg` | Strongest spectrum bin using Kraken’s unit-circle convention | Diagnostic/reference value. |
| `spectrum_peak_compass_deg` | Same peak converted to compass convention | Compare against `doa_deg`. |
| `peak_width_deg` | Approximate width of the main peak at `-3 dB` | Lower usually means a sharper bearing. |
| `peak_to_second_peak_db` | Separation between the main peak and competing peak | Higher usually means less ambiguity. |
| `spectrum_entropy_0_1` | How spread out the spectrum is | Lower usually means more concentrated. |
| `spectrum_peakiness_0_1` | `1 - entropy` | Higher usually means more concentrated. |

These calculations are implemented in [calculate_spectrum_metrics](</src/loggers/kraken_logger.py:101>).

## 3. Temporary time-based fields

The logger maintains a bounded in-memory history of recent bearings:

| Field | Meaning | Prefer |
|---|---|---|
| `temporal_sample_count` | Number of recent samples used | Larger is more meaningful. |
| `temporal_stability_0_1` | Circular consistency of recent bearings | Higher is better. |
| `temporal_jitter_deg` | Circular bearing variation | Lower is better. |

These are calculated by [update_temporal_metrics](</src/loggers/kraken_logger.py:177>).

Important: these temporal fields currently exist in the Kraken JSONL but are not forwarded into the fusion JSONL by [fuse](</src/loggers/fusion_logger.py:91>).

## What the fusion output looks like

A new fusion record will contain both Kraken and UAV telemetry fields:

```json
{
  "run_id": "20261112_120000",
  "kraken_seq": 123,
  "telemetry_seq": 456,

  "t_rx_ms": 1794500000123,
  "dt_ms": 18,

  "doa_deg": 105.0,
  "doa_papr_db": 7.78,
  "rssi_power_db": -34.2,
  "frequency_hz": 416588000,
  "antenna_array_arrangement": "UCA",
  "kraken_latency_ms": 122.0,

  "spectrum_peak_unit_deg": 90.0,
  "spectrum_peak_compass_deg": 0.0,
  "peak_width_deg": 13.0,
  "peak_to_second_peak_db": 5.4,
  "spectrum_entropy_0_1": 0.21,
  "spectrum_peakiness_0_1": 0.79,
  "spectrum_parse_valid": true,

  "lat_deg": 34.0401,
  "lon_deg": -117.8102,
  "altitude_rel_ft": 85.4,
  "yaw_deg": 42.1,
  "pitch_deg": 2.4,
  "roll_deg": 4.2,
  "ground_speed_ft_s": 38.5,
  "vel_north_m_s": 8.1,
  "vel_east_m_s": 2.3,
  "vel_down_m_s": 0.1,

  "usable_for_triangulation": true
}
```

The fusion fields are assembled in [fusion_logger.py](</src/loggers/fusion_logger.py:183>).

The existing archived fusion files, such as [fusion_20260801_144502.jsonl](</logs/fusion/fusion_20260801_144502.jsonl:1>), use the old schema and still contain `confidence_0_1`. New records from the updated logger will use `doa_papr_db`.

## How to identify the best bearings

Do not rank bearings using `doa_papr_db` alone. A sharp peak can still be caused by multipath or interference.

A strong candidate should generally have:

```text
spectrum_parse_valid == true
usable_for_triangulation == true
high doa_papr_db
high spectrum_peakiness_0_1
low spectrum_entropy_0_1
low peak_width_deg
high peak_to_second_peak_db
high temporal_stability_0_1
low temporal_jitter_deg
small abs(dt_ms)
reasonable roll
valid UAV position
```

The most useful fields are:

1. `spectrum_peakiness_0_1` — Is the DoA spectrum concentrated?
2. `peak_width_deg` — Is the main bearing narrow?
3. `peak_to_second_peak_db` — Is there a competing direction?
4. `temporal_stability_0_1` — Does the bearing remain stable?
5. `temporal_jitter_deg` — How much does it fluctuate?
6. `doa_papr_db` — How sharp is Kraken’s primary peak?
7. `dt_ms` — Are Kraken and telemetry measurements synchronized?
8. `roll_deg` and position validity — Is the UAV state suitable for triangulation?

`usable_for_triangulation` is only a gate. In the current [is_usable](</src/loggers/fusion_logger.py:86>) logic, it checks PAPR threshold, DoA range, position, roll, time difference, and ground speed. It does not yet use peak width, entropy, peak separation, or temporal stability.

So for actual ranking, use `usable_for_triangulation` first, then score the spectrum and temporal fields. A future fusion improvement should forward the three temporal fields into the fusion output so all bearing-selection logic can operate from one file.
