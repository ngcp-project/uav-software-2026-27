import json
import sys
import csv

FT_PER_DEG_LAT = 364000
log_file_path = "test_3.csv"
# log_file_path = "ngcp-uav-software-2025-26/logs/fusion/fusion_20260801_144502.jsonl"

def has_gps(lat, lon):
  if lat is None or lon is None:
    return False
  return not (abs(lat) < 1 and abs(lon) < 1)



def convert_csv_to_json(cols):
  cols = [c.strip() for c in cols]
  return {
    "t_rx_ms": int(cols[0]),
    "doa_deg": float(cols[1]),
    "confidence": float(cols[2]),
    "rssi_db": float(cols[3]),
    "freq_hz": int(cols[4]),
    "array_type": cols[5],
    "latency_ms": float(cols[6]),
    "station_id": cols[7],
    "lat_deg": float(cols[8]),
    "lon_deg": float(cols[9]),
    "gps_heading_deg": float(cols[10]),
    "compass_heading_deg": float(cols[11]),
    "heading_source": cols[12],
    "spectrum": [float(v) for v in cols[17:377]],
  }


def csv_to_json(csv_file_path, json_file_path):
  with open(csv_file_path, "r", encoding="utf-8") as csv_file:
    with open(json_file_path, "w", encoding="utf-8") as json_file:
      for cols in csv.reader(csv_file):
        if len(cols) < 377:
          continue
        try:
          log_entry = convert_csv_to_json(cols)
        except ValueError:
          continue
        json.dump(log_entry, json_file)
        json_file.write("\n")


kept = skipped = 0

if log_file_path.endswith(".csv"):
  json_copy_path = log_file_path.rsplit(".", 1)[0] + ".jsonl"
  csv_to_json(log_file_path, json_copy_path)
  log_file_path = json_copy_path


with open(log_file_path, "r", encoding="utf-8") as file:
  for line in file:
    line = line.strip()
    if not line:
      continue
    if line:
      try:
        log_entry = json.loads(line)
      except json.JSONDecodeError:
        skipped += 1
        continue
    if  log_entry.get("usable_for_triangulation") is False:
      skipped += 1
      continue
    if not has_gps(log_entry.get("lat_deg"), log_entry.get("lon_deg")):
      skipped += 1
      continue
    heading = log_entry.get("yaw_deg", log_entry.get("gps_heading_deg"))
    if "gps_heading_deg" in log_entry and heading == 0:
      skipped += 1
      continue
    kept += 1
    print(
      log_entry.get("t_rx_ms"),
      log_entry.get("lat_deg"),
      log_entry.get("lon_deg"),
      heading,
      log_entry.get("doa_deg"),
      log_entry.get("confidence", log_entry.get("confidence_0_1")),
      )
  print(f"Kept {kept} entries, skipped {skipped}")
