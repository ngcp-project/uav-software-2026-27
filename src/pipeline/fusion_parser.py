import json
log_file_path = "ngcp-uav-software-2025-26/logs/fusion/fusion_20260801_144502.jsonl"
with open(log_file_path, "r", encoding="utf-8") as file:
  for line in file:
    line = line.strip()
    if not line:
      continue
    if line:
      try:
        log_entry = json.loads(line)
      except json.JSONDecodeError:
        continue
    if not log_entry["usable_for_triangulation"]:
      continue
    print(
      log_entry["t_rx_ms"],
      log_entry["lat_deg"],
      log_entry["lon_deg"],
      log_entry["yaw_deg"],
      log_entry["doa_deg"],
      log_entry["confidence_0_1"]
      )