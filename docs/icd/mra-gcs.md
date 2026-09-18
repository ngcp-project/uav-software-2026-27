# GCS-MRA Interface Control Document

## Link Management & Time Synchronization
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| HEARTBEAT | MRA | GCS | 1 Hz | mra_state (str), flight_mode (str), armed (bool), in_air (bool), last_cmd_seq (int), link_rssi (int), battery_pct (float) | SEARCH, AUTO, true, true, 1842, -68, 74.0 | Basic liveness and state heartbeat. |
| LINK_STATUS | MRA | GCS | 0.5–1 Hz | link_state (str), rtt_ms (int), packet_loss_pct (float), rssi (int) | DEGRADED, 180, 12.5, -79 | Communications health telemetry. |
| LOST_LINK_EVENT | MRA | GCS | Event-driven | last_contact_time (str), failsafe_action (str), current_pos (Tuple[float,float,float]) | 21:35Z, RTL, (34.02,-118.48,120) | MRA enters lost-link failsafe. |
| LINK_RECOVERED_EVENT | MRA | GCS | Event-driven | recovery_time (str), current_mode (str), resume_capable (bool) | 21:36Z, LOITER, true | Link restored notification. |
| TIME_SYNC | GCS | MRA | On connect / hourly | timestamp_utc (str), time_source (str) | 2026-02-06T21:14:33Z, GNSS | Synchronizes time. |

## Mission Configuration & Control
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| MISSION_ASSIGN | GCS | MRA | Event-driven | mission_id (str), search_area_polygon (List[Tuple[float,float]]), search_alt_m (float), search_pattern (str), geofence_polygon (List[Tuple[float,float]]), rtb_point (Tuple[float,float,float]), drop_constraints (Dict[str,str]) | M-381, [...], 120.0, lawnmower, [...], (..), {min_alt:30} | Mission geometry and constraints. |
| MISSION_ACK | MRA | GCS | Event-driven | mission_id (str), accepted (bool), reject_reason (Optional[str]) | M-381, true, null | Confirms mission acceptance. |
| MODE_CHANGE | GCS | MRA | Event-driven | requested_mode (str), reason (str) | AUTO, Resume mission | Explicit mode change. |
| CMD_ABORT_MISSION | GCS | MRA | Event-driven | abort_reason (str), immediate_action (str) | AIRSPACE_CONFLICT, RTB | Terminates mission. |

## Preflight, Takeoff & Airspace Coordination
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| PREFLIGHT_STATUS | MRA | GCS | On request | checks_passed (bool), failures (List[str]), battery_pct (float), gps_ok (bool), payload_ok (bool), storage_ok (bool) | false, [PAYLOAD_NOT_DETECTED], 88.0, true, false, true | Preflight readiness. |
| TAKEOFF_REQUEST | MRA | GCS | Event-driven | requested_time_window (Tuple[str,str]), requested_alt_m (float), launch_site (Tuple[float,float]), reason (str), airspace_slot_request_id (str) | 21:20–21:30Z, 120.0, (..), Start mission, SLOT-77 | Requests takeoff. |
| TAKEOFF_CLEARANCE | GCS | MRA | Event-driven | approved (bool), clearance_id (str), valid_until (str), assigned_slot_id (str), conditions (List[str]) | true, CLR-991, 21:28Z, SLOT-77, [max_alt=150] | Grants takeoff. |
| TAKEOFF_DENIED | GCS | MRA | Event-driven | approved (bool), deny_reason (str), retry_after_s (int) | false, MEA_IN_AIR, 120 | Denies takeoff. |
| TAKEOFF_STARTED | MRA | GCS | Event-driven | clearance_id (str), takeoff_time (str), takeoff_latlon (Tuple[float,float]), target_alt_m (float) | CLR-991, 21:23Z, (..), 120.0 | Takeoff committed. |
| TAKEOFF_ABORTED | MRA | GCS | Event-driven | clearance_id (str), abort_reason (str), safe_state (str) | CLR-991, MOTOR_FAULT, DISARMED | Takeoff aborted. |

## Navigation & Vehicle Control
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| STATUS_REPORT | MRA | GCS | 1–2 Hz | lat (float), lon (float), alt_m (float), vel_mps (float), heading_deg (float), battery_pct (float), est_time_remaining_s (int), gps_fix (str), hdop (float), payload_status (str), current_task (str) | 34.02, -118.48, 120, 12.2, 87, 72, 900, 3D, 0.9, READY, SEARCH | Full telemetry. |
| CMD_HOLD | GCS | MRA | Event-driven | hold_type (str), hold_point (Tuple[float,float,float]), radius_m (float), duration_s (int) | LOITER, (..), 80, 300 | Hold/loiter command. |
| CMD_RTB | GCS | MRA | Event-driven | rtb_reason (str), rtb_point (Tuple[float,float,float]), land_on_arrival (bool) | LINK_RISK, (..), true | Return to base. |
| CMD_LAND | GCS | MRA | Event-driven | land_point (Tuple[float,float,float]), approach_heading_deg (float), reason (str) | (..), 270, End mission | Landing command. |

## Search & Survivor Detection
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| SEARCH_START | GCS | MRA | Event-driven | mission_id (str), start_time (str), search_area_id (str) | M-381, 21:24Z, AREA-A | Start search. |
| SEARCH_PROGRESS | MRA | GCS | 0.2–1 Hz | search_area_id (str), percent_covered (float), last_sweep_line_id (str), detections_count (int) | AREA-A, 43.2, L12, 0 | Search status. |
| DETECTION_REPORT | MRA | GCS | Event-driven | detection_type (str), lat (float), lon (float), confidence (float), sensor (str), snapshot_ref (str), uncertainty_m (float) | SURVIVOR, (..), 0.87, EO, img_01821, 6.5 | Candidate detection. |
| SURVIVOR_FOUND | MRA | GCS | Event-driven | survivor_id (str), lat (float), lon (float), confidence (float), timestamp_utc (str), uncertainty_m (float), method (str) | S-1, (..), 0.93, 21:29Z, 4.2, VISION | Survivor confirmed. |
| SURVIVOR_FOUND_ACK | GCS | MRA | Event-driven | survivor_id (str), received (bool), next_action (str) | S-1, true, PROCEED_TO_DROP | Acknowledgment. |
| SURVIVOR_FOUND_FROM_GCS | GCS | MRA | Event-driven | survivor_id (str), lat (float), lon (float), source (str), confidence (float), timestamp_utc (str), uncertainty_m (float) | S-1, (..), UGV, 0.88, 21:27Z, 7.0 | Survivor via UGV. |
| LOCATION_CONFLICT | MRA | GCS | Event-driven | survivor_id (str), loc_a (Tuple[float,float]), loc_b (Tuple[float,float]), delta_m (float), recommended_action (str) | S-1, MRA_LOC, UGV_LOC, 35.4, REQUEST_RECONFIRM | Location conflict. |
| NAV_TO_SURVIVOR_STARTED | MRA | GCS | Event-driven | survivor_id (str), eta_s (int), route_summary (str) | S-1, 180, direct | Navigate to survivor. |

## Payload / Supply Drop Operations
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| DROP_ARM_REQUEST | MRA | GCS | Event-driven | survivor_id (str), planned_drop_point (Tuple[float,float,float]), planned_alt_m (float), wind_est_mps (float), safety_checks (List[str]) | S-1, (..), 35, 6.2, [zone_clear] | Request payload arm. |
| DROP_ARM_CLEARANCE | GCS | MRA | Event-driven | approved (bool), clearance_id (str), constraints (List[str]) | true, DROPCLR-55, [min_alt=25] | Authorize arm. |
| DROP_DENIED | GCS | MRA | Event-driven | approved (bool), deny_reason (str), next_step (str) | false, CIVILIANS_NEARBY, HOLD | Deny drop. |
| DROP_EXECUTE | MRA | GCS | Event-driven | clearance_id (str), actual_drop_point (Tuple[float,float,float]), drop_time (str), altitude_m (float) | DROPCLR-55, (..), 21:34Z, 32 | Execute drop. |
| DROP_RESULT | MRA | GCS | Event-driven | clearance_id (str), success (bool), reason_if_fail (Optional[str]), payload_state (str), evidence_ref (str) | DROPCLR-55, true, null, EMPTY, img_01910 | Drop result. |
| POST_TASK_STATUS | MRA | GCS | Event-driven | task_completed (str), next_intent (str), battery_pct (float), eta_rtb_s (int) | DROP, RTB, 48, 210 | Post-task summary. |

## Health Monitoring & FDIR
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| FDIR_EVENT | MRA | GCS | Event-driven | fault_id (str), fault_class (str), severity (str), detected_by (str), effect (str), mitigation_action (str) | BAT_LOW, POWER, WARN, BMS, reduced_endurance, RTB | Fault reporting. |
| HEALTH_DETAIL | MRA | GCS | On request | subsystem (str), metrics (Dict[str,float]), thresholds (Dict[str,float]) | GPS, {sat:9,hdop:2.1}, {hdop_warn:2.0} | Detailed health. |
| CMD_REQUEST_HEALTH | GCS | MRA | Event-driven | subsystems (List[str]) | [GPS, PAYLOAD, MOTORS] | Health request. |

## Acknowledgment & Reliability
| Message Name | Sender | Receiver | Frequency | Data Fields | Example Values | Description |
|-------------|--------|----------|-----------|-------------|----------------|-------------|
| ACK | Either | Either | Per message | ack_for_msg_id (str), status (str), error_code (Optional[str]), error_text (Optional[str]) | c0a8…, OK, null, null | Generic ACK/NACK. |

