import argparse, csv, math, subprocess, time

def detach(detach_topic: str):
    subprocess.run(["gz","topic","-t",detach_topic,"-m","gz.msgs.Empty","-p",""], check=False)

def iter_pose_blocks(lines):
    block = None
    depth = 0
    for line in lines:
        s = line.strip()
        if s.startswith("pose {"):
            block = [line]
            depth = 1
            continue
        if block is not None:
            block.append(line)
            depth += s.count("{")
            depth -= s.count("}")
            if depth <= 0:
                yield block
                block = None
                depth = 0

def parse_pose_block(block_lines):
    """Parse only position {x,y,z} (ignore orientation x/y/z)."""
    name = None
    x = y = z = None
    in_pos = False
    pos_depth = 0

    for line in block_lines:
        s = line.strip()

        if s.startswith('name:'):
            name = s.split(":",1)[1].strip().strip('"')

        if s.startswith("position {"):
            in_pos = True
            pos_depth = 1
            continue

        if in_pos:
            pos_depth += s.count("{")
            pos_depth -= s.count("}")
            if s.startswith("x:"):
                x = float(s.split(":",1)[1].strip())
            elif s.startswith("y:"):
                y = float(s.split(":",1)[1].strip())
            elif s.startswith("z:"):
                z = float(s.split(":",1)[1].strip())

            if pos_depth <= 0:
                in_pos = False

    # If any are missing, treat as no valid world position
    if x is None or y is None or z is None:
        return name, None, None, None
    return name, x, y, z

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-x", type=float, required=True)
    ap.add_argument("--target-y", type=float, required=True)
    ap.add_argument("--dx", type=float, required=True)
    ap.add_argument("--dy", type=float, required=True)
    ap.add_argument("--world", type=str, default="payload_drop")
    ap.add_argument("--detach-topic", type=str, default="/payload/detach")
    ap.add_argument("--csv", type=str, default="payload_trajectory_target.csv")
    ap.add_argument("--drop-tol", type=float, default=0.20)
    ap.add_argument("--landed-z", type=float, default=0.55)  # 1m cube => ~0.5
    args = ap.parse_args()

    pose_info = f"/world/{args.world}/pose/info"

    x_release = args.target_x - args.dx
    y_release = args.target_y - args.dy

    print(f"[INFO] World pose topic: {pose_info}")
    print(f"[INFO] Target: x={args.target_x:.2f}, y={args.target_y:.2f}")
    print(f"[INFO] Using calibration dx={args.dx:.2f}, dy={args.dy:.2f}")
    print(f"[INFO] Required release point: x_release={x_release:.2f}, y_release={y_release:.2f}")
    print(f"[NOTE] Set carrier/payload start Y in payload_drop_world.sdf to y_release ({y_release:.2f}).\n")

    p = subprocess.Popen(["gz","topic","-e","-t",pose_info],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)

    t0 = time.time()
    last = None
    released = False
    release_state = None
    landed_since = None
    first_payload_pose = None

    LANDED_SPEED = 0.30
    LANDED_STABLE_TIME = 1.0

    with open(args.csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec","x","y","z","vx","vy","vz","speed"])

        try:
            for block in iter_pose_blocks(p.stdout):
                name, x, y, z = parse_pose_block(block)

                # In pose/info, the model name is usually "payload"
                if name != "payload":
                    continue
                if x is None:
                    continue  # no valid position in this block

                now = time.time()
                t = now - t0

                if first_payload_pose is None:
                    first_payload_pose = (x, y, z, t)

                if last is None:
                    vx = vy = vz = 0.0
                else:
                    dt = now - last["time"]
                    if dt > 1e-6:
                        vx = (x - last["x"]) / dt
                        vy = (y - last["y"]) / dt
                        vz = (z - last["z"]) / dt
                    else:
                        vx = vy = vz = 0.0

                speed = math.sqrt(vx*vx + vy*vy + vz*vz)
                w.writerow([f"{t:.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}",
                            f"{vx:.3f}", f"{vy:.3f}", f"{vz:.3f}", f"{speed:.3f}"])
                f.flush()

                last = {"x":x,"y":y,"z":z,"time":now}

                # If the sim wasn't restarted, you'll start already past x_release
                if (not released) and first_payload_pose and (first_payload_pose[0] > x_release + 5):
                    print(f"[ERROR] Payload already at x={first_payload_pose[0]:.2f} > x_release={x_release:.2f}.")
                    print("        Restart Gazebo (reload the world) and run this script again.")
                    return

                # Drop when reaching release X
                if (not released) and (x >= (x_release - args.drop_tol)):
                    release_state = (x, y, z, t)
                    print(f"[EVENT] RELEASE @ t={t:.2f}s  x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    detach(args.detach_topic)
                    released = True

                # Landing detection
                if released:
                    if z <= args.landed_z and speed <= LANDED_SPEED:
                        if landed_since is None:
                            landed_since = now
                        elif now - landed_since >= LANDED_STABLE_TIME:
                            print(f"[EVENT] LANDED  @ t={t:.2f}s  x={x:.2f}, y={y:.2f}, z={z:.2f}")
                            ex = x - args.target_x
                            ey = y - args.target_y
                            print(f"[ERROR] landing - target: ex={ex:.2f} m, ey={ey:.2f} m")
                            return
                    else:
                        landed_since = None

        finally:
            p.terminate()

if __name__ == "__main__":
    main()
