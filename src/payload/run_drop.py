import csv, math, subprocess, time

WORLD = "payload_drop"
POSE_INFO_TOPIC = f"/world/{WORLD}/pose/info"
DETACH_TOPIC = "/payload/detach"

# Payload is a 1m cube in the world file -> center rests ~0.5m above ground
LANDED_Z = 0.55
LANDED_SPEED = 0.30
LANDED_STABLE_TIME = 1.0

def detach():
    subprocess.run(["gz","topic","-t",DETACH_TOPIC,"-m","gz.msgs.Empty","-p",""], check=False)

def iter_pose_blocks(lines):
    """Yield each 'pose { ... }' block as a list of lines, from gz topic echo output."""
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
            # crude brace depth tracking
            depth += s.count("{")
            depth -= s.count("}")
            if depth <= 0:
                yield block
                block = None
                depth = 0

def parse_pose_block(block_lines):
    """Return (name, x, y, z) for a pose block. Missing x/y/z -> 0.0."""
    name = None
    x = y = z = 0.0
    for line in block_lines:
        s = line.strip()
        if s.startswith('name:'):
            # name: "payload"
            name = s.split(":",1)[1].strip().strip('"')
        elif s.startswith("x:"):
            x = float(s.split(":",1)[1].strip())
        elif s.startswith("y:"):
            y = float(s.split(":",1)[1].strip())
        elif s.startswith("z:"):
            z = float(s.split(":",1)[1].strip())
    return name, x, y, z

def main():
    print(f"[INFO] Reading world poses from: {POSE_INFO_TOPIC}")
    print(f"[INFO] Detach topic: {DETACH_TOPIC}")
    print("[INFO] Will auto-drop ~3 seconds after first payload pose.\n")

    p = subprocess.Popen(["gz","topic","-e","-t",POSE_INFO_TOPIC],
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, bufsize=1)

    t0 = time.time()
    last = None
    released = False
    release_state = None
    landed_since = None
    first_payload_time = None
    last_live_print = 0.0

    with open("payload_trajectory.csv","w",newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sec","x","y","z","vx","vy","vz","speed"])

        try:
            for block in iter_pose_blocks(p.stdout):
                name, x, y, z = parse_pose_block(block)
                if name != "payload":
                    continue

                now = time.time()
                t = now - t0

                if first_payload_time is None:
                    first_payload_time = now

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

                # live print ~2x/sec so it's obvious it works
                if t - last_live_print > 0.5:
                    print(f"[LIVE] t={t:6.2f}  x={x:8.2f}  y={y:8.2f}  z={z:6.2f}  speed={speed:5.2f}")
                    last_live_print = t

                # auto release ~3 sec after first payload pose
                if (not released) and (now - first_payload_time > 3.0):
                    release_state = (x, y, z, t)
                    print(f"\n[EVENT] RELEASE  x={x:.2f}, y={y:.2f}, z={z:.2f} (t={t:.2f}s)")
                    detach()
                    released = True

                # landing detection after release
                if released:
                    if z <= LANDED_Z and speed <= LANDED_SPEED:
                        if landed_since is None:
                            landed_since = now
                        elif now - landed_since >= LANDED_STABLE_TIME:
                            print(f"\n[EVENT] LANDED   x={x:.2f}, y={y:.2f}, z={z:.2f} (t={t:.2f}s)")
                            rx, ry, rz, rt = release_state
                            print(f"[RESULT] dx={x-rx:.2f} m, dy={y-ry:.2f} m (landing - release)")
                            return
                    else:
                        landed_since = None

        finally:
            p.terminate()

if __name__ == "__main__":
    main()
