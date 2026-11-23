"""Record a BAM pendulum trial or a complete gain sweep over USB serial."""

import argparse
import json
import math
import time
from pathlib import Path

import serial

TRAJECTORIES = json.loads(Path(__file__).with_name("trajectories.json").read_text())
SCALE = 2 * math.pi / 4096


def request(port, args):
    port.write(f"@sysid {args}\n".encode())
    deadline = time.monotonic() + 4
    while time.monotonic() < deadline:
        line = port.readline().decode(errors="replace")
        if "@{" in line:
            reply = json.loads(line[line.index("@{") + 1 :])
            if reply.get("re") == "sysid":
                if not reply["ok"]:
                    raise RuntimeError(reply["message"])
                return reply
    raise TimeoutError(args)


def record(port, args, trajectory, kp):
    points = TRAJECTORIES[trajectory]
    limit = math.floor(math.radians(args.limit) / SCALE)
    low, high = max(0, args.zero - limit), min(4095, args.zero + limit)
    if any(not low <= args.zero + q <= high for q, _ in points):
        raise ValueError(f"{trajectory} exceeds travel limits")
    model = request(port, f"inspect {args.id}")["model"]
    run = request(port, f"prepare {args.id} {len(points)} 5000 {kp} {low} {high}")[
        "run"
    ]
    print(f"Trial {run}: {trajectory}, Kp={kp}", flush=True)
    try:
        for offset in range(0, len(points), 12):
            chunk = " ".join(
                f"{q+args.zero},{on}" for q, on in points[offset : offset + 12]
            )
            request(port, f"upload {run} {offset} {chunk}")
        request(port, f"start {run}")
        while True:
            time.sleep(0.25)
            status = request(port, f"status {run}")
            if status["done"]:
                break
        entries, diagnostics = [], []
        while len(entries) < status["count"]:
            for cmd, t, q, on, pos, speed, pwm, current, volts, temp, _ in request(
                port, f"read {run} {len(entries)}"
            )["rows"]:
                entries.append(
                    dict(
                        timestamp=t / 1e6,
                        goal_position=(q - args.zero) * SCALE,
                        torque_enable=bool(on),
                        position=(pos - args.zero) * SCALE,
                        speed=speed * 0.229 * math.pi / 30,
                        load=max(-1, min(1, pwm / 885)),
                        input_volts=volts / 10,
                        temp=temp,
                    )
                )
                diagnostics.append(dict(command_us=cmd, current_raw=current))
        result = status["message"]
        directory = args.out if result == "complete" else args.out / "partial"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{time.time_ns()}-{trajectory}-{kp}.json"
        data = dict(
            motor=model,
            trajectory=trajectory,
            kp=kp,
            vin=args.vin,
            mass=args.mass,
            length=args.length,
            entries=entries,
            diagnostics=diagnostics,
            result=result,
        )
        data["arm-mass"] = args.arm_mass
        data["arm_mass"] = args.arm_mass
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(data, allow_nan=False))
        temp.replace(path)
        request(port, f"release {run}")
        if result != "complete":
            raise RuntimeError(result)
    except KeyboardInterrupt:
        request(port, f"abort {run}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    for key in ("mass", "arm-mass", "length", "limit"):
        parser.add_argument("--" + key, required=True, type=float)
    parser.add_argument("--zero", required=True, type=int)
    parser.add_argument("--vin", default=5.0, type=float)
    parser.add_argument("--id", default=1, type=int)
    parser.add_argument("--kp", default=100, type=int)
    parser.add_argument("--trajectory", choices=TRAJECTORIES, default="up_and_down")
    parser.add_argument("--standard", action="store_true")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    if not (
        0 < args.limit <= 180
        and args.mass >= 0
        and args.arm_mass >= 0
        and args.length > 0
        and 3.7 <= args.vin <= 6
    ):
        parser.error("invalid bench parameters")
    with serial.Serial(args.port, 115200, timeout=0.5) as port:
        time.sleep(2)
        # Start the stored DYNAMIXEL setter after the boot menu timeout.
        time.sleep(5)
        for kp in ([50, 100, 200, 300, 400] if args.standard else [args.kp]):
            for name in (
                ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]
                if args.standard
                else [args.trajectory]
            ):
                record(port, args, name, kp)
