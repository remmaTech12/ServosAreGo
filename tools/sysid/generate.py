"""Generate the browser's 5 ms trajectories using upstream BAM."""

import json
from pathlib import Path
from bam.trajectory import trajectories

out = {}
for name in ("up_and_down", "lift_and_drop", "sin_sin", "sin_time_square", "steps"):
    trajectory = trajectories[name]
    out[name] = []
    for i in range(round(trajectory.duration * 200)):
        angle, enabled = trajectory(i / 200)
        out[name].append(
            [round(float(angle) * 4096 / (2 * 3.141592653589793)), int(enabled)]
        )
Path(__file__).with_name("trajectories.json").write_text(
    json.dumps(out, separators=(",", ":")) + "\n"
)
