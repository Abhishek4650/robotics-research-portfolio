#!/usr/bin/env python3
"""ikpy on the rev I.1 URDF, run in its own interpreter (python3 -s + system
numpy 1.26): the user-site numpy 2.x breaks ikpy's sympy link maths.
  fk: {"q": [[6]...]}                  -> {"T": [[4x4] (mm)...]}
  ik: {"targets": [[4x4] (mm)...], "seed": [6] | "seeds": [[6]...]} -> {"q": [[6]...]}"""
import json
import os
import sys
import time

import numpy as np
from ikpy.chain import Chain

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "EDITABLE_CAD", "urdf", "arm450.urdf")


def main():
    inp = json.load(open(sys.argv[1]))
    chain = Chain.from_urdf_file(URDF, base_elements=["ground"],
                                 active_links_mask=[False, True, True, True, True, True, True])
    out = {}
    if "q" in inp:
        Ts = []
        for q in inp["q"]:
            T = chain.forward_kinematics([0.0] + list(q)); T[:3, 3] *= 1000.0
            Ts.append(T.tolist())
        out["T"] = Ts
    if "targets" in inp:
        qs, ts = [], []
        seeds = inp.get("seeds") or [inp["seed"]] * len(inp["targets"])
        for T, sd in zip(inp["targets"], seeds):
            T = np.array(T); T[:3, 3] /= 1000.0
            t0 = time.perf_counter()
            q = chain.inverse_kinematics(target_position=T[:3, 3], target_orientation=T[:3, :3],
                                         orientation_mode="all", initial_position=[0.0] + list(sd))
            ts.append((time.perf_counter() - t0) * 1e3)
            qs.append([float(v) for v in q[1:]])
        out["q"], out["time_ms"] = qs, ts
    json.dump(out, open(sys.argv[2], "w"))


if __name__ == "__main__":
    main()
