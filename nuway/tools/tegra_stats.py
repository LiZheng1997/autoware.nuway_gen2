#!/usr/bin/env python3
"""Summarise a tegrastats log: GPU, per-core CPU, RAM and power rails.

tegrastats lines look like
  RAM 6619/62840MB (lfb ...) SWAP ... CPU [0%@2201,4%@2201,...] GR3D_FREQ 0% cpu@50.7C
  ... VDD_GPU_SOC 4422mW/4422mW VDD_CPU_CV 1205mW/1205mW VIN_SYS_5V0 4327mW/4327mW
GR3D_FREQ is the GPU utilisation counter.

Collect with:  timeout 120 tegrastats --interval 1000 > tegra.log
Usage:         tegra_stats.py tegra.log [more.log ...]
"""
import re
import sys

import numpy as np


def parse(path):
    ram, gpu, cpu_tot, cpu_cores, power = [], [], [], [], {}
    for line in open(path):
        m = re.search(r"RAM (\d+)/(\d+)MB", line)
        if m:
            ram.append((int(m.group(1)), int(m.group(2))))
        m = re.search(r"GR3D_FREQ (\d+)%", line)
        if m:
            gpu.append(int(m.group(1)))
        m = re.search(r"CPU \[([^\]]+)\]", line)
        if m:
            vals = [int(x.split("%")[0]) for x in m.group(1).split(",") if "%" in x]
            cpu_tot.append(sum(vals))
            cpu_cores.append(vals)
        for rail in ("VDD_GPU_SOC", "VDD_CPU_CV", "VIN_SYS_5V0"):
            m = re.search(rail + r" (\d+)mW", line)
            if m:
                power.setdefault(rail, []).append(int(m.group(1)))
    return ram, gpu, cpu_tot, cpu_cores, power


def q(v, p):
    return np.percentile(v, p) if v else float("nan")


for path in sys.argv[1:]:
    ram, gpu, cpu_tot, cpu_cores, power = parse(path)
    n = len(gpu)
    print(f"\n=== {path}  ({n} samples) ===")
    if not n:
        print("  no data")
        continue
    ncore = len(cpu_cores[0])
    print(f"GPU (GR3D):  median {q(gpu,50):.0f}%  p95 {q(gpu,95):.0f}%  max {max(gpu)}%  "
          f"zero in {100*sum(1 for g in gpu if g==0)/n:.0f}% of samples")
    print(f"CPU total ({ncore} cores = {ncore*100}%):  median {q(cpu_tot,50):.0f}%  "
          f"p95 {q(cpu_tot,95):.0f}%  max {max(cpu_tot)}%  "
          f"-> {q(cpu_tot,50)/100:.1f}/{ncore} cores")
    busy = (np.array(cpu_cores) > 80).mean(axis=0) * 100
    print("per-core share of time above 80%: " + " ".join(f"{b:.0f}" for b in busy))
    print(f"RAM: median {q([r[0] for r in ram],50)/1024:.1f} GB / {ram[0][1]/1024:.1f} GB")
    for rail, v in power.items():
        print(f"{rail:<14} median {q(v,50)/1000:.2f} W  p95 {q(v,95)/1000:.2f} W  "
              f"max {max(v)/1000:.2f} W")
