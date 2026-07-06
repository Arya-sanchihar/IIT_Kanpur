# SAD Optical Flow + EIS

**Vision subsystem** — Real-time adjacent stabilization on a 64 KB RAM Cortex-M4.

## Problem Statement

Dense optical flow (Lucas-Kanade, Horn-Schunck) needs iterative solving and heavy CPU/RAM — infeasible on a Cortex-M4 with 64 KB RAM and no hardware float throughput. A stabilized real-time feed still needs reliable per-frame motion estimation on this hardware.

## Methodology

- SAD block-matching, integer-only, accelerated with Cortex-M4 SIMD (4 abs-diffs/cycle)
- Pipeline: variance-based low-texture rejection → full-search SAD → histogram outlier filter → subpixel parabolic refinement → 2D Kalman smoothing → EIS crop-correction
- Benchmarked on STM32F303RE (Nucleo-64) against the MPI-Sintel dataset (Middlebury "Army" had insufficient motion for reliable detection)
- Compiler opt: `-O2`, reduced search radius (ρ = 6) and variance threshold (50 → 15) tuned for the 64 KB RAM budget

## Results — 11 frame pairs (MPI-Sintel)

| Metric | Value |
|---|---|
| Mean frame rate (95% C.I.) | 19.6 FPS |
| Mean frame time (95% C.I.) | 51.2 ms |
| Valid blocks (ρ/θ), mean | 100% |
| Pixel coincidence score (mean) | 66.5 / 100 |

See `docs/motion-chart.png` (or regenerate from `results/`) for the per-frame-pair horizontal/vertical motion comparison against a Farneback dense optical-flow reference.

## Novelty

Runs a complete 6-stage optical-flow + EIS pipeline — not just raw block matching — at a sustained ~19.6 FPS, with SIMD-accelerated integer SAD replacing a dense float solver. A 7–10 px EIS crop-correction margin absorbs the estimated motion without exposing frame edges, while search-radius and variance tuning trade a small, quantified accuracy margin for throughput headroom under a hard 64 KB RAM ceiling.

## Repository Layout (suggested)

```
03-sad-optical-flow-eis/
├── README.md
├── firmware/            # on-device SAD + Kalman + EIS pipeline (STM32)
├── benchmark/           # MPI-Sintel evaluation scripts, Farneback reference
├── results/             # raw per-frame-pair CSVs, benchmark logs
└── docs/                # motion charts, pipeline diagrams
```

> Populate the folders above with your actual firmware source and benchmark scripts, then commit.
