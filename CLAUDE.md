# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Pure Rust port of the legacy AprilTags/AprilTag2 detector (Tag36h11 family). No external dependencies, no C/C++ FFI, no OpenCV. Behavior is aligned with the C++ `TagDetector::extractTags(gray)` path.

## Commands

```bash
cargo check        # Type-check and compile
cargo build        # Build the crate
cargo test         # Run all tests (unit tests in src/lib.rs)
cargo clippy       # Lint with clippy
cargo doc --open   # Build and open documentation
```

The crate has zero dependencies — `cargo build` is all that's needed.

## Architecture

Single-file crate (`src/lib.rs`, ~2260 lines). The detection pipeline flows through these stages:

1. **Grayscale normalization** — u8 → f32 [0, 1] via `FloatImage`
2. **Gaussian smoothing** — separable 1D convolution (`filter_factored_centered`)
3. **Gradient computation** — theta + magnitude from central differences
4. **Union-Find edge clustering** — `UnionFindSimple` + `Edge::merge_edges`
5. **Line segment fitting** — least-squares fit via `lsq_fit_xyw`
6. **Gridder spatial indexing** — `Gridder` for fast neighbor lookups
7. **Quad search** — recursive 4-segment loop detection in `Quad::search`
8. **Adaptive bit decoding** — `GrayModel` (bilinear surface fit) + `TagFamily::decode` (Hamming distance against Tag36h11 codes)
9. **Overlap deduplication** — remove duplicate detections by center proximity

### Key types

- `AprilTag2Detector` — public entry point, holds `TagFamily` + `DetectorScratch`
- `DetectorScratch` — reusable allocation pool (FloatImages, UnionFind, segments, etc.)
- `TagDetection` — public type alias: `(id, [(f32,f32); 4], (f32,f32), hamming, perimeter)`
- `Homography33` — 3x3 homography estimation via Jacobi eigenvalue decomposition
- `GrayModel` — adaptive threshold via bilinear polynomial fit

### Floating-point fidelity

This port deliberately matches C++ floating-point behavior (f32 products, f64 accumulators, specific FMA vs separate multiply patterns) for bitwise reproducibility. Comments referencing "Match C++" are intentional and should be preserved.

## Repository scope

This crate is intentionally kept independent from the calibration project it was extracted from. Keep camera calibration, OpenCV image loading, and dataset comparison tooling outside the core crate unless added as optional dev-only examples.

## License

LGPL-2.1-only. Do not change the license or introduce GPL-incompatible dependencies.
