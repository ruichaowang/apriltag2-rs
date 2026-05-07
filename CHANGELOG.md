# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-05-07

### Added
- Pure Rust AprilTag2 detector (Tag36h11 family, 587 tags)
- Complete detection pipeline: grayscale normalization, Gaussian smoothing, gradient computation, Union-Find edge clustering, line segment fitting, Gridder spatial indexing, quad search, adaptive bit decoding, overlap deduplication
- Integration tests with 589 real reference images from AprilRobotics/apriltag-imgs
- `AprilTag2Detector::new()` and `detect_on_gray()` public API
