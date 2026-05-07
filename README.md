# apriltag2-rs

Pure Rust port of the legacy AprilTags/AprilTag2 detector core.

This crate is focused on the low-level detector API: pass a row-major grayscale
image buffer and receive tag detections. It does not call C or C++ code and does
not depend on OpenCV.

## Status

Early extraction from an internal calibration port. The current implementation
targets behavior parity with the legacy C++ `TagDetector::extractTags(gray)`
path for Tag36h11.

## License

This crate is distributed under `LGPL-2.1-only`.

The implementation is a Rust port of the legacy AprilTags C++ detector lineage.
See `NOTICE.md` for provenance notes. Before publishing a release, include the
full LGPL-2.1 license text in `LICENSE`.

## Example

```rust
use apriltag2_rs::AprilTag2Detector;

let width = 640usize;
let height = 480usize;
let gray = vec![0u8; width * height];

let mut detector = AprilTag2Detector::new();
let detections = detector.detect_on_gray(&gray, width, height);

for (id, corners, center, hamming, perimeter) in detections {
    println!(
        "id={id}, center=({:.2}, {:.2}), hamming={hamming}, perimeter={:.2}",
        center.0, center.1, perimeter
    );
    println!("corners={corners:?}");
}
```

## Repository Shape

This repository is intended to stay independent from the calibration project it
was extracted from. Keep camera calibration, OpenCV image loading, and dataset
comparison tooling outside the core crate unless they are added as optional
examples or dev-only tools.
