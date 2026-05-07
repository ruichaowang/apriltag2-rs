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

## Testing

Integration tests use real Tag36h11 images from the official
[AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)
repository. The `tests/test-images/` directory contains all 587 Tag36h11 reference
PNGs plus a mosaic image, sourced from the `tag36h11/` subdirectory of that repo.

Run tests:

```bash
cargo test              # all tests (~19s)
cargo test --test tag36h11_detection -- detect_empty  # single test
```

### Test coverage

- **`detect_all_tag36h11_individual_tags`** — verifies every Tag36h11 reference image
  is correctly detected with the expected tag ID (587 images).
- **`detect_mosaic_all_tags`** — verifies the mosaic image produces valid detections
  within the Tag36h11 ID range.
- **`detect_empty_image_returns_nothing`** — verifies an empty image produces no false positives.
