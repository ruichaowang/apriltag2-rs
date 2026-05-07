# Notice

`apriltag2-rs` is an unofficial Rust port of the legacy AprilTags C++ detector
lineage.

Original project reference:

- AprilTags C++ Library
- https://people.csail.mit.edu/kaess/apriltags/

The original AprilTags C++ library is distributed under GNU LGPL version 2.1.
This crate is therefore distributed under `LGPL-2.1-only`.

Local changes:

- Ported detector core to pure Rust.
- Removed C/C++ FFI requirements.
- Kept behavior aligned with C++ `TagDetector::extractTags(gray)`.
- Added Rust-owned scratch buffers and allocation reuse for detector execution.

This project is not affiliated with AprilRobotics, ETH Zurich, or the original
AprilTags authors.
