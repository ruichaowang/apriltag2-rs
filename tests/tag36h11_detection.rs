// Integration tests for Tag36h11 detection using real test images from
// https://github.com/AprilRobotics/apriltag-imgs

use apriltag2_rs::AprilTag2Detector;
use std::path::PathBuf;

/// Convert an image to row-major grayscale buffer.
fn to_gray(path: &std::path::Path) -> (Vec<u8>, u32, u32) {
    let img = image::open(path).expect("failed to open image");
    let gray = img.to_luma8();
    let (w, h) = gray.dimensions();
    (gray.to_vec(), w, h)
}

/// Embed a tag pattern onto a white canvas with padding, so the black border
/// forms detectable edges. Returns a grayscale buffer at `canvas_size`.
fn embed_tag_on_white(
    tag_gray: &[u8],
    tag_w: u32,
    tag_h: u32,
    canvas_size: u32,
) -> (Vec<u8>, usize, usize) {
    // Scale tag to fit within canvas with at least 1 cell of white padding on each side.
    // The tag's outer 1px should be black (border), and we add white padding around it.
    // We just center the tag on a white canvas.
    let mut out = vec![255u8; (canvas_size as usize) * (canvas_size as usize)];

    let tag_s = tag_w.max(tag_h) as f32;
    let scale = (canvas_size as f32 * 0.8) / tag_s; // leave ~10% padding each side
    let scale = scale.max(1.0);

    let scaled_w = (tag_w as f32 * scale) as usize;
    let scaled_h = (tag_h as f32 * scale) as usize;
    let ox = (canvas_size as usize - scaled_w) / 2;
    let oy = (canvas_size as usize - scaled_h) / 2;

    for y in 0..scaled_h {
        for x in 0..scaled_w {
            let sx = ((x as f32 / scale) as u32).min(tag_w - 1) as usize;
            let sy = ((y as f32 / scale) as u32).min(tag_h - 1) as usize;
            out[(oy + y) * canvas_size as usize + (ox + x)] = tag_gray[sy * tag_w as usize + sx];
        }
    }

    (out, canvas_size as usize, canvas_size as usize)
}

/// Nearest-neighbor upscale by integer factor (used for upscaling the mosaic).
fn upscale_gray(data: &[u8], src_w: u32, src_h: u32, factor: u32) -> Vec<u8> {
    let dst_w = (src_w as usize) * factor as usize;
    let dst_h = (src_h as usize) * factor as usize;
    let mut out = vec![0u8; dst_w * dst_h];
    for y in 0..dst_h {
        for x in 0..dst_w {
            let sx = (x as u32 / factor) as usize;
            let sy = (y as u32 / factor) as usize;
            out[y * dst_w + x] = data[sy * src_w as usize + sx];
        }
    }
    out
}

/// Extract expected tag ID from filename: tag36_11_00042.png → 42
fn tag_id_from_filename(name: &str) -> Option<u32> {
    let stem = name.strip_suffix(".png")?;
    stem.strip_prefix("tag36_11_").and_then(|n| n.parse().ok())
}

fn test_images_dir() -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("tests/test-images");
    p
}

#[test]
fn detect_all_tag36h11_individual_tags() {
    let dir = test_images_dir();
    let mut total = 0usize;
    let mut failed: Vec<(String, String)> = Vec::new();

    for entry in std::fs::read_dir(&dir).expect("cannot read test-images dir") {
        let entry = entry.expect("bad dir entry");
        let path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();
        let Some(expected_id) = tag_id_from_filename(&name) else {
            continue; // skip non-tag files (mosaic.png, alltags.ps, etc.)
        };

        let (gray_data, w, h) = to_gray(&path);

        // Embed the small tag on a white canvas at a realistic size.
        let canvas = 300; // px
        let (scaled, sw, sh) = embed_tag_on_white(&gray_data, w, h, canvas);

        let mut detector = AprilTag2Detector::new();
        let detections = detector.detect_on_gray(&scaled, sw, sh);

        let found = detections.iter().any(|(id, _, _, _, _)| *id == expected_id as i32);
        if !found {
            let det_ids: Vec<i32> = detections.iter().map(|(id, _, _, _, _)| *id).collect();
            failed.push((name, format!("expected {expected_id}, got {det_ids:?}")));
        }
        total += 1;
    }

    assert!(
        failed.is_empty(),
        "\nTag36h11 detection failures ({}/{})\n{}",
        failed.len(),
        total,
        failed
            .iter()
            .map(|(n, m)| format!("  {n}: {m}"))
            .collect::<Vec<_>>()
            .join("\n")
    );
}

#[test]
fn detect_mosaic_all_tags() {
    // mosaic.png contains multiple tag patterns tiled at 10x10 resolution.
    // At native resolution the tags are too small for reliable detection,
    // so we upscale the whole mosaic and check for at least some detections.
    let mut path = test_images_dir();
    path.push("mosaic.png");
    if !path.exists() {
        return;
    }

    let (gray_data, w, h) = to_gray(&path);
    // Upscale mosaic 5x so tag bit cells are resolvable
    let scale: u32 = 5;
    let scaled = upscale_gray(&gray_data, w, h, scale);
    let sw = (w * scale) as usize;
    let sh = (h * scale) as usize;

    let mut detector = AprilTag2Detector::new();
    let detections = detector.detect_on_gray(&scaled, sw, sh);

    // Mosaic should contain multiple tags; verify we detect at least some.
    assert!(
        !detections.is_empty(),
        "Expected at least one tag in upscaled mosaic.png, got zero detections"
    );

    // All detected IDs should be within valid Tag36h11 range (0..587).
    for (id, _, _, _hamming, _) in &detections {
        assert!(
            (0..587).contains(&(*id as usize)),
            "Detected ID {id} outside valid Tag36h11 range"
        );
    }
}

#[test]
fn detect_empty_image_returns_nothing() {
    let gray = vec![128u8; 100 * 100];
    let mut detector = AprilTag2Detector::new();
    let detections = detector.detect_on_gray(&gray, 100, 100);
    assert!(
        detections.is_empty(),
        "Empty image should produce no detections, got {detections:?}"
    );
}
