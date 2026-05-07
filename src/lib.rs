// SPDX-License-Identifier: LGPL-2.1-only

//! Legacy AprilTags/AprilTag2 detector — pure Rust implementation.
//!
//! This crate ports the legacy AprilTags C++ detector core to Rust and keeps
//! behavior aligned with the C++ `TagDetector::extractTags(gray)` path.
//!
//! Core pipeline:
//!
//! 1. 灰度归一化 + 高斯平滑
//! 2. 梯度计算（方向 + 幅值）
//! 3. Union-Find 边缘聚类
//! 4. 最小二乘线段拟合
//! 5. 空间哈希（Gridder）链接线段，搜索 4 段环路（Quad）
//! 6. 自适应阈值采样 + 位解码
//! 7. 重叠检测去重
//!
//! # 使用示例
//!
//! ```no_run
//! use apriltag2_rs::AprilTag2Detector;
//!
//! let width = 1usize;
//! let height = 1usize;
//! let gray_data = vec![0u8; width * height];
//! let mut detector = AprilTag2Detector::new();
//! let detections = detector.detect_on_gray(&gray_data, width, height);
//! for (id, corners, center, hamming, perimeter) in &detections {
//!     println!("Tag {} at center ({}, {})", id, center.0, center.1);
//! }
//! ```

use std::collections::BTreeMap;

/// Tag36h11 编码表（587 个条目，来自 AprilTag2 C++ 头文件）
const TAG36H11_CODES: &[u64] = &[
    0xd5d628584,
    0xd97f18b49,
    0xdd280910e,
    0xe479e9c98,
    0xebcbca822,
    0xf31dab3ac,
    0x056a5d085,
    0x10652e1d4,
    0x22b1dfead,
    0x265ad0472,
    0x34fe91b86,
    0x3ff962cd5,
    0x43a25329a,
    0x474b4385f,
    0x4e9d243e9,
    0x5246149ae,
    0x5997f5538,
    0x683bb6c4c,
    0x6be4a7211,
    0x7e3158eea,
    0x81da494af,
    0x858339a74,
    0x8cd51a5fe,
    0x9f21cc2d7,
    0xa2cabc89c,
    0xadc58d9eb,
    0xb16e7dfb0,
    0xb8c05eb3a,
    0xd25ef139d,
    0xd607e1962,
    0xe4aba3076,
    0x2dde6a3da,
    0x43d40c678,
    0x5620be351,
    0x64c47fa65,
    0x686d7002a,
    0x6c16605ef,
    0x6fbf50bb4,
    0x8d06d39dc,
    0x9f53856b5,
    0xadf746dc9,
    0xbc9b084dd,
    0xd290aa77b,
    0xd9e28b305,
    0xe4dd5c454,
    0xfad2fe6f2,
    0x181a8151a,
    0x26be42c2e,
    0x2e10237b8,
    0x405cd5491,
    0x7742eab1c,
    0x85e6ac230,
    0x8d388cdba,
    0x9f853ea93,
    0xc41ea2445,
    0xcf1973594,
    0x14a34a333,
    0x31eacd15b,
    0x6c79d2dab,
    0x73cbb3935,
    0x89c155bd3,
    0x8d6a46198,
    0x91133675d,
    0xa708d89fb,
    0xae5ab9585,
    0xb9558a6d4,
    0xb98743ab2,
    0xd6cec68da,
    0x1506bcaef,
    0x4becd217a,
    0x4f95c273f,
    0x658b649dd,
    0xa76c4b1b7,
    0xecf621f56,
    0x1c8a56a57,
    0x3628e92ba,
    0x53706c0e2,
    0x5e6b3d231,
    0x7809cfa94,
    0xe97eead6f,
    0x5af40604a,
    0x7492988ad,
    0xed5994712,
    0x5eceaf9ed,
    0x7c1632815,
    0xc1a0095b4,
    0xe9e25d52b,
    0x3a6705419,
    0xa8333012f,
    0x4ce5704d0,
    0x508e60a95,
    0x877476120,
    0xa864e950d,
    0xea45cfce7,
    0x19da047e8,
    0x24d4d5937,
    0x6e079cc9b,
    0x99f2e11d7,
    0x33aa50429,
    0x499ff26c7,
    0x50f1d3251,
    0x66e7754ef,
    0x96ad633ce,
    0x9a5653993,
    0xaca30566c,
    0xc298a790a,
    0x8be44b65d,
    0xdc68f354b,
    0x16f7f919b,
    0x4dde0e826,
    0xd548cbd9f,
    0xe0439ceee,
    0xfd8b1fd16,
    0x76521bb7b,
    0xd92375742,
    0xcab16d40c,
    0x730c9dd72,
    0xad9ba39c2,
    0xb14493f87,
    0x52b15651f,
    0x185409cad,
    0x77ae2c68d,
    0x94f5af4b5,
    0x0a13bad55,
    0x61ea437cd,
    0xa022399e2,
    0x203b163d1,
    0x7bba8f40e,
    0x95bc9442d,
    0x41c0b5358,
    0x8e9c6cc81,
    0x0eb549670,
    0x9da3a0b51,
    0xd832a67a1,
    0xdcd4350bc,
    0x4aa05fdd2,
    0x60c7bb44e,
    0x4b358b96c,
    0x067299b45,
    0xb9c89b5fa,
    0x6975acaea,
    0x62b8f7afa,
    0x33567c3d7,
    0xbac139950,
    0xa5927c62a,
    0x5c916e6a4,
    0x260ecb7d5,
    0x29b7bbd9a,
    0x903205f26,
    0xae72270a4,
    0x3d2ec51a7,
    0x82ea55324,
    0x11a6f3427,
    0x1ca1c4576,
    0xa40c81aef,
    0xbddccd730,
    0x0e617561e,
    0x969317b0f,
    0x67f781364,
    0x610912f96,
    0xb2549fdfc,
    0x06e5aaa6b,
    0xb6c475339,
    0xc56836a4d,
    0x844e351eb,
    0x4647f83b4,
    0x0908a04f5,
    0x7f51034c9,
    0xaee537fca,
    0x5e92494ba,
    0xd445808f4,
    0x28d68b563,
    0x04d25374b,
    0x2bc065f65,
    0x96dc3ea0c,
    0x4b2ade817,
    0x07c3fd502,
    0xe768b5caf,
    0x17605cf6c,
    0x182741ee4,
    0x62846097c,
    0x72b5ebf80,
    0x263da6e13,
    0xfa841bcb5,
    0x7e45e8c69,
    0x653c81fa0,
    0x7443b5e70,
    0x0a5234afd,
    0x74756f24e,
    0x157ebf02a,
    0x82ef46939,
    0x80d420264,
    0x2aeed3e98,
    0xb0a1dd4f8,
    0xb5436be13,
    0x7b7b4b13b,
    0x1ce80d6d3,
    0x16c08427d,
    0xee54462dd,
    0x1f7644cce,
    0x9c7b5cc92,
    0xe369138f8,
    0x5d5a66e91,
    0x485d62f49,
    0xe6e819e94,
    0xb1f340eb5,
    0x09d198ce2,
    0xd60717437,
    0x0196b856c,
    0xf0a6173a5,
    0x12c0e1ec6,
    0x62b82d5cf,
    0xad154c067,
    0xce3778832,
    0x6b0a7b864,
    0x4c7686694,
    0x5058ff3ec,
    0xd5e21ea23,
    0x9ff4a76ee,
    0x9dd981019,
    0x1bad4d30a,
    0xc601896d1,
    0x973439b48,
    0x1ce7431a8,
    0x57a8021d6,
    0xf9dba96e6,
    0x83a2e4e7c,
    0x8ea585380,
    0xaf6c0e744,
    0x875b73bab,
    0xda34ca901,
    0x2ab9727ef,
    0xd39f21b9a,
    0x8a10b742f,
    0x5f8952dba,
    0xf8da71ab0,
    0xc25f9df96,
    0x06f8a5d94,
    0xe42e63e1a,
    0xb78409d1b,
    0x792229add,
    0x5acf8c455,
    0x2fc29a9b0,
    0xea486237b,
    0xb0c9685a0,
    0x1ad748a47,
    0x03b4712d5,
    0xf29216d30,
    0x8dad65e49,
    0x0a2cf09dd,
    0x0b5f174c6,
    0xe54f57743,
    0xb9cf54d78,
    0x4a312a88a,
    0x27babc962,
    0xb86897111,
    0xf2ff6c116,
    0x82274bd8a,
    0x97023505e,
    0x52d46edd1,
    0x585c1f538,
    0xbddd00e43,
    0x5590b74df,
    0x729404a1f,
    0x65320855e,
    0xd3d4b6956,
    0x7ae374f14,
    0x2d7a60e06,
    0x315cd9b5e,
    0xfd36b4eac,
    0xf1df7642b,
    0x55db27726,
    0x8f15ebc19,
    0x992f8c531,
    0x62dea2a40,
    0x928275cab,
    0x69c263cb9,
    0xa774cca9e,
    0x266b2110e,
    0x1b14acbb8,
    0x624b8a71b,
    0x1c539406b,
    0x3086d529b,
    0x0111dd66e,
    0x98cd630bf,
    0x8b9d1ffdc,
    0x72b2f61e7,
    0x9ed9d672b,
    0x96cdd15f3,
    0x6366c2504,
    0x6ca9df73a,
    0xa066d60f0,
    0xe7a4b8add,
    0x8264647ef,
    0xaa195bf81,
    0x9a3db8244,
    0x014d2df6a,
    0x0b63265b7,
    0x2f010de73,
    0x97e774986,
    0x248affc29,
    0xfb57dcd11,
    0x0b1a7e4d9,
    0x4bfa2d07d,
    0x54e5cdf96,
    0x4c15c1c86,
    0xcd9c61166,
    0x499380b2a,
    0x540308d09,
    0x8b63fe66f,
    0xc81aeb35e,
    0x86fe0bd5c,
    0xce2480c2a,
    0x1ab29ee60,
    0x8048daa15,
    0xdbfeb2d39,
    0x567c9858c,
    0x2b6edc5bc,
    0x2078fca82,
    0xadacc22aa,
    0xb92486f49,
    0x51fac5964,
    0x691ee6420,
    0xf63b3e129,
    0x39be7e572,
    0xda2ce6c74,
    0x20cf17a5c,
    0xee55f9b6e,
    0xfb8572726,
    0xb2c2de548,
    0xcaa9bce92,
    0xae9182db3,
    0x74b6e5bd1,
    0x137b252af,
    0x51f686881,
    0xd672f6c02,
    0x654146ce4,
    0xf944bc825,
    0xe8327f809,
    0x76a73fd59,
    0xf79da4cb4,
    0x956f8099b,
    0x7b5f2655c,
    0xd06b114a6,
    0xd0697ca50,
    0x27c390797,
    0xbc61ed9b2,
    0xcc12dd19b,
    0xeb7818d2c,
    0x092fcecda,
    0x89ded4ea1,
    0x256a0ba34,
    0xb6948e627,
    0x1ef6b1054,
    0x8639294a2,
    0xeda3780a4,
    0x39ee2af1d,
    0xcd257edc5,
    0x2d9d6bc22,
    0x121d3b47d,
    0x37e23f8ad,
    0x119f31cf6,
    0x2c97f4f09,
    0xd502abfe0,
    0x10bc3ca77,
    0x53d7190ef,
    0x90c3e62a6,
    0x7e9ebf675,
    0x979ce23d1,
    0x27f0c98e9,
    0xeafb4ae59,
    0x7ca7fe2bd,
    0x1490ca8f6,
    0x9123387ba,
    0xb3bc73888,
    0x3ea87e325,
    0x4888964aa,
    0xa0188a6b9,
    0xcd383c666,
    0x40029a3fd,
    0xe1c00ac5c,
    0x39e6f2b6e,
    0xde664f622,
    0xe979a75e8,
    0x7c6b4c86c,
    0xfd492e071,
    0x8fbb35118,
    0x40b4a09b7,
    0xaf80bd6da,
    0x70e0b2521,
    0x2f5c54d93,
    0x3f4a118d5,
    0x09c1897b9,
    0x079776eac,
    0x084b00b17,
    0x3a95ad90e,
    0x28c544095,
    0x39d457c05,
    0x7a3791a78,
    0xbb770e22e,
    0x9a822bd6c,
    0x68a4b1fed,
    0xa5fd27b3b,
    0x0c3995b79,
    0xd1519dff1,
    0x8e7eee359,
    0xcd3ca50b1,
    0xb73b8b793,
    0x57aca1c43,
    0xec2655277,
    0x785a2c1b3,
    0x75a07985a,
    0xa4b01eb69,
    0xa18a11347,
    0xdb1f28ca3,
    0x877ec3e25,
    0x31f6341b8,
    0x1363a3a4c,
    0x075d8b9ba,
    0x7ae0792a9,
    0xa83a21651,
    0x7f08f9fb5,
    0x0d0cf73a9,
    0xb04dcc98e,
    0xf65c7b0f8,
    0x65ddaf69a,
    0x2cf9b86b3,
    0x14cb51e25,
    0xf48027b5b,
    0x0ec26ea8b,
    0x44bafd45c,
    0xb12c7c0c4,
    0x959fd9d82,
    0xc77c9725a,
    0x48a22d462,
    0x8398e8072,
    0xec89b05ce,
    0xbb682d4c9,
    0xe5a86d2ff,
    0x358f01134,
    0x8556ddcf6,
    0x67584b6e2,
    0x11609439f,
    0x08488816e,
    0xaaf1a2c46,
    0xf879898cf,
    0x8bbe5e2f7,
    0x101eee363,
    0x690f69377,
    0xf5bd93cd9,
    0xcea4c2bf6,
    0x9550be706,
    0x2c5b38a60,
    0xe72033547,
    0x4458b0629,
    0xee8d9ed41,
    0xd2f918d72,
    0x78dc39fd3,
    0x8212636f6,
    0x7450a72a7,
    0xc4f0cf4c6,
    0x367bcddcd,
    0xc1caf8cc6,
    0xa7f5b853d,
    0x9d536818b,
    0x535e021b0,
    0xa7eb8729e,
    0x422a67b49,
    0x929e928a6,
    0x48e8aefcc,
    0xa9897393c,
    0x5eb81d37e,
    0x1e80287b7,
    0x34770d903,
    0x2eef86728,
    0x59266ccb6,
    0x0110bba61,
    0x1dfd284ef,
    0x447439d1b,
    0xfece0e599,
    0x9309f3703,
    0x80764d1dd,
    0x353f1e6a0,
    0x2c1c12dcc,
    0xc1d21b9d7,
    0x457ee453e,
    0xd66faf540,
    0x44831e652,
    0xcfd49a848,
    0x9312d4133,
    0x3f097d3ee,
    0x8c9ebef7a,
    0xa99e29e88,
    0x0e9fab22c,
    0x4e748f4fb,
    0xecdee4288,
    0xabce5f1d0,
    0xc42f6876c,
    0x7ed402ea0,
    0xe5c4242c3,
    0xd5b2c31ae,
    0x286863be6,
    0x160444d94,
    0x5f0f5808e,
    0xae3d44b2a,
    0x9f5c5d109,
    0x8ad9316d7,
    0x3422ba064,
    0x2fed11d56,
    0xbea6e3e04,
    0x04b029eec,
    0x6deed7435,
    0x3718ce17c,
    0x55857f5e2,
    0x2edac7b62,
    0x085d6c512,
    0xd6ca88e0f,
    0x2b7e1fc69,
    0xa699d5c1b,
    0xf05ad74de,
    0x4cf5fb56d,
    0x5725e07e1,
    0x72f18a2de,
    0x1cec52609,
    0x48534243c,
    0x2523a4d69,
    0x35c1b80d1,
    0xa4d7338a7,
    0x0db1af012,
    0xe61a9475d,
    0x05df03f91,
    0x97ae260bb,
    0x32d627fef,
    0xb640f73c2,
    0x45a1ac9c6,
    0x6a2202de1,
    0x57d3e25f2,
    0x5aa9f986e,
    0x0cc859d8a,
    0xe3ec6cca8,
    0x54e95e1ae,
    0x446887b06,
    0x7516732be,
    0x3817ac8f5,
    0x3e26d938c,
    0xaa81bc235,
    0xdf387ca1b,
    0x0f3a3b3f2,
    0xb4bf69677,
    0xae21868ed,
    0x81e1d2d9d,
    0xa0a9ea14c,
    0x8eee297a9,
    0x4740c0559,
    0xe8b141837,
    0xac69e0a3d,
    0x9ed83a1e1,
    0x5edb55ecb,
    0x07340fe81,
    0x50dfbc6bf,
    0x4f583508a,
    0xcb1fb78bc,
    0x4025ced2f,
    0x39791ebec,
    0x53ee388f1,
    0x7d6c0bd23,
    0x93a995fbe,
    0x8a41728de,
    0x2fe70e053,
    0xab3db443a,
    0x1364edb05,
    0x47b6eeed6,
    0x12e71af01,
    0x52ff83587,
    0x3a1575dd8,
    0x3feaa3564,
    0xeacf78ba7,
    0x0872b94f8,
    0xda8ddf9a2,
    0x9aa920d2b,
    0x1f350ed36,
    0x18a5e861f,
    0x2c35b89c3,
    0x3347ac48a,
    0x7f23e022e,
    0x2459068fb,
    0xe83be4b73,
];
const _: () = assert!(
    TAG36H11_CODES.len() == 587,
    "TAG36H11_CODES must have 587 entries"
);

/// 单个 Tag 检测结果
///
/// 元组字段：`(tag_id, 四个角点, 中心坐标, 汉明距离, 观测周长)`
pub type TagDetection = (i32, [(f32, f32); 4], (f32, f32), usize, f32);

// ---------------------------------------------------------------------------
// Math utilities
// ---------------------------------------------------------------------------

/// 将弧度归一化到 [-π, π] 区间
#[inline]
fn mod2pi(x: f32) -> f32 {
    let two_pi = 2.0 * std::f32::consts::PI;
    x - two_pi * (x / two_pi).round()
}

/// 计算两点间欧氏距离的平方
#[inline]
fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    dx * dx + dy * dy
}

/// 归一化灰度浮点图像
///
/// 用于存储归一化到 [0, 1] 的灰度图像，以及高斯平滑后的中间结果。
#[derive(Clone)]
struct FloatImage {
    data: Vec<f32>,
    width: usize,
    height: usize,
}

impl FloatImage {
    fn empty() -> Self {
        Self {
            data: Vec::new(),
            width: 0,
            height: 0,
        }
    }

    fn resize(&mut self, width: usize, height: usize) {
        self.width = width;
        self.height = height;
        self.data.resize(width * height, 0.0);
    }

    fn resize_zeroed(&mut self, width: usize, height: usize) {
        self.resize(width, height);
        self.data.fill(0.0);
    }

    fn load_from_u8(&mut self, data: &[u8], width: usize, height: usize) {
        self.resize(width, height);
        for (dst, &src) in self.data.iter_mut().zip(data.iter()) {
            *dst = src as f32 / 255.0;
        }
    }

    fn copy_from(&mut self, other: &Self) {
        self.width = other.width;
        self.height = other.height;
        self.data.resize(other.data.len(), 0.0);
        self.data.copy_from_slice(&other.data);
    }

    #[inline]
    fn get(&self, x: usize, y: usize) -> f32 {
        self.data[y * self.width + x]
    }
    #[inline]
    fn set(&mut self, x: usize, y: usize, v: f32) {
        self.data[y * self.width + x] = v;
    }
    #[inline]
    fn width(&self) -> usize {
        self.width
    }
    #[inline]
    fn height(&self) -> usize {
        self.height
    }

    /// 1D separable Gaussian convolution (centered, same-size output).
    fn filter_factored_centered(&mut self, h: &[f32], v: &[f32], tmp: &mut Vec<f32>) {
        // Match C++ Gaussian::convolveSymmetricCentered exactly:
        // C++ accumulates (float * float) -> float, then promotes to double accumulator.
        // So each product a[i]*f[j] is computed in f32, then added to f64 acc.
        // C++ loop: j=0 → a[i+sz/2], j=1 → a[i+sz/2-1], ..., j=sz-1 → a[i-sz/2]
        // (where output index = i - sz/2; j=0 is the rightmost neighbor)
        // Reversing k (sz-1..=0) maps: k=sz-1 → sx=x+sz/2, k=0 → sx=x-sz/2, matching C++ j=0..sz-1 order.
        let w = self.width;
        let hh = self.height;
        let hsz_h = h.len() / 2;
        let hsz_v = v.len() / 2;
        tmp.resize(w * hh, 0.0);
        // horizontal pass
        for y in 0..hh {
            let row_off = y * w;
            for x in 0..w {
                let mut acc = 0.0f64;
                for k in (0..h.len()).rev() {
                    let sx = ((x as isize) + (k as isize) - (hsz_h as isize))
                        .clamp(0, w as isize - 1) as usize;
                    // C++: float * float → float, then widen to double
                    acc += (self.data[row_off + sx] * h[k]) as f64;
                }
                tmp[row_off + x] = acc as f32;
            }
        }
        // vertical pass
        for y in 0..hh {
            let row_off = y * w;
            for x in 0..w {
                let mut acc = 0.0f64;
                for k in (0..v.len()).rev() {
                    let sy = ((y as isize) + (k as isize) - (hsz_v as isize))
                        .clamp(0, hh as isize - 1) as usize;
                    // C++: float * float → float, then widen to double
                    acc += (tmp[sy * w + x] * v[k]) as f64;
                }
                self.data[row_off + x] = acc as f32;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Gaussian filter kernel
// ---------------------------------------------------------------------------

fn make_gaussian(sigma: f32, sz: usize) -> Vec<f32> {
    // Match C++ Gaussian::makeGaussianFilter exactly:
    // double inv_variance = 1./(2*sigma*sigma) — sigma is float, 2*sigma*sigma is float,
    // then 1. (double) / float promotes float to double.
    let sigma_sq_f32 = 2.0f32 * sigma * sigma; // float computation like C++
    let inv_variance = 1.0_f64 / sigma_sq_f32 as f64; // double division like C++
    let center = sz / 2; // integer division like C++: j = i - n/2
    let mut k = vec![0.0f32; sz];
    let mut sum = 0.0f32; // C++ uses float sum
    for (i, k_i) in k.iter_mut().enumerate().take(sz) {
        let j = i as f64 - center as f64;
        *k_i = ((-j * j * inv_variance).exp()) as f32;
        sum += *k_i;
    }
    for v in k.iter_mut() {
        *v /= sum;
    }
    k
}

/// 并查集，用于边缘像素聚类
struct UnionFindSimple {
    parent: Vec<usize>,
    size: Vec<usize>,
}

impl UnionFindSimple {
    fn new(n: usize) -> Self {
        let mut uf = Self {
            parent: Vec::new(),
            size: Vec::new(),
        };
        uf.reset(n);
        uf
    }

    fn reset(&mut self, n: usize) {
        self.parent.clear();
        self.parent.extend(0..n);
        self.size.clear();
        self.size.resize(n, 1);
    }

    fn get_representative(&mut self, mut a: usize) -> usize {
        while self.parent[a] != a {
            self.parent[a] = self.parent[self.parent[a]];
            a = self.parent[a];
        }
        a
    }

    fn get_set_size(&self, a: usize) -> usize {
        self.size[a]
    }

    fn connect_nodes(&mut self, a: usize, b: usize) -> usize {
        let ra = self.get_representative(a);
        let rb = self.get_representative(b);
        if ra == rb {
            return ra;
        }
        let sa = self.size[ra];
        let sb = self.size[rb];
        if sa > sb {
            // a is larger — b joins a, return a (matches C++ asz > bsz branch)
            self.parent[rb] = ra;
            self.size[ra] += sb;
            ra
        } else {
            // b is larger or equal — a joins b, return b (matches C++ else branch)
            self.parent[ra] = rb;
            self.size[rb] += sa;
            rb
        }
    }
}

// ---------------------------------------------------------------------------
// Edge
// ---------------------------------------------------------------------------

struct Edge {
    cost: usize,
    pixel_idx_a: usize,
    pixel_idx_b: usize,
}

impl Edge {
    const MIN_MAG: f32 = 0.004;
    const MAX_EDGE_COST: f32 = 30.0 * std::f32::consts::PI / 180.0;
    const WEIGHT_SCALE: usize = 100;
    const THETA_THRESH: f32 = 100.0;
    const MAG_THRESH: f32 = 1200.0;

    fn edge_cost(theta0: f32, theta1: f32, mag1: f32) -> Option<usize> {
        if mag1 < Self::MIN_MAG {
            return None;
        }
        let theta_err = mod2pi(theta1 - theta0).abs();
        if theta_err > Self::MAX_EDGE_COST {
            return None;
        }
        Some(((theta_err / Self::MAX_EDGE_COST) * Self::WEIGHT_SCALE as f32) as usize)
    }

    fn calc_edges(
        theta0: f32,
        x: usize,
        y: usize,
        theta: &FloatImage,
        mag: &FloatImage,
        edges: &mut Vec<Edge>,
    ) {
        let w = theta.width();
        let this = y * w + x;
        let mut add = |cost: Option<usize>, a, b| {
            if let Some(c) = cost {
                edges.push(Edge {
                    cost: c,
                    pixel_idx_a: a,
                    pixel_idx_b: b,
                });
            }
        };
        add(
            Self::edge_cost(theta0, theta.get(x + 1, y), mag.get(x + 1, y)),
            this,
            y * w + x + 1,
        );
        add(
            Self::edge_cost(theta0, theta.get(x, y + 1), mag.get(x, y + 1)),
            this,
            (y + 1) * w + x,
        );
        add(
            Self::edge_cost(theta0, theta.get(x + 1, y + 1), mag.get(x + 1, y + 1)),
            this,
            (y + 1) * w + x + 1,
        );
        if x > 0 {
            add(
                Self::edge_cost(theta0, theta.get(x - 1, y + 1), mag.get(x - 1, y + 1)),
                this,
                (y + 1) * w + x - 1,
            );
        }
    }

    fn merge_edges(
        edges: &[Edge],
        uf: &mut UnionFindSimple,
        tmin: &mut [f32],
        tmax: &mut [f32],
        mmin: &mut [f32],
        mmax: &mut [f32],
    ) {
        let two_pi = 2.0 * std::f32::consts::PI;
        for e in edges {
            let ida = uf.get_representative(e.pixel_idx_a);
            let idb = uf.get_representative(e.pixel_idx_b);
            if ida == idb {
                continue;
            }
            let sza = uf.get_set_size(ida);
            let szb = uf.get_set_size(idb);
            let tmina = tmin[ida];
            let tmaxa = tmax[ida];
            let tminb = tmin[idb];
            let tmaxb = tmax[idb];
            // Two-argument mod2pi: ref + mod2pi(v - ref), then subtract b_center
            // matches C++: bshift = MathUtil::mod2pi(a_center, b_center) - b_center
            let a_center = (tmina + tmaxa) / 2.0;
            let b_center = (tminb + tmaxb) / 2.0;
            let bshift = a_center + mod2pi(b_center - a_center) - b_center;
            let mut tmaxab = tmaxa.max(tmaxb + bshift);
            let tminab = tmina.min(tminb + bshift);
            if tmaxab - tminab > two_pi {
                tmaxab = tminab + two_pi;
            }
            let mminab = mmin[ida].min(mmin[idb]);
            let mmaxab = mmax[ida].max(mmax[idb]);
            let costab = tmaxab - tminab;
            let costa = tmaxa - tmina;
            let costb = tmaxb - tminb;
            if costab <= costa.min(costb) + Self::THETA_THRESH / (sza + szb) as f32
                && (mmaxab - mminab)
                    <= (mmax[ida] - mmin[ida]).min(mmax[idb] - mmin[idb])
                        + Self::MAG_THRESH / (sza + szb) as f32
            {
                let idab = uf.connect_nodes(ida, idb);
                tmin[idab] = tminab;
                tmax[idab] = tmaxab;
                mmin[idab] = mminab;
                mmax[idab] = mmaxab;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Segment fitting and line geometry
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct XYWeight {
    x: f32,
    y: f32,
    w: f32,
}

fn lsq_fit_xyw(points: &[XYWeight]) -> ((f32, f32), (f32, f32)) {
    // Match C++ GLine2D::lsqFitXYW + GLineSegment2D::lsqFitXYW + normalizeP() exactly.
    let mut mxx = 0.0f32;
    let mut myy = 0.0f32;
    let mut mxy = 0.0f32;
    let mut mx = 0.0f32;
    let mut my = 0.0f32;
    let mut n = 0.0f32;
    for p in points {
        my = p.y.mul_add(p.w, my);
        mx = p.x.mul_add(p.w, mx);
        myy = (p.y * p.y).mul_add(p.w, myy);
        mxx = (p.x * p.x).mul_add(p.w, mxx);
        mxy = (p.x * p.y).mul_add(p.w, mxy);
        n += p.w;
    }
    if n < 1e-10 {
        return ((0.0, 0.0), (0.0, 0.0));
    }
    let ex = mx / n;
    let ey = my / n;
    // Match C++ assembly at -O0 on ARM64:
    // - Cxx/Cyy: compiled as separate fmul (via square() call) then fsub -- no FMA
    // - Cxy: compiled as fmsub (FMA) because it's a*b+c in one expression without function call
    let cxx = mxx / n - (mx / n) * (mx / n);
    let cyy = myy / n - (my / n) * (my / n);
    let cxy = (-(mx / n)).mul_add(my / n, mxy / n); // fmsub: mxy/n - (mx/n)*(my/n) with FMA
    let phi = 0.5f32 * (-2.0 * cxy).atan2(cyy - cxx);
    // Match C++ GLine2D::normalizeSlope(): fmadd(dx, dx, dy*dy) then sqrt
    let (dx_raw, dy_raw) = (-phi.sin(), phi.cos());
    let mag = dx_raw.mul_add(dx_raw, dy_raw * dy_raw).sqrt();
    let (dx, dy) = (dx_raw / mag, dy_raw / mag);
    // C++ getLineCoordinate: fmadd(pt.first, dx, pt.second*dy) -- pt.second*dy first as f32
    let mut tmin = f32::INFINITY;
    let mut tmax = f32::NEG_INFINITY;
    for p in points {
        let t = p.x.mul_add(dx, p.y * dy);
        if t < tmin {
            tmin = t;
        }
        if t > tmax {
            tmax = t;
        }
    }
    // C++ normalizeP(): dotprod = fmsub(dy, p.first, dx*p.second) = dx*p.second - dy*p.first
    // i.e., dx*ey computed as f32 first, then fma(-dy, ex, dx*ey)
    let dotprod = (-dy).mul_add(ex, dx * ey);
    let px = -dy * dotprod;
    let py = dx * dotprod;
    // C++ getPointOfCoordinate: fmadd(coord, dx, p.first), fmadd(coord, dy, p.second)
    (
        (tmin.mul_add(dx, px), tmin.mul_add(dy, py)),
        (tmax.mul_add(dx, px), tmax.mul_add(dy, py)),
    )
}

/// 线段拟合结果
///
/// 包含线段端点、方向角、长度以及子节点索引列表（用于 quad 搜索）。
#[derive(Clone)]
struct Segment {
    children: Vec<usize>,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    theta: f32,
    length: f32,
}

impl Segment {
    const MIN_LINE_LENGTH: f32 = 4.0;

    fn new() -> Self {
        Self {
            children: Vec::new(),
            x0: 0.0,
            y0: 0.0,
            x1: 0.0,
            y1: 0.0,
            theta: 0.0,
            length: 0.0,
        }
    }

    fn from_fit(p0: (f32, f32), p1: (f32, f32)) -> Self {
        let mut s = Self::new();
        s.x0 = p0.0;
        s.y0 = p0.1;
        s.x1 = p1.0;
        s.y1 = p1.1;
        s.length = dist2(p0, p1).sqrt();
        s
    }
}

/// 二维固定网格，用于快速查找相邻线段
///
/// 将线段端点按网格单元分组，加速 quad 搜索中的邻近线段查询。
struct Gridder {
    cells: Vec<Vec<usize>>,
    width: usize,
    height: usize,
    n_cells_x: usize,
    n_cells_y: usize,
    cell_size: f32,
}

impl Gridder {
    fn new() -> Self {
        Self {
            cells: Vec::new(),
            width: 0,
            height: 0,
            n_cells_x: 0,
            n_cells_y: 0,
            cell_size: 1.0,
        }
    }

    fn reset(&mut self, width: usize, height: usize, cell_size: usize) {
        let cell_size = cell_size as f32;
        let n_cells_x = (width as f32 / cell_size + 1.0) as usize;
        let n_cells_y = (height as f32 / cell_size + 1.0) as usize;
        let cell_count = n_cells_x * n_cells_y;
        if self.cells.len() != cell_count {
            self.cells.clear();
            self.cells.resize_with(cell_count, Vec::new);
        } else {
            for cell in &mut self.cells {
                cell.clear();
            }
        }
        self.width = width;
        self.height = height;
        self.n_cells_x = n_cells_x;
        self.n_cells_y = n_cells_y;
        self.cell_size = cell_size;
    }

    fn add(&mut self, x: f32, y: f32, seg_idx: usize) {
        let cx = (x / self.cell_size) as usize;
        let cy = (y / self.cell_size) as usize;
        if cx >= self.n_cells_x || cy >= self.n_cells_y {
            return;
        }
        self.cells[cy * self.n_cells_x + cx].push(seg_idx);
    }

    fn find<F>(&self, x: f32, y: f32, radius: f32, mut f: F)
    where
        F: FnMut(usize),
    {
        let r = radius.max(1.0);
        // Match C++ Gridder: width/height are in pixels, compute cell count the same way
        let n_cells_x = self.n_cells_x as isize;
        let n_cells_y = self.n_cells_y as isize;
        let mut cx_min = ((x - r) / self.cell_size) as isize;
        let mut cx_max = ((x + r) / self.cell_size) as isize;
        let mut cy_min = ((y - r) / self.cell_size) as isize;
        let mut cy_max = ((y + r) / self.cell_size) as isize;
        // Clamp to valid grid bounds (match C++ Gridder Iterator clamping)
        cx_min = cx_min.max(0);
        cx_min = cx_min.min(n_cells_x - 1);
        cx_max = cx_max.max(0);
        cx_max = cx_max.min(n_cells_x - 1);
        cy_min = cy_min.max(0);
        cy_min = cy_min.min(n_cells_y - 1);
        cy_max = cy_max.max(0);
        cy_max = cy_max.min(n_cells_y - 1);
        for cy in cy_min..=cy_max {
            for cx in cx_min..=cx_max {
                let indices = &self.cells[cy as usize * self.n_cells_x + cx as usize];
                // LIFO: iterate in reverse insertion order to match C++ Gridder (linked list prepend)
                for &idx in indices.iter().rev() {
                    f(idx);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Line intersection
// ---------------------------------------------------------------------------

fn line_intersection(
    p1: (f32, f32),
    p2: (f32, f32),
    p3: (f32, f32),
    p4: (f32, f32),
) -> Option<(f32, f32)> {
    // Match C++ GLine2D::intersectionWith exactly:
    // getDx()/getDy() return the RAW (non-normalized) stored dx/dy.
    // getFirst()/getSecond() return the raw reference point (x0, y0) since normalizeP is never called.
    // So BOTH parent and child use their raw (x1-x0, y1-y0) direction vectors.
    let m00 = p2.0 - p1.0; // parent raw dx
    let m10 = p2.1 - p1.1; // parent raw dy
    let m01 = -(p4.0 - p3.0); // -child raw dx (NOT normalized)
    let m11 = -(p4.1 - p3.1); // -child raw dy (NOT normalized)
                              // Match C++: m00*m11 - m01*m10 as separate operations (no FMA)
    let det = m00 * m11 - m01 * m10;
    if det.abs() < 1e-10 {
        return None;
    }
    let i00 = m11 / det;
    let i01 = -m01 / det;
    // Child reference = (p3.0, p3.1) = child.x0, child.y0 (no normalizeP)
    let b00 = p3.0 - p1.0;
    let b10 = p3.1 - p1.1;
    let x00 = i00 * b00 + i01 * b10;
    Some((m00 * x00 + p1.0, m10 * x00 + p1.1))
}

// ---------------------------------------------------------------------------
// Homography33 — matches C++ exactly: accumulates A'A incrementally via addCorrespondence,
// then finds the eigenvector of the smallest eigenvalue via Jacobi iteration.
// This matches C++ Homography33::compute() using Eigen::JacobiSVD<MatrixXd>.
// ---------------------------------------------------------------------------

/// Jacobi eigenvalue decomposition for a 9×9 symmetric matrix.
/// Returns eigenvectors as columns sorted by eigenvalue ascending (last = smallest).
/// Matches Eigen::JacobiSVD behavior for symmetric PSD matrices.
#[allow(clippy::needless_range_loop)]
fn jacobi_eigen_9x9(a_in: &[[f64; 9]; 9]) -> [[f64; 9]; 9] {
    let n = 9usize;
    let mut a = *a_in;
    let mut v = [[0.0f64; 9]; 9];
    for (i, row) in v.iter_mut().enumerate().take(n) {
        row[i] = 1.0;
    }

    for _ in 0..200 {
        // Find off-diagonal element with largest absolute value
        let mut max_val = 0.0f64;
        let mut p = 0usize;
        let mut q = 1usize;
        for (i, a_row) in a.iter().enumerate().take(n) {
            for j in i + 1..n {
                let v = a_row[j].abs();
                if v > max_val {
                    max_val = v;
                    p = i;
                    q = j;
                }
            }
        }
        if max_val < 1e-14 {
            break;
        }

        // Stable Givens angle (Numerical Recipes symmetric Jacobi)
        let (c, s) = {
            let apq = a[p][q];
            let tau = (a[q][q] - a[p][p]) / (2.0 * apq);
            let t = if tau >= 0.0 {
                1.0 / (tau + (1.0 + tau * tau).sqrt())
            } else {
                1.0 / (tau - (1.0 + tau * tau).sqrt())
            };
            let c = 1.0 / (1.0 + t * t).sqrt();
            (c, t * c)
        };

        // Update diagonal (numerically stable: t = s/c)
        let t_val = s / c;
        let app_new = a[p][p] - t_val * a[p][q];
        let aqq_new = a[q][q] + t_val * a[p][q];

        a[p][p] = app_new;
        a[q][q] = aqq_new;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        // Update off-diagonal rows (symmetric: a[k][p]=a[p][k], a[k][q]=a[q][k])
        for k in 0..n {
            if k == p || k == q {
                continue;
            }
            let apk = c * a[p][k] - s * a[q][k];
            let aqk = s * a[p][k] + c * a[q][k];
            a[p][k] = apk;
            a[k][p] = apk;
            a[q][k] = aqk;
            a[k][q] = aqk;
        }

        for row in v.iter_mut().take(n) {
            let vpk = c * row[p] - s * row[q];
            let vqk = s * row[p] + c * row[q];
            row[p] = vpk;
            row[q] = vqk;
        }
    }

    // Sort columns of v by eigenvalue ascending (a[i][i]) — last column = smallest eigenvalue
    let mut order: [usize; 9] = [0, 1, 2, 3, 4, 5, 6, 7, 8];
    order.sort_by(|&i, &j| {
        a[i][i]
            .partial_cmp(&a[j][j])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let mut result = [[0.0f64; 9]; 9];
    for (new_col, &old_col) in order.iter().enumerate() {
        for row in 0..n {
            result[row][new_col] = v[row][old_col];
        }
    }
    result
}

/// 单应性矩阵估计器
///
/// 通过 2D-2D 对应点累积 A'A 矩阵，使用 Jacobi 特征分解求解最小特征值对应的
/// 特征向量作为单应性参数。匹配 C++ `Homography33` 实现。
#[derive(Clone)]
struct Homography33 {
    cxy: (f32, f32),
    fa: [[f64; 9]; 9], // upper triangle of A'A, matches C++ fA
    h: [f64; 9],
    valid: bool,
}

impl Homography33 {
    fn new(cxy: (f32, f32)) -> Self {
        Self {
            cxy,
            fa: [[0.0; 9]; 9],
            h: [0.0; 9],
            valid: false,
        }
    }

    /// Matches C++ Homography33::addCorrespondence exactly.
    /// Accumulates A'A from 3 equation rows per correspondence.
    fn add_correspondence(&mut self, world_x: f32, world_y: f32, image_x: f32, image_y: f32) {
        self.valid = false;
        let wx = world_x as f64;
        let wy = world_y as f64;
        let ix = (image_x - self.cxy.0) as f64;
        let iy = (image_y - self.cxy.1) as f64;

        // Row type 0: [-wx, -wy, -1, 0, 0, 0, wx*iy, wy*iy, iy]  (cols 3-8)
        let a03 = -wx;
        let a04 = -wy;
        let a05 = -1.0;
        let a06 = wx * iy;
        let a07 = wy * iy;
        let a08 = iy;
        self.fa[3][3] += a03 * a03;
        self.fa[3][4] += a03 * a04;
        self.fa[3][5] += a03 * a05;
        self.fa[3][6] += a03 * a06;
        self.fa[3][7] += a03 * a07;
        self.fa[3][8] += a03 * a08;
        self.fa[4][4] += a04 * a04;
        self.fa[4][5] += a04 * a05;
        self.fa[4][6] += a04 * a06;
        self.fa[4][7] += a04 * a07;
        self.fa[4][8] += a04 * a08;
        self.fa[5][5] += a05 * a05;
        self.fa[5][6] += a05 * a06;
        self.fa[5][7] += a05 * a07;
        self.fa[5][8] += a05 * a08;
        self.fa[6][6] += a06 * a06;
        self.fa[6][7] += a06 * a07;
        self.fa[6][8] += a06 * a08;
        self.fa[7][7] += a07 * a07;
        self.fa[7][8] += a07 * a08;
        self.fa[8][8] += a08 * a08;

        // Row type 1: [wx, wy, 1, 0, 0, 0, -wx*ix, -wy*ix, -ix]  (cols 0-2, 6-8)
        let a10 = wx;
        let a11 = wy;
        let a12 = 1.0;
        let a16 = -wx * ix;
        let a17 = -wy * ix;
        let a18 = -ix;
        self.fa[0][0] += a10 * a10;
        self.fa[0][1] += a10 * a11;
        self.fa[0][2] += a10 * a12;
        self.fa[0][6] += a10 * a16;
        self.fa[0][7] += a10 * a17;
        self.fa[0][8] += a10 * a18;
        self.fa[1][1] += a11 * a11;
        self.fa[1][2] += a11 * a12;
        self.fa[1][6] += a11 * a16;
        self.fa[1][7] += a11 * a17;
        self.fa[1][8] += a11 * a18;
        self.fa[2][2] += a12 * a12;
        self.fa[2][6] += a12 * a16;
        self.fa[2][7] += a12 * a17;
        self.fa[2][8] += a12 * a18;
        self.fa[6][6] += a16 * a16;
        self.fa[6][7] += a16 * a17;
        self.fa[6][8] += a16 * a18;
        self.fa[7][7] += a17 * a17;
        self.fa[7][8] += a17 * a18;
        self.fa[8][8] += a18 * a18;

        // Row type 2: [-wx*iy, -wy*iy, -iy, wx*ix, wy*ix, ix, 0, 0, 0]  (cols 0-5)
        let a20 = -wx * iy;
        let a21 = -wy * iy;
        let a22 = -iy;
        let a23 = wx * ix;
        let a24 = wy * ix;
        let a25 = ix;
        self.fa[0][0] += a20 * a20;
        self.fa[0][1] += a20 * a21;
        self.fa[0][2] += a20 * a22;
        self.fa[0][3] += a20 * a23;
        self.fa[0][4] += a20 * a24;
        self.fa[0][5] += a20 * a25;
        self.fa[1][1] += a21 * a21;
        self.fa[1][2] += a21 * a22;
        self.fa[1][3] += a21 * a23;
        self.fa[1][4] += a21 * a24;
        self.fa[1][5] += a21 * a25;
        self.fa[2][2] += a22 * a22;
        self.fa[2][3] += a22 * a23;
        self.fa[2][4] += a22 * a24;
        self.fa[2][5] += a22 * a25;
        self.fa[3][3] += a23 * a23;
        self.fa[3][4] += a23 * a24;
        self.fa[3][5] += a23 * a25;
        self.fa[4][4] += a24 * a24;
        self.fa[4][5] += a24 * a25;
        self.fa[5][5] += a25 * a25;
    }

    #[allow(clippy::needless_range_loop)]
    fn compute(&mut self) {
        if self.valid {
            return;
        }
        // Make symmetric (copy upper triangle to lower)
        let mut a = self.fa;
        for i in 0..9 {
            for j in i + 1..9 {
                a[j][i] = a[i][j];
            }
        }
        // Jacobi eigendecomposition: first column of result = eigenvector of smallest eigenvalue
        let eigvecs = jacobi_eigen_9x9(&a);
        for (eigvec_row, h_i) in eigvecs.iter().zip(self.h.iter_mut()) {
            *h_i = eigvec_row[0];
        }
        self.valid = true;
    }

    /// 单应性矩阵投影（带计算）
    #[allow(dead_code)]
    fn project(&mut self, x: f32, y: f32) -> (f32, f32) {
        self.compute();
        self.project_computed(x, y)
    }

    /// Project without recomputing — assumes compute() was already called.
    #[allow(dead_code)]
    fn project_computed(&self, x: f32, y: f32) -> (f32, f32) {
        let ix = self.h[0] * x as f64 + self.h[1] * y as f64 + self.h[2];
        let iy = self.h[3] * x as f64 + self.h[4] * y as f64 + self.h[5];
        let z = self.h[6] * x as f64 + self.h[7] * y as f64 + self.h[8];
        if z.abs() < 1e-10 {
            return (0.0, 0.0);
        }
        ((ix / z) as f32 + self.cxy.0, (iy / z) as f32 + self.cxy.1)
    }

    fn get_h(&self) -> [f64; 9] {
        self.h
    }
    fn get_cxy(&self) -> (f32, f32) {
        self.cxy
    }
}

/// 自适应双线性灰度模型
///
/// 通过二元多项式曲面拟合（基函数 [x, y, xy, 1]），为 bit 采样提供自适应阈值。
/// 匹配 C++ `GrayModel` 实现。
struct GrayModel {
    // Accumulate A'A (4x4 upper triangle) and A'b (4x1)
    // basis: [x, y, x*y, 1]
    aa: [[f64; 4]; 4],
    b: [f64; 4],
    v: [f64; 4],
    nobs: usize,
    dirty: bool,
}

impl GrayModel {
    fn new() -> Self {
        Self {
            aa: [[0.0; 4]; 4],
            b: [0.0; 4],
            v: [0.0; 4],
            nobs: 0,
            dirty: false,
        }
    }

    fn add_observation(&mut self, x: f32, y: f32, gray: f32) {
        // Match C++ GrayModel::addObservation: products computed as float, then stored in double.
        let xy = x * y; // f32 * f32 = f32
                        // basis [x, y, x*y, 1] — accumulate upper triangle of A'A
        self.aa[0][0] += (x * x) as f64;
        self.aa[0][1] += xy as f64;
        self.aa[0][2] += (x * xy) as f64;
        self.aa[0][3] += x as f64;
        self.aa[1][1] += (y * y) as f64;
        self.aa[1][2] += (y * xy) as f64;
        self.aa[1][3] += y as f64;
        self.aa[2][2] += (xy * xy) as f64;
        self.aa[2][3] += xy as f64;
        self.aa[3][3] += 1.0;
        self.b[0] += (x * gray) as f64;
        self.b[1] += (y * gray) as f64;
        self.b[2] += (xy * gray) as f64;
        self.b[3] += gray as f64;
        self.nobs += 1;
        self.dirty = true;
    }

    fn interpolate(&mut self, x: f32, y: f32) -> f32 {
        if self.dirty {
            self.compute();
        }
        // Match C++: v[0]*x + v[1]*y + v[2]*x*y + v[3] (x*y as float product first)
        (self.v[0] * x as f64 + self.v[1] * y as f64 + self.v[2] * (x * y) as f64 + self.v[3])
            as f32
    }

    #[allow(clippy::needless_range_loop)]
    fn compute(&mut self) {
        self.dirty = false;
        if self.nobs >= 6 {
            // Fill symmetric lower triangle
            let mut a = self.aa;
            for i in 0..4 {
                for j in i + 1..4 {
                    a[j][i] = a[i][j];
                }
            }
            // Solve 4x4 system via Cramer or Gaussian elimination
            if let Some(sol) = solve4x4(&a, &self.b) {
                self.v = sol;
                return;
            }
        }
        // Fallback: constant model
        self.v = [0.0; 4];
        self.v[3] = if self.nobs > 0 {
            self.b[3] / self.nobs as f64
        } else {
            0.0
        };
    }
}

/// Solve 4x4 linear system A*x = b using Gaussian elimination with partial pivoting.
#[allow(clippy::needless_range_loop)]
fn solve4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut m = [[0.0f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][4] = b[i];
    }
    for col in 0..4 {
        // partial pivot
        let mut max_row = col;
        for row in col + 1..4 {
            if m[row][col].abs() > m[max_row][col].abs() {
                max_row = row;
            }
        }
        m.swap(col, max_row);
        if m[col][col].abs() < 1e-12 {
            return None;
        }
        let pivot = m[col][col];
        for m_col in m[col].iter_mut().skip(col) {
            *m_col /= pivot;
        }
        for row in 0..4 {
            if row == col {
                continue;
            }
            let factor = m[row][col];
            for j in col..5 {
                m[row][j] -= factor * m[col][j];
            }
        }
    }
    Some([m[0][4], m[1][4], m[2][4], m[3][4]])
}

/// 四边形（Quad）
///
/// 由 4 条线段首尾相连构成的候选 Tag 边界框。包含角点、周长和单应性矩阵。
struct Quad {
    quad_points: [(f32, f32); 4],
    observed_perimeter: f32,
    homography: Homography33,
}

/// Quad 搜索过程中的统计计数器
struct QuadSearchStats {
    quad_reject_parallel: usize,
    quad_reject_winding: usize,
    quad_reject_edge: usize,
    quad_reject_aspect: usize,
    quad_accept: usize,
}

impl Quad {
    const MAX_ASPECT_RATIO: f32 = 32.0;
    const MIN_EDGE_LENGTH: f32 = 6.0; // matches C++ Quad::minimumEdgeLength = 6

    fn new(quad_points: [(f32, f32); 4], optical_center: (f32, f32)) -> Self {
        let mut homography = Homography33::new(optical_center);
        // Map from tag coords (-1,-1)..(1,1) to pixel coords
        homography.add_correspondence(-1.0, -1.0, quad_points[0].0, quad_points[0].1);
        homography.add_correspondence(1.0, -1.0, quad_points[1].0, quad_points[1].1);
        homography.add_correspondence(1.0, 1.0, quad_points[2].0, quad_points[2].1);
        homography.add_correspondence(-1.0, 1.0, quad_points[3].0, quad_points[3].1);
        homography.compute(); // pre-compute so get_h()/get_cxy() work on shared refs
        Self {
            quad_points,
            observed_perimeter: 0.0,
            homography,
        }
    }

    /// Matches C++ Quad::interpolate() with INTERPOLATE define — bilinear interpolation of corners.
    fn interpolate(&self, x: f32, y: f32) -> (f32, f32) {
        let p0 = self.quad_points[0];
        let p3 = self.quad_points[3];
        let p01 = (self.quad_points[1].0 - p0.0, self.quad_points[1].1 - p0.1);
        let p32 = (self.quad_points[2].0 - p3.0, self.quad_points[2].1 - p3.1);
        let t = (x + 1.0) / 2.0;
        let s = (y + 1.0) / 2.0;
        let r1 = (p0.0 + p01.0 * t, p0.1 + p01.1 * t);
        let r2 = (p3.0 + p32.0 * t, p3.1 + p32.1 * t);
        (r1.0 + (r2.0 - r1.0) * s, r1.1 + (r2.1 - r1.1) * s)
    }

    fn interpolate01(&self, x: f32, y: f32) -> (f32, f32) {
        self.interpolate(2.0 * x - 1.0, 2.0 * y - 1.0)
    }

    /// Recursive search for 4-segment loops forming a quad.
    fn search(
        path: &mut [usize; 5],
        parent_idx: usize,
        depth: usize,
        quads: &mut Vec<Self>,
        segments: &[Segment],
        optical_center: (f32, f32),
        stats: &mut QuadSearchStats,
    ) {
        if depth == 4 {
            if path[4] == path[0] {
                let mut p: [(f32, f32); 4] = [(0.0, 0.0); 4];
                let mut calc_perimeter = 0.0;
                let mut bad = false;

                for i in 0..4 {
                    let sa = &segments[path[i]];
                    let sb = &segments[path[i + 1]];
                    if let Some(inter) = line_intersection(
                        (sa.x0, sa.y0),
                        (sa.x1, sa.y1),
                        (sb.x0, sb.y0),
                        (sb.x1, sb.y1),
                    ) {
                        p[i] = inter;
                        calc_perimeter += sa.length;
                    } else {
                        stats.quad_reject_parallel += 1;
                        bad = true;
                    }
                }

                if !bad {
                    // Check winding order
                    let t0 = (p[1].1 - p[0].1).atan2(p[1].0 - p[0].0);
                    let t1 = (p[2].1 - p[1].1).atan2(p[2].0 - p[1].0);
                    let t2 = (p[3].1 - p[2].1).atan2(p[3].0 - p[2].0);
                    let t3 = (p[0].1 - p[3].1).atan2(p[0].0 - p[3].0);
                    let ttheta =
                        mod2pi(t1 - t0) + mod2pi(t2 - t1) + mod2pi(t3 - t2) + mod2pi(t0 - t3);
                    if !(-7.0..=-5.0).contains(&ttheta) {
                        stats.quad_reject_winding += 1;
                        bad = true;
                    }
                }

                if !bad {
                    let d0 = dist2(p[0], p[1]).sqrt();
                    let d1 = dist2(p[1], p[2]).sqrt();
                    let d2 = dist2(p[2], p[3]).sqrt();
                    let d3 = dist2(p[3], p[0]).sqrt();
                    let d4 = dist2(p[0], p[2]).sqrt();
                    let d5 = dist2(p[1], p[3]).sqrt();
                    if d0 < Self::MIN_EDGE_LENGTH
                        || d1 < Self::MIN_EDGE_LENGTH
                        || d2 < Self::MIN_EDGE_LENGTH
                        || d3 < Self::MIN_EDGE_LENGTH
                        || d4 < Self::MIN_EDGE_LENGTH
                        || d5 < Self::MIN_EDGE_LENGTH
                    {
                        stats.quad_reject_edge += 1;
                        bad = true;
                    }

                    let dmax = d0.max(d1).max(d2).max(d3);
                    let dmin = d0.min(d1).min(d2).min(d3);
                    if dmax > dmin * Self::MAX_ASPECT_RATIO {
                        stats.quad_reject_aspect += 1;
                        bad = true;
                    }
                }

                if !bad {
                    let mut q = Self::new(p, optical_center);
                    q.observed_perimeter = calc_perimeter;
                    quads.push(q);
                    stats.quad_accept += 1;
                }
            }
            return;
        }

        let parent = &segments[parent_idx];
        for &child_idx in &parent.children {
            let child = &segments[child_idx];
            // Pruning: require child theta <= first segment theta
            if child.theta > segments[path[0]].theta {
                continue;
            }
            path[depth + 1] = child_idx;
            Self::search(
                path,
                child_idx,
                depth + 1,
                quads,
                segments,
                optical_center,
                stats,
            );
        }
    }
}

/// Tag 家族定义
///
/// 当前使用 Tag36h11：36 位编码，最小汉明距离 11，可纠正 1 位错误。
struct TagFamily {
    #[allow(dead_code)]
    bits: usize,
    dimension: usize,
    #[allow(dead_code)]
    min_hamming_distance: usize,
    error_recovery_bits: usize,
    codes: &'static [u64],
    black_border: usize,
}

impl TagFamily {
    fn new(black_border: usize) -> Self {
        Self {
            bits: 36,
            dimension: 6,
            min_hamming_distance: 11,
            error_recovery_bits: 1,
            codes: TAG36H11_CODES,
            black_border,
        }
    }

    fn rotate90(w: u64, d: usize) -> u64 {
        let mut wr: u64 = 0;
        // C++ AprilTag2 uses column-major: bit position b = row + d*col
        // rotate90: for r from d-1 to 0, for c from 0 to d-1, read bit (r + d*c) from original
        for r in (0..d).rev() {
            for c in 0..d {
                let b = r + d * c;
                wr <<= 1;
                if (w & (1u64 << b)) != 0 {
                    wr |= 1;
                }
            }
        }
        wr
    }

    fn hamming_distance(a: u64, b: u64) -> usize {
        (a ^ b).count_ones() as usize
    }

    fn decode(&self, r_code: u64) -> Option<(usize, usize, usize)> {
        // Try all rotations of r_code, find best match
        let mut best_id = 0;
        let mut best_ham = usize::MAX;
        let mut best_rot = 0;

        for (id, &code) in self.codes.iter().enumerate() {
            let mut rotated = r_code;
            for rot in 0..4 {
                let ham = Self::hamming_distance(rotated, code);
                if ham < best_ham {
                    best_ham = ham;
                    best_id = id;
                    best_rot = rot;
                }
                rotated = Self::rotate90(rotated, self.dimension);
            }
        }

        if best_ham <= self.error_recovery_bits {
            Some((best_id, best_ham, best_rot))
        } else {
            None
        }
    }
}

struct DetectorScratch {
    fim_orig: FloatImage,
    fim_seg: FloatImage,
    fim_theta: FloatImage,
    fim_mag: FloatImage,
    filter_tmp: Vec<f32>,
    uf: UnionFindSimple,
    tmin: Vec<f32>,
    tmax: Vec<f32>,
    mmin: Vec<f32>,
    mmax: Vec<f32>,
    edges: Vec<Edge>,
    clusters: BTreeMap<usize, Vec<XYWeight>>,
    segments: Vec<Segment>,
    gridder: Gridder,
    child_additions: Vec<(usize, usize)>,
    quads: Vec<Quad>,
    detections: Vec<TagDetection>,
    final_detections: Vec<TagDetection>,
}

impl DetectorScratch {
    fn new() -> Self {
        Self {
            fim_orig: FloatImage::empty(),
            fim_seg: FloatImage::empty(),
            fim_theta: FloatImage::empty(),
            fim_mag: FloatImage::empty(),
            filter_tmp: Vec::new(),
            uf: UnionFindSimple::new(0),
            tmin: Vec::new(),
            tmax: Vec::new(),
            mmin: Vec::new(),
            mmax: Vec::new(),
            edges: Vec::new(),
            clusters: BTreeMap::new(),
            segments: Vec::new(),
            gridder: Gridder::new(),
            child_additions: Vec::new(),
            quads: Vec::new(),
            detections: Vec::new(),
            final_detections: Vec::new(),
        }
    }

    fn prepare(&mut self, width: usize, height: usize) {
        let npixels = width * height;
        self.fim_theta.resize_zeroed(width, height);
        self.fim_mag.resize_zeroed(width, height);
        self.uf.reset(npixels);
        self.tmin.clear();
        self.tmin.resize(npixels, 0.0);
        self.tmax.clear();
        self.tmax.resize(npixels, 0.0);
        self.mmin.clear();
        self.mmin.resize(npixels, 0.0);
        self.mmax.clear();
        self.mmax.resize(npixels, 0.0);

        self.edges.clear();
        let edge_reserve = (npixels / 2).max(1024);
        if self.edges.capacity() < edge_reserve {
            self.edges.reserve(edge_reserve - self.edges.capacity());
        }

        self.clusters.clear();
        self.segments.clear();
        self.child_additions.clear();
        self.quads.clear();
        self.detections.clear();
        self.final_detections.clear();
    }
}

fn aptag_log_enabled() -> bool {
    std::env::var_os("APTAGS_LOG").is_some() || std::env::var_os("APTAGS_TIMING").is_some()
}

/// AprilTag2 检测器主结构体
///
/// 封装了 Tag36h11 家族定义和完整的检测流程。
pub struct AprilTag2Detector {
    family: TagFamily,
    scratch: DetectorScratch,
}

impl Default for AprilTag2Detector {
    fn default() -> Self {
        Self::new()
    }
}

impl AprilTag2Detector {
    /// 创建新的检测器实例，使用 Tag36h11 家族（黑边框宽度 = 1）
    pub fn new() -> Self {
        Self {
            family: TagFamily::new(1),
            scratch: DetectorScratch::new(),
        }
    }

    /// 在灰度图像上检测 AprilTag
    ///
    /// # 参数
    /// - `gray_data`: 灰度图像像素数据（行优先）
    /// - `width`: 图像宽度
    /// - `height`: 图像高度
    /// # 返回值
    /// Vec\<TagDetection\>，每个元素为 (id, 四个角点, 中心, 汉明距离, 周长)
    pub fn detect_on_gray(
        &mut self,
        gray_data: &[u8],
        width: usize,
        height: usize,
    ) -> Vec<TagDetection> {
        let t_start = std::time::Instant::now();
        let family = &self.family;
        let scratch = &mut self.scratch;
        scratch.prepare(width, height);

        // Step 1: Convert to FloatImage (normalized grayscale)
        scratch.fim_orig.load_from_u8(gray_data, width, height);
        let optical_center = (width as f32 / 2.0, height as f32 / 2.0);

        // Step 2: Gaussian smoothing for segmentation (segSigma=0.8)
        // C++: sigma=0 (no filter for bit sampling), segSigma=0.8 (for gradient)
        let seg_sigma = 0.8f32;
        let filtsz = ((3.0f32.max(3.0 * seg_sigma)) as usize) | 1; // odd
        let filt = make_gaussian(seg_sigma, filtsz);
        scratch.fim_seg.copy_from(&scratch.fim_orig);
        scratch
            .fim_seg
            .filter_factored_centered(&filt, &filt, &mut scratch.filter_tmp);

        // Step 3: Compute gradient (theta + magnitude) from segmentation image
        let w = scratch.fim_seg.width();
        let h = scratch.fim_seg.height();

        for y in 1..h - 1 {
            for x in 1..w - 1 {
                let ix = scratch.fim_seg.get(x + 1, y) - scratch.fim_seg.get(x - 1, y);
                let iy = scratch.fim_seg.get(x, y + 1) - scratch.fim_seg.get(x, y - 1);
                let mag = ix.mul_add(ix, iy * iy);
                let theta = iy.atan2(ix);
                scratch.fim_theta.set(x, y, theta);
                scratch.fim_mag.set(x, y, mag);
            }
        }

        // Step 4: Union-Find edge extraction
        for y in 0..h - 1 {
            for x in 0..w - 1 {
                let mag0 = scratch.fim_mag.get(x, y);
                if mag0 < Edge::MIN_MAG {
                    continue;
                }
                scratch.mmax[y * w + x] = mag0;
                scratch.mmin[y * w + x] = mag0;
                let theta0 = scratch.fim_theta.get(x, y);
                scratch.tmin[y * w + x] = theta0;
                scratch.tmax[y * w + x] = theta0;
                Edge::calc_edges(
                    theta0,
                    x,
                    y,
                    &scratch.fim_theta,
                    &scratch.fim_mag,
                    &mut scratch.edges,
                );
            }
        }
        scratch.edges.sort_by_key(|e| e.cost); // stable sort matches C++ std::stable_sort
        Edge::merge_edges(
            &scratch.edges,
            &mut scratch.uf,
            &mut scratch.tmin,
            &mut scratch.tmax,
            &mut scratch.mmin,
            &mut scratch.mmax,
        );

        // Step 5: Collect cluster statistics
        // Use BTreeMap to match C++ std::map<int, ...> which iterates in sorted key order.
        let min_seg_size = 4; // matches C++ Segment::minimumSegmentSize = 4
        for y in 0..h - 1 {
            for x in 0..w - 1 {
                let rep = scratch.uf.get_representative(y * w + x);
                if scratch.uf.get_set_size(rep) < min_seg_size {
                    continue;
                }
                scratch.clusters.entry(rep).or_default().push(XYWeight {
                    x: x as f32,
                    y: y as f32,
                    w: scratch.fim_mag.get(x, y),
                });
            }
        }

        // Step 6: Fit line segments to clusters
        // C++ uses std::map<int, vector<XYWeight>> which iterates in sorted key order.
        for points in scratch.clusters.values() {
            let (p0, p1) = lsq_fit_xyw(points);
            let length = dist2(p0, p1).sqrt();
            if length < Segment::MIN_LINE_LENGTH {
                continue;
            }

            let mut seg = Segment::from_fit(p0, p1);
            seg.theta = (p1.1 - p0.1).atan2(p1.0 - p0.0);

            // Determine orientation (dark on left, white on right)
            let mut flip = 0.0;
            let mut noflip = 0.0;
            for pt in points {
                let theta = scratch.fim_theta.get(pt.x as usize, pt.y as usize);
                let mag = scratch.fim_mag.get(pt.x as usize, pt.y as usize);
                let err = mod2pi(theta - seg.theta);
                if err < 0.0 {
                    noflip += mag;
                } else {
                    flip += mag;
                }
            }
            if flip > noflip {
                seg.theta += std::f32::consts::PI;
            }

            // Ensure p0->p1 direction matches theta
            let dot = (p1.0 - p0.0) * seg.theta.cos() + (p1.1 - p0.1) * seg.theta.sin();
            if dot > 0.0 {
                std::mem::swap(&mut seg.x0, &mut seg.x1);
                std::mem::swap(&mut seg.y0, &mut seg.y1);
            }

            scratch.segments.push(seg);
        }

        // Step 7: Chain segments via Gridder, search for 4-segment loops
        let grid_size = 10;
        scratch.gridder.reset(w, h, grid_size);
        for (i, seg) in scratch.segments.iter().enumerate() {
            scratch.gridder.add(seg.x0, seg.y0, i);
        }

        // Find children
        let n = scratch.segments.len();
        for i in 0..n {
            let sx = scratch.segments[i].x1;
            let sy = scratch.segments[i].y1;
            let sr = 0.5 * scratch.segments[i].length;
            let p0 = (scratch.segments[i].x0, scratch.segments[i].y0);
            let p1 = (scratch.segments[i].x1, scratch.segments[i].y1);
            let parent_theta = scratch.segments[i].theta;
            let parent_len = scratch.segments[i].length;

            scratch.gridder.find(sx, sy, sr, |child_idx| {
                // Skip self-links: in C++, these are counted separately as self_pass_all
                if child_idx == i {
                    return;
                }
                let child = &scratch.segments[child_idx];
                if mod2pi(child.theta - parent_theta) > 0.0 {
                    return;
                }

                let p = line_intersection(p0, p1, (child.x0, child.y0), (child.x1, child.y1));
                if let Some(inter) = p {
                    let pd = dist2(inter, p1).sqrt();
                    let cd = dist2(inter, (child.x0, child.y0)).sqrt();
                    let max_pd_cd = pd.max(cd);
                    if max_pd_cd <= parent_len {
                        scratch.child_additions.push((i, child_idx));
                    }
                }
            });
        }
        for &(pi, ci) in &scratch.child_additions {
            scratch.segments[pi].children.push(ci);
        }

        // Search for 4-segment loops
        let mut stats = QuadSearchStats {
            quad_reject_parallel: 0,
            quad_reject_winding: 0,
            quad_reject_edge: 0,
            quad_reject_aspect: 0,
            quad_accept: 0,
        };
        for i in 0..scratch.segments.len() {
            let mut path = [i; 5];
            Quad::search(
                &mut path,
                i,
                0,
                &mut scratch.quads,
                &scratch.segments,
                optical_center,
                &mut stats,
            );
        }

        // Step 8: Decode quads
        let dd = 2 * family.black_border + family.dimension;

        for quad in scratch.quads.iter() {
            // Find threshold by sampling border and data regions
            let mut black_model = GrayModel::new();
            let mut white_model = GrayModel::new();

            for iy in -1..=(dd as i32) {
                let y = (iy as f32 + 0.5) / dd as f32;
                for ix in -1..=(dd as i32) {
                    let x = (ix as f32 + 0.5) / dd as f32;
                    let (px, py) = quad.interpolate01(x, y);
                    // Match C++: check irx < 0 explicitly (negative f32 as usize saturates to 0, not caught by >= w)
                    let irx = px.round() as i32;
                    let iry = py.round() as i32;
                    if irx < 0 || iry < 0 || irx as usize >= w || iry as usize >= h {
                        continue;
                    }
                    let v = scratch.fim_orig.get(irx as usize, iry as usize);
                    if iy == -1 || iy == dd as i32 || ix == -1 || ix == dd as i32 {
                        white_model.add_observation(x, y, v);
                    } else if iy == 0 || iy == (dd as i32 - 1) || ix == 0 || ix == (dd as i32 - 1) {
                        black_model.add_observation(x, y, v);
                    }
                }
            }

            // Read bits
            let mut bad = false;
            let mut tag_code: u64 = 0;
            for iy in (0..family.dimension).rev() {
                let y = (family.black_border as f32 + iy as f32 + 0.5) / dd as f32;
                for ix in 0..family.dimension {
                    let x = (family.black_border as f32 + ix as f32 + 0.5) / dd as f32;
                    let (px, py) = quad.interpolate01(x, y);
                    // Match C++: check irx < 0 explicitly (negative f32 as usize saturates to 0)
                    let irx = px.round() as i32;
                    let iry = py.round() as i32;
                    if irx < 0 || iry < 0 || irx as usize >= w || iry as usize >= h {
                        bad = true;
                        continue;
                    }
                    let threshold =
                        (black_model.interpolate(x, y) + white_model.interpolate(x, y)) * 0.5;
                    let v = scratch.fim_orig.get(irx as usize, iry as usize);
                    tag_code <<= 1;
                    if v > threshold {
                        tag_code |= 1;
                    }
                }
            }

            if bad {
                continue;
            }

            // Decode
            if let Some((tag_id, ham_dist, rotation)) = family.decode(tag_code) {
                if ham_dist <= family.error_recovery_bits {
                    // C++: rotate homography by `rotation * 90°` CCW, then project (-1,-1)
                    // to find the bottom-left corner of the tag in image space.
                    // R = [[c,-s,0],[s,c,0],[0,0,1]], R*[-1,-1,1]^T = [-c+s, -s-c, 1]^T
                    let angle = rotation as f64 * std::f64::consts::PI / 2.0;
                    let (c, s) = (angle.cos(), angle.sin());
                    let rx = -c + s;
                    let ry = -s - c;
                    let h = quad.homography.get_h();
                    let cxy = quad.homography.get_cxy();
                    let ix = h[0] * rx + h[1] * ry + h[2];
                    let iy = h[3] * rx + h[4] * ry + h[5];
                    let z = h[6] * rx + h[7] * ry + h[8];
                    let bottom_left: (f32, f32) = if z.abs() < 1e-10 {
                        (0.0, 0.0)
                    } else {
                        ((ix / z) as f32 + cxy.0, (iy / z) as f32 + cxy.1)
                    };

                    // Find quad corner closest to bottom_left, reorder from there
                    let mut best_rot = 0;
                    let mut best_dist = f32::MAX;
                    for i in 0..4 {
                        let d = dist2(bottom_left, quad.quad_points[i]).sqrt();
                        if d < best_dist {
                            best_dist = d;
                            best_rot = i;
                        }
                    }
                    let mut ordered_corners = [(0.0f32, 0.0f32); 4];
                    for (i, dest) in ordered_corners.iter_mut().enumerate() {
                        *dest = quad.quad_points[(i + best_rot) % 4];
                    }

                    let center = quad.interpolate01(0.5, 0.5);
                    scratch.detections.push((
                        tag_id as i32,
                        ordered_corners,
                        (center.0, center.1),
                        ham_dist,
                        quad.observed_perimeter,
                    ));
                }
            }
        }

        // Step 9: Remove overlapping duplicate detections — matches C++ TagDetector step 9.
        // Overlap: same ID AND center distance < average quad "radius" (perimeter/8).
        // Conflict resolution: keep lower hamming distance; if equal, keep larger perimeter.
        for &det in &scratch.detections {
            let mut conflict_idx: Option<usize> = None;
            for (i, other) in scratch.final_detections.iter().enumerate() {
                if det.0 != other.0 {
                    continue;
                }
                let radius = (dist2(det.1[0], det.1[1]).sqrt()
                    + dist2(det.1[1], det.1[2]).sqrt()
                    + dist2(det.1[2], det.1[3]).sqrt()
                    + dist2(det.1[3], det.1[0]).sqrt()
                    + dist2(other.1[0], other.1[1]).sqrt()
                    + dist2(other.1[1], other.1[2]).sqrt()
                    + dist2(other.1[2], other.1[3]).sqrt()
                    + dist2(other.1[3], other.1[0]).sqrt())
                    / 16.0;
                if dist2(det.2, other.2).sqrt() < radius {
                    conflict_idx = Some(i);
                    break;
                }
            }
            match conflict_idx {
                None => {
                    scratch.final_detections.push(det);
                }
                Some(i) => {
                    // keep the better detection: lower hamming wins; same hamming → larger perimeter
                    if det.3 < scratch.final_detections[i].3
                        || (det.3 == scratch.final_detections[i].3
                            && det.4 > scratch.final_detections[i].4)
                    {
                        scratch.final_detections[i] = det;
                    }
                }
            }
        }

        scratch.final_detections.sort_by_key(|d| d.0);
        if aptag_log_enabled() {
            let elapsed = t_start.elapsed();
            eprintln!(
                "[TIME] Rust AT2 detect_on_gray: {:.1} ms ({} tags, {}x{})",
                elapsed.as_secs_f64() * 1000.0,
                scratch.final_detections.len(),
                width,
                height
            );
        }
        scratch.final_detections.clone()
    }
}
