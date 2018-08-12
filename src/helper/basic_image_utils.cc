/******************************************************************************
 * Copyright 2017-2018 Baidu Robotic Vision Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <driver/helper/xp_logging.h>
#include <driver/helper/timer.h>
#include <driver/helper/basic_image_utils.h>
#include <driver/xp_aec_table.h>
#include <driver/AR0141_aec_table.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

#ifndef __DEVELOPMENT_DEBUG_MODE__
#define __IMAGE_UTILS_NO_DEBUG__
#endif

namespace XPDRIVER {

// only use central area in the image
constexpr int kMarginRow = 50;
constexpr int kMarginCol = 100;
constexpr int kPixelStep = 2;

// Compute the histogram of a sampled area of the input image and return the number of
// sampled pixels
int sampleBrightnessHistogram(const cv::Mat& raw_img,
                              std::vector<int>* histogram,
                              int* avg_pixel_val_ptr) {
  const int end_row = raw_img.rows - kMarginRow;
  const int end_col = raw_img.cols - kMarginCol;

  // Given the current algorithm, collecting histogram is not
  // necessary. But we still do so in case later we switch to a better
  // algorithm
  int pixel_num = 0;
  int avg_pixel_val = 0;
  histogram->clear();
  histogram->resize(256, 0);
  int over_exposure_pixel_num = 0;
  for (int i = kMarginRow; i < end_row; i += kPixelStep) {
    for (int j = kMarginCol; j < end_col; j += kPixelStep) {
      const uint8_t pixel_val = raw_img.data[i * raw_img.cols + j];
      avg_pixel_val += pixel_val;
      (*histogram)[pixel_val]++;
      ++pixel_num;
    }
  }
  if (avg_pixel_val_ptr) {
    *avg_pixel_val_ptr = avg_pixel_val / pixel_num;
  }
  return pixel_num;
}

void gridBrightDarkAdjustBrightness(const cv::Mat& raw_img,
                                    int* adjusted_pixel_val_ptr) {
  // Bright / dark region settings
  constexpr int kBrightRegionThres = 240;
  constexpr int kDarkRegionThres = 25;
  constexpr float kBrightRegionWeight = 1.2f;
  constexpr float kDarkRegionWeight = 0.75f;

  // Grid settings
  constexpr int kGridSize = 10;
  constexpr int kPixelsPerGrid = kGridSize * kGridSize / kPixelStep / kPixelStep;

  const int grid_rows = (raw_img.rows - 2 * kMarginRow) / kGridSize;
  const int grid_cols = (raw_img.cols - 2 * kMarginCol) / kGridSize;
  int adjusted_pixel_val = 0;
  for (int grid_r = 0; grid_r < grid_rows; ++grid_r) {
    for (int grid_c = 0; grid_c < grid_cols; ++grid_c) {
      int start_row = grid_r * kGridSize + kMarginRow;
      int end_row = start_row + kGridSize;
      int start_col = grid_c * kGridSize + kMarginCol;
      int end_col = start_col + kGridSize;
      int grid_pixel_val = 0;

      for (int i = start_row; i < end_row; i += kPixelStep) {
        for (int j = start_col; j < end_col; j += kPixelStep) {
          grid_pixel_val += raw_img.data[i * raw_img.cols + j];
        }
      }
      grid_pixel_val /= kPixelsPerGrid;
      if (grid_pixel_val > kBrightRegionThres) {
        int tmp = grid_pixel_val * kBrightRegionWeight;
        grid_pixel_val *= kBrightRegionWeight;
        XP_CHECK_EQ(tmp, grid_pixel_val);
      } else if (grid_pixel_val < kDarkRegionThres) {
        int tmp = grid_pixel_val * kDarkRegionWeight;
        grid_pixel_val *= kDarkRegionWeight;
        XP_CHECK_EQ(tmp, grid_pixel_val);
      }
      adjusted_pixel_val += grid_pixel_val;
    }
  }
  *adjusted_pixel_val_ptr = adjusted_pixel_val / (grid_rows * grid_cols);
}

// return true if new aec_index is found
bool computeNewAecTableIndex(const cv::Mat& raw_img,
                             const bool smooth_aec,
                             const uint32_t AEC_steps,
                             int* aec_index_ptr) {
  XP_CHECK_NOTNULL(aec_index_ptr);
  int& aec_index = *aec_index_ptr;
  XP_CHECK_LT(aec_index, AEC_steps);
  XP_CHECK_GE(aec_index, 0);

  cv::Mat mono_img;
  if (raw_img.channels() == 1) {
    mono_img = raw_img;
  } else if (raw_img.channels() == 3) {
    cv::cvtColor(raw_img, mono_img, cv::COLOR_BGR2GRAY);
  } else {
    XP_LOG_INFO("Unsupported raw_img channels: " << raw_img.channels());
    return false;
  }

  std::vector<int> histogram;
  int avg_pixel_val = 0;
  int pixel_num = sampleBrightnessHistogram(mono_img, &histogram, &avg_pixel_val);
  if (pixel_num == 0) {
    // Nothing is sampled.  Something is wrong with raw_image
    return false;
  }

  int adjusted_pixel_val = 0;
  gridBrightDarkAdjustBrightness(mono_img, &adjusted_pixel_val);

#ifndef __IMAGE_UTILS_NO_DEBUG__
  int acc_pixel_counts = 0;
  int median_pixel_val = 0;
  for (int i = 0; i < 256; ++i) {
    acc_pixel_counts += histogram[i];
    if (acc_pixel_counts >= pixel_num / 2) {
      median_pixel_val = i;
      break;
    }
  }

  int saturate_pixel_counts = 0;
  for (int i = 253; i < 256; ++i) {
    saturate_pixel_counts += histogram[i];
  }
  float saturate_ratio = static_cast<float>(saturate_pixel_counts) / pixel_num;
  XP_VLOG(1, " pixel_val avg = " << avg_pixel_val
          << " adj_avg = " << adjusted_pixel_val
          << " median = " << median_pixel_val
          << " sat_ratio = " << saturate_ratio);
#endif

  // Heuristically adjust AEC table index
  // [NOTE] a step in AEC table is in average ~4% brightness change.  We calculate a rough
  // step number that will drag the adjusted avg_pixel_val close to 128.
  // We simply use add/minus instead multiply/divide here
  // [NOTE] Due to mono aec table, the brightness changes in the first few rows of
  // are very abrupt, e.g., index 0 -> index 1, ratio = 100%
  constexpr float kStepRatioNormal = 0.05f;
  constexpr float kStepRatioBright = 0.10f;
  constexpr float kStepRatioVeryBright = 0.20f;
  constexpr int kMaxStepNumNormal = 5;
  constexpr int kMaxStepNumBright = 2;
  constexpr int kMaxStepNumVeryBright = 1;
  float step_ratio;
  int max_step_num;
  if (aec_index < 16) {
    max_step_num = kMaxStepNumVeryBright;
    step_ratio = kStepRatioVeryBright;
  } else if (aec_index < 32) {
    max_step_num = kMaxStepNumBright;
    step_ratio = kStepRatioBright;
  } else {
    max_step_num = kMaxStepNumNormal;
    step_ratio = kStepRatioNormal;
  }

  // [NOTE] We need to hand tune the target brightness to avoid saturation
  // e.g., 128 can be too high
  constexpr float target_brightness = 100.f;
  float brightness_ratio = adjusted_pixel_val / target_brightness;
  int rough_step_num = (brightness_ratio - 1.f) / step_ratio;

  // If smooth_aec is true, clip the step number to avoid sudden jump in brightness.
  // Otherwise, try to jump directly to the target aec index.
  int actual_step_num;
  if (!smooth_aec) {
    actual_step_num = rough_step_num;
  } else  if (rough_step_num > max_step_num) {
    actual_step_num = max_step_num;
  } else if (rough_step_num < -max_step_num) {
    actual_step_num = -max_step_num;
  } else {
    actual_step_num = rough_step_num;
  }

#ifndef __IMAGE_UTILS_NO_DEBUG__
  XP_VLOG(1, " brightness_ratio = " << brightness_ratio
          << " rough_step_num = " << rough_step_num
          << " actual_step_num = " << actual_step_num);
#endif

  // Compute the new aec_index
  constexpr int kLowestAecIndex = 1;
  aec_index -= actual_step_num;
  if (aec_index < kLowestAecIndex) {
    aec_index = kLowestAecIndex;
  } else if (aec_index > AEC_steps - 1) {
    aec_index = AEC_steps - 1;
  }
  return true;
}

#ifdef __ARM_NEON__
void AutoWhiteBalance::compute_RGB_mean_neon(const cv::Mat& rgb_img_,
                                             uint32_t* ptr_r_mean,
                                             uint32_t* ptr_g_mean,
                                             uint32_t* ptr_b_mean) const {
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;
  uint32x4_t r_mean = vmovq_n_u32(0);
  uint32x4_t g_mean = vmovq_n_u32(0);
  uint32x4_t b_mean = vmovq_n_u32(0);
  uint16x8_t vr, vg, vb;
  for (int r = 0; r < height; r += 8) {
    for (int c = 0; c < width; c += 16) {
      uint8x16x3_t raw_data = vld3q_u8(rgb_img_.ptr(r) + 3 * c);
      __builtin_prefetch(rgb_img_.ptr(r) + 3 * (c + 16), 0, 1);
      vr = vaddl_u8(vget_low_u8(raw_data.val[0]), vget_high_u8(raw_data.val[0]));
      vg = vaddl_u8(vget_low_u8(raw_data.val[1]), vget_high_u8(raw_data.val[1]));
      vb = vaddl_u8(vget_low_u8(raw_data.val[2]), vget_high_u8(raw_data.val[2]));
      r_mean = vaddq_u32(r_mean, vaddl_u16(vget_low_u16(vr), vget_high_u16(vr)));
      g_mean = vaddq_u32(g_mean, vaddl_u16(vget_low_u16(vg), vget_high_u16(vg)));
      b_mean = vaddq_u32(b_mean, vaddl_u16(vget_low_u16(vb), vget_high_u16(vb)));
    }
  }
  *ptr_r_mean = vgetq_lane_u32(r_mean, 0) + vgetq_lane_u32(r_mean, 1) +
                vgetq_lane_u32(r_mean, 2) + vgetq_lane_u32(r_mean, 3);
  *ptr_g_mean = vgetq_lane_u32(g_mean, 0) + vgetq_lane_u32(g_mean, 1) +
                vgetq_lane_u32(g_mean, 2) + vgetq_lane_u32(g_mean, 3);
  *ptr_b_mean = vgetq_lane_u32(b_mean, 0) + vgetq_lane_u32(b_mean, 1) +
                vgetq_lane_u32(b_mean, 2) + vgetq_lane_u32(b_mean, 3);
}
#endif  // __ARM_NEON__

inline void AutoWhiteBalance::compute_RGB_mean(const cv::Mat &rgb_img_,
                                               uint32_t *ptr_r_mean,
                                               uint32_t *ptr_g_mean,
                                               uint32_t *ptr_b_mean) const {
#ifdef __ARM_NEON__
    return compute_RGB_mean_neon(rgb_img_, ptr_r_mean, ptr_g_mean, ptr_b_mean);
#else
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;
  uint32_t mr[8] __attribute__((aligned(16))) = {0};
  uint32_t mg[8] __attribute__((aligned(16))) = {0};
  uint32_t mb[8] __attribute__((aligned(16))) = {0};
  for (int r = 0; r < height; r += 8) {
    const uint8_t* r_img_data_ptr = rgb_img_.ptr(r);
    const uint8_t* g_img_data_ptr = r_img_data_ptr + 1;
    const uint8_t* b_img_data_ptr = r_img_data_ptr + 2;
    for (int c = 0; c < width; c += 8) {
      mr[0] += *(r_img_data_ptr + 0);
      mr[1] += *(r_img_data_ptr + 3);
      mr[2] += *(r_img_data_ptr + 6);
      mr[3] += *(r_img_data_ptr + 9);
      mr[4] += *(r_img_data_ptr + 12);
      mr[5] += *(r_img_data_ptr + 15);
      mr[6] += *(r_img_data_ptr + 18);
      mr[7] += *(r_img_data_ptr + 21);

      mg[0] += *(g_img_data_ptr + 0);
      mg[1] += *(g_img_data_ptr + 3);
      mg[2] += *(g_img_data_ptr + 6);
      mg[3] += *(g_img_data_ptr + 9);
      mg[4] += *(g_img_data_ptr + 12);
      mg[5] += *(g_img_data_ptr + 15);
      mg[6] += *(g_img_data_ptr + 18);
      mg[7] += *(g_img_data_ptr + 21);

      mb[0] += *(b_img_data_ptr + 0);
      mb[1] += *(b_img_data_ptr + 3);
      mb[2] += *(b_img_data_ptr + 6);
      mb[3] += *(b_img_data_ptr + 9);
      mb[4] += *(b_img_data_ptr + 12);
      mb[5] += *(b_img_data_ptr + 15);
      mb[6] += *(b_img_data_ptr + 18);
      mb[7] += *(b_img_data_ptr + 21);

      r_img_data_ptr += 3 * 8;
      g_img_data_ptr += 3 * 8;
      b_img_data_ptr += 3 * 8;
    }
  }
  int total[3] = {0};
  for (int i = 0; i < 8; ++i) {
    total[0] += mr[i];
    total[1] += mg[i];
    total[2] += mb[i];
  }
  *ptr_r_mean = total[0];
  *ptr_g_mean = total[1];
  *ptr_b_mean = total[2];
#endif  // __ARM_NEON__
}

void AutoWhiteBalance::compute_AWB_coefficients(const cv::Mat& rgb_img_) {
  uint32_t r_mean = 0, g_mean = 0, b_mean = 0;
  compute_RGB_mean(rgb_img_, &r_mean, &g_mean, &b_mean);

  if (g_mean > r_mean && g_mean > b_mean) {
    XP_VLOG(1, "Green channel based.");
    m_coeff_g_ = 1.f;
    m_coeff_r_ = g_mean / static_cast<float>(r_mean);
    m_coeff_b_ = g_mean / static_cast<float>(b_mean);
  } else if (r_mean > g_mean && r_mean > b_mean) {
    XP_VLOG(1, "Red channel based.");
    m_coeff_g_ = r_mean / static_cast<float>(g_mean);
    m_coeff_r_ = 1.f;
    m_coeff_b_ = r_mean / static_cast<float>(b_mean);
  } else {
    XP_VLOG(1, "Blue channel based.");
    m_coeff_g_ = b_mean / static_cast<float>(g_mean);
    m_coeff_r_ = b_mean / static_cast<float>(r_mean);
    m_coeff_b_ = 1.f;
  }
}

#ifdef __ARM_NEON__
#if defined(__aarch64__)
void AutoWhiteBalance::correct_white_balance_coefficients_neon(cv::Mat* rgb_img_ptr) {
  uint8x16x3_t src_raw_data;
  int width = rgb_img_ptr->cols;
  int height = rgb_img_ptr->rows;
  const uint32x4_t v_coeffs[3] = {
    vdupq_n_u32(static_cast<uint32_t>(m_coeff_r_ * 4194304.0f)),
    vdupq_n_u32(static_cast<uint32_t>(m_coeff_g_ * 4194304.0f)),
    vdupq_n_u32(static_cast<uint32_t>(m_coeff_b_ * 4194304.0f))
  };
  const bool do_not_compute[3] = {
    (m_coeff_r_ == 1.f),
    (m_coeff_g_ == 1.f),
    (m_coeff_b_ == 1.f)
  };
  for (int r = 0; r < height; ++r) {
    uint8_t* rgb_img_row_ptr = rgb_img_ptr->ptr(r);
    for (int c = 0; c < width; c += 16) {
      src_raw_data = vld3q_u8(rgb_img_row_ptr + c * 3);
      __builtin_prefetch(rgb_img_row_ptr + (c + 16) * 3, 0, 1);
      for (int ch = 0; ch < 3; ++ch) {
        if (do_not_compute[ch]) continue;
        uint16x8_t low8 = vmovl_u8(vget_low_u8(src_raw_data.val[ch]));
        uint16x8_t high8 = vmovl_u8(vget_high_u8(src_raw_data.val[ch]));
        uint32x4_t low80 = vmovl_u16(vget_low_u16(low8));
        uint32x4_t low81 = vmovl_u16(vget_high_u16(low8));
        uint32x4_t high80 = vmovl_u16(vget_low_u16(high8));
        uint32x4_t high81 = vmovl_u16(vget_high_u16(high8));
        uint32x4_t rlow80 = vshrq_n_u32(vmulq_u32(low80, v_coeffs[ch]), 22);
        uint32x4_t rlow81 = vshrq_n_u32(vmulq_u32(low81, v_coeffs[ch]), 22);
        uint32x4_t rhigh80 = vshrq_n_u32(vmulq_u32(high80, v_coeffs[ch]), 22);
        uint32x4_t rhigh81 = vshrq_n_u32(vmulq_u32(high81, v_coeffs[ch]), 22);
        src_raw_data.val[ch] = vcombine_u8(
                    vqmovn_u16(vcombine_u16(vqmovn_u32(rlow80), vqmovn_u32(rlow81))),
                    vqmovn_u16(vcombine_u16(vqmovn_u32(rhigh80), vqmovn_u32(rhigh81))));
      }
      vst3q_u8(rgb_img_row_ptr + c * 3, src_raw_data);
    }
  }
}

#elif defined(__arm__)
// the data of rgb_img_ptr must be hold in a continuous meomory
void AutoWhiteBalance::correct_white_balance_coefficients_neon(cv::Mat* rgb_img_ptr) {
  uint8x16x3_t src_raw_data;
  int width = rgb_img_ptr->cols;
  int height = rgb_img_ptr->rows;
  uint16_t r_uint16 = static_cast<uint16_t>(m_coeff_r_ * 128.f);
  uint16_t g_uint16 = static_cast<uint16_t>(m_coeff_g_ * 128.f);
  uint16_t b_uint16 = static_cast<uint16_t>(m_coeff_b_ * 128.f);
  int count = width / 16;
  uint8_t* rgb_img_row_ptr = rgb_img_ptr->ptr();
  asm volatile (
    "VDUP.U16 q4, %2\t\n"
    "VDUP.U16 q5, %3\t\n"
    "VDUP.U16 q6, %4\t\n"

    "HEIGHT_LOOP: \t\n"
    // width loop
    "MOV r2, %1\t\n"
    "WIDTH_LOOP: \t\n"

    // r3 = rgb_img_row_ptr + i * 48;
    // r4 = rgb_img_row_ptr + i * 48 + 24;
    "MOV r4, #24\t\n"
    "MOV r3, %5\t\n"
    "ADD r4, r4, %5\t\n"

    // r: d0, d1, g: d2, d3, b: d4, d5
    "VLD3.8 {d0, d2, d4}, [r3]\t\n"
    "VLD3.8 {d1, d3, d5}, [r4]\t\n"

    "PLD [%5, 48]\t\n"

    // q7 ~ q12 holds 16bit raw data
    "VMOVL.U8 q7,   d0\t\n"
    "VMOVL.U8 q8,   d1\t\n"
    "VMOVL.U8 q9,   d2\t\n"
    "VMOVL.U8 q10,  d3\t\n"
    "VMOVL.U8 q11,  d4\t\n"
    "VMOVL.U8 q12,  d5\t\n"

    "VMUL.U16 q7,   q7,   q4\t\n"
    "VMUL.U16 q8,   q8,   q4\t\n"
    "VMUL.U16 q9,   q9,   q5\t\n"
    "VMUL.U16 q10,  q10,  q5\t\n"
    "VMUL.U16 q11,  q11,  q6\t\n"
    "VMUL.U16 q12,  q12,  q6\t\n"

    "VRSHR.U16 q7,   #7\t\n"
    "VRSHR.U16 q8,   #7\t\n"
    "VRSHR.U16 q9,   #7\t\n"
    "VRSHR.U16 q10,  #7\t\n"
    "VRSHR.U16 q11,  #7\t\n"
    "VRSHR.U16 q12,  #7\t\n"

    "VQMOVN.U16 d0, q7 \t\n"
    "VQMOVN.U16 d1, q8 \t\n"
    "VQMOVN.U16 d2, q9 \t\n"
    "VQMOVN.U16 d3, q10\t\n"
    "VQMOVN.U16 d4, q11\t\n"
    "VQMOVN.U16 d5, q12\t\n"

    "VST3.8 {d0, d2, d4}, [r3]\t\n"
    "VST3.8 {d1, d3, d5}, [r4]\t\n"

    "ADD %5, %5, #48\t\n"

    "SUBS r2, r2, #1\t\n"
    "BNE WIDTH_LOOP\t\n"

    "SUBS %0, %0, #1\t\n"
    "BNE HEIGHT_LOOP\t\n"

  : "+r" (height), "+r" (count)
  : "r" (r_uint16), "r" (g_uint16), "r" (b_uint16), "r" (rgb_img_row_ptr)
  : "r2", "r3", "r4", "memory", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8",
    "d9", "d10", "d11", "d12", "d13", "d14", "d15", "d16",
    "d17", "d18", "d19", "d20", "d21", "d22", "d23"
  );
}
#endif  // __aarch64__
#endif  // __ARM_NEON__
void AutoWhiteBalance::correct_white_balance_coefficients(cv::Mat* rgb_img_ptr) {
  cv::Mat& rgb_img_ = *rgb_img_ptr;
  int width = rgb_img_.cols;
  int height = rgb_img_.rows;

  uint8_t buf[48];
  const bool do_not_compute_r = (m_coeff_r_ == 1.f);
  const bool do_not_compute_g = (m_coeff_g_ == 1.f);
  const bool do_not_compute_b = (m_coeff_b_ == 1.f);
  uint32_t coeff_r = static_cast<uint32_t>(m_coeff_r_ * 4194304);
  uint32_t coeff_g = static_cast<uint32_t>(m_coeff_g_ * 4194304);
  uint32_t coeff_b = static_cast<uint32_t>(m_coeff_b_ * 4194304);

  for (int r = 0; r < height; ++r) {
    uint8_t* rgb_img_row_ptr = rgb_img_.ptr(r);
    for (int c = 0; c < width; c += 16) {
      memcpy(buf, rgb_img_row_ptr, 48);
      for (int i = 0; i < 48; i += 12) {
        if (!do_not_compute_r) {
          uint32_t midlle_val_r[4];
          midlle_val_r[0] = ((buf[i + 0] * coeff_r) >> 22);
          midlle_val_r[1] = ((buf[i + 3] * coeff_r) >> 22);
          midlle_val_r[2] = ((buf[i + 6] * coeff_r) >> 22);
          midlle_val_r[3] = ((buf[i + 9] * coeff_r) >> 22);
          buf[i + 0] = midlle_val_r[0] < 255 ? midlle_val_r[0] : 255;
          buf[i + 3] = midlle_val_r[1] < 255 ? midlle_val_r[1] : 255;
          buf[i + 6] = midlle_val_r[2] < 255 ? midlle_val_r[2] : 255;
          buf[i + 9] = midlle_val_r[3] < 255 ? midlle_val_r[3] : 255;
        }
        if (!do_not_compute_g) {
          uint32_t midlle_val_g[4];
          midlle_val_g[0] = ((buf[i + 1] * coeff_g) >> 22);
          midlle_val_g[1] = ((buf[i + 4] * coeff_g) >> 22);
          midlle_val_g[2] = ((buf[i + 7] * coeff_g) >> 22);
          midlle_val_g[3] = ((buf[i + 10] * coeff_g) >> 22);
          buf[i + 1] = midlle_val_g[0] < 255 ? midlle_val_g[0] : 255;
          buf[i + 4] = midlle_val_g[1] < 255 ? midlle_val_g[1] : 255;
          buf[i + 7] = midlle_val_g[2] < 255 ? midlle_val_g[2] : 255;
          buf[i + 10] = midlle_val_g[3] < 255 ? midlle_val_g[3] : 255;
        }
        if (!do_not_compute_b) {
          uint32_t midlle_val_b[4];
          midlle_val_b[0] = ((buf[i + 2] * coeff_b) >> 22);
          midlle_val_b[1] = ((buf[i + 5] * coeff_b) >> 22);
          midlle_val_b[2] = ((buf[i + 8] * coeff_b) >> 22);
          midlle_val_b[3] = ((buf[i + 11] * coeff_b) >> 22);
          buf[i + 2] = midlle_val_b[0] < 255 ? midlle_val_b[0] : 255;
          buf[i + 5] = midlle_val_b[1] < 255 ? midlle_val_b[1] : 255;
          buf[i + 8] = midlle_val_b[2] < 255 ? midlle_val_b[2] : 255;
          buf[i + 11] = midlle_val_b[3] < 255 ? midlle_val_b[3] : 255;
        }
      }
      memmove(rgb_img_row_ptr, buf, 48);
      rgb_img_row_ptr += 16 * 3;
    }
  }
}
}  // namespace XPDRIVER
