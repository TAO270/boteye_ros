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
#include <driver/xp_sensors_wb_table.h>
#include <driver/XP_sensor_driver.h>
#include <driver/v4l2.h>
#include <driver/helper/timer.h>  // for profiling timer
#include <driver/xp_aec_table.h>
#include <driver/AR0141_aec_table.h>
#include <opencv2/imgproc.hpp>
#ifdef __linux__
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <errno.h>
#include <iconv.h>
#include <linux/uvcvideo.h>
#include <fcntl.h>
#include <unistd.h>
#endif  // __linux__
#include <chrono>
#include <iostream>
#include <fstream>
#include <list>
#include <cassert>
#ifdef __ANDORID__
#include <android/log.h>
#include <utilbase.h>
#endif  // __ANDORID__

using std::chrono::steady_clock;

namespace XPDRIVER {

#ifdef __linux__  // XP sensor driver only supports Linux for now.
XpSensorMultithread::XpSensorMultithread(const std::string& sensor_type_str,
                                         const bool use_auto_gain,
                                         const bool imu_from_image,
                                         const std::string& dev_name,
                                         const std::string& wb_mode) :
    sensor_type_str_(sensor_type_str),
    dev_name_(dev_name),
    is_running_(false),
    imu_from_image_(imu_from_image),
    open_rgb_ir_mode_(false),
    rgb_ir_period_(2),
    use_auto_gain_(use_auto_gain),
    aec_index_updated_(false),
    aec_index_(100),
    aec_settle_(!use_auto_gain),
    use_auto_infrared_(false),
    ir_ctl_updated_(false),
    congested_ms_(0),
    infrared_index_(100),
    video_sensor_file_id_(-1),
    imaging_FPS_(25),
    wb_mode_str_(wb_mode),
    raw_sensor_img_mmap_ptr_queue_("raw_sensor_img_mmap_ptr_queue") {
    pull_imu_rate_ = 0;
    stream_images_rate_ = 0;
    stream_ir_images_rate_ = 0;
}
XpSensorMultithread::~XpSensorMultithread() {
  if (is_running_) {
    this->stop();
  }
}

bool XpSensorMultithread::init() {
  // TODO(mingyu): Add an is_init flag to protect from double initialization
  // TODO(mingyu): re-org v4l2_init to a better place
  if (!init_v4l2(dev_name_, &video_sensor_file_id_, &bufferinfo_)) {
    XP_LOG_ERROR(dev_name_ << " cannot be init");
    // try to turn stream off
    // TODO(mingyu): Make sure this "turn off" thing is needed
    stop_v4l2(&video_sensor_file_id_, bufferinfo_);
    return false;
  }

  if (!XP_SENSOR::get_XP_sensor_spec(video_sensor_file_id_, &XP_sensor_spec_)) {
    return false;
  }

  if (!sensor_type_str_.empty()) {
    if (sensor_type_str_ == "XP") {
      sensor_type_ = SensorType::XP;
    } else if (sensor_type_str_ == "XP2") {
      sensor_type_ = SensorType::XP2;
    } else if (sensor_type_str_ == "XP3") {
      sensor_type_ = SensorType::XP3;
    } else if (sensor_type_str_ == "FACE") {
      sensor_type_ = SensorType::FACE;
    } else if (sensor_type_str_ == "XPIRL") {
      sensor_type_ = SensorType::XPIRL;
    } else if (sensor_type_str_ == "XPIRL2") {
      sensor_type_ = SensorType::XPIRL2;
    } else if (sensor_type_str_ == "XPIRL3") {
      sensor_type_ = SensorType::XPIRL3;
    } else {
      XP_LOG_FATAL("Unsupported input sensor type: " << sensor_type_str_);
      return false;
    }
  } else {
    // use auto-detected sensor_type without input sensor_type.
    const std::string sensor_name  = XPDRIVER::SensorName[
                                     static_cast<uint8_t>(XP_sensor_spec_.sensor_type)];
    XP_LOG_INFO("auto-detect sensor type: " << sensor_name);
    sensor_type_ = XP_sensor_spec_.sensor_type;
  }
  // always enable imu embed img funciton of firmware
  XP_SENSOR::xp_imu_embed_img(video_sensor_file_id_, true);
  if (sensor_type_ == SensorType::XPIRL2 || sensor_type_ == SensorType::XPIRL3) {
    infrared_index_ = 50;
    rgb_ir_period_ = 0;
    XP_SENSOR::xp_infrared_ctl(video_sensor_file_id_, XP_SENSOR::OFF , infrared_index_,
                               rgb_ir_period_);
    aec_index_ = 200;
  } else if (sensor_type_ == SensorType::XP3) {
    aec_index_ =  150;
  } else {
    aec_index_ = 120;
  }
  constexpr bool verbose = false;  // Do NOT turn verbose on if not using the latest firmware
  XP_SENSOR::set_registers_to_default(video_sensor_file_id_,
                                      sensor_type_,
                                      aec_index_,
                                      verbose);
  // white balance
  if (sensor_type_ == SensorType::XP3  ||
      sensor_type_ == SensorType::FACE ||
      sensor_type_ == SensorType::XPIRL3 ||
      sensor_type_ == SensorType::XPIRL2) {
    assert(wb_mode_str_.empty() != true);
    if (wb_mode_str_ == "auto") {
      whiteBalanceCorrector_.reset(new AutoWhiteBalance(false));
      // Don't need to do anything in auto white balance mode
      std::cout << "driver works in white balance auto mode" << std::endl;
    } else if (wb_mode_str_ == "disabled") {
      // if we disable white balance, we don't need a whiteBalanceCorrector_
      assert(whiteBalanceCorrector_ == nullptr);
      std::cout << "driver disable white balance" << std::endl;
    } else if (wb_mode_str_ == "preset") {
      whiteBalanceCorrector_.reset(new AutoWhiteBalance(true));
      int index = static_cast<int>(sensor_type_);
      assert(index < sizeof(XP_SENSOR::s_wb_param) / sizeof(XP_SENSOR::WB_PARAM));
      XP_SENSOR::WB_PARAM wbp = XP_SENSOR::s_wb_param[index];

      whiteBalanceCorrector_->setWhiteBalancePresetMode(wbp.r_, wbp.g_, wbp.b_);
      std::cout << "driver works in white balance preset mode, (r, g, b) = ("
                << wbp.r_ << ", " << wbp.g_ << ", " << wbp.b_ << ")" << std::endl;
    }
  }

  ts_ring_buffer_.set_capacity(V4L2_BUFFER_NUM + 2);
  return true;
}

bool XpSensorMultithread::run() {
  if (is_running_) {
    // This sensor is already up and running.
    return false;
  }
  is_running_ = true;
  first_imu_clock_count_ = 0;  // TODO(mingyu): verify if we need to reset everytime

  thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_ioctl_control, this));
  thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_stream_images, this));
  #ifndef __ANDROID__
    if (!imu_from_image_) {
      thread_pool_.push_back(std::thread(&XpSensorMultithread::thread_pull_imu, this));
    }
  #endif
  return true;
}

bool XpSensorMultithread::stop() {
  if (!is_running_) {
    // This sensor is NOT running.  Nothing to stop.
    return false;
  }
  is_running_ = false;
  raw_sensor_img_mmap_ptr_queue_.kill();
  for (std::thread& t : thread_pool_) {
    t.join();
  }
  stop_v4l2(&video_sensor_file_id_, bufferinfo_);
  return true;
}

bool XpSensorMultithread::set_image_data_callback(
    const XpSensorMultithread::ImageDataCallback& callback) {
  if (callback) {
    image_data_callback_ = callback;
    return true;
  }
  return false;
}

bool XpSensorMultithread::set_IR_data_callback(
    const XpSensorMultithread::ImageDataCallback& callback) {
  if (callback) {
    IR_data_callback_ = callback;
    return true;
  }
  return false;
}

bool XpSensorMultithread::set_imu_data_callback(
    const XpSensorMultithread::ImuDataCallback& callback) {
  if (callback) {
    imu_data_callback_ = callback;
    return true;
  }
  return false;
}

void XpSensorMultithread::thread_ioctl_control() {
  // TODO(mingyu): Put back thread param control
  XP_VLOG(1, "======== start thread_ioctl_control");

  uint8_t v4l2_buffer_cout = 0;
  while (is_running_) {
    XPDRIVER::ScopedLoopProfilingTimer loopProfilingTimer(
        "DuoVioTracker::thread_ioctl_control", 1);
    if (raw_sensor_img_mmap_ptr_queue_.size() >= V4L2_BUFFER_NUM - 1) {
      XP_LOG_ERROR("raw_sensor_img_mmap_ptr_queue_.size() = "
                   << raw_sensor_img_mmap_ptr_queue_.size()
                   << " images are used too slow");
      // [NOTE] We should NOT push more access and queue images as the img_data_ptr
      //        in mmap will wrap around, which will mess up the pointers already pushed
      //        into raw_sensor_img_mmap_ptr_queue_.
      usleep(100000);
      congested_ms_ += 100;
      continue;
    }

    uint8_t* img_data_ptr = nullptr;
    if (!access_next_img_and_queue_next(video_sensor_file_id_,
                                        &bufferinfo_,
                                        &img_data_ptr)) {
      continue;
    }
    // must drop beginning queue data as they are all zero.
    if (v4l2_buffer_cout <= V4L2_BUFFER_NUM) {
      v4l2_buffer_cout++;
      continue;
    }
    congested_ms_ = 0;  // reset
    raw_sensor_img_mmap_ptr_queue_.push_back({img_data_ptr, steady_clock::now()});
  }
  XP_VLOG(1, "======== terminate thread_ioctl_control raw_sensor_img_mmap_ptr_queue_.size() "
          << raw_sensor_img_mmap_ptr_queue_.size());
}

void XpSensorMultithread::convert_imu_axes(const XP_20608_data& imu_data,
                                           const SensorType sensor_type,
                                           XPDRIVER::ImuData* xp_imu_ptr) const {
  XPDRIVER::ImuData& xp_imu = *xp_imu_ptr;
  if (sensor_type == SensorType::XP) {
    xp_imu.accel[0] = imu_data.accel[0];
    xp_imu.accel[1] = imu_data.accel[1];
    xp_imu.accel[2] = imu_data.accel[2];
    xp_imu.ang_v[0] = imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] = imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::XP2 ||
             sensor_type == SensorType::XP3) {
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] = - imu_data.accel[1];
    xp_imu.accel[2] =   imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = - imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] =   imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::FACE) {
    // TODO(mingyu): Fix the imu axes here
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] = - imu_data.accel[1];
    xp_imu.accel[2] =   imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] = - imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] =   imu_data.gyro[2] / 180.f * M_PI;
  } else if (sensor_type == SensorType::XPIRL ||
             sensor_type == SensorType::XPIRL2 ||
             sensor_type == SensorType::XPIRL3) {
    xp_imu.accel[0] = - imu_data.accel[0];
    xp_imu.accel[1] =   imu_data.accel[1];
    xp_imu.accel[2] = - imu_data.accel[2];
    xp_imu.ang_v[0] = - imu_data.gyro[0] / 180.f * M_PI;
    xp_imu.ang_v[1] =   imu_data.gyro[1] / 180.f * M_PI;
    xp_imu.ang_v[2] = - imu_data.gyro[2] / 180.f * M_PI;
  } else {
    XP_LOG_FATAL("Non-supported sensor type");
  }
}
void XpSensorMultithread::thread_pull_imu() {
  // TODO(mingyu): Put back thread param control
  const float clock_unit_ms = 1;
  Counter32To64 counter32To64(XP_CLOCK_32BIT_MAX_COUNT);
  XP_VLOG(1, "====== start thread_pull_imu ======");
  uint64_t last_clock_count_64 = 0;
  int startup_imu_count = 0;
  while (is_running_ && !imu_from_image_) {
    XPDRIVER::ScopedLoopProfilingTimer pull_imu_profiling_timer(
      "XpSensorMultithread::thread_pull_imu", 1);
    // sleep to get ~100 Hz rate
    std::this_thread::sleep_for(std::chrono::microseconds(9900));

    XP_20608_data imu_data;
    bool imu_access_ok = (XP_SENSOR::IMU_DataAccess(video_sensor_file_id_, &imu_data));
    if (imu_access_ok) {
      // when working in IMU pulling mode, the time stamp of the first several IMU is not stable
      // we drop the first 5 IMU frame here.
      if (startup_imu_count < 5) {
        ++startup_imu_count;
        continue;
      }
      uint64_t clock_count_wo_overflow = counter32To64.convertNewCount32(imu_data.clock_count);
      if (first_imu_clock_count_ == 0) {
        first_imu_clock_count_ = clock_count_wo_overflow;
      }
      if (last_clock_count_64 > 0) {
        if (last_clock_count_64 == clock_count_wo_overflow) {
          XP_LOG_WARNING("WARNING: Imu pulling too fast");
          continue;
        }
        // only for debug
        if (clock_count_wo_overflow < last_clock_count_64) {
          XP_LOG_FATAL(" clock_count_wo_overflow " << clock_count_wo_overflow
                       << " < last_clock_count_64 " << last_clock_count_64);
        }
        // TODO(mingyu): verify the time unit here
        if ((clock_count_wo_overflow - last_clock_count_64) * clock_unit_ms > 100) {
          XP_LOG_ERROR("IMU clock jumps. clock_count_64 " << clock_count_wo_overflow
                       << " last_clock_count_64 " << last_clock_count_64);
        }
      }
      last_clock_count_64 = clock_count_wo_overflow;
      // [NOTE] We have to flip the axes properly to align IMU coordinates with Camera L
      // The stored time_stamp is in 100us
      XPDRIVER::ImuData xp_imu;
      convert_imu_axes(imu_data, sensor_type_, &xp_imu);
      xp_imu.time_stamp = (clock_count_wo_overflow - first_imu_clock_count_) * clock_unit_ms * 10;
      if (imu_data_callback_ != nullptr) {
        imu_data_callback_(xp_imu);
      }

      ++pull_imu_count_;
      if (pull_imu_count_ > 10) {
        const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            steady_clock::now() - thread_pull_imu_pre_timestamp_).count();
        thread_pull_imu_pre_timestamp_ = steady_clock::now();
        pull_imu_rate_ = pull_imu_count_ * 1000 / ms;
        pull_imu_count_ = 0;
      }
    } else {
      XP_LOG_ERROR("XPDRIVER::IMU_DataAccess failed");
    }
  }
  XP_VLOG(1, "========= thread_pull_imu terminated =========");
}

void XpSensorMultithread::thread_stream_images() {
  // TODO(mingyu): Put back thread param control
  XP_VLOG(1, "======== start thread_stream_images thread");
  Counter32To64 counter32To64_img(XP_CLOCK_32BIT_MAX_COUNT);
  int frame_counter = 0;
  uint64_t last_img_count_wo_overflow_debug = 0;
  XPDRIVER::XP_SENSOR::ImuReader imu_reader;  // read IMU encoded in img

  // Compute running rate
  thread_stream_images_pre_timestamp_ = steady_clock::now();
  // images refer to RGB or GRAY image.
  stream_images_count_ = 0;
  stream_images_rate_ = 0;
  stream_ir_images_count_ = 0;
  stream_ir_images_rate_ = 0;

  // we need to copy mmap data to a buffer immediately
  // create a buffer that's twice the size of the anticipated data
  // imu data in memory
  int imu_data_pos = 0;
  const int imu_data_len = 17;
  uint8_t image_imcomplete_cout = 0;
  // check first 8 frame whether image is complete
  #define IMAGE_COMPLETE_CHECK_NUM 8
  // prevent imu data overflow
  Counter32To64 counter32To64_imu(XP_CLOCK_32BIT_MAX_COUNT);
  while (is_running_) {
    XPDRIVER::ScopedLoopProfilingTimer loopProfilingTimer(
      "XpSensorMultithread::thread_stream_images", 1);
    RawPtrAndSysTime raw_ptr_n_sys_time;
    if (!raw_sensor_img_mmap_ptr_queue_.wait_and_pop_front(&raw_ptr_n_sys_time)) {
      break;
    }
    uint8_t* img_data_ptr = raw_ptr_n_sys_time.first;
    ++frame_counter;
#ifdef __ARM_NEON__
    // since arm platform is buggy, signal the user that at least
    // 1 image is received.
    // TODO(mingyu): Does this problem still persist in our current sensor?
    if (frame_counter == 1) {
      XP_LOG_INFO("Receiving imgs" << std::endl);
    }
#endif
    // The first few imgs contain garbage data
    if (frame_counter == 1) {
      continue;
    } else if (frame_counter > 1) {
      // Only check the left image tag starting from row = 6 to avoid checking on embedded imu data
      bool frame_incomplete = false;
      for (int i = 6; i < XP_sensor_spec_.RowNum; i++) {
        uint8_t *line_start_ptr = img_data_ptr + XP_sensor_spec_.ColNum * i * 2;
        if (*line_start_ptr != (i & 0xFF)) {
          frame_incomplete = true;
        }
      }
      if (frame_incomplete) {
        image_imcomplete_cout++;
        XP_LOG_ERROR("Warning: frame :" << frame_counter << " detect imcomplete image");
        continue;
      }
    }
    if (frame_counter < IMAGE_COMPLETE_CHECK_NUM && image_imcomplete_cout >= 2) {
      imu_from_image_ = true;
      XP_LOG_ERROR("Warniing: auto-swithc to imu from image moe");
      // skip 2 image after auto switcn imu_from_image mode
      if (frame_counter < IMAGE_COMPLETE_CHECK_NUM + 2)
        continue;
    }
    // get and check image timestamp
    uint64_t clock_count_with_overflow = 0;
    if (sensor_type_ == SensorType::XP ||
        sensor_type_ == SensorType::XP2 ||
        sensor_type_ == SensorType::XP3 ||
        sensor_type_ == SensorType::XPIRL ||
        sensor_type_ == SensorType::XPIRL2 ||
        sensor_type_ == SensorType::XPIRL3 ||
        sensor_type_ == SensorType::FACE) {
      clock_count_with_overflow = XP_SENSOR::get_timestamp_in_img(img_data_ptr + imu_data_pos);
    } else {
      XP_LOG_FATAL("Wrong sensor type");
    }
    if (!timestamp_check(clock_count_with_overflow, raw_ptr_n_sys_time.second)) {
      continue;
    }
    /*
    if (!time_orderness_check(clock_count_with_overflow)) {
      XP_LOG_ERROR("detect disorder image, timestamp: " << clock_count_with_overflow);
      continue;
    }
    if (!timestamp_jump_check(clock_count_with_overflow)) {
      XP_LOG_ERROR("detect jump timestamp image, timestamp: " << clock_count_with_overflow);
      continue;
    }
    */
    // Get IMU data if requested (only available for XP series sensor)
    // ignore imu data on android
      if (imu_from_image_) {
      uint8_t* imu_burst_data_pos =  img_data_ptr;
      uint32_t imu_num = 0;
      // Support different versions of firmware. Old version is 25 Hz,
      // and the new version is 500 Hz, and we downsample to ~100 Hz
      if (*(imu_burst_data_pos + 16) == 0) {
        imu_num = 1;
      } else {
        imu_num = *(reinterpret_cast<uint32_t *>(imu_burst_data_pos + imu_data_len));
        // update burst imu data position.
        imu_burst_data_pos = imu_burst_data_pos + imu_data_len + 4;
      }
      if (imu_num > 20)
          imu_num = 20;
      pull_imu_count_ += imu_num;  // Will calculate the effective imu rate w/ image rate
      XP_20608_data imu_data;  // read IMU encoded in
      constexpr bool use_100us = false;
      for (int imu_i = 0; imu_i < imu_num; imu_i += 5) {
        if (imu_reader.get_imu_from_img(imu_burst_data_pos + imu_i * imu_data_len, &imu_data,
                                        use_100us)) {
          if (first_imu_clock_count_ == 0) {
            first_imu_clock_count_ = imu_reader.first_imu_clock_count();
            XP_VLOG(1, "Setting first_imu_clock_count_ " << first_imu_clock_count_);
          }
          // TODO(mingyu): the timestamp / clock count is so messy here...
          // Need to UNIFY
          uint64_t clock_count_wo_overflow =
              counter32To64_imu.convertNewCount32(imu_data.clock_count);

          XPDRIVER::ImuData xp_imu;
          convert_imu_axes(imu_data, sensor_type_, &xp_imu);
          // The XP clock unit is ms.  1 ms = 10 100us
          xp_imu.time_stamp = (clock_count_wo_overflow - first_imu_clock_count_) * 10;  // in 100us

          if (imu_data_callback_ != nullptr) {
            imu_data_callback_(xp_imu);
          }
        }
      }
    }

    // Start receiving image once we have received the first imu (for correct clock offset)
    if (first_imu_clock_count_ == 0) {
      continue;
    }

    // [NOTE] Out-of-order image timestamp will cause counter332To64 falsely
    //        think the timestamp is overflow
    uint64_t clock_count_wo_overflow =
        counter32To64_img.convertNewCount32(clock_count_with_overflow);
    if (last_img_count_wo_overflow_debug > clock_count_wo_overflow) {
      XP_LOG_FATAL("last_img_count_wo_overflow_debug > clock_count_wo_overflow "
                   << last_img_count_wo_overflow_debug
                   << " > " << clock_count_wo_overflow
                   << " overflow count " << counter32To64_img.getOverflowCount());
    }
    last_img_count_wo_overflow_debug = clock_count_wo_overflow;

    // [NOTE] img_time in sec
    const float clock_unit_sec = 1e-3;
    float img_time_sec =
        static_cast<float>(clock_count_wo_overflow - first_imu_clock_count_) * clock_unit_sec;
    const float time_100us = img_time_sec * 10000;
    // negative time usually suggests garbage data
    if (img_time_sec < 0) {
      continue;
    }

    // Get stereo images
    // [NOTE] The returned cv::Mat is CV_8UC1 if the sensor is mono-color,
    //        and CV_8UC3 if the sensor is color
    cv::Mat img_l, img_r;
    cv::Mat img_l_IR, img_r_IR;

    get_images_from_raw_data(img_data_ptr, &img_l, &img_r, &img_l_IR, &img_r_IR);
    if (img_l.rows != 0 && img_r.rows != 0) {
      // Control brightness with user input aec_index or aec (adjust every 5 frames)
      if (use_auto_gain_ && frame_counter % 5 == 3) {
        int new_aec_index = aec_index_;
        using XPDRIVER::XP_SENSOR::kAEC_steps;
        using XPDRIVER::XP_SENSOR::kAR0141_AEC_steps;
        // [NOTE] Color image is converted to gray inside computeNewAecTableIndex
        bool is_ar0141_sensor = (sensor_type_ == SensorType::XPIRL2 || \
                                 sensor_type_ == SensorType::XPIRL3);
        if (XPDRIVER::computeNewAecTableIndex(img_l, aec_settle_,
            is_ar0141_sensor ? kAR0141_AEC_steps : kAEC_steps, &new_aec_index)) {
          if (new_aec_index != aec_index_) {
            aec_index_ = new_aec_index;
            aec_index_updated_ = true;
          } else {
            // new_aec_index == aec_index_.  Let's mark aec settled down.
            if (!aec_settle_) {
              aec_settle_ = true;
            }
          }
        } else {
          XP_LOG_ERROR("computeNewAecTableIndex fails");
          aec_index_updated_ = false;
        }
      }

      if (aec_index_updated_) {
        aec_index_updated_ = false;  // reset
        const bool verbose = !use_auto_gain_;
        XP_SENSOR::set_aec_index(video_sensor_file_id_, aec_index_, sensor_type_, verbose);
      }

      // Start to output images only if
      // 1) AEC has settled, or
      // 2) after a long waiting period, AEC still has problems to settle.
      // [NOTE] aec_settle_ is initialized to true if we are NOT using auto gain/exp control.
      if (!aec_settle_) {
        if (img_time_sec < 3.f) {
          // Keep trying to adjust aec for next iteration
          continue;
        } else {
          // Timeout and give up.  Mark aec settled down
          XP_LOG_ERROR("aec cannot settle during start up");
          aec_settle_ = true;
        }
      }

      // Intensionally process images after a short delay,
      // so that we can have IMU measurements queued up before the first image.
      if (img_time_sec <  0.05) continue;

      if (image_data_callback_ != nullptr) {
        image_data_callback_(img_l, img_r, time_100us, raw_ptr_n_sys_time.second);
      }
      ++stream_images_count_;
    }
    if (img_l_IR.rows != 0 && img_r_IR.rows != 0) {
      bool is_ir_sensor = (sensor_type_ == SensorType::XPIRL2 ||\
                           sensor_type_ == SensorType::XPIRL3);
      if (IR_data_callback_ != nullptr && is_ir_sensor) {
        IR_data_callback_(img_l_IR, img_r_IR, time_100us, raw_ptr_n_sys_time.second);
      }
      ++stream_ir_images_count_;
    }
    if (ir_ctl_updated_ == true) {
      ir_ctl_updated_ = false;  // reset
      XP_SENSOR::xp_infrared_ctl(video_sensor_file_id_, ir_mode_, infrared_index_, rgb_ir_period_);
    }

    if (stream_images_count_ > 10) {
      const auto now_ts = steady_clock::now();
      const int ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now_ts - thread_stream_images_pre_timestamp_).count();
      thread_stream_images_pre_timestamp_ = now_ts;
      if (ms != 0) {
        stream_images_rate_ = stream_images_count_ * 1000 / ms;
      } else {
        stream_images_rate_ = 0;
      }
      stream_images_count_ = 0;
      if (imu_from_image_) {
        if (ms != 0) {
          pull_imu_rate_ = pull_imu_count_ * 1000 / ms;
        } else {
          pull_imu_rate_ = 0;
        }
        pull_imu_count_ = 0;
      }
      if (open_rgb_ir_mode_) {
        if (ms != 0) {
          stream_ir_images_rate_ = stream_ir_images_count_ * 1000 / ms;
        } else {
          stream_ir_images_rate_ = 0;
        }
        stream_ir_images_count_ = 0;
      }
    }

    XP_VLOG(1, "thread_stream_images pushed new img time " << img_time_sec);
    XP_VLOG(1, "======== thread_stream_images loop ends");
  }
  XP_VLOG(1, "======== terminate thread_stream_images");
}

namespace internal {
// [NOTE] The timestamp here may overflow (in 32bit representation)
inline bool reverse_timestamp_check(const int64_t ts_prev,
                                    const int64_t ts_curr) {
  bool check_ok = false;
  if (ts_prev < ts_curr) {
    // No overfow
    check_ok = true;
  } else if (ts_prev > ts_curr) {
    if (ts_curr + XP_CLOCK_32BIT_MAX_COUNT - ts_prev < 10000) {
      // Indeed an overflow occurs
      check_ok = true;
    } else {
      // No overflow occurs. Timestamp is out-of-order.
      XP_LOG_ERROR("Skip out-of-order timestamp curr_ts = " << ts_curr);
    }
  } else {
    XP_LOG_ERROR("Skip duplicate timestamp curr_ts = " << ts_curr);
  }
  return check_ok;
}

inline bool duplicate_timestamp_check(
    const RingBuffer<XpSensorMultithread::TimestampAndSysTime>& ts_ring_buffer,
    const uint64_t image_timestamp) {
  for (size_t i = 0; i < ts_ring_buffer.size(); ++i) {
    if (ts_ring_buffer[i].first == image_timestamp) {
      // This image_timestamp is a duplicate within ts_ring_buffer
      return false;
    }
  }
  return true;
}

inline bool delta_timestamp_check(const XpSensorMultithread::TimestampAndSysTime& ts_prev,
                                  const XpSensorMultithread::TimestampAndSysTime& ts_curr,
                                  const int64_t margin_ms,
                                  int64_t* delta_ts_ms,
                                  int64_t* delta_sys_ms) {
  *delta_sys_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      ts_curr.second - ts_prev.second).count();
  if (ts_prev.first < ts_curr.first) {
    // No overflow occurs.
    *delta_ts_ms = ts_curr.first - ts_prev.first;
  } else {
    // A potential overflow occurs or a corrupted timestamp or a out-of-order timestamp
    // A disorder timestamp will be easily discovered at this step because
    // delta_ts_ms will be a very large number.
    *delta_ts_ms = ts_curr.first + XP_CLOCK_32BIT_MAX_COUNT - ts_prev.first;
  }
  return *delta_ts_ms < *delta_sys_ms + margin_ms;
}
}  // namespace internal

bool XpSensorMultithread::timestamp_check(
    const uint64_t ts_with_overflow,
    const std::chrono::time_point<std::chrono::steady_clock>& sys_time) {
  // Timestamp jump may be caused by three factors:
  // 1) V4L2 accidentally flushes all buffers out, which causes receiving V4L2_BUFFER_NUM
  //    frames that previously received.
  // 2) The system really runs slow, i.e., thread_ioctl_control is forced to sleep due to
  //    congestion in raw_sensor_img_mmap_ptr_queue_.  This will reflect in sys_time.
  // 3) The timestamp data is corrupted and results in an totally incorrect timestamp.
  if (!ts_ring_buffer_.full()) {
    // ts_ring_buffer being non-full can only happen at the beginning, which is impossible
    // for timestamp overflow.
    if (ts_ring_buffer_.empty() || ts_with_overflow > ts_ring_buffer_.back().first) {
      ts_ring_buffer_.push_back({ts_with_overflow, sys_time});
      latest_img_ts_with_overflow_ = ts_with_overflow;
      return true;
    } else {
      return false;
    }
  }

  // Check case 1: duplicate timestamps
  if (!internal::duplicate_timestamp_check(ts_ring_buffer_, ts_with_overflow)) {
    XP_LOG_ERROR("Skip duplicated timestamp curr_ts = " << ts_with_overflow);
    return false;
  }

  // Check case 2: timestamp jump
  //       case 3: timestamp corrupted
  TimestampAndSysTime ts_curr{ts_with_overflow, sys_time};
  int64_t delta_ts_ms, delta_sys_ms;  // ts_curr and ts_prev
  bool check_ok = internal::delta_timestamp_check(ts_ring_buffer_.back(), ts_curr, 50,
                                                  &delta_ts_ms, &delta_sys_ms);
  ts_ring_buffer_.push_back(ts_curr);  // store this TimestampAndSysTime anyway
  if (!check_ok) {
    XP_LOG_ERROR("Skip abnormal timestamp! prev_ts = " << ts_ring_buffer_.back().first
                 << " curr_ts = " << ts_with_overflow
                 << " delta_ts = " << delta_ts_ms << " delta_sys_ms = " << delta_sys_ms);
    return false;
  }

  // Lastly, make sure ts_with_overflow_ is in-order with the latest_img_ts_with_overflow_
  // [IMPORTANT] Image timestamp must *ONLY* grow monotonically.
  check_ok = internal::reverse_timestamp_check(latest_img_ts_with_overflow_,
                                               ts_with_overflow);
  if (check_ok) {
    latest_img_ts_with_overflow_ = ts_with_overflow;  // Remember the latest healthy ts
  }
  return check_ok;
}

bool XpSensorMultithread::set_auto_gain(const bool use_aec) {
  use_auto_gain_ = use_aec;
  return true;
}

bool XpSensorMultithread::set_aec_index(const int aec_index) {
  if (use_auto_gain_) {
    return false;
  }
  aec_index_ = aec_index;
  aec_index_updated_ = true;
  return true;
}

bool XpSensorMultithread::set_infrared_param(const XP_SENSOR::infrared_mode_t IR_mode,
                                             const int infrared_index, const int ir_period) {
  infrared_index_ = infrared_index;
  rgb_ir_period_ = ir_period;
  ir_mode_ = IR_mode;

  if (IR_mode == XP_SENSOR::OFF) {
    // set rgb_ir_period zero to close all IR light
    rgb_ir_period_ = 0;
  }
  ir_ctl_updated_ = true;

  return true;
}

bool XpSensorMultithread::get_sensor_deviceid(std::string* device_id) {
  *device_id = std::string(XP_sensor_spec_.dev_id);
  return true;
}

bool XpSensorMultithread::get_sensor_resolution(uint16_t* width, uint16_t* height) {
  if (XP_sensor_spec_.RowNum > 0 && XP_sensor_spec_.ColNum > 0) {
    *width = XP_sensor_spec_.ColNum;
    *height = XP_sensor_spec_.RowNum;
    return true;
  }
  return false;
}

bool XpSensorMultithread::get_sensor_soft_ver(XpSoftVersion* soft_ver) {
  if ((XP_sensor_spec_.firmware_soft_version.soft_ver_major |
       XP_sensor_spec_.firmware_soft_version.soft_ver_minor |
       XP_sensor_spec_.firmware_soft_version.soft_ver_patch) != 0) {
    *soft_ver = XP_sensor_spec_.firmware_soft_version;
    return true;
  } else {
    return false;
  }
}

bool XpSensorMultithread::get_sensor_type(SensorType* sensor_type) {
  *sensor_type = sensor_type_;
  return true;
}

bool XpSensorMultithread::is_color() const {
  return (sensor_type_ == SensorType::XP3 ||
          sensor_type_ == SensorType::XPIRL2 ||
          sensor_type_ == SensorType::XPIRL3 ||
          sensor_type_ == SensorType::FACE);
}

// handle XP XP2 XPIRL gray sensor image from raw data
bool XpSensorMultithread::get_v024_img_from_raw_data(const uint8_t* img_data_ptr,
                                                       cv::Mat* img_l_ptr,
                                                       cv::Mat* img_r_ptr) {
  XP_CHECK_NOTNULL(img_data_ptr);
  int xp_shift_num = 0;

  zero_col_shift_detect(img_data_ptr, &xp_shift_num);
  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  if (!sensor_MT9V_image_separate(img_data_ptr, img_l_ptr, img_r_ptr)) {
      XP_LOG_ERROR("MT9V34 separate img error");
      return false;
  }
  return true;
}

// handle XP3 FACE color sensor image from raw data
bool XpSensorMultithread::get_v034_img_from_raw_data(const uint8_t* img_data_ptr,
                                                     cv::Mat* img_l_ptr,
                                                     cv::Mat* img_r_ptr) {
  XP_CHECK_NOTNULL(img_data_ptr);
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;
  // use static to avoid temporary memory for every frame.
  static cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
  static cv::Mat img_r_mono(row_num, col_num, CV_8UC1);

  int xp_shift_num = 0;
  zero_col_shift_detect(img_data_ptr, &xp_shift_num);
  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  if (!sensor_MT9V_image_separate(img_data_ptr, &img_l_mono, &img_r_mono)) {
    XP_LOG_ERROR("MT9V34 separate img error");
    return false;
  }
  cv::Mat img_l_color(row_num, col_num, CV_8UC3);
  cv::Mat img_r_color(row_num, col_num, CV_8UC3);
  cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_BayerGR2BGR);
  cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_BayerGR2BGR);
  if (whiteBalanceCorrector_) {
    whiteBalanceCorrector_->run(&img_l_color);
    whiteBalanceCorrector_->run(&img_r_color);
  }
  // FACE is basically XP3 with a special orientation configuration
  if (sensor_type_ == SensorType::FACE) {
    // TODO(mingyu): Figure out the flip / transpose used here
    cv::transpose(img_l_color, img_l_color);
    cv::flip(img_l_color, img_l_color, 1);
    cv::transpose(img_r_color, img_r_color);
    cv::flip(img_r_color, img_r_color, 0);
    *img_r_ptr = img_l_color;
    *img_l_ptr = img_r_color;
  } else {
    *img_l_ptr = img_l_color;
    *img_r_ptr = img_r_color;
  }
  return true;
}


bool XpSensorMultithread::get_XPIRL2_img_from_raw_data(const uint8_t* img_data_ptr,
                                                       cv::Mat* img_l_ptr,
                                                       cv::Mat* img_r_ptr,
                                                       cv::Mat* img_l_IR_ptr,
                                                       cv::Mat* img_r_IR_ptr) {
  // stage 1: parsing RGB or IR images and correcting column shift, time cost: 4.5ms
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;
  ImageType frame_type = ImageType::RGB;

  assert(img_l_ptr != nullptr);
  assert(img_r_ptr != nullptr);
  assert(img_l_IR_ptr != nullptr);
  assert(img_r_IR_ptr != nullptr);

  int xp_shift_num = 0;
  zero_col_shift_detect(img_data_ptr, &xp_shift_num);

  if (img_data_ptr[0] == 'I' && img_data_ptr[1] == 'R') {
    frame_type = ImageType::IR;
  } else if (img_data_ptr[0] == 'R' && img_data_ptr[1] == 'G' && img_data_ptr[2] == 'B') {
    frame_type = ImageType::RGB;
  } else {
    frame_type = ImageType::Unkown_type;
    XP_LOG_ERROR(std::hex << "sensor can't get RGB image value:" << img_data_ptr[0]
                 << img_data_ptr[1] << img_data_ptr[2]
                 << ", please check sensor type or firmware version");
  }

  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);

  if (frame_type == ImageType::IR) {
    // only allocate memory when neccessary.
    if (img_l_IR_ptr->data == nullptr) {
      img_l_IR_ptr->create(row_num / 2 , col_num / 2, CV_8UC1);
    }
    if (img_r_IR_ptr->data == nullptr) {
      img_r_IR_ptr->create(row_num / 2, col_num / 2, CV_8UC1);
    }
    // IR image
    for (int i = 0; i < row_num / 2; ++i) {
      uint8_t* ir_l_row_ptr = img_l_IR_ptr->ptr(i);
      uint8_t* ir_r_row_ptr = img_r_IR_ptr->ptr(i);
      const uint8_t* raw_data_row_ptr = img_data_ptr + 4 * i * col_num;
      for (int j = 0; j < col_num / 2; j += 8) {
        *(ir_l_row_ptr + j + 0) = *(raw_data_row_ptr + 4 * j);
        *(ir_r_row_ptr + j + 0) = *(raw_data_row_ptr + 4 * j + 1);
        *(ir_l_row_ptr + j + 1) = *(raw_data_row_ptr + 4 * (j + 1));
        *(ir_r_row_ptr + j + 1) = *(raw_data_row_ptr + 4 * (j + 1) + 1);
        *(ir_l_row_ptr + j + 2) = *(raw_data_row_ptr + 4 * (j + 2));
        *(ir_r_row_ptr + j + 2) = *(raw_data_row_ptr + 4 * (j + 2) + 1);
        *(ir_l_row_ptr + j + 3) = *(raw_data_row_ptr + 4 * (j + 3));
        *(ir_r_row_ptr + j + 3) = *(raw_data_row_ptr + 4 * (j + 3) + 1);
        *(ir_l_row_ptr + j + 4) = *(raw_data_row_ptr + 4 * (j + 4));
        *(ir_r_row_ptr + j + 4) = *(raw_data_row_ptr + 4 * (j + 4) + 1);
        *(ir_l_row_ptr + j + 5) = *(raw_data_row_ptr + 4 * (j + 5));
        *(ir_r_row_ptr + j + 5) = *(raw_data_row_ptr + 4 * (j + 5) + 1);
        *(ir_l_row_ptr + j + 6) = *(raw_data_row_ptr + 4 * (j + 6));
        *(ir_r_row_ptr + j + 6) = *(raw_data_row_ptr + 4 * (j + 6) + 1);
        *(ir_l_row_ptr + j + 7) = *(raw_data_row_ptr + 4 * (j + 7));
        *(ir_r_row_ptr + j + 7) = *(raw_data_row_ptr + 4 * (j + 7) + 1);
      }
    }
  } else {
    // use static to avoid allocate new memory for each frame
    static cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
    static cv::Mat img_r_mono(row_num, col_num, CV_8UC1);
    cv::Mat img_l_color(row_num, col_num, CV_8UC3);
    cv::Mat img_r_color(row_num, col_num, CV_8UC3);
    uint8_t top[2][8] __attribute__((aligned(16))) = {0};
    uint8_t mid[2][8] __attribute__((aligned(16))) = {0};
    uint8_t bot[2][8] __attribute__((aligned(16))) = {0};

    uint8_t* img_l_mono_data_ptr = img_l_mono.ptr();
    uint8_t* img_r_mono_data_ptr = img_r_mono.ptr();
    const uint8_t* mid_row_ptr = img_data_ptr;
    const uint8_t* bot_row_ptr = img_data_ptr + col_num * 2;
    const int row_buf_size = 2 * col_num;
    uint8_t cache_top_bot[2][2] = {0};
#ifdef __SSE3__
    const __m128i mask = _mm_setr_epi8(0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15);
#endif  // __SSE3__
    for (int c = 0; c < row_buf_size; c += 16) {
#ifdef __SSE3__
      _mm_storeu_si128(reinterpret_cast<__m128i*>(mid), _mm_shuffle_epi8(
                       _mm_loadu_si128(reinterpret_cast<const __m128i*>(mid_row_ptr + c)), mask));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(bot), _mm_shuffle_epi8(
                       _mm_loadu_si128(reinterpret_cast<const __m128i*>(bot_row_ptr + c)), mask));
#else
      for (int i = 0; i < 8; ++i) {
        mid[0][i] = mid_row_ptr[c + 2 * i];
        mid[1][i] = mid_row_ptr[c + 2 * i + 1];
        bot[0][i] = bot_row_ptr[c + 2 * i];
        bot[1][i] = bot_row_ptr[c + 2 * i + 1];
      }
#endif  // __SSE3__
      // for consistency, we use dividing by 2 for all pixels
      mid[0][0] = (bot[0][1] + cache_top_bot[0][1]) / 2;
      mid[1][0] = (bot[1][1] + cache_top_bot[1][1]) / 2;
      // compute G and store to IR place
      for (int i = 2; i < 8; i += 2) {
        mid[0][i] = (bot[0][i + 1] + bot[0][i - 1]) / 2;
        mid[1][i] = (bot[1][i + 1] + bot[1][i - 1]) / 2;
      }

      memmove(img_l_mono_data_ptr, mid[0], 8);
      memmove(img_l_mono_data_ptr + col_num, bot[0], 8);
      memmove(img_r_mono_data_ptr, mid[1], 8);
      memmove(img_r_mono_data_ptr + col_num, bot[1], 8);

      img_l_mono_data_ptr += 8;
      img_r_mono_data_ptr += 8;
      // update cache row
      cache_top_bot[0][1] = bot[0][7];
      cache_top_bot[1][1] = bot[1][7];
    }

    const uint8_t* top_row_ptr;
    // for now we have process the first two row
    for (int r = 2; r < row_num; r += 2) {
      top_row_ptr = img_data_ptr + (r - 1) * row_buf_size;
      mid_row_ptr = img_data_ptr + r * row_buf_size;
      bot_row_ptr = img_data_ptr + (r + 1) * row_buf_size;
      img_l_mono_data_ptr = img_l_mono.ptr(r);
      img_r_mono_data_ptr = img_r_mono.ptr(r);
      // the very left is always zero
      cache_top_bot[0][0] = 0;
      cache_top_bot[0][1] = 0;
      cache_top_bot[1][0] = 0;
      cache_top_bot[1][1] = 0;

      for (int c = 0; c < row_buf_size; c += 16) {
#ifdef __SSE3__
        _mm_storeu_si128(reinterpret_cast<__m128i*>(top), _mm_shuffle_epi8(
                      _mm_loadu_si128(reinterpret_cast<const __m128i*>(top_row_ptr + c)), mask));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(mid), _mm_shuffle_epi8(
                      _mm_loadu_si128(reinterpret_cast<const __m128i*>(mid_row_ptr + c)), mask));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(bot), _mm_shuffle_epi8(
                      _mm_loadu_si128(reinterpret_cast<const __m128i*>(bot_row_ptr + c)), mask));
#else
        for (int i = 0; i < 8; ++i) {
          top[0][i] = top_row_ptr[c + 2 * i];
          top[1][i] = top_row_ptr[c + 2 * i + 1];
          mid[0][i] = mid_row_ptr[c + 2 * i];
          mid[1][i] = mid_row_ptr[c + 2 * i + 1];
          bot[0][i] = bot_row_ptr[c + 2 * i];
          bot[1][i] = bot_row_ptr[c + 2 * i + 1];
        }
#endif  // __SSE3__
        // for consistency, we use dividing by 4 for all pixels
        mid[0][0] = (static_cast<uint32_t>(bot[0][1]) + static_cast<uint32_t>(top[0][1]) + \
                     static_cast<uint32_t>(cache_top_bot[0][0]) + \
                     static_cast<uint32_t>(cache_top_bot[0][1])) >> 2;
        mid[1][0] = (static_cast<uint32_t>(bot[1][1]) + static_cast<uint32_t>(top[1][1]) + \
                     static_cast<uint32_t>(cache_top_bot[1][0]) + \
                     static_cast<uint32_t>(cache_top_bot[1][1])) >> 2;

        for (int i = 2; i < 8; i += 2) {
          mid[0][i] = ((static_cast<uint32_t>(bot[0][i + 1]) + \
                        static_cast<uint32_t>(bot[0][i - 1])) + \
                       (static_cast<uint32_t>(top[0][i + 1]) + \
                        static_cast<uint32_t>(top[0][i - 1]))) >> 2;
          mid[1][i] = ((static_cast<uint32_t>(bot[1][i + 1]) + \
                        static_cast<uint32_t>(bot[1][i - 1])) + \
                       (static_cast<uint32_t>(top[1][i + 1]) + \
                        static_cast<uint32_t>(top[1][i - 1]))) >> 2;
        }
        memmove(img_l_mono_data_ptr, mid[0], 8);
        memmove(img_l_mono_data_ptr + col_num, bot[0], 8);
        memmove(img_r_mono_data_ptr, mid[1], 8);
        memmove(img_r_mono_data_ptr + col_num, bot[1], 8);
        img_l_mono_data_ptr += 8;
        img_r_mono_data_ptr += 8;
        // update cache row
        // left
        cache_top_bot[0][0] = top[0][7];
        cache_top_bot[0][1] = bot[0][7];
        // right
        cache_top_bot[1][0] = top[1][7];
        cache_top_bot[1][1] = bot[1][7];
      }
    }
    // stage 2: convert to color image: 2 ms
    cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_BayerGB2BGR);
    cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_BayerGB2BGR);
    // stage 3: apply auto white balance: 10 ms, using preset mode can even faster.
    if (whiteBalanceCorrector_) {
      whiteBalanceCorrector_->run(&img_l_color);
      whiteBalanceCorrector_->run(&img_r_color);
    }
    *img_l_ptr = img_l_color;
    *img_r_ptr = img_r_color;
  }

  return true;
}

#ifdef __ARM_NEON__
#include <arm_neon.h>
bool XpSensorMultithread::get_XPIRL2_img_from_raw_data_neon(const uint8_t* img_data_ptr,
                                                       cv::Mat* img_l_ptr,
                                                       cv::Mat* img_r_ptr,
                                                       cv::Mat* img_l_IR_ptr,
                                                       cv::Mat* img_r_IR_ptr) {
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;
  ImageType frame_type = ImageType::RGB;
  assert(img_l_ptr != nullptr);
  assert(img_r_ptr != nullptr);
  assert(img_l_IR_ptr != nullptr);
  assert(img_r_IR_ptr != nullptr);
  int xp_shift_num = 0;
  zero_col_shift_detect(img_data_ptr, &xp_shift_num);

  if (img_data_ptr[0] == 'I' && img_data_ptr[1] == 'R') {
    frame_type = ImageType::IR;
  } else if (img_data_ptr[0] == 'R' && img_data_ptr[1] == 'G' && img_data_ptr[2] == 'B') {
    frame_type = ImageType::RGB;
  } else {
    frame_type = ImageType::Unkown_type;
    XP_LOG_ERROR(std::hex << "sensor can't get RGB image value:" << img_data_ptr[0]
                 << img_data_ptr[1] << img_data_ptr[2]
                 << ", please check sensor type or firmware version");
  }

  handle_col_shift_case(const_cast<uint8_t*>(img_data_ptr), xp_shift_num);
  if (frame_type == ImageType::IR) {
    // only allocate memory when neccessary.
    if (img_l_IR_ptr->data == nullptr) {
      img_l_IR_ptr->create(row_num / 2 , col_num / 2, CV_8UC1);
    }
    if (img_r_IR_ptr->data == nullptr) {
      img_r_IR_ptr->create(row_num / 2, col_num / 2, CV_8UC1);
    }
    // IR image
    for (int i = 0; i < row_num / 2; ++i) {
      uint8_t* ir_l_row_ptr = img_l_IR_ptr->ptr(i);
      uint8_t* ir_r_row_ptr = img_r_IR_ptr->ptr(i);
      const uint8_t* raw_data_row_ptr = img_data_ptr + 4 * i * col_num;
      uint8x16x4_t raw_data;
      for (int j = 0; j < col_num / 2; j += 16) {
        raw_data = vld4q_u8(raw_data_row_ptr + 4 * j);
        vst1q_u8(ir_l_row_ptr + j, raw_data.val[0]);
        vst1q_u8(ir_r_row_ptr + j, raw_data.val[1]);
      }
    }
  } else {
    // use static to avoid allocate new memory for each frame
    static cv::Mat img_l_mono(row_num, col_num, CV_8UC1);
    static cv::Mat img_r_mono(row_num, col_num, CV_8UC1);
    cv::Mat img_l_color(row_num, col_num, CV_8UC3);
    cv::Mat img_r_color(row_num, col_num, CV_8UC3);
    union {
      uint8x16x4_t raw_buf;  // val: IRl, IRr, Rl, Rr, or Bl, Br, Gl, Gr
      struct {
        uint8_t IR[2][16];
        uint8_t R[2][16];
      };
      struct {
        uint8_t B[2][16];
        uint8_t G[2][16];
      };
    } top, mid, bot __attribute__((aligned(16)));
    memset(&top, 0, sizeof(top));
    memset(&mid, 0, sizeof(mid));
    memset(&bot, 0, sizeof(bot));

    uint8_t* img_l_mono_data_ptr = img_l_mono.ptr();
    uint8_t* img_r_mono_data_ptr = img_r_mono.ptr();
    const uint8_t* mid_row_ptr = img_data_ptr;
    const uint8_t* bot_row_ptr = img_data_ptr + col_num * 2;
    const int row_buf_size = 2 * col_num;
    uint8_t cache_top_bot[2][2] = {0};

    for (int c = 0; c < row_buf_size; c += 64) {
      mid.raw_buf = vld4q_u8(mid_row_ptr + c);
      bot.raw_buf = vld4q_u8(bot_row_ptr + c);
      // for consistency, we use dividing by 2 for all pixels
      mid.IR[0][0] = (bot.G[0][0] + cache_top_bot[0][1]) / 2;
      mid.IR[1][0] = (bot.G[1][0] + cache_top_bot[1][1]) / 2;
      // compute G and store to IR place
      for (int i = 1; i < 16; ++i) {
        mid.IR[0][i] = (bot.G[0][i] + bot.G[0][i - 1]) / 2;
        mid.IR[1][i] = (bot.G[1][i] + bot.G[1][i - 1]) / 2;
      }
      uint8x16x2_t reorder_data_IR_R_L = {mid.raw_buf.val[0], mid.raw_buf.val[2]};
      uint8x16x2_t reorder_data_IR_R_R = {mid.raw_buf.val[1], mid.raw_buf.val[3]};
      vst2q_u8(img_l_mono_data_ptr, reorder_data_IR_R_L);
      vst2q_u8(img_r_mono_data_ptr, reorder_data_IR_R_R);
      uint8x16x2_t reorder_data_BG_L = {bot.raw_buf.val[0], bot.raw_buf.val[2]};
      uint8x16x2_t reorder_data_BG_R = {bot.raw_buf.val[1], bot.raw_buf.val[3]};
      vst2q_u8(img_l_mono_data_ptr + col_num, reorder_data_BG_L);
      vst2q_u8(img_r_mono_data_ptr + col_num, reorder_data_BG_R);

      img_l_mono_data_ptr += 32;
      img_r_mono_data_ptr += 32;
      // update cache row
      cache_top_bot[0][1] = bot.G[0][15];
      cache_top_bot[1][1] = bot.G[1][15];
    }

    const uint8_t* top_row_ptr;
    // for now we have process the first two row
    for (int r = 2; r < row_num; r += 2) {
      top_row_ptr = img_data_ptr + (r - 1) * row_buf_size;
      mid_row_ptr = img_data_ptr + r * row_buf_size;
      bot_row_ptr = img_data_ptr + (r + 1) * row_buf_size;
      img_l_mono_data_ptr = img_l_mono.ptr(r);
      img_r_mono_data_ptr = img_r_mono.ptr(r);
      // the very left is always zero
      cache_top_bot[0][0] = 0;
      cache_top_bot[0][1] = 0;
      cache_top_bot[1][0] = 0;
      cache_top_bot[1][1] = 0;

      for (int c = 0; c < row_buf_size; c += 64) {
        top.raw_buf = vld4q_u8(top_row_ptr + c);
        mid.raw_buf = vld4q_u8(mid_row_ptr + c);
        bot.raw_buf = vld4q_u8(bot_row_ptr + c);

        // for consistency, we use dividing by 4 for all pixels
        mid.IR[0][0] = (static_cast<uint32_t>(bot.G[0][0]) + static_cast<uint32_t>(top.G[0][0]) + \
                        static_cast<uint32_t>(cache_top_bot[0][0]) + \
                        static_cast<uint32_t>(cache_top_bot[0][1])) >> 2;
        mid.IR[1][0] = (static_cast<uint32_t>(bot.G[1][0]) + static_cast<uint32_t>(top.G[1][0]) + \
                        static_cast<uint32_t>(cache_top_bot[1][0]) + \
                        static_cast<uint32_t>(cache_top_bot[1][1])) >> 2;

        for (int i = 1; i < 16; ++i) {
          mid.IR[0][i] = ((static_cast<uint32_t>(bot.G[0][i]) + \
                           static_cast<uint32_t>(bot.G[0][i - 1])) + \
                          (static_cast<uint32_t>(top.G[0][i]) + \
                           static_cast<uint32_t>(top.G[0][i - 1]))) >> 2;
          mid.IR[1][i] = ((static_cast<uint32_t>(bot.G[1][i]) + \
                           static_cast<uint32_t>(bot.G[1][i - 1])) + \
                          (static_cast<uint32_t>(top.G[1][i]) + \
                           static_cast<uint32_t>(top.G[1][i - 1]))) >> 2;
        }

        uint8x16x2_t reorder_data_IR_R_L = {mid.raw_buf.val[0], mid.raw_buf.val[2]};
        uint8x16x2_t reorder_data_IR_R_R = {mid.raw_buf.val[1], mid.raw_buf.val[3]};
        vst2q_u8(img_l_mono_data_ptr, reorder_data_IR_R_L);
        vst2q_u8(img_r_mono_data_ptr, reorder_data_IR_R_R);

        uint8x16x2_t reorder_data_BG_L = {bot.raw_buf.val[0], bot.raw_buf.val[2]};
        uint8x16x2_t reorder_data_BG_R = {bot.raw_buf.val[1], bot.raw_buf.val[3]};
        vst2q_u8(img_l_mono_data_ptr + col_num, reorder_data_BG_L);
        vst2q_u8(img_r_mono_data_ptr + col_num, reorder_data_BG_R);

        img_l_mono_data_ptr += 32;
        img_r_mono_data_ptr += 32;
        // update cache row
        // left
        cache_top_bot[0][0] = top.G[0][15];
        cache_top_bot[0][1] = bot.G[0][15];
        // right
        cache_top_bot[1][0] = top.G[1][15];
        cache_top_bot[1][1] = bot.G[1][15];
      }
    }
    // stage 2: very time-consuming here
    cv::cvtColor(img_l_mono, img_l_color, cv::COLOR_BayerGB2BGR);
    cv::cvtColor(img_r_mono, img_r_color, cv::COLOR_BayerGB2BGR);
    // stage 3: apply auto white balance: very time-comsuming
    if (whiteBalanceCorrector_) {
      whiteBalanceCorrector_->run(&img_l_color);
      whiteBalanceCorrector_->run(&img_r_color);
    }
    *img_l_ptr = img_l_color;
    *img_r_ptr = img_r_color;
  }
  return true;
}
#endif  // __ARM_NEON__
bool XpSensorMultithread::get_images_from_raw_data(const uint8_t* img_data_ptr,
                                                   cv::Mat* img_l_ptr,
                                                   cv::Mat* img_r_ptr,
                                                   cv::Mat* img_l_IR_ptr,
                                                   cv::Mat* img_r_IR_ptr) {
  XP_CHECK_NOTNULL(img_data_ptr);
  bool return_value = true;
  switch (sensor_type_) {
    case SensorType::XP:
    case SensorType::XP2:
    case SensorType::XPIRL:
      if (!get_v024_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr)) {
        XP_LOG_ERROR("get v024 image from raw data fail");
        return_value = false;
      }
      break;
    case SensorType::XP3:
    case SensorType::FACE:
      if (!get_v034_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr)) {
        XP_LOG_ERROR("get v034 image from raw data fail");
        return_value = false;
      }
      break;
    case SensorType::XPIRL2:
    case SensorType::XPIRL3:
#ifdef __ARM_NEON__
      if (!get_XPIRL2_img_from_raw_data_neon(img_data_ptr, img_l_ptr, img_r_ptr, img_l_IR_ptr, \
                                             img_r_IR_ptr)) {
        XP_LOG_ERROR("get XPIRL2 image from raw data neon fail");
      }
#else
      if (!get_XPIRL2_img_from_raw_data(img_data_ptr, img_l_ptr, img_r_ptr, img_l_IR_ptr, \
                                      img_r_IR_ptr)) {
        XP_LOG_ERROR("get XPIRL2 image from raw data fail");
      }
#endif  // __ARM_NEON__
      break;
    default:
      XP_LOG_ERROR("Error sensor_type ");
      return_value = false;
  }
  return return_value;
}

bool XpSensorMultithread::zero_col_shift_detect(const uint8_t* img_data_ptr,
                                                int* xp_shift_num) {
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;
  static int frame_cout = 0;
  int R_start = col_num - 1;
  int L_start = 0;
  bool nonzero_det = false;
  *xp_shift_num = 0;

  frame_cout++;
  // detect right zero column
  for (int j = R_start; j > 0; --j) {
    nonzero_det = false;
    for (int i = row_num / 10; i < row_num; i += row_num / 8) {
      nonzero_det |= (img_data_ptr[i * col_num * 2 + j * 2 + 1] > 0);
    }
    if (nonzero_det) {
      if (*xp_shift_num == 0) {
        break;
      } else {
        if (frame_cout < 3) {
          XP_LOG_INFO("R2L_shift_num = " << *xp_shift_num);
        }
        return true;
      }
    } else {
      *xp_shift_num = col_num - j;
    }
  }
  // detect Left zero column
  for (int j = L_start; j < col_num; ++j) {
    nonzero_det = false;
    for (int i = row_num / 5; i < row_num; i += row_num / 8) {
      nonzero_det |= (img_data_ptr[i * col_num * 2 + j * 2 + 1] > 0);
    }
    if (nonzero_det) {
      if (*xp_shift_num == 0) {
        break;
      } else {
        if (frame_cout < 3) {
          XP_LOG_INFO("L2R_shift_num = " << *xp_shift_num);
        }
        return true;
      }
    } else {
      *xp_shift_num = -(j + 1);
    }
  }
  *xp_shift_num = 0;
  return false;
}

bool XpSensorMultithread::sensor_MT9V_image_separate(const uint8_t* img_data_ptr,
                                                     cv::Mat* img_l_ptr,
                                                     cv::Mat* img_r_ptr) {
  XP_CHECK_NOTNULL(img_data_ptr);
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;
  if (img_l_ptr->data == NULL) {
    img_l_ptr->create(row_num, col_num, CV_8UC1);
  }
  if (img_r_ptr->data == NULL) {
    img_r_ptr->create(row_num, col_num, CV_8UC1);
  }

  const int total_size = row_num * col_num * 2;
  uint8_t* l_row_ptr = img_l_ptr->ptr();
  uint8_t* r_row_ptr = img_r_ptr->ptr();
  const uint8_t* ptr_to_raw_image_data = img_data_ptr;
#ifndef __ARM_NEON__
  for (int i = 0; i < total_size; i += 2) {
    *l_row_ptr = ptr_to_raw_image_data[i];
    *r_row_ptr = ptr_to_raw_image_data[i + 1];
    l_row_ptr++;
    r_row_ptr++;
  }
#else
  for (int i = 0; i < total_size; i += 16) {
    uint8x8x2_t data = vld2_u8(ptr_to_raw_image_data + i);
    vst1_u8(l_row_ptr, data.val[0]);
    vst1_u8(r_row_ptr, data.val[1]);
    l_row_ptr += 8;
    r_row_ptr += 8;
  }
#endif
  return true;
}

void XpSensorMultithread::handle_col_shift_case(uint8_t* img_data_ptr, int col_shift) {
  // the normal case
  if (col_shift == 0) {
    return;
  }
  // shift raises up
  const int row_num = XP_sensor_spec_.RowNum;
  const int col_num = XP_sensor_spec_.ColNum;

  if (col_shift < 0) {
    col_shift = -col_shift;
    for (int r = 0; r < row_num; ++r) {
      uint8_t* row_ptr = img_data_ptr + col_num * r * 2 + 1;
      int c = col_shift;
      for (; c < col_num - 16; c += 16) {
#ifdef __ARM_NEON__
        uint8x16x2_t raw_src = vld2q_u8(row_ptr + 2 * c - 1);
        uint8x16x2_t raw_src_shift = vld2q_u8(row_ptr + 2 * (c - col_shift) - 1);
        raw_src_shift.val[1] = raw_src.val[1];
        vst2q_u8(row_ptr + 2 * (c - col_shift) - 1, raw_src_shift);
#else
        row_ptr[2 * (c - col_shift + 0)] = row_ptr[2 * (c + 0)];
        row_ptr[2 * (c - col_shift + 1)] = row_ptr[2 * (c + 1)];
        row_ptr[2 * (c - col_shift + 2)] = row_ptr[2 * (c + 2)];
        row_ptr[2 * (c - col_shift + 3)] = row_ptr[2 * (c + 3)];
        row_ptr[2 * (c - col_shift + 4)] = row_ptr[2 * (c + 4)];
        row_ptr[2 * (c - col_shift + 5)] = row_ptr[2 * (c + 5)];
        row_ptr[2 * (c - col_shift + 6)] = row_ptr[2 * (c + 6)];
        row_ptr[2 * (c - col_shift + 7)] = row_ptr[2 * (c + 7)];
        row_ptr[2 * (c - col_shift + 8)] = row_ptr[2 * (c + 8)];
        row_ptr[2 * (c - col_shift + 9)] = row_ptr[2 * (c + 9)];
        row_ptr[2 * (c - col_shift + 10)] = row_ptr[2 * (c + 10)];
        row_ptr[2 * (c - col_shift + 11)] = row_ptr[2 * (c + 11)];
        row_ptr[2 * (c - col_shift + 12)] = row_ptr[2 * (c + 12)];
        row_ptr[2 * (c - col_shift + 13)] = row_ptr[2 * (c + 13)];
        row_ptr[2 * (c - col_shift + 14)] = row_ptr[2 * (c + 14)];
        row_ptr[2 * (c - col_shift + 15)] = row_ptr[2 * (c + 15)];
#endif  // __ARM_NEON__
      }
      for (; c < col_num; ++c) {
        row_ptr[2 * (c - col_shift)] = row_ptr[2 * c];
      }

      for (c = c - col_shift; c < col_num; ++c) {
        row_ptr[2 * c] = 255;
      }
    }
  } else {
    for (int r = 0; r < row_num; ++r) {
      uint8_t* row_ptr = img_data_ptr + col_num * r * 2 + 1;
      int c = col_num - 1;

      for (; c >= col_shift + 16; c -= 16) {
#ifdef __ARM_NEON__
        uint8x16x2_t raw_src_shift = vld2q_u8(row_ptr + 2 * (c - col_shift) - 1);
        uint8x16x2_t raw_src = vld2q_u8(row_ptr + 2 * c - 1);
        raw_src.val[1] = raw_src_shift.val[1];
        vst2q_u8(row_ptr + 2 * c - 1, raw_src);
#else
        uint8_t img_data_tmp[16];
        img_data_tmp[0] = row_ptr[2 * (c - col_shift - 0)];
        img_data_tmp[1] = row_ptr[2 * (c - col_shift - 1)];
        img_data_tmp[2] = row_ptr[2 * (c - col_shift - 2)];
        img_data_tmp[3] = row_ptr[2 * (c - col_shift - 3)];
        img_data_tmp[4] = row_ptr[2 * (c - col_shift - 4)];
        img_data_tmp[5] = row_ptr[2 * (c - col_shift - 5)];
        img_data_tmp[6] = row_ptr[2 * (c - col_shift - 6)];
        img_data_tmp[7] = row_ptr[2 * (c - col_shift - 7)];
        img_data_tmp[8] = row_ptr[2 * (c - col_shift - 8)];
        img_data_tmp[9] = row_ptr[2 * (c - col_shift - 9)];
        img_data_tmp[10] = row_ptr[2 * (c - col_shift - 10)];
        img_data_tmp[11] = row_ptr[2 * (c - col_shift - 11)];
        img_data_tmp[12] = row_ptr[2 * (c - col_shift - 12)];
        img_data_tmp[13] = row_ptr[2 * (c - col_shift - 13)];
        img_data_tmp[14] = row_ptr[2 * (c - col_shift - 14)];
        img_data_tmp[15] = row_ptr[2 * (c - col_shift - 15)];
        row_ptr[2 * (c - 0)] = img_data_tmp[0];
        row_ptr[2 * (c - 1)] = img_data_tmp[1];
        row_ptr[2 * (c - 2)] = img_data_tmp[2];
        row_ptr[2 * (c - 3)] = img_data_tmp[3];
        row_ptr[2 * (c - 4)] = img_data_tmp[4];
        row_ptr[2 * (c - 5)] = img_data_tmp[5];
        row_ptr[2 * (c - 6)] = img_data_tmp[6];
        row_ptr[2 * (c - 7)] = img_data_tmp[7];
        row_ptr[2 * (c - 8)] = img_data_tmp[8];
        row_ptr[2 * (c - 9)] = img_data_tmp[9];
        row_ptr[2 * (c - 10)] = img_data_tmp[10];
        row_ptr[2 * (c - 11)] = img_data_tmp[11];
        row_ptr[2 * (c - 12)] = img_data_tmp[12];
        row_ptr[2 * (c - 13)] = img_data_tmp[13];
        row_ptr[2 * (c - 14)] = img_data_tmp[14];
        row_ptr[2 * (c - 15)] = img_data_tmp[15];
#endif  // __ARM_NEON__
      }
      for (; c >= col_shift; --c) {
        row_ptr[2 * c] = row_ptr[2 * (c - col_shift)];
      }
      for (; c >= 0; --c) {
        row_ptr[2 * c] = 255;
      }
    }
  }
}
#endif  // __linux__
}  // namespace XPDRIVER
