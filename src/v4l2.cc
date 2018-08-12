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
#include <driver/v4l2.h>
#include <driver/helper/xp_logging.h>
#include <driver/helper/basic_image_utils.h>
#include <iostream>
#include <fstream>
#include <list>

#ifdef __ANDROID__
#include <android/log.h>
#include <utilbase.h>
#endif  // __ANDROID__
#ifdef __linux__  // Only support Linux for now

namespace XPDRIVER {
// V4L2 related utility functions implementation
/* config the buffer */
/* local structure */
struct Buffer {
  void *start;
  __u32 offset;
  size_t length;
};
static std::vector<struct Buffer> mmap_buffers;
int v4l2_width, v4l2_height;

bool init_mmap(int fd) {
  int n_buffers = 0;
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  // we need quite a few buffers since odroid may be slow to fetch buffer on time
  req.count = V4L2_BUFFER_NUM;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    if (EINVAL == errno) {
      XP_LOG_ERROR("Not support memory mapping");
      return false;
    } else {
      XP_LOG_ERROR("VIDIOC_REQBUFS");
      return false;
    }
  }

  mmap_buffers.resize(req.count);

  struct v4l2_buffer buf;
  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    memset(&buf, 0, sizeof(buf));

    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      XP_LOG_ERROR("[VIDIOC_QUERYBUF]");
      return false;
    }

    mmap_buffers[n_buffers].length = buf.length;
    mmap_buffers[n_buffers].start =
      mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    mmap_buffers[n_buffers].offset = buf.m.offset;

    if (MAP_FAILED == mmap_buffers[n_buffers].start) {
      XP_LOG_ERROR("[VIDIOC_QUERYBUF]");
      return false;
    }
  }
  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    memset(&buf, 0, sizeof(buf));
    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory      = V4L2_MEMORY_MMAP;
    buf.index       = n_buffers;
    if (ioctl(fd, VIDIOC_QBUF, &buf)) {
      XP_LOG_ERROR("VIDIOC_QBUFS");
      return false;
    }
  }
  return true;
}

bool init_v4l2(const std::string& dev_name_in,
               int* fd_ptr,
               struct v4l2_buffer* bufferinfo_ptr) {
  int& fd = *fd_ptr;
  bool find_cam = false;
  const bool dev_name_given = !dev_name_in.empty();
  std::list<std::string> possible_dev_names;
  if (dev_name_given) {
    possible_dev_names.push_back(dev_name_in);
  } else {
    possible_dev_names.push_back("/dev/video0");
    possible_dev_names.push_back("/dev/video1");
    possible_dev_names.push_back("/dev/video2");
    possible_dev_names.push_back("/dev/video3");
  }
  for (const auto& dev_name : possible_dev_names) {
    // Check the existence of dev_name
    {
      std::ifstream infile(dev_name);
      if (!infile.good()) {
        if (dev_name_given) {
          XP_LOG_ERROR(dev_name << " doesn't exist");
          return false;
        } else {
          // try next name
          XP_LOG_INFO(dev_name << " doesn't exist");
          continue;
        }
      }
    }
    // open the device
    fd = open(dev_name.c_str(), O_RDWR);
    if (fd < 0) {
      if (dev_name_given) {
        XP_LOG_ERROR(dev_name << " OPEN FAILED");
        return false;
      } else {
        // try next name
        XP_LOG_INFO(dev_name << " OPEN FAILED");
        continue;
      }
    }
    // check if the device matches our device name
    find_cam = dev_name_given;  // if the name is given, always use it
    struct v4l2_capability video_cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &video_cap) == -1) {
      XP_LOG_ERROR("init_v4l2: Can't get video_capability");
    } else {
      // match dev name
      std::string xp_dev_name = std::string(reinterpret_cast<char*>(video_cap.card));
      if (xp_dev_name != "FX3"
          && xp_dev_name != "BaiduCam2"
          && xp_dev_name.substr(0, 21) != "Baidu_Robotics_vision") {
        XP_LOG_INFO("Skip " << dev_name << " name " << video_cap.card);
      } else {
        find_cam = true;
        XP_LOG_INFO("Find a " << video_cap.card << " dev at " << dev_name);
        break;
      }
    }
  }
  if (!find_cam) {
    XP_LOG_ERROR("Cannot find xPerception devices");
    return false;
  }
  struct v4l2_buffer& bufferinfo = *bufferinfo_ptr;
  // set frame rate
  if (false) {
    struct v4l2_streamparm stream_parm;
    if (ioctl(fd, VIDIOC_G_PARM, &stream_parm) < 0) {
      XP_LOG_ERROR("ioctl VIDIOC_G_PARM failed");
      return false;
    }
    XP_LOG_INFO("int capture.timeperframe "
                << stream_parm.parm.capture.timeperframe.numerator << "/"
                << stream_parm.parm.capture.timeperframe.denominator
                << " V4L2_CAP_TIMEPERFRAME "
                << (V4L2_CAP_TIMEPERFRAME & stream_parm.parm.capture.capability)
                << std::endl);
    stream_parm.parm.capture.timeperframe.numerator = 1;
    stream_parm.parm.capture.timeperframe.denominator = 30;
    stream_parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_S_PARM, &stream_parm) < 0) {
      XP_LOG_ERROR("ioctl VIDIOC_S_PARM failed.");
      return false;
    }
    // verify
    if (ioctl(fd, VIDIOC_G_PARM, &stream_parm) < 0) {
      XP_LOG_ERROR("ioctl VIDIOC_G_PARM failed");
      return false;
    }
    XP_LOG_INFO("capture.timeperframe " << stream_parm.parm.capture.timeperframe.numerator << "/"
                << stream_parm.parm.capture.timeperframe.denominator << std::endl);
  }

  // set format
  struct v4l2_format format;
  get_v4l2_resolution(fd, &v4l2_width, &v4l2_height);
  memset(&format, 0, sizeof(format));

  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  format.fmt.pix.width  = v4l2_width;
  format.fmt.pix.height = v4l2_height;
  int r = ioctl(fd, VIDIOC_S_FMT, &format);
  if (r < 0) {
    XP_LOG_ERROR("ioctl(fd, VIDIOC_S_FMT, &format) failed. code " << r
               << " dev " << dev_name_in);
    return false;
  }
  init_mmap(fd);
  // Activate streaming
  memset(&bufferinfo, 0, sizeof(bufferinfo));
  auto type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    XP_LOG_ERROR("VIDIOC_STREAMON failed. fd " << fd);
    return false;
  }
  return true;
}
bool get_v4l2_resolution(int fd, int* width, int* height) {
  struct v4l2_format format;
  // get format
  format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret = ioctl(fd, VIDIOC_G_FMT, &format);
  if (ret < 0) {
    XP_LOG_ERROR("ioctl(fd, VIDIOC_G_FMT, &format) failed. code " << ret);
    return false;
  }
  *width = format.fmt.pix.width;
  *height = format.fmt.pix.height;
  return true;
}
bool stop_v4l2(int* fd_ptr, const struct v4l2_buffer& bufferinfo) {
  XP_CHECK_NOTNULL(fd_ptr);
  int& fd = *fd_ptr;
  auto type = bufferinfo.type;
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    XP_LOG_ERROR("VIDIOC_STREAMOFF failed. fd " << fd);
    return false;
  }
  // uninit mmap
  for (int i = 0; i < mmap_buffers.size(); ++i) {
    if (munmap(mmap_buffers[i].start, mmap_buffers[i].length) == -1) {
      XP_LOG_ERROR("munmap " << i << " failed");
    }
  }
  close(fd);
  return true;
}

bool access_next_img_and_queue_next(int fd,
                                    struct v4l2_buffer* bufferinfo_ptr,
                                    uint8_t ** img_data_ptr_ptr) {
  bool valid_img_buffer_data = true;
  bool restart_ok = false;
  if (!access_next_img_pair_data(fd,
                                 bufferinfo_ptr,
                                 img_data_ptr_ptr)) {
    XP_LOG_ERROR("access_next_img_pair_data failed");
    valid_img_buffer_data = false;
  }
  uint8_t* & img_data_ptr = *img_data_ptr_ptr;
  if (img_data_ptr == reinterpret_cast<uint8_t*>(0xffffffff)) {
    // This is what could happen in Odroid that caused crash
    // This happens mostly due to high system load
    XP_LOG_ERROR("mmap returns 0xffffffff ind "
                 << bufferinfo_ptr->index
                 << " seq " << bufferinfo_ptr->sequence);
    valid_img_buffer_data = false;
    // Restart the camera driver if 0xffffffff
    if (ioctl(fd, VIDIOC_STREAMOFF, &bufferinfo_ptr->type) == 0) {
      usleep(30000);  // 30 ms
      if (ioctl(fd, VIDIOC_STREAMON, &bufferinfo_ptr->type) == 0) {
        restart_ok = true;
        XP_LOG_INFO("restart camera OK\n");
      } else {
        XP_LOG_ERROR("Restarting VIDIOC_STREAMON failed " << errno);
      }
    } else {
      XP_LOG_ERROR("Restarting VIDIOC_STREAMOFF failed " << errno);
    }
  }

  if (img_data_ptr == nullptr) {
    // Never see this happens
    XP_LOG_ERROR("img_data_ptr = null");
    valid_img_buffer_data = false;
  }

  // duplicate a buffer info
  // do not return bufferinfo modified by queue_next_img_buffer
  struct v4l2_buffer bufferinfo_new = *bufferinfo_ptr;
  if (!queue_next_img_buffer(fd, &bufferinfo_new)) {
    XP_LOG_ERROR("queue_next_img_buffer failed");
    return false;
  }
  return valid_img_buffer_data;
}

bool access_next_img_pair_data(int fd,
                               struct v4l2_buffer* bufferinfo_ptr,
                               uint8_t ** img_data_ptr) {
  XP_CHECK_NOTNULL(bufferinfo_ptr);
  memset(bufferinfo_ptr, 0, sizeof(struct v4l2_buffer));
  bufferinfo_ptr->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufferinfo_ptr->memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_DQBUF, bufferinfo_ptr) < 0) {
    XP_LOG_ERROR("VIDIOC_DQBUF");
    return false;
  }
  if (bufferinfo_ptr->length != v4l2_width * v4l2_height * 2) {
    XP_LOG_ERROR("bufferinfo.length " << bufferinfo_ptr->length);
    return false;
  }
  *img_data_ptr = static_cast<uint8_t*>(mmap_buffers[bufferinfo_ptr->index].start);
  XP_VLOG(1, "VIDIOC_DQBUF ind " << bufferinfo_ptr->index
          << " addr " << static_cast<void*>(*img_data_ptr)
          << " seq " << bufferinfo_ptr->sequence
          << " len " << bufferinfo_ptr->length
          << " offset " << bufferinfo_ptr->m.offset);
  if (*img_data_ptr == nullptr) {
    return false;
  }
  return true;
}

bool queue_next_img_buffer(int fd, struct v4l2_buffer* bufferinfo_ptr) {
  // Put the buffer in the incoming queue.
  int r = ioctl(fd, VIDIOC_QBUF, bufferinfo_ptr);
  if (r < 0) {
    XP_LOG_ERROR("ioctl(fd, VIDIOC_QBUF, bufferinfo_ptr) error."
               << " buf ind " << bufferinfo_ptr->index << " code: " << r);
    return false;
  }
  return true;
}

}   // namespace XPDRIVER

#endif  // __linux__
