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

#include <driver/helper/counter_32_to_64.h>
#include <driver/helper/xp_logging.h>

namespace XPDRIVER {
// counter converter class
Counter32To64::Counter32To64(uint64_t max_clock_count) :
  max_clock_count_(max_clock_count) {}

uint64_t Counter32To64::convertNewCount32(uint64_t counter32) {
  if (counter32 > max_clock_count_) {
    XP_LOG_FATAL("counter32 > max_clock_count_ "
               << counter32 << " > " << max_clock_count_);
  }
  if (counter32 < last_counter32_) {
    XP_VLOG(1, "++overflow_count_ " << overflow_count_
            << " counter32 " << counter32
            << " last_counter32_ " << last_counter32_);
    ++overflow_count_;
  }
  last_counter32_ = counter32;
  return (static_cast<uint64_t>(counter32) + (overflow_count_ * max_clock_count_));
}

// Not allowed. Will crash for debug purpose
uint64_t Counter32To64::convertNewCount32(int32_t counter32) {
  XP_LOG_FATAL("wrong type input");
  return 0;
}

uint64_t Counter32To64::convertNewCount32(uint32_t counter32) {
  XP_LOG_FATAL("wrong type input");
  return 0;
}

uint64_t Counter32To64::convertNewCount32(int64_t counter32) {
  XP_LOG_FATAL("wrong type input");
  return 0;
}

uint64_t Counter32To64::getOverflowCount() const {
  return overflow_count_;
}

uint64_t Counter32To64::getLastCounter32() const {
  return last_counter32_;
}

}  // namespace XPDRIVER
