/*
 * Copyright 2019, FZI Forschungszentrum Informatik (templating)
 *
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <chrono>
#include "ur_rtde_driver/comm/pipeline.h"
#include "ur_rtde_driver/comm/parser.h"
#include "ur_rtde_driver/comm/stream.h"
#include "ur_rtde_driver/comm/package.h"
#include "ur_rtde_driver/exceptions.h"

namespace ur_driver
{
namespace comm
{
template <typename HeaderT>
class URProducer : public IProducer<HeaderT>
{
private:
  URStream<HeaderT>& stream_;
  Parser<HeaderT>& parser_;
  std::chrono::seconds timeout_;

public:
  URProducer(URStream<HeaderT>& stream, Parser<HeaderT>& parser) : stream_(stream), parser_(parser), timeout_(1)
  {
  }

  void setupProducer()
  {
    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    stream_.setReceiveTimeout(tv);
    if (!stream_.connect())
    {
      throw UrException("Failed to connect to robot. Please check if the robot is booted and connected.");
    }
  }
  void teardownProducer()
  {
    stopProducer();
  }
  void stopProducer()
  {
  }

  bool tryGet(std::vector<std::unique_ptr<URPackage<HeaderT>>>& products)
  {
    // 4KB should be enough to hold any packet received from UR
    uint8_t buf[4096];
    size_t read = 0;
    // expoential backoff reconnects
    while (true)
    {
      if (stream_.read(buf, sizeof(buf), read))
      {
        // reset sleep amount
        timeout_ = std::chrono::seconds(1);
        break;
      }

      if (stream_.closed())
        return false;

      LOG_WARN("Failed to read from stream, reconnecting in %ld seconds...", timeout_.count());
      std::this_thread::sleep_for(timeout_);

      if (stream_.connect())
        continue;

      auto next = timeout_ * 2;
      if (next <= std::chrono::seconds(120))
        timeout_ = next;
    }

    BinParser bp(buf, read);
    return parser_.parse(bp, products);
  }
};
}  // namespace comm
}  // namespace ur_driver
