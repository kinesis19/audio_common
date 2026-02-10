// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef AUDIO_COMMON__AUDIO_CAPTURER_NODE
#define AUDIO_COMMON__AUDIO_CAPTURER_NODE

#include <memory>
#include <portaudio.h>
#include <rclcpp/rclcpp.hpp>

#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace audio_common {

class AudioCapturerNode : public rclcpp_lifecycle::LifecycleNode {
public:
  AudioCapturerNode();
  ~AudioCapturerNode() override;

  // Lifecycle callback methods
  using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  LifecycleCallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleCallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleCallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleCallbackReturn on_cleanup(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleCallbackReturn on_shutdown(
      const rclcpp_lifecycle::State &previous_state) override;
  
  // Lifecycle 사용 전의 work method
  void work();

private:

  // Timer Callback (work 대체용)
  void capture_callback();
  
  PaStream *stream_;
  int format_;
  int channels_;
  int rate_;
  int chunk_;
  std::string frame_id_;

  rclcpp_lifecycle::LifecyclePublisher<audio_common_msgs::msg::AudioStamped>::SharedPtr audio_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Methods
  template <typename T> std::vector<T> read_data();
};

} // namespace audio_common

#endif