#include "audio_common/audio_capturer_node.hpp"
#include "audio_common_msgs/msg/audio_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace audio_common;

AudioCapturerNode::AudioCapturerNode() : rclcpp_lifecycle::LifecycleNode("audio_capturer_node") {
  this->declare_parameter<int>("format", paInt16);
  this->declare_parameter<int>("channels", 1);
  this->declare_parameter<int>("rate", 16000);
  this->declare_parameter<int>("chunk", 512);
  this->declare_parameter<int>("device", -1);
  this->declare_parameter<std::string>("frame_id", "");

  RCLCPP_INFO(this->get_logger(), "AudioCapturer node initialized");
}

AudioCapturerNode::~AudioCapturerNode() {
  if (this->stream_) {
    Pa_StopStream(this->stream_);
    Pa_CloseStream(this->stream_);
  }
  Pa_Terminate();
}


AudioCapturerNode::LifecycleCallbackReturn AudioCapturerNode::on_configure(const rclcpp_lifecycle::State &) {
  this->format_ = this->get_parameter("format").as_int();
  this->channels_ = this->get_parameter("channels").as_int();
  this->rate_ = this->get_parameter("rate").as_int();
  this->chunk_ = this->get_parameter("chunk").as_int();
  this->frame_id_ = this->get_parameter("frame_id").as_string();
  int device = this->get_parameter("device").as_int();

  if (Pa_Initialize() != paNoError) {
    return AudioCapturerNode::LifecycleCallbackReturn::FAILURE;
  }

  PaStreamParameters inputParameters;
  inputParameters.device = (device >= 0) ? device : Pa_GetDefaultInputDevice();
  inputParameters.channelCount = this->channels_;
  inputParameters.sampleFormat = this->format_;
  inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
  inputParameters.hostApiSpecificStreamInfo = nullptr;

  PaError err = Pa_OpenStream(&this->stream_, &inputParameters, nullptr, this->rate_, this->chunk_, paClipOff, nullptr, nullptr);
  if (err != paNoError) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream: %s", Pa_GetErrorText(err));
    return AudioCapturerNode::LifecycleCallbackReturn::FAILURE;
  }

  this->audio_pub_ = this->create_publisher<audio_common_msgs::msg::AudioStamped>("audio", rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(), "Configured: AudioCapturerNode");
  return AudioCapturerNode::LifecycleCallbackReturn::SUCCESS;
}

AudioCapturerNode::LifecycleCallbackReturn AudioCapturerNode::on_activate(const rclcpp_lifecycle::State &) {
  Pa_StartStream(this->stream_);
  this->audio_pub_->on_activate();
  
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&AudioCapturerNode::capture_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "Activated: AudioCapturerNode");
  return AudioCapturerNode::LifecycleCallbackReturn::SUCCESS;
}

AudioCapturerNode::LifecycleCallbackReturn AudioCapturerNode::on_deactivate(const rclcpp_lifecycle::State &) {
  timer_.reset();
  Pa_StopStream(this->stream_);
  this->audio_pub_->on_deactivate();
  
  RCLCPP_INFO(this->get_logger(), "Deactivated: AudioCapturerNode");
  return AudioCapturerNode::LifecycleCallbackReturn::SUCCESS;
}

AudioCapturerNode::LifecycleCallbackReturn AudioCapturerNode::on_cleanup(const rclcpp_lifecycle::State &) {
  Pa_CloseStream(this->stream_);
  Pa_Terminate();
  this->audio_pub_.reset();
  
  RCLCPP_INFO(this->get_logger(), "Cleanup: AudioCapturerNode");
  return AudioCapturerNode::LifecycleCallbackReturn::SUCCESS;
}

AudioCapturerNode::LifecycleCallbackReturn AudioCapturerNode::on_shutdown(const rclcpp_lifecycle::State &) {
  return AudioCapturerNode::LifecycleCallbackReturn::SUCCESS;
}

void AudioCapturerNode::capture_callback() {
  if (!this->audio_pub_->is_activated()) {
    return;
  }

  auto msg = audio_common_msgs::msg::AudioStamped();
  msg.header.frame_id = this->frame_id_;
  msg.header.stamp = this->get_clock()->now();

  switch (this->format_) {
    case paFloat32: msg.audio.audio_data.float32_data = this->read_data<float>(); break;
    case paInt32:   msg.audio.audio_data.int32_data = this->read_data<int32_t>(); break;
    case paInt16:   msg.audio.audio_data.int16_data = this->read_data<int16_t>(); break;
    case paInt8:    msg.audio.audio_data.int8_data = this->read_data<int8_t>(); break;
    case paUInt8:   msg.audio.audio_data.uint8_data = this->read_data<uint8_t>(); break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unsupported format");
      return;
  }

  msg.audio.info.format = this->format_;
  msg.audio.info.channels = this->channels_;
  msg.audio.info.chunk = this->chunk_;
  msg.audio.info.rate = this->rate_;

  this->audio_pub_->publish(msg);
}

template <typename T> 
std::vector<T> AudioCapturerNode::read_data() {
  std::vector<T> data(this->chunk_ * this->channels_);
  Pa_ReadStream(this->stream_, data.data(), this->chunk_);
  return data;
}