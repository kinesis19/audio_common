# audio_capture

This repositiory provides a set of ROS 2 packages for audio. It provides a C++ version to capture and play audio data using PortAudio.

Original: [mgonzs13/audio_common](https://github.com/mgonzs13/audio_common)

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/audio_common.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Docs
[audio_common for ROS2](https://mgonzs13.github.io/audio_common/4.0.8/index.html)