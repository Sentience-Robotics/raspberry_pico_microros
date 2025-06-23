```bash
docker run --rm -d -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200

cd /home/dev/micro_ros_raspberrypi_pico_sdk/build
cmake ..
make
picotool load pico_micro_ros_example.uf2 -f
picotool reboot

# ???
cd /home/dev/lucy_ws/lucy_ros2/install
source setup.zsh

ros2 topic echo rp2040_topic
```
