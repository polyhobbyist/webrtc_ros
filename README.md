# WebRTC ROS 2 Node
This is a ROS 2 package that provides a WebRTC peer that can be configured to stream a ROS image topic and recieve a stream that is published to a ROS image topic.

## Installation

```bash
sudo apt-get install libgtk-3-dev pulseaudio
```

## Testing
Suggest downloading a test image to use for testing.  The following is a good test image: https://en.wikipedia.org/wiki/Test_card#/media/File:Philips_PM5544.svg. Copy this into the media folder and run:

```bash
ros2 run image_transport_tutorials my_publisher webrtc_ros/src/webrtc_ros/media/Phillips_PM5544.svg.png
```
This creates a camera with just the test image.

Then run the webrtc_ros node:

```bash
ros2 launch webrtc_ros webrtc_ros_server.launch.py
```
You can then launch the test web server by running:

```bash
python webrtc_ros/scripts/server.py
```

Then navigate to http://localhost:8080 in a web browser. 




## License
webrtc_ros is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.

## References
This project is derived from the [Robot Web Tools](http://robotwebtools.org/) effort.
