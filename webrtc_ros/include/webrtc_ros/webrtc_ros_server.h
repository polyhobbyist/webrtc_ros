#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <webrtc_ros/webrtc_client.h>
#include <condition_variable>

namespace webrtc_ros
{

class WebrtcRosServer
{
public:
  WebrtcRosServer(rclcpp::Node::SharedPtr nh);
  ~WebrtcRosServer();
  void run();
  void stop();

  void cleanupWebrtcClient(WebrtcClient *client);

  std::unique_ptr<rtc::Thread>signaling_thread_;
private:

  std::condition_variable shutdown_cv_;
  std::mutex clients_mutex_;
  std::map<WebrtcClient*, WebrtcClientWeakPtr> clients_;

  rclcpp::Node::SharedPtr nh_;
  std::string image_transport_;
  ImageTransportFactory itf_;
};

}

#endif
