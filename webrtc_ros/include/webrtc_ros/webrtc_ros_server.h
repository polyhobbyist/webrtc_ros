#ifndef WEBRTC_ROS_WEBRTC_ROS_SERVER_H_
#define WEBRTC_ROS_WEBRTC_ROS_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>
#include <webrtc_ros/webrtc_client.h>
#include <condition_variable>
#include <webrtc_ros_msgs/srv/create_client.hpp>
#include <webrtc_ros_msgs/srv/close_client.hpp>

namespace webrtc_ros
{

class WebrtcRosServer
{
public:
  WebrtcRosServer(rclcpp::Node::SharedPtr nh);
  ~WebrtcRosServer();
  void run();
  void stop();
private:

  void closeClientCallback(webrtc_ros_msgs::srv::CloseClient::Request::SharedPtr request,
                  webrtc_ros_msgs::srv::CloseClient::Response::SharedPtr response);
  void createClientCallback(webrtc_ros_msgs::srv::CreateClient::Request::SharedPtr request,
                  webrtc_ros_msgs::srv::CreateClient::Response::SharedPtr response);


  std::condition_variable shutdown_cv_;
  std::mutex clients_mutex_;
  std::map<std::string, WebrtcClientWeakPtr> clients_;

  rclcpp::Node::SharedPtr nh_;
  ImageTransportFactory itf_;

  uint16_t nextClientId_;

  std::shared_ptr<rclcpp::Service<webrtc_ros_msgs::srv::CreateClient>> createClientService_;
  std::shared_ptr<rclcpp::Service<webrtc_ros_msgs::srv::CloseClient>> closeClientService_;
};

}

#endif
