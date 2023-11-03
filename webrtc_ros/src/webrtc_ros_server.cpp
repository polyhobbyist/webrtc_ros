#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_ros_server.h>
#include "webrtc/rtc_base/ssl_adapter.h"

namespace webrtc_ros
{

WebrtcRosServer::WebrtcRosServer(rclcpp::Node::SharedPtr nh)
  : nh_(nh), itf_(nh, std::make_shared<image_transport::ImageTransport>(nh)), nextClientId_(0)
{
  rtc::InitializeSSL();
}

// Implement the CloseClient ROS Service callback
void WebrtcRosServer::closeClientCallback(webrtc_ros_msgs::srv::CloseClient::Request::SharedPtr request,
                  webrtc_ros_msgs::srv::CloseClient::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lock(clients_mutex_);
  auto client = clients_.find(request->instance_id);
  if (client != clients_.end())
  {
    WebrtcClientPtr client_ptr = client->second.lock();
    if (client_ptr)
    {
      clients_.erase(request->instance_id);
      shutdown_cv_.notify_all();

      response->success = true;
    }
    else
    {
      response->success = false;
    }
  }
  else
  {
    response->success = false;
  }
}

// Implement the CreateClient ROS Service callback
void WebrtcRosServer::createClientCallback(webrtc_ros_msgs::srv::CreateClient::Request::SharedPtr request,
                  webrtc_ros_msgs::srv::CreateClient::Response::SharedPtr response)
{
  std::string instanceId = std::to_string(nextClientId_++);
  auto client = std::make_shared<WebrtcClient>(nh_, itf_, request->image_transport, instanceId);

  if (client->init(client))
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    clients_[instanceId] = WebrtcClientWeakPtr(client);
    response->instance_id = instanceId;
  }
}

WebrtcRosServer::~WebrtcRosServer()
{
  stop();
}

void WebrtcRosServer::run()
{
  stop();
  
  signaling_thread_ = rtc::Thread::CreateWithSocketServer();
  signaling_thread_->Start();

  createClientService_ = nh_->create_service<webrtc_ros_msgs::srv::CreateClient>("create_webrtc_client", std::bind(&WebrtcRosServer::createClientCallback, this, std::placeholders::_1, std::placeholders::_2));
  closeClientService_ = nh_->create_service<webrtc_ros_msgs::srv::CloseClient>("close_webrtc_client", std::bind(&WebrtcRosServer::closeClientCallback, this, std::placeholders::_1, std::placeholders::_2)); 
}

void WebrtcRosServer::stop()
{
  if (createClientService_)
  {
    createClientService_.reset();
  }

  if (closeClientService_)
  {
    closeClientService_.reset();
  }

  // Send all clients messages to shutdown, cannot call dispose of share ptr while holding clients_mutex_
  // It will deadlock if it is the last shared_ptr because it will try to remove it from the client list
  std::vector<WebrtcClientWeakPtr> to_invalidate;
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    for(auto& client_entry : clients_) {
      to_invalidate.push_back(client_entry.second);
    }
  }
  for(WebrtcClientWeakPtr& client_weak : to_invalidate) {
    std::shared_ptr<WebrtcClient> client = client_weak.lock();
    if (client)
      client->invalidate();
  }

  // Wait for all our clients to shown
  {
    std::unique_lock<std::mutex> lock(clients_mutex_);
    shutdown_cv_.wait(lock, [this]{ return this->clients_.size() == 0; });
  }

  if (signaling_thread_)
  {
    signaling_thread_->Stop();
    signaling_thread_.reset();
  }

  rtc::CleanupSSL();
}

}
