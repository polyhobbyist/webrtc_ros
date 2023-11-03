#include <rclcpp/rclcpp.hpp>
#include <webrtc_ros/webrtc_client.h>
#include <webrtc_ros/webrtc_ros_message.h>
#include <webrtc_ros/sdp_message.h>
#include <webrtc_ros/ice_candidate_message.h>
//#include "talk/media/devices/devicemanager.h"
#include <webrtc/api/video/video_source_interface.h>
#include <webrtc_ros/ros_video_capturer.h>
#include <webrtc_ros_msgs/srv/get_ice_servers.hpp>

#include <chrono>
using namespace std::chrono_literals;

namespace webrtc_ros
{

WebrtcClientObserverProxy::WebrtcClientObserverProxy(WebrtcClientWeakPtr client_weak)
  : client_weak_(client_weak) {}

void WebrtcClientObserverProxy::OnSuccess(webrtc::SessionDescriptionInterface* description)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionSuccess(description);
}
void WebrtcClientObserverProxy::OnFailure(webrtc::RTCError error)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnSessionDescriptionFailure(error.message());
}
void WebrtcClientObserverProxy::OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnAddRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnRemoveRemoteStream(media_stream);
}
void WebrtcClientObserverProxy::OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface>)
{
}
void WebrtcClientObserverProxy::OnRenegotiationNeeded()
{
}
void WebrtcClientObserverProxy::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
  WebrtcClientPtr client = client_weak_.lock();
  if (client)
    client->OnIceCandidate(candidate);
}
void WebrtcClientObserverProxy::OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState)
{
}
void WebrtcClientObserverProxy::OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState)
{
}
void WebrtcClientObserverProxy::OnIceCandidatesRemoved(const std::vector<cricket::Candidate>&)
{
}
void WebrtcClientObserverProxy::OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState)
{
}


WebrtcClient::WebrtcClient(rclcpp::Node::SharedPtr nh, const ImageTransportFactory& itf, const std::string& transport, const std::string& client_id)
  : nh_(nh), itf_(itf), transport_(transport),
    signaling_thread_(rtc::Thread::Current()), worker_thread_(rtc::Thread::CreateWithSocketServer()),
    client_id_(client_id)
{

}

WebrtcClient::~WebrtcClient()
{
  if(valid()) {
    RCLCPP_FATAL(nh_->get_logger(), "WebrtcClient destructor should only be called once it's invalidated");
  }
  RCLCPP_INFO(nh_->get_logger(),"Destroying Webrtc Client");
}

void WebrtcClient::invalidate()
{
  keep_alive_this_.reset();
}
bool WebrtcClient::valid()
{
  return keep_alive_this_ != nullptr;
}

bool WebrtcClient::start(std::shared_ptr<WebrtcClient>& keep_alive_ptr)
{
  keep_alive_this_ = keep_alive_ptr;
  worker_thread_->Start();

  it_ = std::make_shared<image_transport::ImageTransport>(nh);
  
  peer_connection_factory_  = webrtc::CreatePeerConnectionFactory(
        worker_thread_.get(), worker_thread_.get(), worker_thread_.get(),
        nullptr, webrtc::CreateBuiltinAudioEncoderFactory(),
        webrtc::CreateBuiltinAudioDecoderFactory(),
        std::unique_ptr<webrtc::VideoEncoderFactory>(
            new webrtc::MultiplexEncoderFactory(
                std::make_unique<webrtc::InternalEncoderFactory>())),
        std::unique_ptr<webrtc::VideoDecoderFactory>(
            new webrtc::MultiplexDecoderFactory(
                std::make_unique<webrtc::InternalDecoderFactory>())),
        nullptr, nullptr);

  if (!peer_connection_factory_.get())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Could not create peer connection factory");
    invalidate();
    return false;
  }

  ping_timer_ = nh_->create_wall_timer(10.0s, std::bind(&WebrtcClient::ping_timer_callback, this));

  std::string signalingTopic = "webrtc_client_signaling/" + client_id_;
  rtc_signal_pub_ = nh_->create_publisher<std_msgs::msg::String>(signalingTopic, 1);

  std::string messageTopic = "webrtc_client_message/" + client_id_;
  rtc_message_sub_ = nh_->create_subscription<webrtc_ros_msgs::msg::WebRTCMessage>(messageTopic, 1, std::bind(&WebrtcClient::rtc_message_callback, this, std::placeholders::_1));

  return true;

}


bool WebrtcClient::initPeerConnection()
{
  if(!valid()) {
    RCLCPP_ERROR(nh_->get_logger(),"Tried to initialize invalidated webrtc client");
    return false;
  }
  if (!peer_connection_)
  {
    webrtc::PeerConnectionInterface::RTCConfiguration config;

    auto iceClient = nh_->create_client<webrtc_ros_msgs::srv::GetIceServers>("get_ice_servers");

    if (iceClient->wait_for_service(10s)){
      auto request = std::make_shared<webrtc_ros_msgs::srv::GetIceServers::Request>();
      auto result = iceClient->async_send_request(request);

      // Wait for the result.
      if (rclcpp::spin_until_future_complete(nh_, result) == rclcpp::FutureReturnCode::SUCCESS)
      {      
        for(int i=0; i<result.get()->servers.size(); i++){
          webrtc::PeerConnectionInterface::IceServer server;
          server.uri = result.get()->servers[i].uri;
          if(!result.get()->servers[i].username.empty() && !result.get()->servers[i].password.empty()){
            server.username = result.get()->servers[i].username;
            server.password = result.get()->servers[i].password;
          }
          config.servers.push_back(server);
        }
      }
    }

    WebrtcClientWeakPtr weak_this(keep_alive_this_);
    webrtc_observer_proxy_ = new rtc::RefCountedObject<WebrtcClientObserverProxy>(weak_this);
    peer_connection_ = peer_connection_factory_->CreatePeerConnection(
            config,
            nullptr,
            nullptr,
            webrtc_observer_proxy_.get()
    );
    if (!peer_connection_.get())
    {
      RCLCPP_WARN(nh_->get_logger(), "Could not create peer connection");
      invalidate();
      return false;
    }
    return true;
  }
  else
  {
    return true;
  }
}

void WebrtcClient::ping_timer_callback()
{
    auto s = std_msgs::msg::String();
    s.data = "ping";
    rtc_signal_pub_->publish(s);
}


class DummySetSessionDescriptionObserver
  : public webrtc::SetSessionDescriptionObserver
{
public:
  virtual void OnSuccess()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("webrtc_ros"), "Set Session Succeeded in %s",  __FUNCTION__);
  }
  virtual void OnFailure(webrtc::RTCError error)
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("webrtc_ros"), __FUNCTION__ << " " << error.message());
  }

protected:
  DummySetSessionDescriptionObserver() {}
  ~DummySetSessionDescriptionObserver() {}
};

static bool parseUri(const std::string& uri, std::string* scheme_name, std::string* path) {
  size_t split = uri.find_first_of(':');
  if(split == std::string::npos)
    return false;
  *scheme_name = uri.substr(0, split);
  if(uri.length() > split + 1)
    *path = uri.substr(split + 1, uri.length() - split - 1);
  else
    *path = "";
  return true;
}

// handle webrtc_ros_msgs::msg::WebRTCMessage
void WebrtcClient::rtc_message_callback(webrtc_ros_msgs::msg::WebRTCMessage::SharedPtr msg) 
{
    signaling_thread_->BlockingCall(
      [&] { this->handle_webrtc_message_on_thread(msg); });
}

void WebrtcClient::handle_webrtc_message_on_thread(webrtc_ros_msgs::msg::WebRTCMessage::SharedPtr msg)
{
  if (msg->type == webrtc_ros_msgs::msg::WebRTCMessage::TEXT)
  {
    Json::Reader reader;
    Json::Value message_json;
    RCLCPP_INFO(nh_->get_logger(),"JSON: %s", msg->raw_message .c_str());
    if (!reader.parse(msg->raw_message, message_json))
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not parse message: " << msg->raw_message);
      invalidate();
      return;
    }

    if (ConfigureMessage::isConfigure(message_json))
    {
      ConfigureMessage message;
      if (!message.fromJson(message_json))
      {
        RCLCPP_WARN(nh_->get_logger(), "Can't parse received configure message.");
        return;
      }

      if (!initPeerConnection())
      {
        RCLCPP_WARN(nh_->get_logger(), "Failed to initialize peer connection");
        return;
      }

      // RCLCPP_DEBUG(_node->get_logger(),"Configuring webrtc connection");

      for(const ConfigureAction& action: message.actions)
      {
            // Macro that simply checks if a key is specified and will ignore the action
            // is not specified
          #define FIND_PROPERTY_OR_CONTINUE(key, name)				\
            if(action.properties.find(key) == action.properties.end()) {	\
              RCLCPP_WARN_STREAM(nh_->get_logger(), "No " << #name << " specified");		\
              continue;							\
            }								\
            std::string name = action.properties.at(key)
            // END OF MACRO

            if(action.type == ConfigureAction::kAddStreamActionName) {
              FIND_PROPERTY_OR_CONTINUE("id", stream_id);

                    rtc::scoped_refptr<webrtc::MediaStreamInterface> stream = peer_connection_factory_->CreateLocalMediaStream(stream_id);

                    if (!peer_connection_->AddStream(stream.get()))
                    {
                      RCLCPP_WARN(nh_->get_logger(), "Adding stream to PeerConnection failed");
                continue;
                    }
            }
            else if(action.type == ConfigureAction::kRemoveStreamActionName) {
              FIND_PROPERTY_OR_CONTINUE("id", stream_id);

                    rtc::scoped_refptr<webrtc::MediaStreamInterface> stream = peer_connection_factory_->CreateLocalMediaStream(stream_id);

              if(!stream) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Stream not found with id: " << stream_id);
                continue;
              }
                    peer_connection_->RemoveStream(stream.get());
            }
            else if(action.type == ConfigureAction::kAddVideoTrackActionName) {
              FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
              FIND_PROPERTY_OR_CONTINUE("id", track_id);
              FIND_PROPERTY_OR_CONTINUE("src", src);

              std::string video_type;
              std::string video_path;
              if(!parseUri(src, &video_type, &video_path)) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << src);
                continue;
              }

                    webrtc::MediaStreamInterface* stream = peer_connection_->local_streams()->find(stream_id);
              if(!stream) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Stream not found with id: " << stream_id);
                continue;
              }

              if(video_type == "ros_image") {
                      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Subscribing to ROS topic: " << video_path);
                      rtc::scoped_refptr<RosVideoCapturer> capturer(new rtc::RefCountedObject<RosVideoCapturer>(itf_, video_path, transport_));
                      rtc::scoped_refptr<webrtc::VideoTrackInterface> video_track(
                        peer_connection_factory_->CreateVideoTrack(
                          track_id,
                          capturer.get()));
                      stream->AddTrack(video_track);
                      capturer->Start();
              }
              else {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Unknown video source type: " << video_type);
              }

            }
            else if(action.type == ConfigureAction::kAddAudioTrackActionName) {
              FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
              FIND_PROPERTY_OR_CONTINUE("id", track_id);
              FIND_PROPERTY_OR_CONTINUE("src", src);

              std::string audio_type;
              std::string audio_path;
              if(!parseUri(src, &audio_type, &audio_path)) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << src);
                continue;
              }

                    webrtc::MediaStreamInterface* stream = peer_connection_->local_streams()->find(stream_id);
              if(!stream) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Stream not found with id: " << stream_id);
                continue;
              }

              if(audio_type == "local") {
                cricket::AudioOptions options;
                rtc::scoped_refptr<webrtc::AudioTrackInterface> audio_track(
                  peer_connection_factory_->CreateAudioTrack(
                    track_id,
                      peer_connection_factory_->CreateAudioSource(options).get()));
                      stream->AddTrack(audio_track);
              }
              else {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Unknown video source type: " << audio_type);
              }

            }
            else if(action.type == ConfigureAction::kExpectStreamActionName) {
              FIND_PROPERTY_OR_CONTINUE("id", stream_id);
              if(expected_streams_.find(stream_id) != expected_streams_.end()) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Stream id: " << stream_id << " is already expected");
                continue;
              }
              expected_streams_[stream_id] = std::map<std::string, std::string>();
            }
            else if(action.type == ConfigureAction::kExpectVideoTrackActionName) {
              FIND_PROPERTY_OR_CONTINUE("stream_id", stream_id);
              FIND_PROPERTY_OR_CONTINUE("id", track_id);
              FIND_PROPERTY_OR_CONTINUE("dest", dest);

              if(expected_streams_.find(stream_id) == expected_streams_.end()) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Stream id: " << stream_id << " is not expected");
                continue;
              }
              if(expected_streams_[stream_id].find(track_id) != expected_streams_[stream_id].end()) {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Track id: " << track_id << " is already expected in stream id: " << stream_id);
                continue;
              }
              expected_streams_[stream_id][track_id] = dest;
            }
            else {
              RCLCPP_WARN_STREAM(nh_->get_logger(), "Unknown configure action type: " << action.type);
	}
      }      
      webrtc::PeerConnectionInterface::RTCOfferAnswerOptions options(0,0,false,false,false);
      peer_connection_->CreateOffer(webrtc_observer_proxy_.get(),options);
      // TODO check media constraints
    }
    else if (SdpMessage::isSdpAnswer(message_json))
    {
      SdpMessage message;

      if (!message.fromJson(message_json))
      {
        RCLCPP_WARN(nh_->get_logger(), "Can't parse received session description message.");
        return;
      }

      webrtc::SessionDescriptionInterface* session_description(message.createSessionDescription());
      if (!session_description)
      {
        RCLCPP_WARN(nh_->get_logger(), "Can't create session description");
        return;
      }

      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Received remote description: " << message.sdp);
      rtc::scoped_refptr<DummySetSessionDescriptionObserver> dummy_set_description_observer(new rtc::RefCountedObject<DummySetSessionDescriptionObserver>());
      peer_connection_->SetRemoteDescription(dummy_set_description_observer.get(), session_description);
    }
    else if (IceCandidateMessage::isIceCandidate(message_json))
    {
      IceCandidateMessage message;
      if (!message.fromJson(message_json))
      {
        RCLCPP_WARN(nh_->get_logger(), "Can't parse received ice candidate message.");
        return;
      }

      std::unique_ptr<webrtc::IceCandidateInterface> candidate(message.createIceCandidate());
      if (!candidate.get())
      {
        RCLCPP_WARN(nh_->get_logger(), "Can't parse received candidate message.");
        return;
      }
      if (!peer_connection_->AddIceCandidate(candidate.get()))
      {
        RCLCPP_WARN(nh_->get_logger(), "Failed to apply the received candidate");
        return;
      }
      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Received remote candidate :" << message.toJson());
      return;
    }
    else
    {
      std::string message_type;
      WebrtcRosMessage::getType(message_json, &message_type);
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected message type: " << message_type << ": " << msg->raw_message);
    }
  }
  else if (msg->type == webrtc_ros_msgs::msg::WebRTCMessage::PONG)
  {
    // got a pong from the last ping
  }
  else if (msg->type == webrtc_ros_msgs::msg::WebRTCMessage::CLOSE)
  {
    invalidate();
  }
  else
  {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected signaling message type: " << msg->type << ": " << msg->raw_message);
  }
}

void WebrtcClient::OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface* description)
{
  rtc::scoped_refptr<DummySetSessionDescriptionObserver> dummy_set_description_observer(new rtc::RefCountedObject<DummySetSessionDescriptionObserver>());
  peer_connection_->SetLocalDescription(dummy_set_description_observer.get(), description);

  SdpMessage message;
  if (message.fromSessionDescription(*description))
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Created local description: " << message.sdp);

    auto msg = std_msgs::msg::String();
    msg.data = message.toJson();
    rtc_signal_pub_->publish(msg);
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "Failed to serialize description");
  }
}
void WebrtcClient::OnSessionDescriptionFailure(const std::string& error)
{
  RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not create local description: " << error);
  invalidate();
}
void WebrtcClient::OnIceCandidate(const webrtc::IceCandidateInterface* candidate)
{
  IceCandidateMessage message;
  if (message.fromIceCandidate(*candidate))
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Got local ICE candidate: " << message.toJson());
    auto msg = std_msgs::msg::String();
    msg.data = message.toJson();
    rtc_signal_pub_->publish(msg);
  }
  else
  {
    RCLCPP_WARN(nh_->get_logger(), "Failed to serialize local candidate");
  }
}

void WebrtcClient::OnAddRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  std::string stream_id = media_stream->id();
  if(expected_streams_.find(stream_id) != expected_streams_.end()) {
    for(auto& track : media_stream->GetVideoTracks()) {
      if(expected_streams_[stream_id].find(track->id()) != expected_streams_[stream_id].end()) {
	std::string dest = expected_streams_[stream_id][track->id()];

	std::string video_type;
	std::string video_path;
	if(!parseUri(dest, &video_type, &video_path)) {
	  RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid URI: " << dest);
	  continue;
	}

	if(video_type == "ros_image") {
	  auto renderer = std::make_shared<RosVideoRenderer>(it_, video_path);
	  track->AddOrUpdateSink(renderer.get(), rtc::VideoSinkWants());
	  video_renderers_[stream_id].push_back(renderer);
	}
	else {
	  RCLCPP_WARN_STREAM(nh_->get_logger(), "Unknown video destination type: " << video_type);
	}

      }
      else {
	RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected video track: " << track->id());
      }
    }
    // Currently audio tracks play to system default output without any action taken
    // It does not appear to be simple to change this
  }
  else {
    RCLCPP_WARN_STREAM(nh_->get_logger(), "Unexpected stream: " << stream_id);
  }
}
void WebrtcClient::OnRemoveRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface> media_stream)
{
  std::string stream_id = media_stream->id();
  video_renderers_.erase(stream_id);
}

}
