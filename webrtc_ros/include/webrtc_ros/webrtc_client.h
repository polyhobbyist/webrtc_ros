#ifndef WEBRTC_ROS_WEBRTC_CLIENT_H_
#define WEBRTC_ROS_WEBRTC_CLIENT_H_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <webrtc_ros/ros_video_renderer.h>
#include <webrtc/api/media_stream_interface.h>
#include <webrtc/api/peer_connection_interface.h>
#include <webrtc/api/audio_options.h>
#include <webrtc/pc/peer_connection_factory.h>
#include <webrtc/api/create_peerconnection_factory.h>
#include <webrtc/api/audio_codecs/builtin_audio_encoder_factory.h>
#include <webrtc/api/audio_codecs/builtin_audio_decoder_factory.h>

#include <webrtc/media/engine/internal_decoder_factory.h>
#include <webrtc/media/engine/internal_encoder_factory.h>


#include <webrtc/media/base/adapted_video_track_source.h>

#include <webrtc/media/engine/multiplex_codec_factory.h>
#include <webrtc_ros/configure_message.h>
#include <webrtc_ros/image_transport_factory.h>
#include <webrtc/rtc_base/thread.h>
#include <webrtc_ros_msgs/msg/web_rtc_message.hpp>

#include <std_msgs/msg/string.hpp>



namespace webrtc_ros
{
class WebrtcClient;
typedef std::shared_ptr<WebrtcClient> WebrtcClientPtr;
typedef std::weak_ptr<WebrtcClient> WebrtcClientWeakPtr;

class WebrtcClientObserverProxy : public webrtc::PeerConnectionObserver,
  public webrtc::CreateSessionDescriptionObserver
{
public:
  WebrtcClientObserverProxy(WebrtcClientWeakPtr client_weak);

  void OnSuccess(webrtc::SessionDescriptionInterface*) override;
  void OnFailure(webrtc::RTCError error) override;
  void OnAddStream(rtc::scoped_refptr<webrtc::MediaStreamInterface>) override;
  void OnRemoveStream(rtc::scoped_refptr<webrtc::MediaStreamInterface>) override;
  void OnDataChannel(rtc::scoped_refptr<webrtc::DataChannelInterface>) override;
  void OnRenegotiationNeeded() override;
  void OnIceCandidate(const webrtc::IceCandidateInterface*) override;
  void OnSignalingChange(webrtc::PeerConnectionInterface::SignalingState) override;
  void OnIceConnectionChange(webrtc::PeerConnectionInterface::IceConnectionState) override;
  void OnIceGatheringChange(webrtc::PeerConnectionInterface::IceGatheringState) override;
  void OnIceCandidatesRemoved(const std::vector<cricket::Candidate>& candidates) override;

private:
  WebrtcClientWeakPtr client_weak_;

};

class WebrtcClient
{
public:
  WebrtcClient(rclcpp::Node::SharedPtr nh, const ImageTransportFactory& itf, const std::string& transport, const std::string& client_id);
  ~WebrtcClient();

  bool start(std::shared_ptr<WebrtcClient>& keep_alive_ptr);

  void invalidate();
  bool valid();


private:
  WebrtcClientPtr keep_alive_this_;

  bool initPeerConnection();

  void ping_timer_callback();

  void rtc_message_callback(webrtc_ros_msgs::msg::WebRTCMessage::SharedPtr msg);
  void handle_webrtc_message_on_thread(webrtc_ros_msgs::msg::WebRTCMessage::SharedPtr msg);

  void OnSessionDescriptionSuccess(webrtc::SessionDescriptionInterface*);
  void OnSessionDescriptionFailure(const std::string&);
  void OnAddRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface>);
  void OnRemoveRemoteStream(rtc::scoped_refptr<webrtc::MediaStreamInterface>);
  void OnIceCandidate(const webrtc::IceCandidateInterface*);

  rclcpp::Node::SharedPtr nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ImageTransportFactory itf_;
  std::string transport_;
  std::string client_id_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> rtc_signal_pub_;
  std::shared_ptr<rclcpp::Subscription<webrtc_ros_msgs::msg::WebRTCMessage>> rtc_message_sub_;

  std::unique_ptr<rtc::Thread> worker_thread_;

  rtc::scoped_refptr<webrtc::PeerConnectionFactoryInterface> peer_connection_factory_;
  std::map<std::string, std::vector<std::shared_ptr<RosVideoRenderer>>> video_renderers_;
  rtc::scoped_refptr<WebrtcClientObserverProxy> webrtc_observer_proxy_;
  rtc::scoped_refptr<webrtc::PeerConnectionInterface> peer_connection_;

  std::map<std::string, std::map<std::string, std::string>> expected_streams_;

  rclcpp::TimerBase::SharedPtr ping_timer_;

  std::unique_ptr<rtc::Thread>signaling_thread_;

  friend WebrtcClientObserverProxy;
};

}

#endif
