# This script allows you to directly call the WebRTC ROS node from the command line.
# It is a simple script that takes in the arguments and sends them to the WebRTC ROS node.

# This script creates a web server that serves up an html webpage with a basic webRTC video chat interface. When a user attempts to connect
# to the server, the server will send a message to the WebRTC ROS node to establish a connection with the user. The WebRTC ROS node will then
# send a message back to the server with the necessary information to establish a connection with the user. The server will then send this
# information to the user, allowing the user to establish a connection with the WebRTC ROS node.

import rclpy
import rclpy.wait_for_message
from rclpy.node import Node
from std_msgs.msg import String
from webrtc_ros_msgs.msg import WebRTCMessage
from webrtc_ros_msgs.srv import CreateClient, CloseClient
import sys
import json
from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
import threading

# Create a fastapi endpoint to serve up an html webpage, and an API to receive webrtc offers from the webpage and relay answers
rclpy.init()
app = FastAPI()

class WebRTCServer(Node):
    def __init__(self):
        super().__init__('webrtc_server')
        self.websocket = None
        self.sig_pub = None
        self.sig_sub = None

        self.get_logger().info('WebRTC Server starting up on')
        app.add_websocket_route("/ws", self.websocket_endpoint)

        self.close_client_service = self.create_client(CloseClient, 'close_webrtc_client')
        while not self.close_client_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('close service not available, waiting again...')

        closeClientRequest = CloseClient.Request()
        closeClientRequest.instance_id = "myId"

        future = self.close_client_service.call_async(closeClientRequest)
        rclpy.spin_until_future_complete(self, future)

        self.create_client_service = self.create_client(CreateClient, 'create_webrtc_client')
        while not self.create_client_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('create service not available, waiting again...')

        createClientRequest = CreateClient.Request()
        createClientRequest.id = "myId"
        createClientRequest.image_transport = "camera/raw"

        future = self.create_client_service.call_async(createClientRequest)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is not None:
            self.get_logger().info('Service call successful')
        else:
            self.get_logger().info('Service call failed')
            exit(1)
    
        self.get_logger().info("WebRTC Server up and running with id: " + response.instance_id)
    
        self.sig_sub = self.create_subscription(String, 
            "/webrtc_client_signaling/myId",
            self.websocket_response,
            1)

        self.sig_pub = self.create_publisher(WebRTCMessage, 
            "/webrtc_client_message/myId",
            1)


    async def websocket_endpoint(self, websocket: WebSocket):
        self.get_logger().info("Websocket connection established")
        self.websocket = websocket
        await websocket.accept()
        while True:
            data = await websocket.receive_text()
            if (data == "pong"):
                msg = WebRTCMessage()
                msg.id = "myId"
                msg.type = 1
                self.sig_pub.publish(msg)
            else:
                self.get_logger().info(f"Message text was: {data}")
                msg = WebRTCMessage()
                msg.id = "myId"
                msg.type = 0
                msg.raw_message = data
                self.sig_pub.publish(msg)
            

    async def websocket_response(self, msg):
        self.get_logger().info("received message: " + msg.data)
        if self.websocket is not None:
            await self.websocket.send_text(msg.data)
        else:
            self.get_logger().info("Websocket not connected")

# Implement an index endpoint
@app.get("/")
def index():
    # get the directory of this file
    dir = sys.path[0]

    return FileResponse(dir + '/index.html')

def run_server():
    webrtc_server = WebRTCServer()
    rclpy.spin(webrtc_server)

thread = threading.Thread(target=run_server)
thread.start()