#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from webrtc_ros_msgs.msg import IceServer
from webrtc_ros_msgs.srv import GetIceServers
import requests

class IceServerManager(Node):
    """ Manages providing ice server information to the webrtc system """

    def __init__(self):
        super().__init__('ice_server_provider')

        self.stun_servers = self.get_parameter_or('stun_servers', [
            'stun:stun1.l.google.com:19302', 'stun:stun2.l.google.com:19302'])
        self.turn_server_uris = self.get_parameter_or('turn_server_uris', [])
        self.turn_creds_uri = self.get_parameter_or('turn_server_creds_uri', '')
        self.turn_creds_username = self.get_parameter_or(
            'turn_server_creds_username', '')
        self.turn_creds_password = self.get_parameter_or(
            'turn_server_creds_password', '')

        self.get_ice_servers_service = self.create_service(
            GetIceServers, 'get_ice_servers', self.get_ice_servers)

        self.get_logger().info('Ice Server Provider Up')

    def get_turn_creds(self):
        """Get the credentials from the turn server."""
        if self.turn_creds_uri:
            self.get_logger().debug('getting turn server credentials')
            resp = requests.post(self.turn_creds_uri, json={
                'username': self.turn_creds_username,
                'password': self.turn_creds_password
            })
            try:
                self.get_logger().info('trying to parse response from server')
                data = resp.json()
                if 'username' in data and 'password' in data:
                    self.get_logger().info('successfully received turn credentials')
                    return data
                self.get_logger().warn(
                    'response did not have username and password fields')
            except AttributeError:
                self.get_logger().error(
                    'server did not respond with JSON, response code: %i',
                    resp.status_code)
        else:
            self.get_logger().debug('No URI provided for turn credentials')
        return False

    def get_ice_servers(self, request, response):
        """Callback for service. Returns the ice servers"""
        turn_creds = self.get_turn_creds()
        if turn_creds:
            for uri in self.turn_server_uris:
                serv = IceServer()
                serv.uri = uri
                serv.username = turn_creds['username']
                serv.password = turn_creds['password']
                response.servers.append(serv)
        for suri in self.stun_servers:
            serv = IceServer()
            serv.uri = suri
            response.servers.append(serv)
        return response

def main(args=None):
    rclpy.init(args=args)
    ice_server_manager = IceServerManager()
    rclpy.spin(ice_server_manager)
    ice_server_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
