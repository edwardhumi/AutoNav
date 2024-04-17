import requests
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def open_door(esp_ip, robot_id):

    url = 'http://'+ esp_ip+ '/openDoor'
    payload = {
            "action": "openDoor",
            "parameters": {"robotId" : robot_id}
            }
    headers = {'Content-Type': 'application/json'}

    #while True:
    response = requests.post(url,json=payload, headers=headers)

    if response.status_code == 200:
        response_data = response.json()
        return response_data['data']['message']

    elif (response.status_code == 400):
        response_data = response.json()
        print("Error:",response_data['data']['message'])
        return "0"

#open_door("172.20.10.4", 39)

class ESPServer(Node):
    def __init__(self):
        super().__init__('esp_server')
        self.publisher_ = self.create_publisher(String, 'door', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.server_callback)
        self.subscription = self.create_subscription(
                String,
                'stage',
                self.listener_callback,
                10)
        self.subscription
        self.startCall = False

    def server_callback(self):
        if self.startCall:
            self.get_logger().info("STARTING HTTP CALL")
            door = "0"
            try:
                door = open_door("192.168.65.116", 39)
            except:
                print("Trying to connect")
            msg = String()
            msg.data = door
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing Door: "%s"' % msg.data)

    def listener_callback(self, msg):
        self.get_logger().info('Received Stage: "%s":' % msg.data)
        if (msg.data == "server"):
            self.startCall = True


def main(args=None):
    rclpy.init(args=args)
    esp_server = ESPServer()
    rclpy.spin(esp_server)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
