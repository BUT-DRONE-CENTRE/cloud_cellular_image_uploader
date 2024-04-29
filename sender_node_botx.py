#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import requests
import json

class BotXUploader(Node):
    def __init__(self):
        super().__init__('botx_uploader')

        # Request BotX JWT token
        self.jwt_token = self.request_botx_token()
        if not self.jwt_token:
            self.get_logger().error('Failed to obtain BotX JWT token. Exiting...')
            return

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            1 # QoS profile depth
        )
        self.upload_url = "https://dcs.botx.cloud/static/02161038-ccdb-4a44-baf3-ee0eb56ab9ea/99995517-fcee-4a0c-8878-6ee451bac3cf/Images/Upload"

    def request_botx_token(self):
        url = "https://api.botx.cloud/token"
        email = "240415@vutbr.cz"  # Replace with your BotX email
        password = "qwe123456"  # Replace with your BotX password

        payload = json.dumps({
            "Email": email,
            "Password": password
        })
        headers = {
            'Content-Type': 'application/json'
        }

        response = requests.post(url, headers=headers, data=payload)

        if response.status_code == 200:
            token = response.json().get('token')
            self.get_logger().info('BotX JWT token obtained successfully')
            return token
        else:
            self.get_logger().error('Failed to obtain BotX JWT token. Status code: %d', response.status_code)
            return None

    def image_callback(self, msg):
        self.get_logger().info('Received image')

        # Use the received compressed image data
        image_data = msg.data

        # Prepare the headers with the JWT token
        headers = {'Authorization': 'Bearer ' + self.jwt_token}

        # Create FormData object containing the file
        files = {'file': ('image.png', image_data, 'image/png')}

        # Make a POST request to upload the image
        response = requests.post(self.upload_url, headers=headers, files=files)

        if response.status_code == 200:
            self.get_logger().info('Image uploaded successfully')
        else:
            self.get_logger().error('Failed to upload image. Status code: %d' % response.status_code)


def main(args=None):
    rclpy.init(args=args)
    botx_uploader = BotXUploader()
    rclpy.spin(botx_uploader)
    botx_uploader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
