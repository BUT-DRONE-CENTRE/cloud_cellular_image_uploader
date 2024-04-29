#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from firebase_admin import credentials, initialize_app, storage
import tempfile
import time
import os
 
class FirebaseUploader(Node):
    def __init__(self):
        super().__init__('firebase_uploader')
 
        # Initialize Firebase app
        cred = credentials.Certificate("credential_json_file_location") #insert location of your credentials json file (generate in Project settings -> service accounts -> generate new private key)
        initialize_app(cred, {'storageBucket': 'storage_bucket_url'}) #insert url of your storage bucket (copy folder path from storage)
 
        self.bucket = storage.bucket()
 
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            1 # QoS profile depth
        )
 
        # Create unique folder for each run
        self.folder_name = "images_" + str(int(time.time()))
        self.folder_path = os.path.join(tempfile.gettempdir(), self.folder_name)
        os.makedirs(self.folder_path, exist_ok=True)
 
    def image_callback(self, msg):
        self.get_logger().info('Received image')
 
        # Use the received compressed image data
        image_data = msg.data
 
        # Generate unique file name using current timestamp
        file_name = "image_" + str(int(time.time())) + "-" + str(int(time.monotonic_ns())) + ".jpg"
 
        # Create temporary file to store image data
        file_path = os.path.join(self.folder_path, file_name)
        with open(file_path, 'wb') as temp_file:
            temp_file.write(image_data)
 
        # Get file size
        file_size = os.path.getsize(file_path)/1000
 
        # Get current time
        time_created = time.strftime('%Y-%m-%d-%H:%M:%S', time.localtime())
 
        # Define metadata
        metadata = {
            'contentType': 'image/jpeg',
            'size': str(file_size) + 'Kb',
            'timeCreated': time_created
        }
 
        # Create file path in storage
        storage_path = os.path.join(self.folder_name, file_name)
 
        # Upload image to Firebase Storage with metadata
        blob = self.bucket.blob(storage_path)
        blob.metadata = metadata
        blob.upload_from_filename(file_path)
 
        # Get download URL
        download_url = blob.generate_signed_url(expiration=3600)  # URL expires in 1 hour
 
        self.get_logger().info('File uploaded successfully. Download URL: %s' % download_url)
 
 
def main(args=None):
    rclpy.init(args=args)
    firebase_uploader = FirebaseUploader()
    rclpy.spin(firebase_uploader)
    firebase_uploader.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()