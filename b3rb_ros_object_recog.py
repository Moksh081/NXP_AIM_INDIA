# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from synapse_msgs.msg import TrafficStatus

import cv2
import numpy as np
import torch

from sensor_msgs.msg import CompressedImage

QOS_PROFILE_DEFAULT = 10


class ObjectRecognizer(Node):
    """ Initializes object recognizer node with the required publishers and subscriptions.

        Returns:
            None
    """
    def __init__(self):
        super().__init__('object_recognizer')

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT)

        # Publisher for traffic status.
        self.publisher_traffic = self.create_publisher(
            TrafficStatus,
            '/traffic_status',
            QOS_PROFILE_DEFAULT)

        # Load YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    """ Analyzes the image received from /camera/image_raw/compressed to detect traffic signs.
        Publishes the existence of traffic signs in the image on the /traffic_status topic.

        Args:
            message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

        Returns:
            None
    """
    def camera_image_callback(self, message):
        # Convert message to an n-dimensional numpy array representation of image.
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Perform inference with YOLOv5 model
        results = self.model(image)

        # Extract bounding boxes and labels
        detections = results.xyxy[0].cpu().numpy()

        # COCO class names
        with open('b3rb_ros_line_follower/b3rb_ros_line_follower/data1/coco.names', 'r') as f:
            classes = [line.strip() for line in f.readlines()]

        # Variable to store stop sign detection status
        stop_sign_detected = False

        # Iterate through detections and check for 'stop sign'
        for detection in detections:
            x1, y1, x2, y2, conf, class_id = detection
            label = classes[int(class_id)]
            if label == 'stop sign':
                stop_sign_detected = True  # Set the variable to True if stop sign is detected
                cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(image, f'{label} {conf:.2f}', (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

       
        cv2.imshow('Detection Result', image)
        cv2.waitKey(1)

        # Publish traffic status based on detection
        traffic_status_message = TrafficStatus()
        traffic_status_message.stop_sign = stop_sign_detected
        self.publisher_traffic.publish(traffic_status_message)


def main(args=None):
    rclpy.init(args=args)
    object_recognizer = ObjectRecognizer()
    rclpy.spin(object_recognizer)
    object_recognizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
