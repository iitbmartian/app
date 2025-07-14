import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal
import importlib

class MessageSignals(QObject):
    """Qt signals for ROS message communication"""
    image_received = pyqtSignal(np.ndarray, int)  # cv_image, camera_id
    pointcloud_received = pyqtSignal(object, str)  # msg, topic_name
    generic_received = pyqtSignal(str, str, str)  # msg_str, topic_name, msg_type


class ImageSubscriber(Node):
    """Regular image subscriber for sensor_msgs/msg/Image"""
    
    def __init__(self, topic_name, callback, camera_id=0, node_name=None):
        if node_name is None:
            # Create a unique node name based on topic and camera ID
            safe_topic = topic_name.replace('/', '_').replace('-', '_').lstrip('_')
            node_name = f'image_subscriber_{safe_topic}_{camera_id}'
        
        super().__init__(node_name)
        
        self.callback = callback
        self.camera_id = camera_id
        self.bridge = CvBridge()
        self.topic_name = topic_name
        
        self.get_logger().info(f"Creating regular image subscriber for topic: {topic_name}")
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        
        self.subscription  # prevent unused variable warning
    
    def image_callback(self, msg):
        """Callback for regular Image messages"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Call the provided callback
            self.callback(cv_image, self.camera_id)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image from {self.topic_name}: {str(e)}")


class CompressedImageSubscriber(Node):
    """Dedicated subscriber for sensor_msgs/msg/CompressedImage"""
    
    def __init__(self, topic_name, callback, camera_id=0, node_name=None):
        if node_name is None:
            # Create a unique node name based on topic and camera ID
            safe_topic = topic_name.replace('/', '_').replace('-', '_').lstrip('_')
            node_name = f'compressed_image_subscriber_{safe_topic}_{camera_id}'
        
        super().__init__(node_name)
        
        self.callback = callback
        self.camera_id = camera_id
        self.topic_name = topic_name
        
        self.get_logger().info(f"Creating compressed image subscriber for topic: {topic_name}")
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.compressed_image_callback,
            10
        )
        
        self.subscription  # prevent unused variable warning
    
    def compressed_image_callback(self, msg):
        """Callback for CompressedImage messages"""
        try:
            # Convert compressed image data to numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)
            
            # Decode the compressed image
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().error(f"Failed to decode compressed image from {self.topic_name}")
                return
            
            # Call the provided callback
            self.callback(cv_image, self.camera_id)
            
        except Exception as e:
            self.get_logger().error(f"Error processing compressed image from {self.topic_name}: {str(e)}")


class PointCloudSubscriber(Node):
    """Subscriber for PointCloud2 messages"""
    
    def __init__(self, topic_name, callback, subscriber_id="pointcloud"):
        # Create a unique node name
        safe_topic = topic_name.replace('/', '_').replace('-', '_').lstrip('_')
        node_name = f'pointcloud_subscriber_{safe_topic}_{subscriber_id}'
        
        super().__init__(node_name)
        
        self.callback = callback
        self.topic_name = topic_name
        
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info(f"Created PointCloud subscriber for topic: {topic_name}")
    
    def pointcloud_callback(self, msg):
        """Callback for PointCloud2 messages"""
        try:
            # Call the provided callback with the message and topic name
            self.callback(msg, self.topic_name)
        except Exception as e:
            self.get_logger().error(f"Error processing pointcloud from {self.topic_name}: {str(e)}")


class GenericSubscriber(Node):
    """Generic subscriber for various ROS message types"""
    
    def __init__(self, topic_name, msg_type, callback, subscriber_id=0):
        # Create a unique node name
        safe_topic = topic_name.replace('/', '_').replace('-', '_').lstrip('_')
        safe_type = msg_type.replace('/', '_').replace('-', '_')
        node_name = f'generic_subscriber_{safe_topic}_{safe_type}_{subscriber_id}'
        
        super().__init__(node_name)
        
        self.callback = callback
        self.topic_name = topic_name
        self.msg_type = msg_type
        
        # Import the message type dynamically
        try:
            msg_class = self._get_message_class(msg_type)
            
            self.subscription = self.create_subscription(
                msg_class,
                topic_name,
                self.generic_callback,
                10
            )
            
            self.get_logger().info(f"Created generic subscriber for topic: {topic_name} ({msg_type})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to create subscriber for {topic_name}: {str(e)}")
            raise
    
    def _get_message_class(self, msg_type):
        """Dynamically import message class based on type string"""
        # Common message type mappings
        type_mapping = {
            'std_msgs/msg/String': (String, 'std_msgs.msg'),
            'geometry_msgs/msg/Twist': (Twist, 'geometry_msgs.msg'),
            'nav_msgs/msg/Odometry': (Odometry, 'nav_msgs.msg'),
            'sensor_msgs/msg/LaserScan': (LaserScan, 'sensor_msgs.msg'),
            'sensor_msgs/msg/Image': (Image, 'sensor_msgs.msg'),
            'sensor_msgs/msg/CompressedImage': (CompressedImage, 'sensor_msgs.msg'),
            'sensor_msgs/msg/PointCloud2': (PointCloud2, 'sensor_msgs.msg'),
        }
        
        if msg_type in type_mapping:
            return type_mapping[msg_type][0]
        
        # Try to import dynamically
        try:
            # Parse the message type (e.g., 'geometry_msgs/msg/Twist')
            parts = msg_type.split('/')
            if len(parts) != 3:
                raise ValueError(f"Invalid message type format: {msg_type}")
            
            package_name, msg_folder, class_name = parts
            module_name = f"{package_name}.{msg_folder}"
            
            # Import the module
            module = importlib.import_module(module_name)
            
            # Get the message class
            msg_class = getattr(module, class_name)
            
            return msg_class
            
        except Exception as e:
            self.get_logger().error(f"Failed to import message type {msg_type}: {str(e)}")
            raise
    
    def generic_callback(self, msg):
        """Generic callback for any message type"""
        try:
            # Call the provided callback with message, topic name, and type
            self.callback(msg, self.topic_name, self.msg_type)
        except Exception as e:
            self.get_logger().error(f"Error processing message from {self.topic_name}: {str(e)}")


def get_topic_type(node, topic_name):
    """Utility function to get the message type of a topic"""
    try:
        topic_names_and_types = node.get_topic_names_and_types()
        
        for name, types in topic_names_and_types:
            if name == topic_name:
                return types[0] if types else None
        
        return None
    except Exception as e:
        print(f"Error getting topic type for {topic_name}: {str(e)}")
        return None


def is_image_topic(topic_type):
    """Check if a topic type is an image topic (regular or compressed)"""
    return topic_type in ['sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage']


def is_compressed_image_topic(topic_type):
    """Check if a topic type is a compressed image topic"""
    return topic_type == 'sensor_msgs/msg/CompressedImage'


def determine_image_topic_type(node, topic_name):
    """Determine if an image topic is compressed or regular"""
    try:
        topic_names_and_types = node.get_topic_names_and_types()
        
        for name, types in topic_names_and_types:
            if name == topic_name:
                if 'sensor_msgs/msg/CompressedImage' in types:
                    return 'compressed'
                elif 'sensor_msgs/msg/Image' in types:
                    return 'regular'
        
        # Fallback: check if topic name suggests compression
        if 'compressed' in topic_name.lower():
            return 'compressed'
        else:
            return 'regular'
            
    except Exception as e:
        print(f"Could not determine topic type for {topic_name}: {e}")
        # Fallback: assume regular if unable to determine
        return 'regular'