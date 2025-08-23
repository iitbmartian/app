#!/usr/bin/env python3
"""
ROS2 WebSocket Bridge for 3D Visualization
Subscribes to rtabmap, costmap, markers, and nav2 topics
Streams data efficiently to web frontend
"""

import asyncio
import json
import gzip
import time
from typing import Dict, List, Any, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

import fastapi
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
import uvicorn

# ROS2 message types
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration


class ROS2DataCollector(Node):
    """ROS2 node that collects data from various topics"""
    
    def __init__(self):
        super().__init__('ros2_viz_bridge')
        
        # Data storage
        self.latest_data = {
            'map': None,
            'local_costmap': None,
            'global_path': None,
            'local_path': None,
            'markers': None,
            'point_cloud': None,
            'robot_pose': None,
            'timestamp': time.time()
        }
        
        # Transform handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        map2_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/rtabmap/map', self.map_callback, map_qos)
        
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, map_qos)
        
        self.global_path_sub = self.create_subscription(
            Path, '/plan', self.global_path_callback, sensor_qos)
        
        self.local_path_sub = self.create_subscription(
            Path, '/local_plan', self.local_path_callback, sensor_qos)
        
        self.marker_sub = self.create_subscription(
            MarkerArray, '/visualization_marker_array', self.marker_callback, sensor_qos)
        
        # Alternative marker topic
        self.single_marker_sub = self.create_subscription(
            Marker, '/visualization_marker', self.single_marker_callback, sensor_qos)
        
        # Point cloud from rtabmap
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map', self.point_cloud_callback, map2_qos)
        
        # Robot pose (odometry)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/rtabmap/global_pose', self.pose_callback, map_qos)
        
        self.get_logger().info('ROS2 Data Collector initialized')
    
    def map_callback(self, msg: OccupancyGrid):
        """Process rtabmap occupancy grid"""
        try:
            map_data = {
                'type': 'map',
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'data': list(msg.data),
                'frame_id': msg.header.frame_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            self.latest_data['map'] = map_data
            self.latest_data['timestamp'] = time.time()
        except Exception as e:
            self.get_logger().error(f'Error processing map: {e}')
    
    def local_costmap_callback(self, msg: OccupancyGrid):
        """Process local costmap"""
        try:
            costmap_data = {
                'type': 'local_costmap',
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'z': msg.info.origin.position.z
                },
                'data': list(msg.data),
                'frame_id': msg.header.frame_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            self.latest_data['local_costmap'] = costmap_data
        except Exception as e:
            self.get_logger().error(f'Error processing local costmap: {e}')
    
    def global_path_callback(self, msg: Path):
        """Process global navigation path"""
        try:
            path_data = {
                'type': 'global_path',
                'poses': [],
                'frame_id': msg.header.frame_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            
            for pose_stamped in msg.poses:
                pose_data = {
                    'position': {
                        'x': pose_stamped.pose.position.x,
                        'y': pose_stamped.pose.position.y,
                        'z': pose_stamped.pose.position.z
                    },
                    'orientation': {
                        'x': pose_stamped.pose.orientation.x,
                        'y': pose_stamped.pose.orientation.y,
                        'z': pose_stamped.pose.orientation.z,
                        'w': pose_stamped.pose.orientation.w
                    }
                }
                path_data['poses'].append(pose_data)
            
            self.latest_data['global_path'] = path_data
        except Exception as e:
            self.get_logger().error(f'Error processing global path: {e}')
    
    def local_path_callback(self, msg: Path):
        """Process local navigation path"""
        try:
            path_data = {
                'type': 'local_path',
                'poses': [],
                'frame_id': msg.header.frame_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            
            for pose_stamped in msg.poses:
                pose_data = {
                    'position': {
                        'x': pose_stamped.pose.position.x,
                        'y': pose_stamped.pose.position.y,
                        'z': pose_stamped.pose.position.z
                    },
                    'orientation': {
                        'x': pose_stamped.pose.orientation.x,
                        'y': pose_stamped.pose.orientation.y,
                        'z': pose_stamped.pose.orientation.z,
                        'w': pose_stamped.pose.orientation.w
                    }
                }
                path_data['poses'].append(pose_data)
            
            self.latest_data['local_path'] = path_data
        except Exception as e:
            self.get_logger().error(f'Error processing local path: {e}')
    
    def marker_callback(self, msg: MarkerArray):
        """Process visualization markers"""
        try:
            markers_data = {
                'type': 'markers',
                'markers': [],
                'timestamp': time.time()
            }
            
            for marker in msg.markers:
                marker_data = self._process_marker(marker)
                markers_data['markers'].append(marker_data)
            
            self.latest_data['markers'] = markers_data
        except Exception as e:
            self.get_logger().error(f'Error processing markers: {e}')
    
    def single_marker_callback(self, msg: Marker):
        """Process single visualization marker"""
        try:
            marker_data = self._process_marker(msg)
            # Store as single marker array
            self.latest_data['markers'] = {
                'type': 'markers',
                'markers': [marker_data],
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().error(f'Error processing single marker: {e}')
    
    def _process_marker(self, marker: Marker) -> Dict:
        """Convert ROS marker to dict format"""
        return {
            'id': marker.id,
            'ns': marker.ns,
            'type': marker.type,
            'action': marker.action,
            'frame_id': marker.header.frame_id,
            'position': {
                'x': marker.pose.position.x,
                'y': marker.pose.position.y,
                'z': marker.pose.position.z
            },
            'orientation': {
                'x': marker.pose.orientation.x,
                'y': marker.pose.orientation.y,
                'z': marker.pose.orientation.z,
                'w': marker.pose.orientation.w
            },
            'scale': {
                'x': marker.scale.x,
                'y': marker.scale.y,
                'z': marker.scale.z
            },
            'color': {
                'r': marker.color.r,
                'g': marker.color.g,
                'b': marker.color.b,
                'a': marker.color.a
            },
            'text': marker.text if hasattr(marker, 'text') else '',
            'points': [{'x': p.x, 'y': p.y, 'z': p.z} for p in marker.points],
            'colors': [{'r': c.r, 'g': c.g, 'b': c.b, 'a': c.a} for c in marker.colors]
        }
    
    def point_cloud_callback(self, msg: PointCloud2):
        """Process point cloud data with proper coordinate transformation"""
        try:
            points = []
            colors = []
            point_step = 10
            count = 0

            # Get the original frame ID from the message
            source_frame = msg.header.frame_id
            target_frame = "map"
            
            # Try to get transform from source frame to map frame
            transform = None
            try:
                # Look up the transform
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    msg.header.stamp,
                    Duration(seconds=1.0)
                )
            except Exception as tf_error:
                self.get_logger().warn(f'Could not transform from {source_frame} to {target_frame}: {tf_error}')
                # If transform fails, we'll use the points as-is but with correct frame_id
                target_frame = source_frame

            for point in pc2.read_points(msg, skip_nans=True):
                if count % point_step == 0:
                    if transform is not None:
                        # Transform the point to map frame
                        point_stamped = PointStamped()
                        point_stamped.header.frame_id = source_frame
                        point_stamped.header.stamp = msg.header.stamp
                        point_stamped.point.x = float(point[0])
                        point_stamped.point.y = float(point[1])
                        point_stamped.point.z = float(point[2])
                        
                        try:
                            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                            points.extend([
                                float(transformed_point.point.x), 
                                float(transformed_point.point.y), 
                                float(transformed_point.point.z)
                            ])
                        except Exception as transform_error:
                            self.get_logger().warn(f'Error transforming point: {transform_error}')
                            # Fall back to original coordinates
                            points.extend([float(point[0]), float(point[1]), float(point[2])])
                    else:
                        # No transform available, use original coordinates
                        points.extend([float(point[0]), float(point[1]), float(point[2])])
                    
                    # Handle colors if available in the point cloud
                    if len(point) >= 4:  # Has color information
                        # Assuming RGB packed as float
                        rgb = int(point[3]) if not np.isnan(point[3]) else 0xFFFFFF
                        r = ((rgb >> 16) & 0xFF) / 255.0
                        g = ((rgb >> 8) & 0xFF) / 255.0
                        b = (rgb & 0xFF) / 255.0
                        colors.extend([r, g, b])
                    else:
                        colors.extend([1.0, 1.0, 1.0])  # default white
                
                count += 1
                if len(points) > 30000:  # cap to ~10k points
                    break

            cloud_data = {
                'type': 'point_cloud',
                'points': points,
                'colors': colors,
                'frame_id': target_frame,  # Use the actual target frame
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }

            self.latest_data['point_cloud'] = cloud_data
            self.latest_data['timestamp'] = time.time()

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {e}')

    
    def pose_callback(self, msg: PoseStamped):
        """Process robot pose"""
        try:
            pose_data = {
                'type': 'robot_pose',
                'position': {
                    'x': msg.pose.position.x,
                    'y': msg.pose.position.y,
                    'z': msg.pose.position.z
                },
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                },
                'frame_id': msg.header.frame_id,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
            self.latest_data['robot_pose'] = pose_data
        except Exception as e:
            self.get_logger().error(f'Error processing robot pose: {e}')


class WebSocketManager:
    """Manages WebSocket connections and data streaming"""
    
    def __init__(self, ros_node: ROS2DataCollector):
        self.ros_node = ros_node
        self.active_connections: List[WebSocket] = []
        self.last_sent_timestamp = 0
        self.update_rate = 1.0 / 30.0  # 30 Hz max
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"WebSocket connected. Total connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        print(f"WebSocket disconnected. Total connections: {len(self.active_connections)}")
    
    async def broadcast_data(self):
        """Broadcast ROS data to all connected clients"""
        if not self.active_connections:
            return
        
        current_time = time.time()
        if current_time - self.last_sent_timestamp < self.update_rate:
            return
        
        # Check if we have new data
        if self.ros_node.latest_data['timestamp'] <= self.last_sent_timestamp:
            return
        
        print(f"Broadcast check: {len(self.active_connections)} clients, "
          f"latest_data timestamp={self.ros_node.latest_data['timestamp']}, "
          f"last_sent_timestamp={self.last_sent_timestamp}")
        
        try:
            # Prepare data for transmission
            data_to_send = {}
            
            # Include all available data
            for key, value in self.ros_node.latest_data.items():
                if value is not None and key != 'timestamp':
                    data_to_send[key] = value

            if data_to_send:
                # Compress large data
                json_data = json.dumps(data_to_send, default=lambda o: float(o) if isinstance(o, np.generic) else o.tolist() if isinstance(o, np.ndarray) else str(o))
                # Send to all connected clients
                disconnected_clients = []
                for websocket in self.active_connections:
                    try:
                        await websocket.send_text(json_data)
                    except WebSocketDisconnect:
                        disconnected_clients.append(websocket)
                    except Exception as e:
                        print(f"Error sending to client: {e}")
                        disconnected_clients.append(websocket)
                
                # Remove disconnected clients
                for client in disconnected_clients:
                    self.disconnect(client)
                
                self.last_sent_timestamp = current_time
        
        except Exception as e:
            print(f"Error in broadcast_data: {e}")


# FastAPI app
app = FastAPI(title="ROS2 3D Visualization Bridge")

# Global variables
ros_node = None
websocket_manager = None
executor = None


@app.on_event("startup")
async def startup_event():
    """Initialize ROS2 node and executor"""
    global ros_node, websocket_manager, executor
    
    # Initialize ROS2
    rclpy.init()
    ros_node = ROS2DataCollector()
    websocket_manager = WebSocketManager(ros_node)
    
    # Start ROS2 executor in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    # Start data broadcasting task
    asyncio.create_task(data_broadcast_loop())
    
    print("ROS2 WebSocket Bridge started successfully!")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup ROS2 resources"""
    global ros_node, executor
    
    if executor:
        executor.shutdown()
    if ros_node:
        ros_node.destroy_node()
    
    rclpy.shutdown()
    print("ROS2 WebSocket Bridge shut down.")


async def data_broadcast_loop():
    """Main loop for broadcasting data to WebSocket clients"""
    while True:
        try:
            if websocket_manager:
                await websocket_manager.broadcast_data()
            await asyncio.sleep(0.01)  # 100Hz check rate
        except Exception as e:
            print(f"Error in data broadcast loop: {e}")
            await asyncio.sleep(1.0)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time data streaming"""
    await websocket_manager.connect(websocket)
    try:
        while True:
            # Keep connection alive and handle client messages
            message = await websocket.receive_text()
            # Handle client commands if needed
            if message == "ping":
                await websocket.send_text("pong")
    except WebSocketDisconnect:
        websocket_manager.disconnect(websocket)


@app.get("/")
async def get_root():
    return {"message": "ROS2 3D Visualization Bridge is running"}


@app.get("/status")
async def get_status():
    """Get current status and data info"""
    if ros_node:
        status = {
            "active_connections": len(websocket_manager.active_connections),
            "latest_data_keys": list(ros_node.latest_data.keys()),
            "last_update": ros_node.latest_data.get('timestamp', 0)
        }
        return status
    return {"error": "ROS node not initialized"}


# Mount static files for serving the frontend
app.mount("/static", StaticFiles(directory="static"), name="static")


if __name__ == "__main__":
    uvicorn.run(
        "map_web:app",
        host="0.0.0.0",
        port=8000,
        log_level="info",
        reload=False
    )
