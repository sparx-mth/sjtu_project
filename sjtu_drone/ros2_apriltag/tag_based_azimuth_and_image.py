#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32
import tf2_ros
from tf2_ros import TransformException

class TagBasedAzimuth(Node):
    """
    Computes the continuous camera azimuth (yaw) relative to a world frame (0-360 degrees) 
    by leveraging the known absolute azimuth of AprilTags placed on the walls. 
    
    It selects the most reliable tag by finding the one closest to the center of the camera frame.
    """
    def __init__(self):
        super().__init__("tag_based_azimuth")

        # --- Configuration Parameters ---
        self.declare_parameter("tag_family", "36h11")
        # Ensure this is the correct camera frame name for your setup
        self.declare_parameter("camera_frame", "simple_drone/front_cam_link")

        self.tag_family = self.get_parameter("tag_family").get_parameter_value().string_value
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value

        # Clean up camera frame name
        if self.camera_frame.startswith("/"):
            self.camera_frame = self.camera_frame[1:]

        # --- Known Tag Configuration ---
        # Define the absolute world azimuth (in degrees, clockwise from North=0) 
        # for each known tag ID on the walls.
        self.tag_config = {
            13: 0.0,   # Assuming Tag 10 is on the East Wall (90 degrees)
            11: 90.0,  # Assuming Tag 11 is on the West Wall (270 degrees)
            12: 180.0,    # Assuming Tag 13 is on the North Wall (0 degrees)
            12: 180.0,  # Assuming Tag 12 is on the South Wall (180 degrees)
            14: 0.0,    # Placeholder tags
            15: 0.0     # Placeholder tags
        }
        
        self.known_tag_ids = list(self.tag_config.keys())

        # --- TF Setup ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Publisher ---
        self.azimuth_pub = self.create_publisher(Float32, "camera_azimuth", 10)
        
        # --- Timer (5 Hz) ---
        self.timer = self.create_timer(0.2, self.timer_callback) 

        self.get_logger().info(f"TagBasedAzimuth Node Started. Camera Frame: '{self.camera_frame}'")
        self.get_logger().info(f"Tracking Tag IDs: {self.known_tag_ids}")

    def get_camera_yaw(self):
        """
        Scans all known tags, calculates the azimuth from each visible one, 
        and returns the result from the tag closest to the center of the frame.
        """
        last_error = None
        best_tag_yaw_deg = None
        best_tag_id = None
        # Use infinity to find the smallest absolute relative yaw
        min_abs_relative_yaw = float('inf') 

        for tid in self.known_tag_ids:
            # --- Step 1: Find Transform (Camera -> Tag) ---
            candidate_frames = [f"tag{self.tag_family}:{tid}", f"tag_{tid}", f"tag{tid}"]
            transform = None
            
            for tag_frame in candidate_frames:
                try:
                    # Lookup transform from camera frame to tag frame
                    transform = self.tf_buffer.lookup_transform(
                        self.camera_frame, # Target frame
                        tag_frame,         # Source frame
                        rclpy.time.Time()
                    )
                    break 
                except TransformException as e:
                    last_error = e
                    continue
            
            # --- Step 2: If Transform Found, Calculate Yaw and Quality ---
            if transform:
                t = transform.transform.translation
                
                # Calculate relative yaw (angle deviation of tag from camera center)
                # Assumes standard optical frame (Z forward, X right).
                # The sign might need adjustment (t.x vs -t.x) depending on specific TF definition.
                relative_yaw_rad = math.atan2(-t.x, t.z) 
                relative_yaw_deg = math.degrees(relative_yaw_rad)

                # The absolute relative yaw indicates how close the tag is to the center
                abs_relative_yaw = abs(relative_yaw_deg)
                
                # --- Step 3: Select the Best Tag (Closest to Center) ---
                if abs_relative_yaw < min_abs_relative_yaw:
                    min_abs_relative_yaw = abs_relative_yaw
                    
                    wall_azimuth_deg = self.tag_config[tid]
                    
                    # Compute the absolute world azimuth of the camera
                    camera_yaw = wall_azimuth_deg + relative_yaw_deg
                    
                    # Normalize to 0-360 range
                    camera_yaw = camera_yaw % 360.0
                    
                    # Store data for the current best tag
                    best_tag_yaw_deg = camera_yaw
                    best_tag_id = tid
        
        # --- Step 4: Return the Best Result ---
        if best_tag_yaw_deg is not None:
            return best_tag_yaw_deg, best_tag_id
        
        # No tags visible
        return None, last_error

    def timer_callback(self):
        """
        Periodically calls the yaw calculation and publishes the result.
        """
        yaw_deg, info = self.get_camera_yaw()
        
        if yaw_deg is not None:
            # --- Success ---
            msg = Float32()
            msg.data = float(yaw_deg)
            self.azimuth_pub.publish(msg)
            
            # Logging the result and the tag used for calculation
            self.get_logger().info(
                f"Azimuth: {yaw_deg:.1f}° (Based on Tag {info})"
            )
        else:
            # --- Failure (No tags visible) ---
            # Log a warning to indicate the node is running but has no data
            self.get_logger().warn(
                f"No tags visible. Last TF error: {info}", throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = TagBasedAzimuth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()