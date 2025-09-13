import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from pyproj import Proj, transform
import atexit
import os

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')

        # --- Parameter for Save File Path ---
        self.declare_parameter('save_path', '~/waypoint.csv')
        save_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.save_path = os.path.expanduser(save_path)

        # --- File Handling Logic ---
        try:
            self.waypoint_file = open(self.save_path, 'w')
            # New CSV Header for UTM format
            self.waypoint_file.write('easting,northing,altitude\n')
            self.get_logger().info(f'Successfully opened waypoint file at: {self.save_path}')
        except IOError as e:
            self.get_logger().error(f'Could not open waypoint file for writing: {e}')
            self.waypoint_file = None

        atexit.register(self.shutdown)

        # --- Subscriber to the /fix topic ---
        self.subscription = self.create_subscription(
            NavSatFix,
            '/ublox_gps_node/fix',
            self.fix_callback,
            10)

        # --- Publishers ---
        self.path_publisher = self.create_publisher(Path, '/trajectory', 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/current_pose', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # --- Coordinate Conversion ---
        self.gps_proj = Proj(proj='latlong', datum='WGS84')
        self.map_proj = Proj(proj='utm', zone=52, datum='WGS84') # UTM zone for Seoul

        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.get_logger().info('Waypoint Visualizer node started. Waiting for GPS data...')

    def fix_callback(self, msg):
        if msg.status.status == -1: # STATUS_NO_FIX
            self.get_logger().warn('No GPS fix, waiting for valid data...')
            return

        # --- Convert GPS to UTM ---
        utm_x, utm_y = transform(self.gps_proj, self.map_proj, msg.longitude, msg.latitude)

        # --- Save waypoint to file in UTM format ---
        if self.waypoint_file:
            self.waypoint_file.write(f'{utm_x},{utm_y},{msg.altitude}\n')

        # --- Visualization Logic (using relative coordinates) ---
        if not self.origin_set:
            self.origin_x = utm_x
            self.origin_y = utm_y
            self.origin_set = True
            self.get_logger().info(f'Map origin set to UTM: ({self.origin_x}, {self.origin_y})')

        relative_x = utm_x - self.origin_x
        relative_y = utm_y - self.origin_y

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = relative_x
        pose_stamped.pose.position.y = relative_y
        pose_stamped.pose.position.z = msg.altitude 
        pose_stamped.pose.orientation.w = 1.0 # Default orientation (no heading)

        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path_msg)
        self.pose_publisher.publish(pose_stamped)

    def shutdown(self):
        if self.waypoint_file:
            self.get_logger().info('Closing waypoint file.')
            self.waypoint_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
