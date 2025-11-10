import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Point


class Visualization(Node):
    def __init__(self):
        super().__init__('visualization')

        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.drone_names = [
            'x500_mono_cam_0',
            'x500_mono_cam_1',
            'x500_mono_cam_2',
            'x500_mono_cam_3'
        ]

        self.drone_poses = {name: None for name in self.drone_names}
        self.drone_trails = {name: [] for name in self.drone_names}
        self.max_trail_length = 200      # número máximo de puntos en la estela
        self.min_point_distance = 2.0    # distancia mínima entre puntos consecutivos en la estela (m)

        for name in self.drone_names:
            topic = f'/model/{name}/pose'
            self.create_subscription(PoseStamped, topic, self.make_pose_callback(name), 10)

        self.map_timer = self.create_timer(5.0, self.publish_map)
        self.drones_timer = self.create_timer(0.1, self.publish_drones)
        self.map_marker = self.create_map_marker()

    def make_pose_callback(self, drone_name):
        def callback(msg):
            self.drone_poses[drone_name] = msg.pose

            p = Point()
            p.x = msg.pose.position.x
            p.y = msg.pose.position.y
            p.z = msg.pose.position.z

            # Añadir punto solo si está lo suficientemente lejos del último
            if not self.drone_trails[drone_name] or \
               self.distance(self.drone_trails[drone_name][-1], p) >= self.min_point_distance:
                self.drone_trails[drone_name].append(p)

                if len(self.drone_trails[drone_name]) > self.max_trail_length:
                    self.drone_trails[drone_name].pop(0)

        return callback

    @staticmethod
    def distance(p1, p2):
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

    def create_map_marker(self):
        map_marker = Marker()
        map_marker.header.frame_id = "jjaa_swarm"
        map_marker.ns = "map"
        map_marker.id = 0
        map_marker.type = Marker.MESH_RESOURCE
        map_marker.action = Marker.ADD
        map_marker.mesh_resource = "package://visualization/assets/CITY_1.obj"
        map_marker.pose.position.x = 50.0
        map_marker.pose.position.y = -95.0
        map_marker.pose.position.z = 0.02
        map_marker.pose.orientation.x = 0.7071
        map_marker.pose.orientation.y = 0.0
        map_marker.pose.orientation.z = 0.0
        map_marker.pose.orientation.w = 0.7071
        map_marker.scale.x = 23.0
        map_marker.scale.y = 20.0
        map_marker.scale.z = 25.0
        map_marker.mesh_use_embedded_materials = True
        return map_marker

    def publish_map(self):
        self.map_marker.header.stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self.map_marker)
        self.publisher.publish(markers)

    def publish_drones(self):
        markers = MarkerArray()

        for i, name in enumerate(self.drone_names):
            pose = self.drone_poses.get(name)
            if pose is None:
                continue

            drone_marker = Marker()
            drone_marker.header.frame_id = "jjaa_swarm"
            drone_marker.header.stamp = self.get_clock().now().to_msg()
            drone_marker.ns = "drones"
            drone_marker.id = i + 1
            drone_marker.type = Marker.MESH_RESOURCE
            drone_marker.action = Marker.ADD
            drone_marker.mesh_resource = "package://visualization/assets/NXP-HGD-CF.dae"
            drone_marker.pose = pose
            drone_marker.scale.x = 45.0
            drone_marker.scale.y = 45.0
            drone_marker.scale.z = 45.0
            drone_marker.color.r = 1.0
            drone_marker.color.g = 1.0
            drone_marker.color.b = 0.0
            drone_marker.color.a = 1.0
            drone_marker.mesh_use_embedded_materials = False
            markers.markers.append(drone_marker)

            trail_marker = Marker()
            trail_marker.header.frame_id = "jjaa_swarm"
            trail_marker.header.stamp = self.get_clock().now().to_msg()
            trail_marker.ns = "trails"
            trail_marker.id = 100 + i
            trail_marker.type = Marker.LINE_STRIP
            trail_marker.action = Marker.ADD
            trail_marker.scale.x = 5.0
            trail_marker.color.r = 0.0
            trail_marker.color.g = 0.0
            trail_marker.color.b = 1.0
            trail_marker.color.a = 1.0
            trail_marker.points = self.drone_trails[name]
            markers.markers.append(trail_marker)

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Visualization()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
