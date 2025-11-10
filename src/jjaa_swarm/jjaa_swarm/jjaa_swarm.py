import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from offboard_control.control.multi_offboard_controller_assembler import MultiOffboardControllerAssembler
from offboard_control.service.sim_uavs_configuration_service import SimUAVSConfigurationService
import random
from jjaa_swarm.linear_programming_script import solve
from offboard_control.domain.constant.states import States
import time
import cv2

from jjaa_swarm.utils import (
    gps_offset_in_enu,
    load_points_from_csv,
    load_arucos_from_csv,
    barrido_zigzag_poses,
    dividir_rectangulo_en_4,
    puntos_medios_rectangulo,
    get_formation,
    enu_ned,
    enu_ned_list
)

RANDOM=0
HUNGARIAN=1

"""
Example of usage:

ros2 run jjaa_swarm jjaa_swarm --ros-args -p targets:=5 -p plan_type:=hungarian
"""

class JJAASwarm(Node):
  
    def __init__(self):
        super().__init__('jjaa_swarm')

        self.declare_parameter('mode', 'execution')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        self.declare_parameter('perception', 'global')
        self.perception = self.get_parameter('perception').get_parameter_value().string_value

        self.declare_parameter('position', 'corners')
        self.position = self.get_parameter('position').get_parameter_value().string_value

        self.declare_parameter('plan_type', 'hungarian')
        plan_type_str = self.get_parameter('plan_type').get_parameter_value().string_value

        self.plan_type = HUNGARIAN if plan_type_str.lower() == 'hungarian' else RANDOM

        self.declare_parameter('targets', 4)
        self.n_targets = self.get_parameter('targets').get_parameter_value().integer_value
        
        self.declare_parameter("vehicle_ids", [1,2])
        self.vehicle_ids = self.get_parameter('vehicle_ids').get_parameter_value().integer_array_value

        self.declare_parameter("world", "jjaa_swarm")
        self.world = self.get_parameter('world').get_parameter_value().string_value

        self.declare_parameter("uav_model", "x500_mono_cam")
        self.uav_model = self.get_parameter('uav_model').get_parameter_value().string_value

        self.declare_parameter('n_points', 400)
        self.n_points = self.get_parameter('n_points').get_parameter_value().integer_value

        self.map_corners = []
        self.map_midpoints = []
        self.rgb_subs = []
        self.detected_ids = set()
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters()
        self.multi_offboard_controller = None
        self.gps_origin = None
        self.aruco_poses = {}

        self.initial_pose = Pose()
        self.initial_pose.position.x = 90.0
        self.initial_pose.position.y = -81.0
        self.initial_pose.position.z = 140.0
        
        self.configure()

            
    def run(self):
        self.start_formation()
        self.run_global_perception()
        self.execute_mission(self.plan_type)


    def run_local_perception(self):

        self.get_logger().info(f"Starting local Perception")        
        start_time = time.perf_counter()

        waypoints = barrido_zigzag_poses(self.map_corners, paso=100)
        
        waypoint_following_future = self.multi_offboard_controller.local_waypoint_following_all(
            self.gps_origin,
            enu_ned_list(waypoints)
        )

        while not waypoint_following_future.done() or not self.check_perception() :
            rclpy.spin_once(self)

        end_time = time.perf_counter()
        elapsed = end_time - start_time
        self.get_logger().info(f"Fase de percepción finalizada.")
        self.get_logger().info(f"Tiempo transcurrido: {elapsed}")


    def run_global_perception(self):

        waypoints = []
        [waypoints.append(enu_ned_list(barrido_zigzag_poses(sector, paso=20))) for sector in dividir_rectangulo_en_4(self.map_corners)]

        self.get_logger().info(f"Starting global Perception")
        
        start_time = time.perf_counter()

        self.multi_offboard_controller.offboard_mode_all()
        self.multi_offboard_controller.check_offboard()
        self.get_logger().info(f"Vehicles waiting for offboard mode.")

        go_to_all_future = self.multi_offboard_controller.local_waypoint_following_all(
            self.gps_origin,
            waypoints
        )

        while not go_to_all_future.done() or not self.check_perception():
            rclpy.spin_once(self)

        end_time = time.perf_counter()
        elapsed = end_time - start_time
        self.get_logger().info(f"Fase de percepción finalizada.")
        self.get_logger().info(f"Tiempo transcurrido: {elapsed}")


    def image_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, ids, _ = cv2.aruco.detectMarkers(cv_image, self.dictionary, parameters=self.parameters)

        if ids is not None:
            for aruco_id in ids.flatten():
                if aruco_id not in self.detected_ids and aruco_id < len(self.aruco_poses):
                    self.detected_ids.add(aruco_id)
                    self.get_logger().info(f"Nuevo ArUco detectado: {aruco_id}")


    def start_formation(self):

        initial_formation = get_formation(
            self.initial_pose,
            8.0,
            len(self.rgb_subs)
        )

        initial_formation = enu_ned_list(initial_formation)

        arm_future = self.multi_offboard_controller.arm_all()
        while not arm_future.done():
            rclpy.spin_once(self)

        self.multi_offboard_controller.offboard_mode_all()
        self.multi_offboard_controller.check_offboard()
        self.get_logger().info(f"Vehicles waiting for offboard mode.")

        self.get_logger().info(f"Vehicles taking off.")
        takeoff_future = self.multi_offboard_controller.take_off_all(10.0)
        while not takeoff_future.done():
            rclpy.spin_once(self)

        self.get_logger().info(f"Vehicles going to initial formation.")
        goto_future = self.multi_offboard_controller.go_to_all_local(
            self.gps_origin,
            initial_formation,
            n_points = self.n_points
        )
        while not goto_future.done():
            rclpy.spin_once(self)


    def rtl(self):

        rtl_future = self.multi_offboard_controller.return_to_launch_all()
        while not rtl_future.done():
            rclpy.spin_once(self)

        disarm_future = self.multi_offboard_controller.disarm_all()
        while not disarm_future.done():
            rclpy.spin_once(self)

        self.multi_offboard_controller.hold_all()
        self.multi_offboard_controller.disarm_all()

        
    def execute_mission(self, plan_type: int): 
           
        paths = self.get_paths(plan_type)

        self.get_logger().info(f"Ejecutando misión")
        
        self.multi_offboard_controller.offboard_mode_all()
        self.multi_offboard_controller.check_offboard()
        self.get_logger().info(f"Vehicles waiting for offboard mode.")

        goto_future = self.multi_offboard_controller.go_to_all_local(
            self.gps_origin,
            paths,
            n_points = self.n_points
        )

        start_time = time.perf_counter()
                
        while not goto_future.done():
            rclpy.spin_once(self)
        
        end_time = time.perf_counter()
        elapsed = end_time - start_time

        if plan_type == HUNGARIAN:
            method = "Hungarian"
        elif plan_type == RANDOM:
            method = "Random"

        self.get_logger().info(f"Método: {method}, Tiempo transcurrido: {elapsed}")

        land_future = self.multi_offboard_controller.land_all()
                
        while not land_future.done():
            rclpy.spin_once(self)

        self.multi_offboard_controller.hold_all()
        self.multi_offboard_controller.disarm_all()


    def get_vehicle_poses(self):
        
        vehicle_poses = []
        for controller in self.multi_offboard_controller.controllers:
            
            enu_pose = Pose() 
            
            pos_state = controller.state_manager.state_repositories[States.LOCAL_POSITION].get()

            enu_pose.position.x = pos_state.y
            enu_pose.position.y = pos_state.x
            enu_pose.position.z = -pos_state.z

            dx, dy, dz = gps_offset_in_enu(
                lat_ref=pos_state.ref_lat, lon_ref=pos_state.ref_lon, alt_ref=pos_state.ref_alt,
                lat_origin=self.gps_origin.position.latitude, lon_origin=self.gps_origin.position.longitude, alt_origin=self.gps_origin.position.altitude
            )
            enu_pose.position.x += dx
            enu_pose.position.y += dy
            enu_pose.position.z += dz

            vehicle_poses.append(enu_pose)
            
        return vehicle_poses


    def get_paths(self, plan_type: int):

        paths = self.get_vehicle_poses()

        if plan_type == HUNGARIAN:
            
            drone_indices, target_indices = solve(
                paths,
                list(self.aruco_poses.values())[:self.n_targets]
            )

            for drone_idx, target_idx in zip(drone_indices, target_indices):
                paths[drone_idx] = enu_ned(self.aruco_poses[str(target_idx)])

        elif plan_type == RANDOM:
            paths = [enu_ned(self.aruco_poses[str(i)]) for i in list(self.aruco_poses.values())[:self.n_targets]]
            random.shuffle(paths) 
    
        return paths        


    def check_perception(self):
        if len(self.detected_ids) == len(self.aruco_poses):
            self.get_logger().info("¡Todos los ArUco IDs han sido detectados!")
            return True
        return False
    

    def configure(self):

        self.configuration_service = SimUAVSConfigurationService(
            self,
            self.vehicle_ids    
        )
        self.gps_origin = self.configuration_service.get_auto_gps_reference(
            self.vehicle_ids[0]
        )

        multi_offboard_controller_assembler = MultiOffboardControllerAssembler()
        
        self.multi_offboard_controller = multi_offboard_controller_assembler.assemble(
            self,
            self.configuration_service.get_offboard_configurations()
        ) 
        
        self.map_corners = load_points_from_csv(f"corners.csv")
        self.map_midpoints = puntos_medios_rectangulo(
            self.map_corners
        )
        self.aruco_poses = load_arucos_from_csv(f"aruco_poses.csv")
        
        for i in self.vehicle_ids:
            self.rgb_subs.append(self.create_subscription(
                CompressedImage,
                f"/world/{self.world}/model/{self.uav_model}_{i}/link/mono_cam/base_link/sensor/camera/image/compressed",
                self.image_callback,
                10
            ))


def main(args=None):
  
    rclpy.init(args=args)
    node = JJAASwarm()    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

