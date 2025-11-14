from geometry_msgs.msg import Pose
from scipy.optimize import linear_sum_assignment
from math import sqrt


def solve(vehicle_poses, target_poses):
    cost_matrix = build_cost_matrix(vehicle_poses, target_poses)
    return linear_sum_assignment(cost_matrix)


def euclidean_distance(pose1: Pose, pose2: Pose) -> float:
    dx = pose2.position.x - pose1.position.x
    dy = pose2.position.y - pose1.position.y
    dz = pose2.position.z - pose1.position.z
    return sqrt(dx**2 + dy**2 + dz**2)


def build_cost_matrix(drones, targets):
    cost_matrix = []
    for drone in drones:
        row = [euclidean_distance(drone, target) for target in targets]
        cost_matrix.append(row)
    return cost_matrix
