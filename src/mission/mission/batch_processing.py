import csv
import os
from geometry_msgs.msg import Pose


MISSION_FILE = "results/mission_results.csv"
PERCEPTION_FILE = "results/perception_results.csv"
ASSIGNATION_FILE = "results/assignation_results.csv"
TRAJECTORY_RESULTS = "results/trajectory_results.csv"


def save_mission_row(
        mission_id: str,
        n_vehicles: int, 
        plan_type: int,
        perception_start_time: float,
        assignation_start_time: float,
        execution_start_time: float,
        execution_end_time: float,
        spatial_tol: float 
    ):
        file_exists = os.path.exists(MISSION_FILE)
    
        with open(MISSION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "n_vehicles", "plan_type", "perception_start_time", "assignation_start_time", "execution_start_time", "execution_end_time", "spatial_tol"])
            
            writer.writerow([
                mission_id, 
                n_vehicles,
                plan_type,
                perception_start_time,
                assignation_start_time,
                execution_start_time,
                execution_end_time,
                spatial_tol
            ])


def save_perception_row(
        mission_id: str,
        task_id: int,
        pose: Pose,
        detection_time: float
    ):
        file_exists = os.path.exists(PERCEPTION_FILE)
    
        with open(PERCEPTION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "task_id",  "x", "y", "z", "detection_time"])
            
            writer.writerow([
                mission_id, 
                task_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                detection_time
            ])
            

def save_trajectory_row(
        mission_id: str,
        trajectory_length: float
    ):
        file_exists = os.path.exists(TRAJECTORY_RESULTS)
    
        with open(TRAJECTORY_RESULTS, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "trajectory_length"])
            
            writer.writerow([
                mission_id, 
                trajectory_length
            ])


def save_trajectories(
        mission_id: str,
        trajectories, 
        avg_speed: float
    ):
        
    for trajectory in trajectories:
        duration = sum(trajectory[3]) # sum of dts, duration of the trajectory.
        trajectory_length = duration * avg_speed
        save_trajectory_row(mission_id, trajectory_length)


def save_assignation_row(
        mission_id: str,
        vehicle_id: int,
        vehicle_pose: Pose,
        task_id: int,
    ):
        file_exists = os.path.exists(ASSIGNATION_FILE)
    
        with open(ASSIGNATION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "vehicle_id", "x", "y", "z", "task_id"])
            
            writer.writerow([
                mission_id, 
                vehicle_id,
                vehicle_pose.position.x,
                vehicle_pose.position.y,
                vehicle_pose.position.z,
                task_id
            ])