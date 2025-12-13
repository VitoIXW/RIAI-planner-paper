import csv
import os
from geometry_msgs.msg import Pose


MISSION_FILE = "results/mission_results.csv"
PERCEPTION_FILE = "results/perception_results.csv"
ASSIGNATION_FILE = "results/assignation_results.csv"
EXECUTION_FILE = "results/execution_results.csv"


def save_mission_row(
        mission_id: str,
        n_vehicles: int, 
        perception_start_time: float 
    ):
        file_exists = os.path.exists(MISSION_FILE)
    
        with open(MISSION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "n_vehicles","perception_start_time"])
            
            writer.writerow([
                mission_id, 
                n_vehicles,
                perception_start_time
            ])


def save_perception_row(
        mission_id: str,
        task_id: int,
        pose: Pose,
    ):
        file_exists = os.path.exists(PERCEPTION_FILE)
    
        with open(PERCEPTION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "task_id",  "x", "y", "z"])
            
            writer.writerow([
                mission_id, 
                task_id,
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ])


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


def save_execution_row(
        mission_id: str,
        time: float,
        plan_type: int
    ):
        file_exists = os.path.exists(EXECUTION_FILE)
    
        with open(EXECUTION_FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "time", "plan_type"])
            
            writer.writerow([
                mission_id, 
                time, 
                plan_type
            ])
        