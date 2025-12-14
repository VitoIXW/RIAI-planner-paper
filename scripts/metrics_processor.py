#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np

# n vehiculos / Tiempo entre detecciones
# alrotimo de planificación / tiempo de ejecución
# alrotimo de planificación / distancia recorrida
# Para n precisiones espaciales / tiempo de ejecución RRT star


class MissionRow:
    def __init__(self, dict):
        self.mission_id = dict["mission_id"]
        self.n_vehicles = dict["n_vehicles"]
        self.plan_type = dict["plan_type"]
        self.perception_start_time = dict["perception_start_time"]
        self.assignation_start_time = dict["assignation_start_time"]
        self.execution_start_time = dict["execution_start_time"]
        self.execution_end_time = dict["execution_end_time"]
        self.spatial_tol = dict["spatial_tol"]


class PerceptionRow:
    def __init__(self, dict):
        self.mission_id = dict["mission_id"]
        self.task_id = dict["task_id"]
        self.pose = dict["pose"]
        self.detection_time = dict["detection_time"]
            

class AssignationRow:
    def __init__(self, dict):
        self.mission_id = dict["mission_id"]
        self.vehicle_id = dict["vehicle_id"]
        self.vehicle_pose = dict["vehicle_pose"]
        self.task_id = dict["task_id"]
            
            
def load_mission_rows():
    with open("results/mission_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return [MissionRow(row) for row in reader]
    

def load_perception_rows(mission_id):
    with open("results/perception_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return [PerceptionRow(row) for row in reader if row["mission_id"] == mission_id]    
    
    
def load_assignation_rows(mission_id):
    with open("results/assignation_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return [AssignationRow(row) for row in reader if row["mission_id"] == mission_id]  
        
        
def time_between_detections(mission_row: MissionRow, perception_rows: list[PerceptionRow]):
    
    dts = []
    prev = None
    
    for row in perception_rows:
        if prev == None:
            prev = row.detection_time
        else:
            dts.append(row.detection_time - prev)
            prev = row.detection_time  
                  
    return np.mean(dts)        
    
        
def avg_distance_covered(mission_row: MissionRow, assignation_row: list[AssignationRow]):
    ...
    
    
                    
if __name__ == "__main__":
    
    time_between_detections_dict = {}
    avg_distance_covered_dict = {}
    assignation_elapsed_time_dict = {}
    execution_elapsed_time_dict = {}
    
    for mission_row in load_mission_rows():
        
        if time_between_detections_dict[mission_row.n_vehicles] is None:
            time_between_detections_dict[mission_row.n_vehicles] = []
            
        time_between_detections_dict[mission_row.n_vehicles].append(
            time_between_detections(
                mission_row, load_perception_rows(mission_row.mission_id)
            )
        )
        
        if avg_distance_covered_dict[mission_row.plan_type] is None:
            avg_distance_covered_dict[mission_row.plan_type] = {}
        
        if avg_distance_covered_dict[mission_row.plan_type][mission_row.n_vehicles] is None:
            avg_distance_covered_dict[mission_row.plan_type][mission_row.n_vehicles] = []
        
        avg_distance_covered_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            avg_distance_covered(
                mission_row, load_assignation_rows(mission_row.mission_id)
            )
        )
        
        if assignation_elapsed_time_dict[mission_row.plan_type] is None: 
            assignation_elapsed_time_dict[mission_row.plan_type] = []
            
        if assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] is None:
            assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] = []
            
        assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            mission_row.execution_start_time - mission_row.assignation_start_time
        )
        
        if execution_elapsed_time_dict[mission_row.plan_type] is None: 
            execution_elapsed_time_dict[mission_row.plan_type] = []
            
        if execution_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] is None:
            execution_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] = []
            
        execution_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            mission_row.execution_end_time - mission_row.execution_start_time
        )
        
        