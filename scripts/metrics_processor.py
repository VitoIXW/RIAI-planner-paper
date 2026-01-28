import csv
import numpy as np


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
        self.task_pose = np.array([float(dict["x"]), float(dict["y"]), float(dict["z"])])
        self.detection_time = dict["detection_time"]


class TrajectoryRow:
    def __init__(self, dict):
        self.mission_id = dict["mission_id"]
        self.trajectory_length = float(dict["trajectory_length"])
            

class AssignationRow:
    def __init__(self, dict):
        self.mission_id = dict["mission_id"]
        self.vehicle_id = dict["vehicle_id"]
        self.vehicle_pose = np.array([float(dict["x"]), float(dict["y"]), float(dict["z"])])
        self.task_id = dict["task_id"]
            
            
def load_mission_rows() -> dict[str, MissionRow]:
    with open("results/mission_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return {row["mission_id"]: MissionRow(row) for row in reader}
    

def load_perception_rows(mission_id) -> dict[int, PerceptionRow]:
    with open("results/perception_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return {row["task_id"]: PerceptionRow(row) for row in reader if row["mission_id"] == mission_id}    


def load_trajectory_rows(mission_id) -> list[TrajectoryRow]:
    with open("results/trajectory_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return [TrajectoryRow(row) for row in reader if row["mission_id"] == mission_id]  


def load_assignation_rows(mission_id) -> dict[int, AssignationRow]:
    with open("results/assignation_results.csv", newline='') as f:
        reader = csv.DictReader(f)
        return {row["vehicle_id"]: AssignationRow(row) for row in reader if row["mission_id"] == mission_id}  
        
        
def time_between_detections(perception_rows: list[PerceptionRow]):
    
    dts = []
    prev = None
    
    for row in perception_rows:
        if prev == None:
            prev = row.detection_time
        else:
            dts.append(float(row.detection_time) - float(prev))
            prev = row.detection_time  
                  
    return np.mean(dts)        
    
        
def euclidean_distance_sum(assignation_rows: list[AssignationRow]):
    
    distances = []

    for row in assignation_rows:
        task_rows = load_perception_rows(row.mission_id)

        if row.task_id in task_rows:
            distances.append(
                np.linalg.norm(
                    row.vehicle_pose - task_rows[row.task_id].task_pose
                ))
        
    return np.sum(distances)


def compute_time_between_detections():

    time_between_detections_dict = {}
    mission_rows = load_mission_rows().values()

    for mission_row in mission_rows:

        if mission_row.n_vehicles not in time_between_detections_dict:
            time_between_detections_dict[mission_row.n_vehicles] = []

        time_between_detections_dict[mission_row.n_vehicles].append(
            time_between_detections(load_perception_rows(mission_row.mission_id).values()))

    for n in time_between_detections_dict:
        time_between_detections_dict[n] = np.mean(time_between_detections_dict[n])

    return time_between_detections_dict


def compute_euclidean_distance_sum():

    euclidean_distance_sum_dict = {}

    for mission_row in load_mission_rows().values():

        if mission_row.plan_type not in euclidean_distance_sum_dict:
            euclidean_distance_sum_dict[mission_row.plan_type] = {}
            
        if mission_row.n_vehicles not in euclidean_distance_sum_dict[mission_row.plan_type]:
            euclidean_distance_sum_dict[mission_row.plan_type][mission_row.n_vehicles] = []
        
        euclidean_distance_sum_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            euclidean_distance_sum(load_assignation_rows(mission_row.mission_id).values()))
        
    for n in euclidean_distance_sum_dict:
        for m in euclidean_distance_sum_dict[n]:
            euclidean_distance_sum_dict[n][m] = np.mean(euclidean_distance_sum_dict[n][m])
    
    return euclidean_distance_sum_dict


def compute_trajectory_length_sum():

    trajectory_length_sum_dict = {}

    for mission_row in load_mission_rows().values():

        if mission_row.plan_type not in trajectory_length_sum_dict:
            trajectory_length_sum_dict[mission_row.plan_type] = {}
            
        if mission_row.n_vehicles not in trajectory_length_sum_dict[mission_row.plan_type]:
            trajectory_length_sum_dict[mission_row.plan_type][mission_row.n_vehicles] = []
        
        trajectory_length_sum_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            sum(tr.trajectory_length for tr in load_trajectory_rows(mission_row.mission_id))
        )
        
    for n in trajectory_length_sum_dict:
        for m in trajectory_length_sum_dict[n]:
            trajectory_length_sum_dict[n][m] = np.mean(trajectory_length_sum_dict[n][m])
    
    return trajectory_length_sum_dict


def compute_assignation_elapsed_time():

    assignation_elapsed_time_dict = {}

    for mission_row in load_mission_rows().values():

        if mission_row.plan_type not in assignation_elapsed_time_dict: 
            assignation_elapsed_time_dict[mission_row.plan_type] = {}
                
        if mission_row.n_vehicles not in assignation_elapsed_time_dict[mission_row.plan_type]:
            assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] = {}
        
        if mission_row.spatial_tol not in assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles]:
            assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles][mission_row.spatial_tol] = []

        assignation_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles][mission_row.spatial_tol].append(
            float(mission_row.execution_start_time) - float(mission_row.assignation_start_time)
        )

    for n in assignation_elapsed_time_dict:
        for m in assignation_elapsed_time_dict[n]:
            for p in assignation_elapsed_time_dict[n][m]:
                assignation_elapsed_time_dict[n][m][p] = np.mean(assignation_elapsed_time_dict[n][m][p])
    
    return assignation_elapsed_time_dict


def compute_execution_elapsed_time():

    execution_elapsed_time_dict = {}

    for mission_row in load_mission_rows().values():
        
        if mission_row.plan_type not in execution_elapsed_time_dict: 
            execution_elapsed_time_dict[mission_row.plan_type] = {}
            
        if mission_row.n_vehicles not in execution_elapsed_time_dict[mission_row.plan_type]:
            execution_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles] = []
            
        execution_elapsed_time_dict[mission_row.plan_type][mission_row.n_vehicles].append(
            float(mission_row.execution_end_time) - float(mission_row.execution_start_time)
        )

    for n in execution_elapsed_time_dict:
        for m in execution_elapsed_time_dict[n]:
            execution_elapsed_time_dict[n][m] = np.mean(execution_elapsed_time_dict[n][m])

    return execution_elapsed_time_dict


if __name__ == "__main__":
    
    print("=" * 80)
    print("Time between detections for number of vehicles.")
    print("=" * 80)
    t_detections = compute_time_between_detections()
    print(f"{t_detections}")
    
    print("=" * 80)
    print("Covered distance for planning type and number of vehicles.")
    print("=" * 80)
    d_covered = compute_euclidean_distance_sum()
    print(f"{d_covered}")

    print("=" * 80)
    print("Assignation elapsed time for planning type, number of vehicles and spatial tolerance.")
    print("=" * 80)
    t_assignation = compute_assignation_elapsed_time()
    print(f"{t_assignation}")

    print("=" * 80)
    print("Execution elapsed time for planning type and number of vehicles.")
    print("=" * 80)
    t_execution = compute_execution_elapsed_time()
    print(f"{t_execution}")

        
        