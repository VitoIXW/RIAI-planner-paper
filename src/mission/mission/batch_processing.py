import csv
import os

FILE = "results.csv"

def save(
        mission_id: str,
        task_id: str,
        n_vehicles: int, 
        time: float,
        spatial_tol: float = None,
        n_steps: int = None
    ):
        file_exists = os.path.exists(FILE)
    
        with open(FILE, mode='a', newline='') as f:
            writer = csv.writer(f)
            
            if not file_exists:
                writer.writerow(["mission_id", "task_id", "n_vehicles", "time", "spatial_tol", "n_steps"])
            
            writer.writerow([mission_id, task_id, n_vehicles, time, spatial_tol if spatial_tol is not None else "", n_steps if n_steps is not None else ""])

        