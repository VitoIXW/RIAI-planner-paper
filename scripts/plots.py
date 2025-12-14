import matplotlib.pyplot as plt
import numpy as np
from metrics_processor import compute_time_between_detections, compute_avg_distance_covered, compute_assignation_elapsed_time, compute_execution_elapsed_time 


PLAN_TYPES={"0": "RRT", "1": "RRT *", "2": "RRT * + Húngaro", "3": "Aleatorio"}


def plot_time_between_detections():
    t_detections = compute_time_between_detections()
    vehicles = sorted(t_detections.keys())
    times = [t_detections[v] for v in vehicles]
    plt.figure(figsize=(8,5))
    plt.bar(vehicles, times, color='skyblue')
    plt.xlabel("Número de vehículos", fontsize=16)
    plt.ylabel("Tiempo (s)", fontsize=16)
    plt.title("Tiempo medio entre detecciones", fontsize=18)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()


def plot_avg_distance_covered():
    d_covered = compute_avg_distance_covered()
    plan_types = sorted(d_covered.keys())
    all_n_vehicles = sorted({n for data in d_covered.values() for n in data.keys()})
    n_vehicles_count = len(all_n_vehicles)
    x = np.arange(len(plan_types))  
    width = 0.8 / n_vehicles_count
    plt.figure(figsize=(12,6))
    for i, n in enumerate(all_n_vehicles):
        distances = [d_covered[pt].get(n, 0) for pt in plan_types]
        plt.bar(x + i*width, distances, width, label=f'{n} vehículo(s)')
    plt.xlabel("Tipo de planificación", fontsize=16)
    plt.ylabel("Distancia (m)", fontsize=16)
    plt.title("Distancia media recorrida", fontsize=18)
    plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="Número de vehículos", fontsize=15, title_fontsize=15)
    plt.tight_layout()
    plt.show()


def plot_assignation_time_by_spatial_tol():
    t_assignation = compute_assignation_elapsed_time()
    for plan_type, plan_data in t_assignation.items():
        spatial_tols = sorted({st for n_data in plan_data.values() for st in n_data.keys()})
        n_vehicles_list = sorted(plan_data.keys())
        n_vehicles_count = len(n_vehicles_list)
        x = np.arange(len(spatial_tols))
        width = 0.8 / n_vehicles_count
        plt.figure(figsize=(12,6))
        for i, n in enumerate(n_vehicles_list):
            times = [plan_data[n].get(st, 0) for st in spatial_tols]
            plt.bar(x + i*width, times, width, label=f'{n} vehículo(s)')
        plt.xlabel("Tolerancia (m)", fontsize=16)
        plt.ylabel("Tiempo (s)", fontsize=16)
        plt.title(f"Tiempo medio de asignación - Plan {PLAN_TYPES[plan_type]}", fontsize=18)
        plt.xticks(x + width*(n_vehicles_count-1)/2, spatial_tols, fontsize=15)
        plt.yticks(fontsize=15)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.legend(title="Número de vehículos", fontsize=15, title_fontsize=15)
        plt.tight_layout()
        plt.show()


def plot_execution_time_by_plan_type():
    t_execution = compute_execution_elapsed_time()
    plan_types = sorted(t_execution.keys())
    n_vehicles_list = sorted({n for data in t_execution.values() for n in data.keys()})
    n_vehicles_count = len(n_vehicles_list)
    x = np.arange(len(plan_types))
    width = 0.8 / n_vehicles_count
    plt.figure(figsize=(12,6))
    for i, n in enumerate(n_vehicles_list):
        times = [t_execution[plan].get(n, 0) for plan in plan_types]
        plt.bar(x + i*width, times, width, label=f'{n} vehículo(s)')
    plt.xlabel("Tipo de planificación", fontsize=16)
    plt.ylabel("Tiempo (s)", fontsize=16)
    plt.title("Tiempo medio de ejecución", fontsize=18)
    plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="Número de vehículos", fontsize=15, title_fontsize=15)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_time_between_detections()
    plot_avg_distance_covered()
    plot_assignation_time_by_spatial_tol()
    plot_execution_time_by_plan_type()