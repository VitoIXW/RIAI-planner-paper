import matplotlib.pyplot as plt
import numpy as np
from metrics_processor import compute_time_between_detections, compute_euclidean_distance_sum, compute_assignation_elapsed_time, compute_execution_elapsed_time, compute_trajectory_length_sum 


PLAN_TYPES={"0": "M1", "1": "Método propuesto", "2": "M2", "3": "M3"}


def plot_time_between_detections():
    t_detections = compute_time_between_detections()
    vehicles = sorted(t_detections.keys())
    times = [t_detections[v] for v in vehicles]
    plt.figure(figsize=(8,5))
    plt.bar(vehicles, times, color='skyblue')
    plt.xlabel("Número de UAVs", fontsize=16)
    plt.ylabel("Tiempo (s)", fontsize=16)
    plt.title("Promedio del tiempo medio entre detecciones", fontsize=18)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()


def plot_trajectory_length_sum():
    lengths = compute_trajectory_length_sum()
    plan_types = sorted(lengths.keys())
    all_n_vehicles = sorted({n for data in lengths.values() for n in data.keys()})
    n_vehicles_count = len(all_n_vehicles)
    x = np.arange(len(plan_types))  
    width = 0.8 / n_vehicles_count
    plt.figure(figsize=(14,6))
    for i, n in enumerate(all_n_vehicles):
        distances = [lengths[pt].get(n, 0) for pt in plan_types]
        plt.bar(x + i*width, distances, width, label=f'{n} UAV(s)')
    plt.xlabel("Tipo de planificación", fontsize=16, labelpad=15)
    plt.ylabel("Longitud (m)", fontsize=16)
    plt.title("Promedio de la suma de longitudes de trayectorias ejecutadas", fontsize=18)
    plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="Número de UAVs", fontsize=15, title_fontsize=15, bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()


def plot_euclidean_distance_sum():
    d_covered = compute_euclidean_distance_sum()
    plan_types = sorted(d_covered.keys())
    all_n_vehicles = sorted({n for data in d_covered.values() for n in data.keys()})
    n_vehicles_count = len(all_n_vehicles)
    x = np.arange(len(plan_types))  
    width = 0.8 / n_vehicles_count
    plt.figure(figsize=(14,6))
    for i, n in enumerate(all_n_vehicles):
        distances = [d_covered[pt].get(n, 0) for pt in plan_types]
        plt.bar(x + i*width, distances, width, label=f'{n} UAV(s)')
    plt.xlabel("Tipo de planificación", fontsize=16, labelpad=15)
    plt.ylabel("Distancia (m)", fontsize=16)
    plt.title("Promedio de la suma de distancias entre agentes y tareas asignadas", fontsize=18)
    plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="Número de UAVs", fontsize=15, title_fontsize=15, bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()


def plot_assignation_time_by_spatial_tol(n_tolerances=None):
    """
    Grafica el tiempo de asignación por tolerancia espacial.
    
    Args:
        n_tolerances: Número de tolerancias a considerar (las primeras n ordenadas, excluyendo 0.5).
                     Si es None, se consideran todas.
                     Ejemplo: n_tolerances=2 tomará solo 1.0 y 1.5, excluyendo 0.5
    """
    t_assignation = compute_assignation_elapsed_time()
    for plan_type, plan_data in t_assignation.items():
        spatial_tols = sorted({st for n_data in plan_data.values() for st in n_data.keys()})
        
        if n_tolerances is not None:
            spatial_tols = spatial_tols[1:n_tolerances+1]
        
        n_vehicles_list = sorted(plan_data.keys())
        n_vehicles_count = len(n_vehicles_list)
        x = np.arange(len(spatial_tols))
        width = 0.8 / n_vehicles_count
        plt.figure(figsize=(12,6))
        for i, n in enumerate(n_vehicles_list):
            times = [plan_data[n].get(st, 0) for st in spatial_tols]
            plt.bar(x + i*width, times, width, label=f'{n} UAV(s)')
        plt.xlabel("Tolerancia (m)", fontsize=16)
        plt.ylabel("Tiempo (s)", fontsize=16)
        plt.title(f"Promedio del tiempo medio de planificación - Plan {PLAN_TYPES[plan_type]}", fontsize=18)
        plt.xticks(x + width*(n_vehicles_count-1)/2, spatial_tols, fontsize=15)
        plt.yticks(fontsize=15)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.legend(title="Número de UAVs", fontsize=15, title_fontsize=15)
        plt.tight_layout()
        plt.show()


def plot_assignation_time_by_plan_type(n_tolerances=None):
    """
    Grafica el tiempo de asignación agrupado por tipo de planificación.
    Genera una figura por cada tolerancia espacial.
    
    Args:
        n_tolerances: Número de tolerancias a considerar (las primeras n ordenadas, excluyendo 0.5).
                     Si es None, se consideran todas.
                     Ejemplo: n_tolerances=2 tomará solo 1.0 y 1.5, excluyendo 0.5
    """
    t_assignation = compute_assignation_elapsed_time()
    
    # Obtener todas las tolerancias espaciales únicas
    all_spatial_tols = sorted({st for plan_data in t_assignation.values() 
                               for n_data in plan_data.values() 
                               for st in n_data.keys()})
    
    # Filtrar tolerancias si se especifica n_tolerances
    if n_tolerances is not None:
        all_spatial_tols = all_spatial_tols[1:n_tolerances+1]
    
    # Para cada tolerancia espacial, crear una gráfica
    for spatial_tol in all_spatial_tols:
        plan_types = sorted(t_assignation.keys())
        n_vehicles_list = sorted({n for plan_data in t_assignation.values() 
                                 for n in plan_data.keys()})
        n_vehicles_count = len(n_vehicles_list)
        
        x = np.arange(len(plan_types))
        width = 0.8 / n_vehicles_count
        
        plt.figure(figsize=(12,6))
        for i, n in enumerate(n_vehicles_list):
            times = []
            for plan_type in plan_types:
                time_value = t_assignation.get(plan_type, {}).get(n, {}).get(spatial_tol, 0)
                times.append(time_value)
            plt.bar(x + i*width, times, width, label=f'{n} UAV(s)')
        
        plt.xlabel("Tipo de planificación", fontsize=16, labelpad=15)
        plt.ylabel("Tiempo (s)", fontsize=16)
        plt.title(f"Promedio del tiempo medio de planificación - tolerancia {spatial_tol} m", fontsize=18)
        plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
        plt.yticks(fontsize=15)
        plt.grid(axis='y', linestyle='--', alpha=0.7)
        plt.legend(title="Número de UAVs", fontsize=15, title_fontsize=15)
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
        plt.bar(x + i*width, times, width, label=f'{n} UAV(s)')
    plt.xlabel("Tipo de planificación", fontsize=16, labelpad=15)
    plt.ylabel("Tiempo (s)", fontsize=16)
    plt.title("Promedio del tiempo medio de ejecución", fontsize=18)
    plt.xticks(x + width*(n_vehicles_count-1)/2, [PLAN_TYPES[pt] for pt in plan_types], fontsize=15)
    plt.yticks(fontsize=15)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.legend(title="Número de UAVs", fontsize=15, title_fontsize=15)
    plt.tight_layout()
    plt.show()


def plot_legend_only():
    
    d_covered = compute_euclidean_distance_sum()
    all_n_vehicles = sorted({n for data in d_covered.values() for n in data.keys()})
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.axis('off') 

    x = np.arange(len(all_n_vehicles))
    for i, n in enumerate(all_n_vehicles):
        ax.bar(x[i], 0, label=f'{n} UAV(s)')
    
    legend = ax.legend(title="Número de UAVs", 
                      fontsize=18, 
                      title_fontsize=20,
                      loc='center',
                      frameon=True,
                      fancybox=True,
                      shadow=True,
                      ncol=2)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_legend_only()
    plot_time_between_detections()
    plot_trajectory_length_sum()
    plot_euclidean_distance_sum()
    plot_assignation_time_by_plan_type(n_tolerances=2)
    plot_assignation_time_by_spatial_tol(n_tolerances=2)  
    plot_execution_time_by_plan_type()