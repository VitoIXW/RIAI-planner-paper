#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# n vehiculos / Tiempo entre detecciones
# alrotimo de planificación / tiempo de ejecución
# alrotimo de planificación / distancia recorrida
# Para n precisiones espaciales / tiempo de ejecución RRT star


def plot_tasks(file_path: str = "results.csv"):
    
    df = pd.read_csv(file_path)
    
    grouped = (
        df.groupby(["task_id", "n_vehicles"])["time"].mean().reset_index()
    )
    tasks = grouped["task_id"].unique()
    
    plt.figure(figsize=(10,6))
    for task in tasks:
        sub = grouped[grouped["task_id"] == task]
        plt.plot(
            sub["n_vehicles"].astype(int).to_numpy(),
            sub["time"].astype(float).to_numpy(),
            marker="o",
            label=task
        )


    plt.xlabel("Número de vehículos")
    plt.ylabel("Tiempo (s)")
    plt.title("Tiempo por tarea y número de vehículos")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_tasks()