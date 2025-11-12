import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle 
from rtt_star_planner import RttStarPlanner 

CYLINDER_HEIGHT = 1.3 
OBSTACLE_RADIUS = 1.5
Z_BASE = 0.0 
BIAS_PROB = .8
SPACE_COEF = .2
TIME_COEF = .8
SPATIAL_TOL = 1.5
TIME_TOL = 50.0
SPEED = 5.0
LIMIT=True
STEP_SIZE = 1.0
N_STEPS = 2000

upper_limit = np.array([20.0, 20.0, 1.5])
lower_limit = np.array([0.0, 0.0, 0.0])

start = np.array([0.0, 0.0, 1.5, .0])
goal = np.array([20.0, 20.0, 1.5, 70.0])

obstacles_0 = np.array([[5.0, 5.0, CYLINDER_HEIGHT, OBSTACLE_RADIUS],
                      [12.0, 6.0, CYLINDER_HEIGHT, OBSTACLE_RADIUS],
                      [4.0, 3.0, CYLINDER_HEIGHT, OBSTACLE_RADIUS],
                      [3.0, 10.0, CYLINDER_HEIGHT, OBSTACLE_RADIUS], 
                      [15.0, 15.0, CYLINDER_HEIGHT, OBSTACLE_RADIUS]])
obstacles = []
for obstacle in obstacles_0:
    delta_t = 0.5
    t_obs = np.arange(0, 70 + delta_t, delta_t)
    n_points = len(t_obs)
    x_obs = np.full(n_points, obstacle[0])
    y_obs = np.full(n_points, obstacle[1])
    z_obs = np.full(n_points, CYLINDER_HEIGHT)
    r_obs = np.full(n_points, OBSTACLE_RADIUS)

    obstacle = np.column_stack([x_obs, y_obs, z_obs, t_obs, r_obs])
    obstacles.append(obstacle)
    
    
planner = RttStarPlanner(
    lower_limit,
    upper_limit,
    STEP_SIZE,
    N_STEPS,
    SPACE_COEF,
    TIME_COEF
)

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(111, projection='3d')

for obstacle in obstacles_0:

    Xc, Yc, R_dummy, R = obstacle[0], obstacle[1], obstacle[2], obstacle[3]
    u = np.linspace(0, 2 * np.pi, 50) 
    v = np.linspace(Z_BASE, Z_BASE + CYLINDER_HEIGHT, 50) 
    U, V = np.meshgrid(u, v)
    
    X = Xc + R * np.cos(U)
    Y = Yc + R * np.sin(U)
    Z = V
    
    ax.plot_surface(X, Y, Z, color='gray', alpha=0.5)
    
    X_base = Xc + R * np.cos(u)
    Y_base = Yc + R * np.sin(u)
    
    Z_base_const = Z_BASE * np.ones_like(X_base)
    Z_top_const = (Z_BASE + CYLINDER_HEIGHT) * np.ones_like(X_base)
    
    ax.plot_trisurf(X_base, Y_base, Z_base_const, color='gray', alpha=0.6) 
    ax.plot_trisurf(X_base, Y_base, Z_top_const, color='gray', alpha=0.6) 

goal_node, tree, n_iterations = planner.solve(start, goal, SPEED, obstacles, BIAS_PROB, LIMIT, SPATIAL_TOL, TIME_TOL)

def debug_path_cost(goal_node, space_coef, time_coef):
    """Trace back path and calculate expected vs actual cost."""
    if goal_node is None:
        print("No goal node found")
        return
    
    path = []
    node = goal_node
    while node is not None:
        path.append(node)
        node = node._parent
    
    path.reverse()
    
    print(f"\n=== PATH ANALYSIS ===")
    print(f"Number of nodes in path: {len(path)}")
    print(f"Space coef: {space_coef}, Time coef: {time_coef}")
    
    accumulated_cost = 0.0
    accumulated_dist = 0.0
    accumulated_time = 0.0
    
    for i in range(len(path)):
        node = path[i]
        print(f"\nNode {i}:")
        print(f"  Position: {node._position}")
        print(f"  Stored cost: {node._cost:.4f}")
        
        if i > 0:
            parent = path[i-1]
            spatial_dist = np.linalg.norm(node._position[:3] - parent._position[:3])
            temporal_dist = abs(node._position[3] - parent._position[3])
            edge_cost = space_coef * spatial_dist + time_coef * temporal_dist
            
            accumulated_dist += spatial_dist
            accumulated_time += temporal_dist
            accumulated_cost += edge_cost
            
            print(f"  Edge from parent:")
            print(f"    Spatial dist: {spatial_dist:.4f} m")
            print(f"    Temporal dist: {temporal_dist:.4f} s")
            print(f"    Edge cost: {edge_cost:.4f}")
            print(f"    Expected accumulated: {accumulated_cost:.4f}")
            print(f"    Difference: {node._cost - accumulated_cost:.4f}")
    
    print(f"\n=== SUMMARY ===")
    print(f"Total spatial distance: {accumulated_dist:.4f} m")
    print(f"Total temporal distance: {accumulated_time:.4f} s")
    print(f"Expected final cost: {accumulated_cost:.4f}")
    print(f"Actual final cost: {goal_node._cost:.4f}")
    print(f"Final time: {goal_node._position[3]:.4f} s")

#debug_path_cost(goal_node, SPACE_COEF, TIME_COEF)

final_time = .0
total_cost = .0

for node in tree:
    if node._parent:
        ax.plot([node._position[0], node._parent._position[0]],
                [node._position[1], node._parent._position[1]],
                [node._position[2], node._parent._position[2]],
                color='blue', linewidth=0.5, alpha=0.5)

if goal_node:
    n = goal_node
    final_time = goal_node._position[3]
    total_cost = goal_node._cost
    print(f"{final_time},{total_cost}")
    while n._parent:
        ax.plot([n._position[0], n._parent._position[0]],
                [n._position[1], n._parent._position[1]],
                [n._position[2], n._parent._position[2]],
                color='green', linewidth=4.5, alpha=1.0, label='Path')
        n = n._parent

x = [node._position[0] for node in tree]
y = [node._position[1] for node in tree]
z = [node._position[2] for node in tree]

ax.scatter(x, y, z, color='red', marker='.', s=10, label='Nodes')
ax.scatter(start[0], start[1], start[2], color='green', s=100, marker='o', label='Start')
ax.scatter(goal[0], goal[1], goal[2], color='red', s=100, marker='*', label='Goal')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(lower_limit[0], upper_limit[0])
ax.set_ylim(lower_limit[1], upper_limit[1])
ax.set_zlim(lower_limit[2], upper_limit[2])

ax.set_title(f'RTT* Tree PLanner 3D. N.Iterations: {n_iterations}.', fontsize=12)

parameter_summary = (
    f"Final time: {final_time:.2f} s\n"
    f"Total cost: {total_cost:.2f}\n"
    f"Tol: {SPATIAL_TOL} m\n"
    f"Spatial coeficient (α): {SPACE_COEF}\n"
    f"Temporal coeficient(β): {TIME_COEF}\n"
    f"Step time: {STEP_SIZE} m\n"
    f"Bias prob: {BIAS_PROB}"
)

fig.text(
    0.02, 
    0.02, 
    parameter_summary, 
    fontsize=9, 
    bbox=dict(facecolor='lightgray', alpha=0.5, boxstyle='round,pad=0.5')
)

handles, labels = ax.get_legend_handles_labels()
obstacle_proxy = Rectangle((0, 0), 1, 1, fc='gray', alpha=0.5) 

unique_handles = {}
for h, l in zip(handles, labels):
    if l not in unique_handles:
        unique_handles[l] = h

if 'Obstacles' not in unique_handles:
    final_handles = [obstacle_proxy] + list(unique_handles.values())
    final_labels = ['Obstacles'] + list(unique_handles.keys())
else:
    final_handles = list(unique_handles.values())
    final_labels = list(unique_handles.keys())

ax.legend(final_handles, final_labels)
ax.grid(True)

plt.show()