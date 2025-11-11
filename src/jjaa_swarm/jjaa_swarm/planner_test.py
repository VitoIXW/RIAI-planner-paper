import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from rtt_star_planner import RttStarPlanner

upper_limit = np.array([20.0, 20.0])
lower_limit = np.array([0.0, 0.0])

start = np.array([2.0, 0.0])
goal = np.array([20.0, 15.0])

obstacles = np.array([[5.0, 5.0, 1.0],[12.0, 6.0, 1.0],[4.0, 3.0, 1.0],[3.0, 10.0, 1.0]])

step_size = 1.0
n_steps = 1000
tol = .5

planner = RttStarPlanner(
    lower_limit,
    upper_limit,
    step_size,
    n_steps
)

fig, ax = plt.subplots(figsize=(8,8))
for obstacle in obstacles:
    circle = Circle((obstacle[0], obstacle[1]), obstacle[2], color='gray', alpha=0.5)
    ax.add_patch(circle)

goal_node, tree = planner.solve(start, goal, obstacles, tol)

for node in tree:
    if node._parent:
        ax.plot([node._position[0], node._parent._position[0]],
                [node._position[1], node._parent._position[1]],
                color='blue', linewidth=0.5, alpha=0.5)

if goal_node:
    n = goal_node
    while n._parent:
        ax.plot([n._position[0], n._parent._position[0]],
                [n._position[1], n._parent._position[1]],
                color='green', linewidth=1.5, alpha=1.0)
        n = n._parent

x = [node._position[0] for node in tree]
y = [node._position[1] for node in tree]

ax.scatter(x, y, color='red', marker='o', label='Nodes')
ax.scatter(*start, color='green', s=100, marker='o', label='start')
ax.scatter(*goal, color='red', s=100, marker='*', label='goal')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title(f'RTT Tree*, Iterations: {len(tree)}, tol: {tol} m, step_size: {step_size} m')
ax.legend()
ax.grid(True)

plt.show()