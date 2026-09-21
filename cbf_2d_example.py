import numpy as np

import matplotlib
matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import cvxpy as cp

# ==========================================
# 1. Parameters
# ==========================================

# Robot initial position
position = np.array([0.0, 0.0])

# Goal position
goal = np.array([5.0, 0.0])

# Obstacle center
# obstacle = np.array([2.5, 0.0])
obstacle = np.array([2.5, 0.4])

# Safety distance
d_safe = 0.7

# Simulation time step
dt = 0.01


# ==========================================
# 2. Barrier Function
# ==========================================

def barrier_function(position, obstacle, d_safe):


    return h


def barrier_function_gradient(position, obstacle):

    return grad_h

# ==========================================
# 3. Nominal Controller
# ==========================================

def nominal_controller(position, goal, gain=1.0):


    return u_nominal

# ==========================================
# 4. CBF-QP Controller
# ==========================================

def cbf_controller(position, obstacle, d_safe, u_nominal, gamma=5.0):

    # 1. 현재 Barrier value


    # 2. dh/dz


    # 3. QP에서 우리가 결정할 control input


    # 4. u_nominal을 최대한 유지


    # 5. CBF Safety Constraint


    # 6. QP 구성


    # 7. QP 풀기


    # 8. 안전한 control 반환

    return u_safe

# ==========================================
# 5. Simulation
# ==========================================

# 초기 위치 다시 설정
position = np.array([0.0, 0.0])

# Simulation 횟수
num_steps = 1000

# Robot이 지나간 위치 저장
trajectory = []

for step in range(num_steps):

    # 원래 하고 싶은 움직임
    u_nominal = nominal_controller(
        position,
        goal
    )

    # CBF를 통과한 안전한 움직임
    u_safe = cbf_controller(
        position,
        obstacle,
        d_safe,
        u_nominal
    )

    # Robot state update


    # 궤적 저장


# ==========================================
# 6. Visualization
# ==========================================

trajectory = np.array(trajectory)

plt.figure()

# Robot trajectory
plt.plot(
    trajectory[:, 0],
    trajectory[:, 1],
    label="Robot trajectory"
)

# Goal
plt.scatter(
    goal[0],
    goal[1],
    marker="*",
    s=150,
    label="Goal"
)

# Obstacle center
plt.scatter(
    obstacle[0],
    obstacle[1],
    label="Obstacle"
)

# Safety boundary
circle = plt.Circle(
    obstacle,
    d_safe,
    fill=False
)

plt.gca().add_patch(circle)

plt.axis("equal")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()

plt.savefig("results/cbf_trajectory.png", dpi=150)
plt.show()