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
    diff = position - obstacle

    h = np.dot(diff, diff) - d_safe**2

    return h


def barrier_function_gradient(position, obstacle):
    diff = position - obstacle

    grad_h = 2 * diff

    return grad_h

# ==========================================
# 3. Nominal Controller
# ==========================================

def nominal_controller(position, goal, gain=1.0):
    error = goal - position
    u_nominal = gain * error

    return u_nominal

# ==========================================
# 4. CBF-QP Controller
# ==========================================

def cbf_controller(position, obstacle, d_safe, u_nominal, gamma=5.0):

    # 1. 현재 Barrier value
    h = barrier_function(position, obstacle, d_safe)

    # 2. dh/dz
    grad_h = barrier_function_gradient(position, obstacle)

    # 3. QP에서 우리가 결정할 control input
    u = cp.Variable(2)

    # 4. u_nominal을 최대한 유지
    objective = cp.Minimize(cp.sum_squares(u - u_nominal))

    # 5. CBF Safety Constraint
    constraint = [grad_h @ u >= -gamma * h]

    # 6. QP 구성
    problem = cp.Problem(objective, constraint)

    # 7. QP 풀기
    problem.solve()

    if u.value is None:
        raise RuntimeError("CBF-QP를 풀지 못했습니다.")

    # 8. 안전한 control 반환
    u_safe = np.array(u.value)

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
    position = position + u_safe * dt

    # 궤적 저장
    trajectory.append(position.copy())

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