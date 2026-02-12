# Energy Efficient Trajectory· python
import pybullet as p
import pybullet_data
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.optimize import minimize
import time

# ------------------ PyBullet Setup ------------------
physicsClient = p.connect(p.GUI)  # GUI mode for visualization
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0,0,0]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("kuka_iiwa/model.urdf", robotStartPos, robotStartOrientation, useFixedBase=True)
num_joints = p.getNumJoints(robotId)
print(f"Robot has {num_joints} joints")

# ------------------ Start & Goal ------------------
start_joint_positions = np.array([0, 0, 0, -1.57, 0, 1.0, 0])
goal_joint_positions = np.array([0.5, -0.3, 0.2, -1.0, 0.1, 0.5, 0.0])
steps = 50
times = np.linspace(0,1,steps+1)

# ------------------ Cubic Interpolation Initial Trajectory ------------------
trajectory = []
for j in range(num_joints):
    cs = CubicSpline([0,1], [start_joint_positions[j], goal_joint_positions[j]])
    joint_traj = cs(times)
    trajectory.append(joint_traj)
trajectory = np.array(trajectory).T  # shape: (steps+1, num_joints)

# ------------------ Energy Computation ------------------
dt = 0.01

def compute_energy_for_trajectory(traj):
    total_energy = 0
    for joint_pos in traj:
        for j in range(num_joints):
            p.setJointMotorControl2(robotId, j, p.POSITION_CONTROL, targetPosition=joint_pos[j])
        p.stepSimulation()
        torques = []
        for j in range(num_joints):
            state = p.getJointState(robotId, j)
            tau = state[3]   # applied torque
            vel = state[1]   # joint velocity
            torques.append(abs(tau*vel)*dt)
        total_energy += sum(torques)
        time.sleep(dt)  # visualize smoothly
    return total_energy

# ------------------ Optimization Objective (Structure for DIRECT Mode) ------------------
flat_traj = trajectory.flatten()
bounds = [(-np.pi, np.pi)] * len(flat_traj)

def objective(flat_traj):
    traj = flat_traj.reshape((steps+1, num_joints))
    return compute_energy_for_trajectory(traj)

# ------------------ Optimized Trajectory ------------------
# For demo, we visualize cubic interpolation trajectory
# In research use, DIRECT mode can run optimizer and feed optimized_traj here
optimized_trajectory = trajectory

# ------------------ Run Simulation & Compute Energy ------------------
total_energy = compute_energy_for_trajectory(optimized_trajectory)
print(f"Total energy for visualized trajectory: {total_energy:.4f} Joules")

# ------------------ Disconnect ------------------
p.disconnect()
