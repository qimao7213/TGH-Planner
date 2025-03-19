import numpy as np
import matplotlib.pyplot as plt

# Initial state: position p0, velocity v0
p0 = np.array([0, 0, 0])
v0 = np.array([-0, 0, -0.2])
a = np.array([0.2, -0.2, 0.5])  # Control input: acceleration
total_time = 1.0  # Total time

one_time = 0.8 * total_time
# Direct calculation from 0s to 1s
phi_direct = np.eye(6)
for i in range(3):
    phi_direct[i, i + 3] = one_time

state0 = np.zeros(6)
state0[:3] = p0
state0[3:] = v0

integral_direct = np.zeros(6)
integral_direct[:3] = 0.5 * one_time**2 * a
integral_direct[3:] = one_time * a

state_direct = phi_direct @ state0 + integral_direct

# Incremental calculation in 10 steps
steps = 100
dt = total_time / steps

phi_step = np.eye(6)
for i in range(3):
    phi_step[i, i + 3] = dt

state_incremental = np.zeros(6)
state_incremental[:3] = p0
state_incremental[3:] = v0

incremental_positions = [state_incremental[:3].copy()]

for _ in range(steps):
    integral_step = np.zeros(6)
    integral_step[:3] = 0.5 * dt**2 * a
    integral_step[3:] = dt * a
    state_incremental = phi_step @ state_incremental + integral_step
    incremental_positions.append(state_incremental[:3].copy())

# Plot the trajectories
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Direct calculation position
ax.plot([p0[0], state_direct[0]], [p0[1], state_direct[1]], [p0[2], state_direct[2]], 'r-', label='Direct calculation')

# Incremental positions
incremental_positions = np.array(incremental_positions)
ax.plot(incremental_positions[:, 0], incremental_positions[:, 1], incremental_positions[:, 2], 'b--', label='Incremental calculation')

ax.set_xlabel('X position')
ax.set_ylabel('Y position')
ax.set_zlabel('Z position')
ax.set_title('Trajectories: Direct vs Incremental Calculation')
ax.legend()
plt.show()


