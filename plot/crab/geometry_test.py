
## ---------------------------------- 
## ---------------------------------- 

import numpy as np
import matplotlib.pyplot as plt

# Define the vectors
z_bar = np.array([0, 0, -1])  # Current -Z axis
target_vector = np.array([1, 1, 0])  
target_vector = target_vector / np.linalg.norm(target_vector)  # Normalize

# Compute cross product
omega = np.cross(z_bar, target_vector)

# Plot
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Draw vectors
ax.quiver(0, 0, 0, z_bar[0], z_bar[1], z_bar[2], color='b', label='Current -Z axis ($\\bar{z}$)', linewidth=2)
ax.quiver(0, 0, 0, target_vector[0], target_vector[1], target_vector[2], color='r', label='Target Vector ($\\hat{t}$)', linewidth=2)
ax.quiver(0, 0, 0, omega[0], omega[1], omega[2], color='g', label='Rotation Axis ($\\omega$)', linewidth=2)

# Annotations
ax.text(z_bar[0], z_bar[1], z_bar[2], '$\\bar{z}$', color='b', fontsize=12)
ax.text(target_vector[0], target_vector[1], target_vector[2], '$\\hat{t}$', color='r', fontsize=12)
ax.text(omega[0], omega[1], omega[2], '$\\omega$', color='g', fontsize=12)

# Labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Rigid Body Rotation")

# Show legend
ax.legend()
plt.show()
