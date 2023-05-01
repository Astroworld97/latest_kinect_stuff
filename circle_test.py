import numpy as np
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the two points and the center point of the circle
point1 = np.array([0, 0, 0])
point2 = np.array([0, 1, 0])
center = np.array([1, 0.5, 0])

# Calculate the normal vector of the plane containing the two points and the center point
normal = np.cross(point1 - center, point2 - center)

# Choose a point on the circle
point_on_circle = center + normal * np.linalg.norm(point1 - center)

# Calculate the radius of the circle
radius = np.linalg.norm(point_on_circle - center)

# Calculate the equation of the circle in 3D space
theta = np.linspace(0, 2*np.pi, 100)
x = center[0] + radius * np.cos(theta) * normal[0] + \
    radius * np.sin(theta) * np.cross(normal, [1, 0, 0])[0]
y = center[1] + radius * np.cos(theta) * normal[1] + \
    radius * np.sin(theta) * np.cross(normal, [0, 1, 0])[1]
z = center[2] + radius * np.cos(theta) * normal[2] + \
    radius * np.sin(theta) * np.cross(normal, [0, 0, 1])[2]

# create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw the circle
for i in range(len(x) - 1):
    pt1 = (int(x[i]), int(y[i]), int(z[i]))
    pt2 = (int(x[i+1]), int(y[i+1]), int(z[i+1]))
    ax.scatter(pt1[0], pt1[1], pt1[2], c='r')
    ax.scatter(pt2[0], pt2[1], pt2[2], c='b')

# plot a line connecting the points
ax.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], [pt1[2], pt2[2]], c='g')

# set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# show the plot
plt.show()
