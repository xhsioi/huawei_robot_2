import numpy as np

direction_near = [[0, 0], [1, 0], [-1, 0], [0, -1], [1, 1], [-1, -1], [1, -1], [-1, 1], [0, 1]]

v1 = np.array(direction_near[1])
v2 = np.array([-1,-1])
v1_u = v1 / np.linalg.norm(v1)
v2_u = v2 / np.linalg.norm(v2)
theta = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
print(theta)

