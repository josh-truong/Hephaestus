import numpy as np
import matplotlib.pyplot as plt

map = np.load('filter_map.npy')
path = np.load('path.npy')

plt.imshow(map)
plt.scatter(path[:,0],path[:,1], s=0.3)
plt.show()