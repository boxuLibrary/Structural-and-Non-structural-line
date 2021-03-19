import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.gca(projection='3d')

lines = []
with open("../bin/house_model/house.txt", "r") as data:
    for line in data.readlines():
        line = line.strip('\n').split(" ")
        lines.append(line)


position = []
tx_index = 5
position = np.loadtxt("../bin/cam_pose.txt", usecols = (tx_index, tx_index + 1, tx_index + 2))


for data in lines:
     ax.plot([float(data[0]), float(data[3])], [float(data[1]), float(data[4])],'g' ,zs=[float(data[2]), float(data[5])])

ax.plot(position[:,0], position[:,1], position[:,2], label='gt')
ax.plot([position[0,0]], [position[0,1]], [position[0,2]], 'r.', label='start')

    
plt.show()
