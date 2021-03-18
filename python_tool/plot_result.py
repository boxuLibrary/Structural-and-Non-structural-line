import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
fig1 = plt.figure()
fig2 = plt.figure()

ax = fig.gca(projection='3d')
ax1 = fig1.gca(projection='3d')
ax2 = fig2.gca(projection='3d')


coordinate_plucker = []
coordinate_line = []
coordinate_true = []

with open("../bin/res/res_plucker.txt", "r") as data:
    for line in data.readlines():
        line = line.strip('\n').split(" ")
        coordinate_plucker.append(line)

with open("../bin/res/res_ours.txt", "r") as data2:
    for line in data2.readlines():
        line = line.strip('\n').split(" ")
        coordinate_line.append(line)        

with open("../bin/house_model/house.txt", "r") as data1:
    for line in data1.readlines():
        line = line.strip('\n').split(" ")
        coordinate_true.append(line)




for data in coordinate_plucker:
     ax.plot([float(data[0]), float(data[3])], [float(data[1]), float(data[4])],'g' ,zs=[float(data[2]), float(data[5])])



for data1 in coordinate_true:
    ax2.plot([float(data1[0]), float(data1[3])], [float(data1[1]), float(data1[4])],'r' ,zs=[float(data1[2]), float(data1[5])])

for data2 in coordinate_line:
    ax1.plot([float(data2[0]), float(data2[3])], [float(data2[1]), float(data2[4])],'b' ,zs=[float(data2[2]), float(data2[5])])    
 
plt.show()
