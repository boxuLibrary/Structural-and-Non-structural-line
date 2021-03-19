# [PLS-VIO] Comparison of 4-parameter orthogonal expression and 2-parameter expression proposed by our paper of line features.
This repository is supplementary material that provides the simulation code for running and comparison results. After we generate the simulated straight line observations, we initialize them to obtain the straight line in the three-dimensional space and optimize the  line landmarks. The code is the initial version, and it will be improved further in the comming months.

<div align=center><img width="500" height="300" src="https://github.com/xubogithub/Structural-and-Non-structural-line/blob/master/bin/demo/simulate_line.gif"/></div>

<div align=center><img width="640" height="340" src="https://github.com/xubogithub/Structural-and-Non-structural-line/blob/master/bin/demo/simulate_line.png"/></div>



We compared the number of consuming time and accuracy of optimization, as the line observation noise (pixels) in the image increases. The accuracy is obtained by comparing the reconstructed straight line with the groundtruth of the 3D line landmark.

**</center> comparison of optimization consuming time[s] in different pixel noise level[pixel] </center>**  
 -|0 |1.5|3|4.5|
  :-:|:-:|:-:|:-:|:-:|
  2-parameter|20.54|20.34|20.38|20.66
  4-parameter|40.89|41.00|40.75|40.49
    
**</center> comparison of accuracy[cm] in different pixel noise level[pixel] </center>**
-|0|1.5|3|4.5
:-:|:-:|:-:|:-:|:-:
 2-parameter|0.06|7.93|14.99|20.63
4-parameter|0.05|5.34|10.65|16.17

## 1. Prerequisites
1.1 **Ubuntu** and **python**

* Ubuntu 16.04 or Ubuntu18.04
* python3.

1.2. **Dependency**

* C++14 or C++17 Compiler 
* Eigen 3.3.7
* OpenCV 3.3.9 
* Cere-solver 2.0.0: [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

## 2. Build Project with Cmake
Clone the repository and compile the project:
```
git clone https://github.com/xubogithub/Structural-and-Non-structural-line.git
cd ~/Structural-and-Non-structural-line-master/
mkdir build
cd build
cmake ..
make -j4
```
## 3. run program
**Notice**: The executable file **line_optimization** is in the bin directory, and you can run it by **./line_optlimization**

```
cd ../bin
./line_optlimization
```
## 4. Python Tools for Visualization
We provide some visualizaton tools for comparison and analysis.
Visual simulation animation:
```
python draw_points.py
```
Visual simulation trajectory and line landmarks:
```
python plot_simulate.py
```
Visual results of the code:
```
python plot_result.py
```
## 5. Related Papers

- **Leveraging Structural Information to Improve Point Line Visual-Inertial Odometry**.
```
The paper is received.
```

We use [vio_data_simulation ](https://github.com/HeYijia/vio_data_simulation.git) as our base line code and generate the simulation data.

## 6. Acknowledgements

Thank Dr. Yijia He very much. We are still working on improving the code reliability. For any technical issues, please contact Bo Xu boxu1995@whu.edu.cn and Peng Wang cypminxuan@163.com.

## 7. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
