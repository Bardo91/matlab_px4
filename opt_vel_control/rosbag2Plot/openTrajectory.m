clear all
clc
%%read rosbag trajectory
T = readtable('computed_trajectory.txt');
A=table2array(T);
[r,c]=size(A);
x_traj=A(1,3:6:c);
y_traj=A(1,4:6:c);
z_traj=A(1,5:6:c);