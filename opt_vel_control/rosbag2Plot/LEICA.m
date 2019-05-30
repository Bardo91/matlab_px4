%%LEICA
load('/home/emanuela/Scrivania/map_karting_pipes.mat')
pcshow(ptCloud)
%inicial point 
hold on 
plot3(0,0,0,'s')

hold on
% real pose
rosbag_plot
hold on 

% first trajectory
%%read rosbag trajectory
T = readtable('computed_trajectory.txt');
A=table2array(T);
[r,c]=size(A);
x_traj=A(1,3:6:c);
y_traj=A(1,4:6:c);
z_traj=A(1,5:6:c);

plot3(x_traj,y_traj,z_traj,'r')
hold on 

% % wypoints 
% W = readtable('waypoints.txt');
% P = table2array(W);


hold on 

% altre traiettorie replanificate
%il NAN NON DA PROBLEMA COL PLOT

for  i=2:r
    x_traj=A(i,3:6:c);
    y_traj=A(i,4:6:c);
    z_traj=A(i,5:6:c);
   
    plot3(x_traj,y_traj,z_traj,'r')
    hold on 
end