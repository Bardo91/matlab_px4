%%funzione che plotta .rosbag
% inserisci .rosbag
% bag = rosbag('/home/emanuela/Scrivania/rosbags/rosbag_22-05-19/2019-05-22-13-45-02_filtered.bag');
bag = rosbag('/home/emanuela/Scrivania/copia_filtro/2019-05-23-17-00-54_filtro2.bag');
poseSelect = select(bag, 'topic', '/ual/pose');
posemsg = readMessages(poseSelect);

[tam_pm, c]=size(posemsg);
pos_x=zeros(tam_pm,1);
pos_y=zeros(tam_pm,1);
pos_z=zeros(tam_pm,1);
for i=1:tam_pm
    pos_x(i,1)=posemsg{i,1}.Pose.Position.X;
    pos_y(i,1)=posemsg{i,1}.Pose.Position.Y;
    pos_z(i,1)=posemsg{i,1}.Pose.Position.Z;
end
%plot
figure(1)
plot3(pos_x,pos_y,pos_z,'b')
x_in=[pos_x(1,1),pos_y(1,1),pos_z(1,1)]; %pregunta = la traiettoria la calcola a partire da queste coordinate quindi?!
x_take_off=[pos_x(1,1),pos_y(1,1),1];
hold on 
plot3(pos_x(1,1),pos_y(1,1),pos_z(1,1),'*')
hold on 
axis equal
grid on 
% valuta prima traiettoria planificata 


% confronta i due risultati 









% legend('real traj','inicial point','2nd wp','expected state start','1st wp','real state start','compute traj')

hold on 

%compute trajectory 

% waypoints=[1 5;3 5.2;1.5 4];
% waypoints_vel=[0.5 0;0.5 0; 0.5 0];
% waypoints_ac=[0.1 0.0; 0.1 0.0; 0.1 0.0];
% state_start=[3.7;4.5;0.6]; %REAL state start 
% 
% plot3(5,5.2,4,'s')
% hold on 
% plot3(pos_x(1,1),pos_y(1,1),1,'o')
% hold on 
% plot3(1,3,1.5,'p')
% hold on 
% plot3(3.7,4.5,0.6,'h')
% grid on
% 
% computeTrajectory1(waypoints,waypoints_vel,waypoints_ac,state_start)

% legend('real traj','inicial point','2nd wp','expected state start','1st wp','real state start','compute traj')
