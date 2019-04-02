%RIPORTA
clearvars;
%% Read pose
% Subscribe to pose
poseSubscriber = rossubscriber('/ual/pose');  
msg = poseSubscriber.receive(1);          
display(msg.Pose.Position);

%% Initialize State_start
Xo=msg.Pose.Position.X
Yo=msg.Pose.Position.Y
Zo=msg.Pose.Position.Z

%% Initialize Problem Variables
num_axes         = 3;
num_trajectories = 1; %number of waypoints
State_start      = [Xo 0 0; Yo 0 0; Zo 0 0];
X1= 1.0;
Y1= 3.0;
Z1= 3.0;
X2= 0.0;
Y2= 0.0;
Z2= 0.0;
Waypoints(:,:,1) = [ 0.0  0.0  0.0  0.0  0.0; 0.0  0.0  0.0  0.0  0.0; 0.0 0.0 0.0 0.0 0.0];
% Waypoints(:,:,2) = [ 0.0 0.0  0.0  0.0  0.0; 0.0  0.0  0.0  0.0  0.0; 0.0 0.0  0.0  0.0  0.0];
        V_max            =  1.0*ones(num_axes,num_trajectories)
        V_min            = -1.0*ones(num_axes,num_trajectories)
        A_max            =  0.5*ones(num_axes,num_trajectories)
        A_min            = -0.5*ones(num_axes,num_trajectories)
        J_max            =  1.0*ones(num_axes,num_trajectories)
        J_min            = -1.0*ones(num_axes,num_trajectories)
        A_global         =  0.0*ones(num_axes,1)
       

%Boolean variables 
b_comp_global    = false;
b_sync_V         =  true(num_axes,num_trajectories);
b_sync_A         =  true(num_axes,num_trajectories);
b_sync_J         = false(num_axes,num_trajectories);
b_sync_W         =  true(num_axes,num_trajectories);
b_rotate         = false(1,num_trajectories);
b_best_solution  =  true(num_axes,num_trajectories);
b_hard_vel_limit = false(num_axes,num_trajectories);
b_catch_up       =  true(num_axes,num_trajectories);


% %% ----------   Calculate    ----------
solution_in  = -1 * ones(num_axes,2,num_trajectories,'int8');

tic;
[J_setp_struct,solution_out,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);                                                              
toc;



% %% ----------     Output     ----------
disp(['num_axes = ',num2str(num_axes)]);
disp(['num_trajectories = ',num2str(num_trajectories)]);

ts_rollout = 0.04; %%CAMBIATA  %0.01
T_rollout = max(sum(T_waypoints,2));
[P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

% %% Dispaly 3D
% plot3(P(1).signals.values,P(2).signals.values,P(3).signals.values)
% hold on 
% plot3(Xo,Yo,Zo,'o')
% plot3(X1,Y1,Z1,'*') %primo waypoint
% plot3(X2,Y2,Z2,'x') %secondo waypoint
% hold on


%% SEND VELOCITY TO SUBSCRIBER
speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
msg_v = rosmessage(speedPublisher);
%calcolo n_p_trajectory, cioè il numero dei punti in cui sto discretizzando
% la traiettoria
b= size(V(1).signals.values);
n_v_traj=b(1,1);

S_x=zeros(n_v_traj,1);
S_y=zeros(n_v_traj,1);
S_z=zeros(n_v_traj,1);

for i=1:n_v_traj 
   tic
msg_v.Twist.Linear.X= V(1).signals.values(i,1);
msg_v.Twist.Linear.Y = V(2).signals.values(i,1);
msg_v.Twist.Linear.Z = V(3).signals.values(i,1);   
send(speedPublisher, msg_v)
% tsim1=toc;
msg = poseSubscriber.receive(1);
S_x(i,1) = msg.Pose.Position.X;
S_y(i,1) = msg.Pose.Position.Y;
S_z(i,1) = msg.Pose.Position.Z;
tsim=toc;
incT = 0.04 -tsim;
if incT > 0.0
pause(incT)
% pause(incT + tsim1 + 0.0064)
end
end

% plot3(S_x,S_y,S_z);
% 
% t= P(1).time;
%  figure
% subplot(3,1,1)
% plot(t,P(1).signals.values)
% hold on
% plot(t,S_x)
% 
% subplot(3,1,2)
% plot(t,P(2).signals.values)
% hold on
% plot(t,S_y);
% 
% subplot(3,1,3)
% plot(t,P(3).signals.values)
% hold on
% plot(t,S_z);
LANDING