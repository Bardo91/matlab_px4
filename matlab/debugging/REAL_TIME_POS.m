clearvars;
% prova ROS + OPT_CONTROL
% Take off
% COMMENTA QUESTA SEZIONE IN MODALITA' ONLINE
% Init service
clientTakeoff = rossvcclient('/ual/take_off');  
% Get empty message of the right type
takeoffReq = rosmessage(clientTakeoff); 

% Set values
takeoffReq.Blocking = 1;
takeoffReq.Height = 2.0;

% Call client
takeoffResp = call(clientTakeoff,takeoffReq);
pause(3)

%% Read pose
% Subscribe to pose
figure(1)
callbackPose = @(topic, msg) plot3(msg.Pose.Position.X, msg.Pose.Position.Y, msg.Pose.Position.Z, 'ob')
poseSubscriber = rossubscriber('/ual/pose', callbackPose);  
msg = poseSubscriber.receive(1);          
display(msg.Pose.Position);

%% Initialize State_start
Xo=msg.Pose.Position.X
Yo=msg.Pose.Position.Y
Zo=msg.Pose.Position.Z

%% Initialize Problem Variables
num_axes         = 3;
num_trajectories = 2; %number of waypoints
State_start      = [Xo 0 0; Yo 0 0; Zo 0 0];
X1= 1.0;
Y1= 3.0;
Z1= 3.0;
X2= 0.0;
Y2= 0.0;
Z2= 0.0;
Waypoints(:,:,1) = [ 1.0  0.0  0.0  0.0  0.0; 3.0  0.0  0.0  0.0  0.0; 3.0 0.0 0.0 0.0 0.0];
Waypoints(:,:,2) = [ 0.0 0.0  0.0  0.0  0.0; 0.0  0.0  0.0  0.0  0.0; 0.0 0.0  0.0  0.0  0.0];
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

ts_rollout = 0.01;
T_rollout = max(sum(T_waypoints,2));
[P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

%% Dispaly 3D
figure(1)
plot3(P(1).signals.values,P(2).signals.values,P(3).signals.values)
hold on 
plot3(Xo,Yo,Zo,'o')
plot3(X1,Y1,Z1,'*') %primo waypoint
plot3(X2,Y2,Z2,'x') %secondo waypoint
hold on



%% SEND POSITION TO SUBSCRIBER  
% %ual_position 
% posePublisher = rospublisher('/ual/set_pose','geometry_msgs/PoseStamped');
% msg = rosmessage(posePublisher);
% 
% %calcolo n_p_trajectory, cioè il numero dei punti in cui sto discretizzando
% % la traiettoria
% c= size(P(1).signals.values);
% n_p_traj=c(1,1);
% S_x=zeros(n_p_traj,1);
% S_y=zeros(n_p_traj,1);
% S_z=zeros(n_p_traj,1);
% % 
% for i=1:n_p_traj 
% msg.Pose.Position.X = P(1).signals.values(i,1);
% msg.Pose.Position.Y = P(2).signals.values(i,1);
% msg.Pose.Position.Z = P(3).signals.values(i,1);
% msg.Pose.Orientation.W = 1;    
% send(posePublisher, msg)
% msg = poseSubscriber.receive(1);
% S_x(i,1) = msg.Pose.Position.X;
% S_y(i,1) = msg.Pose.Position.Y;
% S_z(i,1) = msg.Pose.Position.Z;
% plot3(S_x(i,1),S_y(i,1),S_z(i,1),'.');
% pause(0.03)
% end
% 
% % plot3(S_x,S_y,S_z)
% 
%% SEND VELOCITY TO SUBSCRIBER
speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
msg = rosmessage(speedPublisher);
%calcolo n_p_trajectory, cioè il numero dei punti in cui sto discretizzando
% la traiettoria
b= size(V(1).signals.values);
n_v_traj=b(1,1);

for i=1:n_v_traj 
    msg.Twist.Linear.X= V(1).signals.values(i,1);
    msg.Twist.Linear.Y = V(2).signals.values(i,1);
    msg.Twist.Linear.Z = V(3).signals.values(i,1);
    % msg.Pose.Orientation.W = 1;    
    send(speedPublisher, msg)
    pause(0.01)
end

% % % % % Evalueate current error
% % e_x= ( P(1).signals.values - S_x);
% % e_y= ( P(2).signals.values - S_y);
% % e_z= ( P(1).signals.values - S_z);
% % 
% % plot with time 
% % t= P(1).time 
% % ref_0= zeros(n_p_traj,1)
% % 
% % nel plot aggiungi anche riferimento dellerrore a zero!
% % figure
% % subplot(3,1,1)
% % plot(t,e_x)
% % hold on
% % plot(t,ref_0);
% % 
% % subplot(3,1,2)
% % plot(t, e_y)
% % hold on
% % plot(t,ref_0);
% % 
% % subplot(3,1,3)
% % plot(t,e_z)
% % hold on
% % plot(t,ref_0);


clientLand = rossvcclient('/ual/land');  
% Get empty message of the right type
landReq = rosmessage(clientLand); 

% Set values
landReq.Blocking = 1;

% Call client
landResp = call(clientLand,landReq);
pause(3)
