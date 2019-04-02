clear all
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
poseSubscriber = rossubscriber('/ual/pose');  
msg = poseSubscriber.receive(1);          
display(msg.Pose.Position);

%% Initialize State_start
Xo=msg.Pose.Position.X
Yo=msg.Pose.Position.Y
Zo=msg.Pose.Position.Z

%% Initialize Problem Variables
num_axes         = 3;
num_trajectories = 1; % 2 number of waypoints
State_start      = [Xo 0 0; Yo 0 0; Zo 0 0];
X1= 1.0;
Y1= 3.0;
Z1= 3.0;
X2= 3.0;
Y2= 3.0;
Z2= 3.0;
Waypoints(:,:,1) = [ 10.0  0.0  0.0  0.0  0.0; 8.0  0.0  0.0  0.0  0.0; 12.0 0.0 0.0 0.0 0.0];
% Waypoints(:,:,2) = [ 3.0 0.0  0.0  0.0  0.0; 3.0  0.0  0.0  0.0  0.0; 3.0 0.0  0.0  0.0  0.0];
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

ts_rollout = 0.2; %%CAMBIATA  %0.01
T_rollout = max(sum(T_waypoints,2));
[P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

% % Dispaly 3D
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
b= size(P(1).signals.values);
n_v_traj=b(1,1);

j=1;
% k=1;



S_x=zeros(j,1);
S_y=zeros(j,1);
S_z=zeros(j,1);

E_x= zeros(j,1); 
E_y= zeros(j,1);
E_z= zeros(j,1);

P_traj_x=zeros(j,1);
P_traj_y=zeros(j,1);
P_traj_z=zeros(j,1);



i=1;
m=200;



while i < n_v_traj 
   
    tic
    msg_v.Twist.Linear.X= V(1).signals.values(i,1); 
    msg_v.Twist.Linear.Y = V(2).signals.values(i,1);
    msg_v.Twist.Linear.Z = V(3).signals.values(i,1);   
    send(speedPublisher, msg_v)
    
     %Store_complete trajectory
    P_traj_x(j,1)=P(1).signals.values(i,1);
    P_traj_y(j,1)=P(2).signals.values(i,1);
    P_traj_z(j,1)=P(3).signals.values(i,1); 
    
    
    %Subscriber position
    msg = poseSubscriber.receive(1);
    S_x(j,1) = msg.Pose.Position.X;
    S_y(j,1) = msg.Pose.Position.Y;
    S_z(j,1) = msg.Pose.Position.Z;
    
    
   %calcolo errore
    E_x= abs(P_traj_x(j,1)-S_x(j,1));
    E_y= abs(P_traj_y(j,1)-S_y(j,1));
    E_z= abs(P_traj_z(j,1)-S_z(j,1));

    tsim=toc; 
     
      
    if (E_x+E_y+E_z) < 0.5  
      incT = 0.2 -tsim;
            if incT > 0.0
            pause(incT)
            end
     
    else 
%         if i> m 
%             break 
%         end
%         if [abs(S_x(j,1)-Waypoints(1,1)); abs(S_y(j,1)-Waypoints(2,1)); abs(S_z(j,1)-Waypoints(3,1))] < [ 3; 3 ; 3]
%            break
%        end 
%         
       State_start = [S_x(j,1) V(1).signals.values(i,1) 0.0; ... 
                      S_y(j,1) V(2).signals.values(i,1) 0.0; ...
                      S_z(j,1) V(3).signals.values(i,1) 0.0];
                  
       display(strcat("replaning in iteration: ",string(i)))
       [J_setp_struct,solution_out,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);                                                              
       [P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);
                        
       %sovrascrivo i valori di i e n_v_traj
           c= size(P(1).signals.values);
           n_v_traj=c(1,1);
           i=0;  %prova con i=1;
          
           
    end
       
   i=i+1;
   j=j+1;
   
end

% n=size(S_x);
% m=n(1,1)
% State_start=[S_x(m,1) 0.0  0.0; S_y(m,1) 0.0  0.0; S_z(m,1) 0.0  0.0  ];
%   
% [J_setp_struct,solution_out,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);                                                              
% [P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);
% c= size(P(1).signals.values);
%              n_v_traj=c(1,1);
%            
% while k <= n_v_traj 
%    
%     
%     msg_v.Twist.Linear.X= V(1).signals.values(k,1); 
%     msg_v.Twist.Linear.Y = V(2).signals.values(k,1);
%     msg_v.Twist.Linear.Z = V(3).signals.values(k,1);   
%     send(speedPublisher, msg_v)
%     
%      
%    k=k+1;
%  
% end

 % Dispaly 3D
plot3(P_traj_x, P_traj_y, P_traj_z,'b')
hold on 
plot3(Xo,Yo,Zo,'o')
% plot3(X1,Y1,Z1,'*') %primo waypoint
hold on 
plot3(X2,Y2,Z2,'x') %secondo waypoint (unico in questo caso)
hold on
plot3(S_x,S_y,S_z,'r');
grid
legend('trajectory','initial pose','waypoint','real pose')
hold on 
plot3(P(1).signals.values,P(2).signals.values,P(3).signals.values)
 

figure
subplot(3,1,1)
plot(P_traj_x, 'b')
hold on
plot(S_x, 'r')
hold on 
% plot(State_start_1(1,1),'*')
grid
legend('Trajectory_x','Real pose_x','last state start')

subplot(3,1,2)
plot(P_traj_y,'b')
hold on
plot(S_y, 'r');
hold on 
% plot(State_start_1(2,1),'*')
grid
legend('Trajectory_y','Real pose_y','last state start')

subplot(3,1,3)
plot(P_traj_z, 'b')
hold on
plot(S_z,'r');
hold on 
% plot(State_start_1(3,1),'*')
grid
legend('Trajectory_z','Real pose_z','last state start')
% LANDING
% 
% %clcolo errore tic toc