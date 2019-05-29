%% Initialize Problem Variables
num_axes         = 3;
num_trajectories = 2; %number of waypoints
State_start      = [0 0 0; 0 0 0; 0 0 0];
X1= 3.0;
Y1= 5.0;
Z1= 1.0;
X2= 6.0;
Y2= 7.0;
Z2= 2.0;
Waypoints(:,:,1) = [ 3.0  0.0  0.0  0.0  0.0; 5.0  0.0  0.0  0.0  0.0; 1.0 0.0 0.0 0.0 0.0];
Waypoints(:,:,2) = [ 6.0 0.0  0.0  0.0  0.0; 7.0  0.0  0.0  0.0  0.0; 2.0 0.0  0.0  0.0  0.0];


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
b_sync_J         =  true(num_axes,num_trajectories);
b_sync_W         =  false(num_axes,num_trajectories);
b_rotate         =  false(1,num_trajectories);
b_best_solution  =  true(num_axes,num_trajectories);
b_hard_vel_limit =  false(num_axes,num_trajectories);
b_catch_up       =  false(num_axes,num_trajectories);


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
plot3(P(1).signals.values,P(2).signals.values,P(3).signals.values,'b')
hold on 
plot3(0,0,0,'o')
plot3(X1,Y1,Z1,'*') %primo waypoint
plot3(X2,Y2,Z2,'x') %secondo waypoint
hold on
hold on 
xlabel('x')
hold on 
ylabel('y')
hold on 
zlabel('z')
hold on 
legend('optimal trajectory')
hold on 
grid on 
hold on 
legend('first waypoint')
hold on 
legend('optimal trajectory','State start', 'First Waypoint', 'Second Waypoints')

