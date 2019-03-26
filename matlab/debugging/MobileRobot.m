classdef MobileRobot < handle
      properties(Access = public)
          pose;
          speed;
          
          poseSubscriber;
          speedSubscriber;
          
          speedPublisher;
          
          trajectory_pos = [];
          trajectory_speed = [];
      end
    methods(Access = public)
        function obj = MobileRobot()  % Constructor   
            obj.poseSubscriber = rossubscriber('/ual/pose', @obj.callbackPose);  
            obj.speedSubscriber = rossubscriber('/ual/velocity', @obj.callbackSpeed);  
            obj.speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
        end
        
        function takeoffResp = takeoff(obj, altitude)
            clientTakeoff = rossvcclient('/ual/take_off');  
            takeoffReq = rosmessage(clientTakeoff); 
            takeoffReq.Blocking = 1;
            takeoffReq.Height = altitude;
            takeoffResp = call(clientTakeoff,takeoffReq);
            pause(3)
        end
        
        function landResp = land(obj)
            clientLand = rossvcclient('/ual/land');  
            landReq = rosmessage(clientLand); 
            landReq.Blocking = 1;
            landResp = call(clientLand,landReq);
            pause(3)
        end
        
        function  callbackPose(obj, src, msg) 
            obj.pose = msg.Pose.Position;
        end
        
        function  callbackSpeed(obj, src, msg) 
            obj.speed = msg.Twist.Linear;
        end
        
        function computeTrajectory(obj, waypoint)
            num_axes         = 3;
            num_trajectories = 1; %number of waypoints
            State_start      = [obj.pose.X 0 0; obj.pose.Y 0 0; obj.pose.Z 0 0];
            Waypoints(:,:,1) = [    waypoint(1)  0.0  0.0  0.0  0.0; ...
                                    waypoint(2)  0.0  0.0  0.0  0.0; ...
                                    waypoint(3)  0.0 0.0 0.0 0.0];
            %Waypoints(:,:,2) = [ 0.0 0.0  0.0  0.0  0.0; 0.0  0.0  0.0  0.0  0.0; 0.0 0.0  0.0  0.0  0.0];
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

            obj.trajectory_pos = P;
            obj.trajectory_speed = V;
        end
        
        function moveToWaypoint(obj, waypoint)
            computeTrajectory(obj, waypoint);
            
            msg = rosmessage(obj.speedPublisher);
            %calcolo n_p_trajectory, cioè il numero dei punti in cui sto discretizzando
            % la traiettoria
            V = obj.trajectory_speed;
            P = obj.trajectory_pos;
            b= size(V(1).signals.values);
            n_v_traj=b(1,1);
            for i=1:n_v_traj 
                msg.Twist.Linear.X= V(1).signals.values(i,1);
                msg.Twist.Linear.Y = V(2).signals.values(i,1);
                msg.Twist.Linear.Z = V(3).signals.values(i,1);
                % msg.Pose.Orientation.W = 1;    
                dist_vector =   [obj.pose.X obj.pose.Y obj.pose.Z] - ... 
                                [P(1).signals.values(i,1) P(2).signals.values(i,1) P(3).signals.values(i,1)]
                if norm(dist_vector) > 0.1
                   display('error'); 
                end
                send(obj.speedPublisher, msg)
                pause(0.01)
            end
            
        end
    end
end