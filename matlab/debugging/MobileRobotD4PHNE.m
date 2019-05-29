classdef MobileRobotD4PHNE < handle
    properties(Access = public)
        
        % subscribers
        poseReadSubscriber;
        poseWpSubscriber;
        poseReplanSubscriber;
        speedSubscriber;
        % Publisher
        speedPublisher;
        computedTrajectoryPublisher;
        waypointsPublisher
        
        % Messages
        speed_msg;
        trajectory_msg;
        waypoints_msg
        
        % Parameters
        pose;
        speed;
        
        % waypoints
        waypoints = [];
        waypoints_vel=[];
        waypoints_ac=[];
        trajectory_pos = [];
        trajectory_speed = [];
        N_traj=[];
        nwps=[];
        currentWaypoint=[];
        
        flying = false
        % Debug variables
        wp_n_imo=0;
        replan_index=0;
        counter_step=1;
        speed_index=1;s
        error=[];
        plotting=0;
        
        magic=0.040; %0.020
        ts_rolloutMagic = 0.08;%0.04
        error_delete_waypoint = 0.4;
        max_error = 1.0;
        
%         axes = figure();
        
    end
    
    methods(Access = public)
        
        function obj = MobileRobotD4PHNE()  % Constructor
            obj.poseReadSubscriber = rossubscriber('/ual/pose', @obj.callbackReadPose);
            obj.poseWpSubscriber = rossubscriber('/ual/pose', @obj.callbackWpPose);
            obj.poseReplanSubscriber = rossubscriber('/ual/pose', @obj.callbackReplanPose);
            obj.speedSubscriber = rossubscriber('/ual/velocity', @obj.callbackSpeed);
            obj.speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
            obj.speed_msg = rosmessage(obj.speedPublisher);
            obj.computedTrajectoryPublisher = rospublisher('/emanuela/computed_trajectory','std_msgs/Float64MultiArray');
            obj.trajectory_msg = rosmessage(obj.computedTrajectoryPublisher);
            obj.waypointsPublisher = rospublisher('/emanuela/waypoints','std_msgs/Float64MultiArray');
            obj.waypoints_msg = rosmessage(obj.waypointsPublisher);
        end
        
        function takeoffResp = takeoff(obj, altitude)
            clientTakeoff = rossvcclient('/ual/take_off');
            takeoffReq = rosmessage(clientTakeoff);
            takeoffReq.Blocking = 1;
            takeoffReq.Height = altitude;
            takeoffResp = call(clientTakeoff,takeoffReq);
            pause(3)
            obj.flying = true;
            disp('take off end')
        end
        
        function landResp = land(obj)
            clientLand = rossvcclient('/ual/land');
            landReq = rosmessage(clientLand);
            landReq.Blocking = 1;
            landResp = call(clientLand,landReq);
            pause(3)
        end
        
        function  callbackReadPose(obj, ~, msg)
            obj.pose = msg.Pose.Position;
            if(obj.plotting)
%                 figure(1)
                %subplot(2,1,1)
                plot3(obj.pose.X, obj.pose.Y, obj.pose.Z,'.')
                hold on
            end
        end
        
        function  callbackWpPose(obj, ~, msg)
            pose = msg.Pose.Position;
            [~ , obj.nwps]=size(obj.waypoints);

%            plot3(obj.waypoints(1,1), obj.waypoints(2,1), obj.waypoints(3,1), 'o')
%            hold on
%            plot3(obj.waypoints(1,2), obj.waypoints(2,2), obj.waypoints(3,2), 'o')
%            hold on
%            plot3(obj.waypoints(1,3), obj.waypoints(2,3), obj.waypoints(3,3), 'o')
%            hold on
            
%             for i=1:obj.nwps
%                 dist_wayps = [obj.pose.X obj.pose.Y obj.pose.Z] - ...
%                     [ obj.waypoints(1,i) obj.waypoints(2,i) obj.waypoints(3,i) ];
%                 
%                 if norm(dist_wayps) < 0.6 %0.6
%                     obj.wp_n_imo=obj.wp_n_imo+1;
%                     disp(strcat('---------------->Deleted waypoint number=', num2str(obj.wp_n_imo)))
%                     
%                     obj.waypoints = obj.waypoints(:,i+1:obj.nwps);
%                     obj.waypoints_vel= obj.waypoints_vel(:,i+1:obj.nwps);
%                     obj.waypoints_ac= obj.waypoints_ac(:,i+1:obj.nwps);
%                     
%                     break %added
%                 end
%             end

            if size(obj.currentWaypoint,2) > 0
                dist_wayps = [pose.X pose.Y pose.Z] - ...
                             [ obj.currentWaypoint(1) obj.currentWaypoint(2) obj.currentWaypoint(3) ];
                if norm(dist_wayps) < obj.error_delete_waypoint  %0.6
                    obj.wp_n_imo=obj.wp_n_imo+1;
                    disp(strcat('------------------->Deleted waypoint number=', num2str(obj.wp_n_imo)))
                    currentWaypointNumber = size(obj.waypoints(1,:),2);
                    obj.waypoints = obj.waypoints(:,2:currentWaypointNumber);
                    obj.waypoints_vel= obj.waypoints_vel(:,2:currentWaypointNumber);
                    obj.waypoints_ac= obj.waypoints_ac(:,2:currentWaypointNumber);
                    obj.currentWaypoint = obj.waypoints(:,1);
                    computeTrajectory(obj);
                end
            end
        end
        
        
        function  callbackSpeed(obj, ~, msg)
            obj.speed = msg.Twist.Linear;
        end
        
        
        function computeTrajectory(obj)
            num_axes         = 3;
            [~ , obj.nwps ]=size(obj.waypoints);
            num_trajectories = obj.nwps; %number of waypoint
            
            State_start      = [obj.pose.X obj.speed.X 0;
                obj.pose.Y obj.speed.Y 0;
                obj.pose.Z obj.speed.Z 0];
            
            Waypoints=[];
            
            for i=1:obj.nwps
                
                Waypoints(:,:,i) = [    obj.waypoints(1,i)  obj.waypoints_vel(1,i)  obj.waypoints_ac(1,i)  0.0  0.0; ...
                    obj.waypoints(2,i)  obj.waypoints_vel(2,i)  obj.waypoints_ac(1,i)  0.0  0.0; ...
                    obj.waypoints(3,i)  obj.waypoints_vel(3,i)  obj.waypoints_ac(1,i)  0.0  0.0];
            end
            
            V_max            =  1.0*ones(num_axes,num_trajectories); %1.0
            V_min            = -1.0*ones(num_axes,num_trajectories);
            A_max            =  0.5*ones(num_axes,num_trajectories); %0.5
            A_min            = -0.5*ones(num_axes,num_trajectories);
            J_max            =  1.0*ones(num_axes,num_trajectories);  %1.0
            J_min            = -1.0*ones(num_axes,num_trajectories);
            A_global         =  0.0*ones(num_axes,1);
            %Boolean variables
            b_comp_global    = false;
            b_sync_V         =  true(num_axes,num_trajectories);
            b_sync_A         =  true(num_axes,num_trajectories);
            b_sync_J         = false(num_axes,num_trajectories);%true
            b_sync_W         = true(num_axes,num_trajectories); %false
            b_rotate         = false(1,num_trajectories); %false
            b_best_solution  =  true(num_axes,num_trajectories);
            b_hard_vel_limit = true(num_axes,num_trajectories); %false
            b_catch_up       =  true(num_axes,num_trajectories); %true
            
            % %% ----------   Calculate    ----------
            solution_in  = -1*ones(num_axes,2,num_trajectories,'int8');
            
            tic;
            [J_setp_struct,~,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);
            toc;
            
            % %% ----------     Output     ----------
            %             disp(['num_axes = ',num2str(num_axes)]);
            %             disp(['num_trajectories = ',num2str(num_trajectories)]);
            
            ts_rollout = obj.ts_rolloutMagic; %0.06
            T_rollout = max(sum(T_waypoints,2));
            [P,V,~,~] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);
            
            obj.trajectory_pos = P;
            obj.trajectory_speed = V;
            b= size(V(1).signals.values);
            
            obj.N_traj=b(1,1);
        end
        
        function  callbackReplanPose(obj,~, msg)
            if obj.flying == true && obj.nwps > 0
                
                pose_r = msg.Pose.Position;
                
                %calcola l'errore
                obj.error(obj.speed_index,:) = [pose_r.X pose_r.Y pose_r.Z] - ...
                    [obj.trajectory_pos(1).signals.values(obj.speed_index,1) obj.trajectory_pos(2).signals.values(obj.speed_index,1) obj.trajectory_pos(3).signals.values(obj.speed_index,1)];
                
                
                if norm(obj.error(obj.speed_index,:)) > obj.max_error % && obj.nwps > 0 %&& obj.dist_waypoint<0.5
                    obj.replan_index=obj.replan_index+1;
                    computeTrajectory(obj)
                    % publish replan trajectory <---------------
                    sendTrajectory(obj);
                    
                    disp(strcat('computed trajectory number=',num2str(obj.replan_index )))
                    
                    obj.speed_index = 1;
                    
                end
            end
        end
        
        
        function moveToWaypoint(obj, waypoints,waypoints_vel,waypoints_ac)
  
            obj.waypoints = waypoints;
            obj.waypoints_vel = waypoints_vel;
            obj.waypoints_ac = waypoints_ac;
            obj.currentWaypoint = waypoints(:,1);
            
            if(obj.plotting)
                for i = 1:size(obj.waypoints(1,:),2)
                    plot3(obj.waypoints(1,i), obj.waypoints(2,i), obj.waypoints(3,i), 'o')
                    hold on
                end
            end
            % publish all waypoints
            sendWaypoints(obj);
            
            %           COMPUTE TRAJECTORY
            computeTrajectory(obj);
            
            % publish first computed trajectory <--------
            sendTrajectory(obj);
            
            %           PLOT OF THE FIRST TRAJECTORY => VERIFICA FUNZIONAMENTO
            if(obj.plotting)
                      plot3(obj.trajectory_pos(1).signals.values,...
                             obj.trajectory_pos(2).signals.values,...
                             obj.trajectory_pos(3).signals.values);
                       hold on
            end
            
            while obj.speed_index < obj.N_traj && obj.nwps>0
                tic
                obj.speed_msg.Twist.Linear.X= obj.trajectory_speed(1).signals.values(obj.speed_index,1);
                obj.speed_msg.Twist.Linear.Y = obj.trajectory_speed(2).signals.values(obj.speed_index,1);
                obj.speed_msg.Twist.Linear.Z = obj.trajectory_speed(3).signals.values(obj.speed_index,1);
                
                send(obj.speedPublisher, obj.speed_msg)
                obj.speed_index = obj.speed_index+1;
                %                disp('speed message')
                inct=toc;
                if(inct<obj.ts_rolloutMagic)
                    %java.lang.Thread.sleep((obj.magic-inct)/1000)
                    pause(obj.magic-inct) %pause(0.020 - inct) %0.009 con Ts_roll=0.03
                end
                
            end
        end
        
        function sendWaypoints(obj)
            disp('Sending waypoints')
            waypointNumber = size(obj.waypoints,2)
            waypointMat = [];
            waypointMat(1:9:waypointNumber*9) = obj.waypoints(1,:);
            waypointMat(2:9:waypointNumber*9) = obj.waypoints(2,:);
            waypointMat(3:9:waypointNumber*9) = obj.waypoints(3,:);
            waypointMat(4:9:waypointNumber*9) = obj.waypoints_vel(1,:);
            waypointMat(5:9:waypointNumber*9) = obj.waypoints_vel(2,:);
            waypointMat(6:9:waypointNumber*9) = obj.waypoints_vel(3,:);
            waypointMat(7:9:waypointNumber*9) = obj.waypoints_ac(1,:);
            waypointMat(8:9:waypointNumber*9) = obj.waypoints_ac(2,:);
            waypointMat(9:9:waypointNumber*9) = obj.waypoints_ac(3,:);
            obj.waypoints_msg.Data = waypointMat;
            send(obj.waypointsPublisher,obj.waypoints_msg);
        end
        
        function sendTrajectory(obj)
            obj.trajectory_msg.Data(1:6:length(obj.trajectory_pos(1).signals.values)*6) =  obj.trajectory_pos(1).signals.values;
            obj.trajectory_msg.Data(2:6:length(obj.trajectory_pos(2).signals.values)*6) =  obj.trajectory_pos(2).signals.values;
            obj.trajectory_msg.Data(3:6:length(obj.trajectory_pos(3).signals.values)*6) =  obj.trajectory_pos(3).signals.values;
            obj.trajectory_msg.Data(4:6:length(obj.trajectory_speed(1).signals.values)*6) =  obj.trajectory_speed(1).signals.values;
            obj.trajectory_msg.Data(5:6:length(obj.trajectory_speed(2).signals.values)*6) =  obj.trajectory_speed(2).signals.values;
            obj.trajectory_msg.Data(6:6:length(obj.trajectory_speed(3).signals.values)*6) =  obj.trajectory_speed(3).signals.values;
            send(obj.computedTrajectoryPublisher, obj.trajectory_msg);
        end
    end
end

