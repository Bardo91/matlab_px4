classdef MobileRobot3 < handle
        properties(Access = public)
            
            % subscribers
            poseReadSubscriber;
            poseWpSubscriber;
            poseReplanSubscriber;
            speedSubscriber;
            % Publisher
            speedPublisher;
            speed_msg;
            
            % Parameters
            pose;
            speed;
            waypoints = ([3 10 11; 3 15 10; 1 15 26]);
            trajectory_pos = [];
            trajectory_speed = [];
            N_traj=[];
            nwps=[];
            flying = false
            % Debug variables
            counter_step=1;
            speed_index=1;
            error=[];
            
        end
      
    methods(Access = public)
        
        function obj = MobileRobot3()  % Constructor   
            obj.poseReadSubscriber = rossubscriber('/ual/pose', @obj.callbackReadPose);  
            obj.poseWpSubscriber = rossubscriber('/ual/pose', @obj.callbackWpPose);  
            obj.poseReplanSubscriber = rossubscriber('/ual/pose', @obj.callbackReplanPose);  
            %obj.speedSubscriber = rossubscriber('/ual/velocity', @obj.callbackSpeed);  
            obj.speedPublisher = rospublisher('/ual/set_velocity','geometry_msgs/TwistStamped');
            obj.speed_msg = rosmessage(obj.speedPublisher);
        end
        
        function takeoffResp = takeoff(obj, altitude)
            clientTakeoff = rossvcclient('/ual/take_off');  
            takeoffReq = rosmessage(clientTakeoff); 
            takeoffReq.Blocking = 1;
            takeoffReq.Height = altitude;
            takeoffResp = call(clientTakeoff,takeoffReq);
            pause(3)
            obj.flying = true;
        end
        
        function landResp = land(obj)
            clientLand = rossvcclient('/ual/land');  
            landReq = rosmessage(clientLand); 
            landReq.Blocking = 1;
            landResp = call(clientLand,landReq);
            pause(3)
        end
        
        function  callbackReadPose(obj, src, msg) 
            obj.pose = msg.Pose.Position;
             figure(1)
%             subplot(2,1,1)
            plot3(obj.pose.X, obj.pose.Y, obj.pose.Z,'.')
            hold on
        end
        
        function  callbackWpPose(obj, src, msg) 
%            obj.pose = msg.Pose.Position;
           [dim nwps]=size(obj.waypoints);
           
%            plot3(obj.waypoints(1,1), obj.waypoints(2,1), obj.waypoints(3,1), 'o')
%            hold on
%            plot3(obj.waypoints(1,2), obj.waypoints(2,2), obj.waypoints(3,2), 'o')
%            hold on
%            plot3(obj.waypoints(1,3), obj.waypoints(2,3), obj.waypoints(3,3), 'o')
%            hold on
           
             for i=1:nwps
                   dist_wayps = [obj.pose.X obj.pose.Y obj.pose.Z] - ... 
                  [ obj.waypoints(1,i) obj.waypoints(2,i) obj.waypoints(3,i) ];
                 
                 if norm(dist_wayps) < 0.4
                   obj.waypoints =obj.waypoints(:,i+1:nwps);
                 end
             end 
        end
         
        
        function  callbackSpeed(obj, src, msg) 
                obj.speed = msg.Twist.Linear;
        end
        
        
        function computeTrajectory(obj)
            num_axes         = 3;
            [dim nwps ]=size(obj.waypoints);
            num_trajectories = nwps; %number of waypoint
            State_start      = [obj.pose.X obj.speed.X 0;... 
                                obj.pose.Y obj.speed.Y 0;...
                                obj.pose.Z obj.speed.Z 0];
            Waypoints = [];
            
            for i=1:nwps
                Waypoints(:,:,i) = [    obj.waypoints(1,i)  0.0  0.0  0.0  0.0; ...
                                        obj.waypoints(2,i)  0.0  0.0  0.0  0.0; ...
                                        obj.waypoints(3,i)  0.0 0.0 0.0 0.0];
            end
            
            V_max            =  1.0*ones(num_axes,num_trajectories);
            V_min            = -1.0*ones(num_axes,num_trajectories);
            A_max            =  0.5*ones(num_axes,num_trajectories); %0.5
            A_min            = -0.5*ones(num_axes,num_trajectories);
            J_max            =  1.0*ones(num_axes,num_trajectories);  %1.5
            J_min            = -1.0*ones(num_axes,num_trajectories);
            A_global         =  0.0*ones(num_axes,1);
            %Boolean variables 
            b_comp_global    = false;
            b_sync_V         =  true(num_axes,num_trajectories);
            b_sync_A         =  true(num_axes,num_trajectories);
            b_sync_J         = true(num_axes,num_trajectories);
            b_sync_W         = false(num_axes,num_trajectories); %false
            b_rotate         = false(1,num_trajectories); %false
            b_best_solution  =  true(num_axes,num_trajectories);
            b_hard_vel_limit = false(num_axes,num_trajectories);
            b_catch_up       =  true(num_axes,num_trajectories);

            % %% ----------   Calculate    ----------
            solution_in  = -1*ones(num_axes,2,num_trajectories,'int8');  

            tic;
            [J_setp_struct,solution_out,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);                                                              
            toc;

            % %% ----------     Output     ----------
%             disp(['num_axes = ',num2str(num_axes)]);
%             disp(['num_trajectories = ',num2str(num_trajectories)]);

            ts_rollout = 0.06; %0.01
            T_rollout = max(sum(T_waypoints,2));
            [P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

            obj.trajectory_pos = P;
            obj.trajectory_speed = V;  
            b= size(V(1).signals.values);
           
            obj.N_traj=b(1,1); 
        end  
        
        
          function  callbackReplanPose(obj,src, msg) 
            if obj.flying == true
%            PREGUNTA:NECESITO TODAS LAS LINEAS COMENTADAS SI LAS VARIABLES SON OBJECTOS?
            obj.pose = msg.Pose.Position;
%            
%           COMPUTE TRAJECTORY
%             
            computeTrajectory(obj);

%            PLOT OF THE FIRST TRAJECTORY
            plot3(obj.trajectory_pos(1).signals.values,...
                  obj.trajectory_pos(2).signals.values,... 
                  obj.trajectory_pos(3).signals.values);
            hold on 
           
%              V = obj.trajectory_speed;
%              P = obj.trajectory_pos;
% 
%              obj.waypoints = waypoints; %E' RIDONDNTE QUESTA INFORMAZIONE?
            [dim nwps ]=size(obj.waypoints);
            
            hasFinishedTraj  = false;
           
            while nwps > 0 && ~hasFinishedTraj
              for i=1: obj.N_traj
                  tic
                  %calcola l'errore
                  obj.error(obj.counter_step,:) = [obj.pose.X obj.pose.Y obj.pose.Z] - ... 
                              [obj.trajectory_pos(1).signals.values(i,1) obj.trajectory_pos(2).signals.values(i,1) obj.trajectory_pos(3).signals.values(i,1)];

                  if norm(obj.error(obj.counter_step,:)) > 0.6
                     computeTrajectory(obj) 
                     break 
                  end
                  obj.counter_step = obj.counter_step+1; 
                  obj.speed_index = i;
                  moveToWaypoint(obj)
                  incT = toc;
%                  pause(10^-09)
                                          
                  if hasFinishedTraj == obj.N_traj
                     hasFinishedTraj = true;
                  end
              end
            end     
             end
         end
               
        
        function moveToWaypoint(obj)  
           
%            display('-----------------------------------------------------------------------------------------')
           obj.speed_msg.Twist.Linear.X= obj.trajectory_speed(1).signals.values(obj.speed_index,1);
           obj.speed_msg.Twist.Linear.Y = obj.trajectory_speed(2).signals.values(obj.speed_index,1);
           obj.speed_msg.Twist.Linear.Z = obj.trajectory_speed(3).signals.values(obj.speed_index,1);
                   
           send(obj.speedPublisher, obj.speed_msg)
                  
           pause(10^-09)
                  
        end
    end   
end
  
