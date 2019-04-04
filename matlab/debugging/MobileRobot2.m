classdef MobileRobot2 < handle
      properties(Access = public)
          pose;
          speed;
          
          poseSubscriber;
          speedSubscriber;
          
          speedPublisher;
         
          
          waypoints = [];
          
          trajectory_pos = [];
          trajectory_speed = [];
          N_traj=[];
          nwps=[];
         
          
          % Debug variables
           counter_step=0;
          error=[];
        
      end
    methods(Access = public)
        function obj = MobileRobot2()  % Constructor   
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
             figure(1)
%             subplot(2,1,1)
            plot3(obj.pose.X, obj.pose.Y, obj.pose.Z,'.')
           
            hold on
            
           [dim nwps ]=size(obj.waypoints);
% 
             for i=1:nwps
                   plot3(obj.waypoints(1,i), obj.waypoints(2,i), obj.waypoints(3,i), 'o')
                 
                 dist_wayps = [obj.pose.X obj.pose.Y obj.pose.Z] - ... 
                  [ obj.waypoints(1,i) obj.waypoints(2,i) obj.waypoints(3,i) ];
                 
                 if norm(dist_wayps) < 0.4
                   obj.waypoints = [obj.waypoints(:,i+1:nwps)]
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
            disp(['num_axes = ',num2str(num_axes)]);
            disp(['num_trajectories = ',num2str(num_trajectories)]);

            ts_rollout = 0.06; %0.01
            T_rollout = max(sum(T_waypoints,2));
            [P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

            obj.trajectory_pos = P;
            obj.trajectory_speed = V;  
            b= size(V(1).signals.values);
           
            obj.N_traj=b(1,1);

           
        end  
          
          
                    
         function moveToWaypoint(obj, waypoints)
            obj.waypoints = waypoints;
            
            %Plan trajectory 
            
            computeTrajectory(obj);
        
            msg = rosmessage(obj.speedPublisher);
          
            %hay estas informaciones in computeTrajectory tambien.
            V = obj.trajectory_speed;
            P = obj.trajectory_pos;
            %initial trajectory
           
            
            
            
            [dim nwps ]=size(obj.waypoints);
            
            
%             hold on;
            hasFinishedTraj  = false;
           
            while nwps > 0 && ~hasFinishedTraj
               for i=1: obj.N_traj
                   tic
                   % compleate replanned trajectory
%                     subplot(2,1,1)
%                     plot3(obj.trajectory_pos(1).signals.values, ...
%                     obj.trajectory_pos(2).signals.values,...
%                     obj.trajectory_pos(3).signals.values);
                
                    msg.Twist.Linear.X= obj.trajectory_speed(1).signals.values(i,1);
                    msg.Twist.Linear.Y = obj.trajectory_speed(2).signals.values(i,1);
                    msg.Twist.Linear.Z = obj.trajectory_speed(3).signals.values(i,1);

                    obj.error = [obj.pose.X obj.pose.Y obj.pose.Z] - ... 
                            [obj.trajectory_pos(1).signals.values(i,1) obj.trajectory_pos(2).signals.values(i,1) obj.trajectory_pos(3).signals.values(i,1)];

                     
%                   subplot(2,1,2)
%                   plot(obj.counter_step,norm(obj.error),'.')
%                   hold on 
%                   plot(obj.counter_step, norm(obj.error), 'bo');
%                   plot3(obj.trajectory_pos(1).signals.values(i,1), obj.trajectory_pos(2).signals.values(i,1), obj.trajectory_pos(3).signals.values(i,1),'°');
                    obj.counter_step = obj.counter_step+1;
                    
                    
                    if norm(obj.error) > 0.6
                
                       computeTrajectory(obj)
                       
                       
                     break 

                    end
%                    
                    send(obj.speedPublisher, msg)
                    incT = toc
                    pause(10^-09)
                    
%                     if incT < 0.05
%                         pause(0.05-incT)   
%                     end
                    
                    if hasFinishedTraj == obj.N_traj
                        hasFinishedTraj = true;
                    end
               end
            end 
             
        end
    end
    end