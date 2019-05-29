 function computeTrajectory1(waypoints,waypoints_vel,waypoints_ac,state_start)
num_axes         = 3;
            [~ , nwps ]=size(waypoints);
            num_trajectories = nwps; %number of waypoint

             State_start      = [state_start(1,1) 0 0; 
                                 state_start(2,1) 0 0;
                                 state_start(3,1) 0 0];


                            
                Waypoints=[];
%                 figure(1)
            
             for i=1:nwps
               
                Waypoints(:,:,i) = [    waypoints(1,i)  waypoints_vel(1,i)  waypoints_ac(1,i)  0.0  0.0; ...
                                        waypoints(2,i)  waypoints_vel(2,i)  waypoints_ac(2,i)  0.0  0.0; ...
                                        waypoints(3,i)  waypoints_vel(3,i)  waypoints_ac(3,i)  0.0  0.0];
                                    
%                 plot3(waypoints(1,i),waypoints(2,i),waypoints(3,i),'*')
%                 hold on 
                                    
             end
            
            
            V_max            =  1.0*ones(num_axes,num_trajectories);
            V_min            = -1.0*ones(num_axes,num_trajectories);
            A_max            =  0.5*ones(num_axes,num_trajectories); 
            A_min            = -0.5*ones(num_axes,num_trajectories);
            J_max            =  1.0*ones(num_axes,num_trajectories);  
            J_min            = -1.0*ones(num_axes,num_trajectories);
            A_global         =  0.0*ones(num_axes,1);
            %Boolean variables 
            b_comp_global    = false;
            b_sync_V         =  true(num_axes,num_trajectories);
            b_sync_A         =  true(num_axes,num_trajectories);
            b_sync_J         = true(num_axes,num_trajectories);%false
            b_sync_W         = false(num_axes,num_trajectories); %false
            b_rotate         = false(1,num_trajectories); %false
            b_best_solution  =  true(num_axes,num_trajectories);
            b_hard_vel_limit = false(num_axes,num_trajectories);
            b_catch_up       = true(num_axes,num_trajectories);%true

            % %% ----------   Calculate    ----------
            solution_in  = -1*ones(num_axes,2,num_trajectories,'int8');  

            tic;
            [J_setp_struct,~,T_waypoints,~] = opt_control_mex(State_start,Waypoints,V_max,V_min,A_max,A_min,J_max,J_min,A_global,b_comp_global,b_sync_V,b_sync_A,b_sync_J,b_sync_W,false(num_axes,num_trajectories),b_rotate,b_best_solution,b_hard_vel_limit,b_catch_up,ones(num_axes,8,1),ones(num_axes,1),zeros(num_axes,1),zeros(num_axes,1),solution_in);                                                              
            toc;

            % %% ----------     Output     ----------
%             disp(['num_axes = ',num2str(num_axes)]);
%             disp(['num_trajectories = ',num2str(num_trajectories)]);

            ts_rollout = 0.05; %0.01
            T_rollout = max(sum(T_waypoints,2));
            [P,V,A,J] = rollout(State_start(:,1),State_start(:,2),State_start(:,3)+A_global*b_comp_global,J_setp_struct,T_rollout,ts_rollout);

            trajectory_pos = P; 
            trajectory_speed = V;
            trajectory_jerk= J;
            trajectory_ac= A;
            plot3(trajectory_pos(1).signals.values,trajectory_pos(2).signals.values,...
                 trajectory_pos(3).signals.values)
             
            xlabel('x')
            ylabel('y')
            zlabel('z')
            
            grid on 
            
%             figure(2) 
%             
%             subplot(3,1,1)
%             plot(trajectory_pos(1).signals.values)
%             xlabel('n')
%             ylabel('X trajectory [m]')
%             grid on
%             
%             subplot(3,1,2)
%              plot(trajectory_pos(2).signals.values)
%             xlabel('n')
%             ylabel('Y trajectory [m]')
%             grid on 
%             
%            
%             subplot(3,1,3)
%             plot(trajectory_pos(3).signals.values)
%             xlabel('n')
%             ylabel('Z trajectory [m]')
%             grid on 
%             
%             T_rollout
%             
%             disp('axes syncronization on')
%             
%             figure(3)
%             
%             subplot(3,1,1)
%             plot(trajectory_speed(1).signals.values)
%             xlabel('n')
%             ylabel('V_x [m/s]')
%             grid on
%             
%             subplot(3,1,2)
%              plot(trajectory_speed(2).signals.values)
%             xlabel('n')
%             ylabel('V_y [m/s]')
%             grid on 
%             
%            
%             subplot(3,1,3)
%             plot(trajectory_speed(3).signals.values)
%             xlabel('n')
%             ylabel('V_z [m/s]')
%             grid on 
%             
%             figure(4)
%             subplot(3,1,1)
%             plot(trajectory_jerk(1).signals.values)
%             xlabel('n')
%             ylabel('J_x [m/s^3]')
%             grid on
%             
%             subplot(3,1,2)
%              plot(trajectory_jerk(2).signals.values)
%             xlabel('n')
%             ylabel('J_y [m/s^3]')
%             grid on 
%             
%            
%             subplot(3,1,3)
%             plot(trajectory_jerk(3).signals.values)
%             xlabel('n')
%             ylabel('J_z [m/s^3]')
%             grid on 
%             
%            
%             figure(5)
%             subplot(3,1,1)
%             plot(trajectory_ac(1).signals.values)
%             xlabel('n')
%             ylabel('a_x [m/s^2]')
%             grid on
%             
%             subplot(3,1,2)
%              plot(trajectory_ac(2).signals.values)
%             xlabel('n')
%             ylabel('a_y [m/s^2]')
%             grid on 
%             
%            
%             subplot(3,1,3)
%             plot(trajectory_ac(3).signals.values)
%             xlabel('n')
%             ylabel('a_z [m/s^2]')
%             grid on 
%             
% %             b= size(V(1).signals.values);
% % 
% %            
% %             N_traj=b(1,1); 
%             
           
        end  