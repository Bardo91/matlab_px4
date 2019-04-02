if (~exist('resolution','var'))
    resolution = [-2880/2 1620/2 2880/2 1620/2];
end
if (~exist('size_font','var'))
    size_font = 16;
end
if (~exist('b_plot_marker','var'))
    b_plot_marker = true;
end
if (~exist('b_record_video','var'))
    b_record_video = false;
end
if (~exist('T_viapoints','var'))
    T_viapoints = [];
end
if (~exist('Obstacles','var'))
    Obstacles = [];
end
if (~exist('b_plot_axis','var'))
    b_plot_axis = ones(num_axes,1);
end

color_start    = [0.0 0.7 0.0];
color_waypoint = [0.0 0.0 0.7];
color_endpoint = [0.7 0.0 0.0];
color_viapoint = [0.5 0.5 0.5];
color_obstacle = [0.5 0.5 0.5];
color_limit    = [1.0 0.0 0.0];
color_marker   = [0.0 0.0 0.0];
size_marker    = 30;
size_waypoints = 150;



%figure;
%clf;
set(gcf,'units','pixels','Resize','off','Position',resolution);
%set(gcf,'MenuBar','none');

num_axes = size(Waypoints,1);
num_trajectories = size(Waypoints,3);

axislimits = zeros(4,4);
axislimits(:,1) = 0;
axislimits(:,2) = T_rollout;
for index_axis = 1:num_axes
    if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
        axislimits(1,3) = min(axislimits(1,3),min(P(index_axis).signals.values));
        axislimits(1,4) = max(axislimits(1,4),max(P(index_axis).signals.values));
        axislimits(2,3) = min(axislimits(2,3),min(V(index_axis).signals.values));
        axislimits(2,4) = max(axislimits(2,4),max(V(index_axis).signals.values));
        axislimits(3,3) = min(axislimits(3,3),min(A(index_axis).signals.values));
        axislimits(3,4) = max(axislimits(3,4),max(A(index_axis).signals.values));
        axislimits(4,3) = min(axislimits(4,3),min(J_setp_struct(index_axis).signals.values));
        axislimits(4,4) = max(axislimits(4,4),max(J_setp_struct(index_axis).signals.values));
    end
end
axislimits(:,3) = axislimits(:,3)-(axislimits(:,4)-axislimits(:,3))/10;
axislimits(:,4) = axislimits(:,4)+(axislimits(:,4)-axislimits(:,3))/10;

h1 = subplot(4,1,1);
    hold all;
    axis manual;
    for index_axis = 1:num_axes
        if (size(b_plot_axis,1) >= index_axis && b_plot_axis(index_axis) == true)
            stairs(P(index_axis).time,P(index_axis).signals.values,'Color','black');
            scatter(0,State_start(index_axis,1),size_waypoints,color_start,'Marker','o','MarkerFaceColor',color_start);
            for index_waypoint = 1:1:num_trajectories
                if (index_waypoint == num_trajectories)
                    color = color_endpoint;
                else
                    color = color_waypoint;
                end
                T = sum(T_waypoints(index_axis,1:index_waypoint));
                scatter(0,Waypoints(index_axis,1,index_waypoint),size_waypoints,color,'Marker','o','MarkerFaceColor',color);
                Waypoint_evolved = evolve_waypoints(Waypoints(:,:,index_waypoint),T);
                scatter(T,Waypoint_evolved(index_axis,1),size_waypoints,color,'Marker','o','MarkerFaceColor',color,'MarkerFaceAlpha',0.0);
                xt = @(t) Waypoints(index_axis,1,index_waypoint)+Waypoints(index_axis,4,index_waypoint).*t+1/2.*Waypoints(index_axis,5,index_waypoint).*t.^2;
                fplot(xt,[0 T],'Color',color,'LineStyle',':');
            end
            for index_times = 2:1:size(J_setp_struct(index_axis).time,1)
                line([J_setp_struct(index_axis).time(index_times,1) J_setp_struct(index_axis).time(index_times,1)],[axislimits(1,3) axislimits(1,4)],'Color','black','LineStyle',':');
                %text(J_setp_struct(index_axis).time(index_times-1) + (J_setp_struct(index_axis).time(index_times) - J_setp_struct(index_axis).time(index_times-1)) / 2,axislimits(1,4),arabic_to_roman(index_times-1),'FontSize',size_font,'FontName','Times','HorizontalAlignment','center','VerticalAlignment','bottom');
                text(J_setp_struct(index_axis).time(index_times-1) + (J_setp_struct(index_axis).time(index_times) - J_setp_struct(index_axis).time(index_times-1)) / 2,axislimits(1,4),num2str(index_times-1),'FontSize',size_font,'FontName','Times','HorizontalAlignment','center','VerticalAlignment','bottom');
                if (b_plot_marker == true)
                    [P_switch,V_switch,A_switch,J_switch] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct(index_axis),J_setp_struct(index_axis).time(index_times));
                    scatter(J_setp_struct(index_axis).time(index_times),P_switch,size_marker,color_marker,'Marker','o','MarkerFaceColor',color_marker);
                end
            end
            for index_viapoint = 1:size(T_viapoints,2)
                [P_viapoint,V_viapoint,A_viapoint,J_viapoint] = evaluate_to_time(State_start(index_axis,1),State_start(index_axis,2),State_start(index_axis,3),J_setp_struct,sum(T_viapoints(index_viapoint)));
                scatter(T_viapoints(index_viapoint),P_viapoint(index_axis),size_waypoints,color_viapoint,'Marker','o','MarkerFaceColor',color_viapoint);
            end    
%             if ~(isempty(Obstacles))
%                 t = linspace(axislimits(1,1),axislimits(1,2),100);
%                 for index_Obstacle = 1:size(Obstacles,3)
%                     Obstacle = Obstacles(:,:,index_Obstacle);
%                     for index_t = 1:size(t,2)
%                         Obstacle_evolved = evolve_obstacles(Obstacle,t(index_t));
%                         y_1(index_t) = Obstacle_evolved(index_axis,1,1);
%                         y_2(index_t) = Obstacle_evolved(index_axis,5,1);
%                     end
%                     alpha_patch = 0.2;
%                     xt = @(t) Obstacle(index_axis,1)-MAV_margin(index_axis)-MAV_d(index_axis)+Obstacle(index_axis,2).*t+1/2.*Obstacle(index_axis,3).*t.^2+1/6.*Obstacle(index_axis,4).*t.^3;
%                     fplot(xt,[axislimits(1,1) axislimits(1,2)],'Color',[0.5,0.5,0.5],'LineStyle',':','LineWidth',2);
%                     xt = @(t) Obstacle(index_axis,5)+MAV_margin(index_axis)+MAV_d(index_axis)+Obstacle(index_axis,6).*t+1/2.*Obstacle(index_axis,7).*t.^2+1/6.*Obstacle(index_axis,8).*t.^3;
%                     fplot(xt,[axislimits(1,1) axislimits(1,2)],'Color',[0.5,0.5,0.5],'LineStyle',':','LineWidth',2);
%                 end
%                 fill([t t(end:-1:1)],[y_1 y_2(end:-1:1)],color_obstacle,'FaceAlpha',alpha_patch);
%             end
        end
    end


