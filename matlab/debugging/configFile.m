clear all
clc
%%
disp('opening waypoints file')
waypointsFile = fopen('configFile.txt','r');
sizeWaypointsMatrix = [9 inf];
formatSpec = '%f %f %f %f %f %f %f %f %f';
waypointsMatrix = fscanf(waypointsFile, formatSpec, sizeWaypointsMatrix)
sizeOfConfigFileMatrix = size(waypointsMatrix,2)

offset = waypointsMatrix(1:3,1)
waypoints = waypointsMatrix(1:3,2:sizeOfConfigFileMatrix)
disp('Adding offset to waypoints')
waypoints = waypoints + offset
waypoints_vel = waypointsMatrix(4:6,2:sizeOfConfigFileMatrix)
waypoints_ac = waypointsMatrix(7:9,2:sizeOfConfigFileMatrix)

lastwaypoint = waypoints_ac(:,size(waypoints_ac,2));
if(     lastwaypoint(1) == 8.8 && ...
        lastwaypoint(2) == 8.8 && ...
        lastwaypoint(3) == 8.8)
    % pick square dimension from config file
    squareDimension = waypoints(:,size(waypoints,2))-offset
    height = squareDimension(1,size(squareDimension,2))
    side = squareDimension(2,size(squareDimension,2))
    
    % extract square dimension from waypoints
    waypoints = waypoints (:,1:size(waypoints,2)-1);
    waypoints_vel = waypoints_vel (:,1:size(waypoints_vel,2)-1);
    waypoints_ac = waypoints_ac (:,1:size(waypoints_ac,2)-1);
    
    % change waypoints to add a square
    squareWaypoints(:,1) = offset + [0,0,height]';
    squareWaypoints(:,2) = offset + [side/2,side/2,height]';
    squareWaypoints(:,3) = offset + [side/2,-side/2,height]';
    squareWaypoints(:,4) = offset + [-side/2,-side/2,height]';
    squareWaypoints(:,5) = offset + [-side/2,side/2,height]';
    squareWaypoints(:,6) = offset + [side/2,side/2,height]';
    squareWaypoints_vel = [0.5 0.5 0.5;
                            0.5 0.5 0.5;
                            0.5 0.5 0.5;
                            0.5 0.5 0.5;
                            0.5 0.5 0.5;
                            0.0 0.0 0.0;]';
    squareWaypoints_acel = zeros(3,6);                   
end

fclose(waypointsFile);
disp('closing waypoints file')


waypointNumber = size(waypoints,2);
waypoints_msg(1:9:waypointNumber*9) = waypoints(1,:);
waypoints_msg(2:9:waypointNumber*9) = waypoints(2,:)';
waypoints_msg(3:9:waypointNumber*9) = waypoints(3,:);
waypoints_msg(4:9:waypointNumber*9) = waypoints_vel(1,:);
waypoints_msg(5:9:waypointNumber*9) = waypoints_vel(2,:);
waypoints_msg(6:9:waypointNumber*9) = waypoints_vel(3,:);
waypoints_msg(7:9:waypointNumber*9) = waypoints_ac(1,:);
waypoints_msg(8:9:waypointNumber*9) = waypoints_ac(2,:);
waypoints_msg(9:9:waypointNumber*9) = waypoints_ac(3,:);