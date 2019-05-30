%% COMMAND LINE
close all
rosinit
disp('Init mobile robot')
robot=MobileRobotD4PHNE(); %robot=MobileRobot4();
disp('take off start')
robot.takeoff(1)

disp('opening waypoints file')
waypointsFile = fopen('newWaypoints.txt','r');
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

lastAcelWaypoint = waypoints_ac(:,size(waypoints_ac,2));

if(     lastAcelWaypoint(1) == 8.8 && ...
        lastAcelWaypoint(2) == 8.8 && ...
        lastAcelWaypoint(3) == 8.8)
    % pick square dimension from config file
    squareDimension = waypoints(:,size(waypoints,2))-offset
    height = squareDimension(1,size(squareDimension,2));
    side = squareDimension(2,size(squareDimension,2));
    disp('Making square')
    disp(strcat(strcat('Height=', num2str(height)),strcat(' Side=', num2str(side))))
    % extract square dimension from waypoints
    waypoints = waypoints (:,1:size(waypoints,2)-1);
    waypoints_vel = waypoints_vel (:,1:size(waypoints_vel,2)-1);
    waypoints_ac = waypoints_ac (:,1:size(waypoints_ac,2)-1);
    
    % change waypoints to add a square
    squareWaypoints(:,1) = offset + [side/2,        side/2-0.001,            height-0.001]';
    squareWaypoints(:,2) = offset + [side/2-0.002,   -side/2,           height-0.002]';
    squareWaypoints(:,3) = offset + [-side/2-0.001,       -side/2-0.002,          height-0.003]';
    squareWaypoints(:,4) = offset + [-side/2-0.003,       side/2-0.003,           height-0.004]';
    squareWaypoints(:,5) = offset + [side/2-0.004,   side/2-0.005,            height-0.005]';
    vel = 0.4;
    squareWaypoints_vel = [ vel vel vel;
                            vel vel vel;
                            vel vel vel;
                            vel vel vel;
                            0.0 0.0 0.0;]';
    acel = 0.1;
    squareWaypoints_ac = [ acel acel acel;
                           acel acel acel;
                           acel acel acel;
                           acel acel acel;
                           0.0 0.0 0.0;]';
    waypoints = squareWaypoints;
    waypoints_vel = squareWaypoints_vel;
    waypoints_ac = squareWaypoints_ac;
end

fclose(waypointsFile);
disp('closing waypoints file')

robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);

%%
% %indice
% index_caso=1;
% switch index_caso
%
% %%-------------------------------------------------------------------------
%     case 1
% % PROVA DI VOLO -->  2 waypoints trajectory
%
% waypoints=[1 5; 3 5.2; 1.5 4];                     %[x1 x2  ;y1  y2  ;z1 z2 ]
% waypoints_vel=[0.5 0; 0.5 0; 0.5 0];              %[u1 u2  ;v1  v2  ;w1 w2 ]
% waypoints_ac=[0.1 0.0; 0.1 0.0; 0.1 0.0];
%
% robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
% % robot.land()
%
% %%-------------------------------------------------------------------------
%     case 2
% %  SINGOLO WAYPOINT IN MOVIMENTO
%
% waypoints=[3  ; 2  ; 1 ]; %prova hasta 5
% waypoints_vel=[0.5 ; 0.0 ; 0.0 ];
% waypoints_ac=[0.0 ; 0.0 ; 0.0 ];
%
% robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
% % robot.land()
%
% %%-------------------------------------------------------------------------
%     case 3
% % 3 WAYPOINTS TRAJECTORY
% waypoints=[4 5 18; 12 11 9; 13 10 7];             %[x1 x2 x3 ;y1  y2 y3 ;z1 z2 z3 ]
% waypoints_vel=[0.5 0.5 0; 0.5 0.5 0; 0.5 0.5 0];  %[u1 u2 u3 ;v1  v2  v3 ;w1 w2 w3]
% robot.moveToWaypoint(waypoints,waypoints_vel);
% % robot.land()
%
%
% %--------------------------------------------------------------------------
%
%     case 4
%  %OPTIMAL TRAJECTORIES GENERATIONS
%
%  waypoints=[3 5 8; 5 8 10; 3 11 7];                                  %[x1 x2  ;y1  y2  ;z1 z2 ]
%  waypoints_vel=[0.2 0.0 0.0; 0.0 0.3 0.0; 0.0 0.0 0.0];              %[u1 u2  ;v1  v2  ;w1 w2 ]
%  waypoints_ac=[0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0];
% % robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
%  computeTrajectory1(waypoints,waypoints_vel,waypoints_ac);
% end



%possibili crash: perchè le componenti y2 y y3 sono uguali;
%                 perchè le componenti del vettore delle velocità sono
%                 uguali (possono esserlo);
%--------------------------------------------------------------------------
% WPS utilizzati
% robot.moveToWaypoint([3 8 11; 3 8 11; 3 10 18]);
% robot.moveToWaypoint([10 ;  8 ; 12 ])
% robot.moveToWaypoint([3 10 11; 3 15 10; 1 15 26]);
%--------------------------------------------------------------------------
%NOTA: REPLANIFICARE MOLTO, PER LA DINAMICA DEL SEGNALE INVIATO OGNI VOLTA,
%E' NEGATIVO, QUINDI AUMENTO LA NORMA DELLO ERRORE PER REPLANIFICARE MENO
%SPESSO.
