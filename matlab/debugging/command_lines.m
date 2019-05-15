%% COMMAND LINE

%PROVA MOBILEROBOT4
% nota: a meno che non sei nel caso 4, descommenta le righe 4-5;
robot=MobileRobot4(); %robot=MobileRobot4();
robot.takeoff(1)

%indice
index_caso=1;
switch index_caso

%%-------------------------------------------------------------------------
    case 1
% PROVA DI VOLO -->  2 wayponit trajectory
%nota= fallo partire con ROS reinizializzato
%nota= volori di utilizzo:  errore > 1.2 y dist_wp < 0.5
waypoints=[3 10; 3 15; 1 15];                     %[x1 x2  ;y1  y2  ;z1 z2 ]
waypoints_vel=[0.5 0; 0.5 0; 0.5 0];              %[u1 u2  ;v1  v2  ;w1 w2 ]  
waypoints_ac=[0.1 0.0; 0.1 0.0; 0.1 0.0];

robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
% robot.land()

%%-------------------------------------------------------------------------
    case 2 
%  SINGOLO WAYPOINT IN MOVIMENTO

waypoints=[3  ; 2  ; 1 ]; %prova hasta 5
waypoints_vel=[0.5 ; 0.0 ; 0.0 ];               
waypoints_ac=[0.0 ; 0.0 ; 0.0 ];

robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
% robot.land()

%%-------------------------------------------------------------------------
    case 3
% 3 WAYPOINTS TRAJECTORY
waypoints=[4 5 18; 12 11 9; 13 10 7];             %[x1 x2 x3 ;y1  y2 y3 ;z1 z2 z3 ]
waypoints_vel=[0.5 0.5 0; 0.5 0.5 0; 0.5 0.5 0];  %[u1 u2 u3 ;v1  v2  v3 ;w1 w2 w3]  
robot.moveToWaypoint(waypoints,waypoints_vel);
% robot.land()


%--------------------------------------------------------------------------

    case 4
 %OPTIMAL TRAJECTORIES GENERATIONS
 
 waypoints=[3 5 8; 5 8 10; 3 11 7];                                  %[x1 x2  ;y1  y2  ;z1 z2 ]
 waypoints_vel=[0.2 0.0 0.0; 0.0 0.3 0.0; 0.0 0.0 0.0];              %[u1 u2  ;v1  v2  ;w1 w2 ]  
 waypoints_ac=[0.0 0.0 0.0; 0.0 0.0 0.0; 0.0 0.0 0.0];
% robot.moveToWaypoint(waypoints,waypoints_vel,waypoints_ac);
 computeTrajectory1(waypoints,waypoints_vel,waypoints_ac);
end 



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