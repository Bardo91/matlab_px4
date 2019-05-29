%% Distance
%initialize variables
% Waypoints
% n_wps=3;

load('wps_1.mat')
load('wps_3.mat')
load('wps_2.mat')
% Real Pose 
load('real_x.mat')
load('real_y.mat')
load('real_z.mat')

%-------------------------------------------------------------------------
%use the locìgical not operator in xr to locate the zeros;
%xr_new es el vector de los indices degli elementi not 0;
xr_new=find(~xr);
n=xr_new(1,1);

x_traj=xr(1:n-1); %trova size di x_traj
y_traj=yr(1:n-1);
z_traj=zr(1:n-1);
%--------------------------------------------------------------------------
%definisci matrice di punti della rel traj

real_traj=[x_traj y_traj z_traj];
%punto in cui arriva dopo il decollo 
z0=[0 0 2]; %[0 0 1]

nwps=[z0;
      w1;
      w2;
      w3];
      
[r , ~]=size(nwps);
%-------------------------------------------------------------------------
%definisco le direzioni 
v0=(w1 - z0);
v1=(w2 -w1);
v2=(w3 -w2);

dir_recta=[v0;
           v1;
           v2];
%-------------------------------------------------------------------------      
%cALCOLA IL NUMERO DI WPS
n_wps=r-1;

%calcola il numero di punti nella real pose x_traj;
[n_traj, c]=size(x_traj);

%definisci il vettore delle distanze minime
% d_min=zeros(n_traj,1);

% t=linspace(0,1);
%-------------------------------------------------------------------------
%PLOTTA FIGURA
plot3(x_traj,y_traj,z_traj,'b')
hold on 
plot3(w1(1),w1(2),w1(3),'*');
hold on 
plot3(w2(1),w2(2),w2(3),'*');
hold on 
plot3(w3(1),w3(2),w3(3),'*');
%-------------------------------------------------------------------------
k=1;
d=zeros(k,1);
t=linspace(0,1);
a=zeros(3,3);

for i=1:n_wps
    
    x = nwps(i,1) +t*a(1,1);
    y = nwps(i,2) +t*a(1,2);
    z = nwps(i,3) +t*a(1,3);
    plot3(x,y,z,'b')
    
    hold on 
    for j=k:10:n_traj
        
        if  nwps(i+1,1)< real_traj(j,1)< nwps(i+1,1) && nwps(i+1,1)<real_traj(j,2) < nwps(i+1,2) && nwps(i+1,1)< real_traj(j,3)< nwps(i+1,3)
            
        b = real_traj(j,:) - nwps(i,:);
        c = real_traj(j,:) - nwps(i+1,:);
        
        d(k,1)=norm(cross(b,c))/norm(a);
        k=k+1;
        
        else 
           break 
        end 
        
    end 
end 

   %calcola equazione del piano ortogonale ad una direzione e passante per
    %un punto per ogni i;
    %pero comunque hai un problema.
    
%     norm(real_traj(1,:) - z0(1,:))
% 
% ans =
% 
%     0.9162