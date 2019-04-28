%% Distance
%initialize variables
% Waypoints
n_wps=3;
load('wps_1.mat')
load('wps_3.mat')
load('wps_2.mat')
% Real Pose 
load('real_x.mat')
load('real_y.mat')
load('real_z.mat')

%use the locìgical not operator in xr to locate the zeros;

%xr_new es el vector de los indices de los el. not 0;
xr_new=find(~xr);
n=xr_new(1,1);

x_traj=xr(1:n-1); %trova size di x_traj
y_traj=yr(1:n-1);
z_traj=zr(1:n-1);

%definisci matrice di punti della rel traj

real_traj=[x_traj y_traj z_traj]

z=[0 0 2];

nwps=[z;
      w1;
      w2;
      w3]
      
[r c]=size(nwps);

%definisco le direzioni 
v0=[w1(1)-z(1) w1(2)-z(2) w1(3)-z(3)];
v1=[w2(1)-w1(1) w2(2)-w1(2) w2(3)-w1(3)];
v2=[w3(1)-w2(1) w3(2)-w2(2) w3(3)-w2(3)];

dir_recta=[v0;
           v1;
           v2]
       

n_wps=r-1;

%calcola il numero di punti nella real pose
[n_traj c]=size(x_traj)

%definisci il vettore delle distanze 
d_min=zeros(n_traj,1);
d=[];
% t=linspace(0,1);
%scrivi ciclo per calcolare la retta 
plot3(x_traj,y_traj,z_traj,'b')
hold on 
plot3(w1(1),w1(2),w1(3),'*');
hold on 
plot3(w2(1),w2(2),w2(3),'*');
hold on 
plot3(w3(1),w3(2),w3(3),'*');
k=1;
% h=1;
d=zeros(k,1);
point=zeros(k,3);

for i=1:n_wps
    %equazione parametrica della retta
    syms t
    line=nwps(i,:)+dir_recta(i,:)*t
    
%     hold on 
%     plot3(x,y,z,'r')
%     grid on
%     hold on 
    for j=1:n_traj
        %Calcola proiezione ortogonale
        %Calcola piano passante per P(j) y ortogonale alla direzione i_esima
        syms x_p y_p z_p
        P=[x_p y_p z_p]
        planefunction=dot(dir_recta(1,:),P-real_traj(j,:)); %plane_fun=0
        newfunction = subs(planefunction, P, line);
        t0 = solve(newfunction);
        
        %punto intersezione fra il piano precedent. e la direzione V(i);
        point(k,:) = subs(line, t, t0);
        subs(planefunction, P, point);
        
        if nwps(i,1)< point(k,1)< nwps(i+1,1) && nwps(i,2)< point(k,2)< nwps(i+1,2) && nwps(i,1)< point(k,1)< nwps(i+1,1)   
        %cross product
        C_product=cross((nwps(i+1,:)-nwps(i,:)),(nwps(i,:)-real_traj(j,:)));
        % distancia entres nwps(i+1) y nwps(i)
        D_recta= norm(nwps(i+1,:)-nwps(i,:));
        % distancia total
        d(k,1)=norm(C_product)/D_recta;
        k=k+1;
        else 
            break 
        end 
        
    end 
end 






