%initialize variables 
load('wps_1.mat')
load('wps_3.mat')
load('wps_2.mat')
n_wps=3;
load('real_x.mat')
load('real_y.mat')
load('real_z.mat')

%use the locìgical not operator in xr to locate the zeros;
%xr_new es el vector de los indices
xr_new=find(~xr);
n=xr_new(1,1);

x_traj=xr(1:n-1); %trova size di x_traj
y_traj=yr(1:n-1);
z_traj=zr(1:n-1);

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
       
t=linspace(0,1);
n_wps=r-1;
%calcola il numero di punti nella real pose
[n_traj c]=size(x_traj)

%definisci il vettore delle distanze 
d_min=zeros(n_traj,1);
d=[];
%scrivi ciclo per calcolare la retta 
plot3(x_traj,y_traj,z_traj,'b')
hold on 
plot3(w1(1),w1(2),w1(3),'*');
hold on 
plot3(w2(1),w2(2),w2(3),'*');
hold on 
plot3(w3(1),w3(2),w3(3),'*');
k=1;
h=1;
d_min=zeros(k,1);

for i= 1 : n_wps
    %equazione parametrica della retta;
    x=nwps(i,1)+dir_recta(i,1)*t;
    y=nwps(i,2)+dir_recta(i,2)*t;
    z=nwps(i,3)+dir_recta(i,3)*t;
    hold on 
    plot3(x,y,z,'r')
    grid on
    hold on 
    % prova solo per il primo punto della traiettoria per il debugging
    for j= k:n_traj
        %calcola la distanza d_way fra i punti della traiettoria e il primo waypoints
         d_way=sqrt((xr(j,1)-w1(1)).^2+(yr(j,1)-w1(2)).^2+(zr(j,1)- w1(3)).^2)
       
         if d_way > 0.4
         %condizione di massima vicinanza al waypoint    
         %calcola la distanza del j-esimo punto della real pose ad un punto
         %variabili di debug
         xr(j,1);
         yr(j,1);
         zr(j,1);
         d(j,:)=sqrt((xr(j,1)-transpose(x)).^2+(yr(j,1)-transpose(y)).^2+(zr(j,1)- transpose(z)).^2);
         %trova il minimo di d
         d_min(k,1)= min(d(j,:));
         k=k+1;
         else 
             break 
         end 
         
    end 
    
end 






    
% for i=1:r
%     
% %     if norm([r_p(1,1) - wps(1,1) r_p(1,2)-wps(1,2) r_p(1,3)-wps(1,3])  %con la norma?distanza?
%         
%         d(i,:)=sqrt((r_p(i,1)-x_1).^2+(r_p(i,2)-y_1).^2+(r_p(i,3)- z_1).^2)
%        
%         %trova il minimo di d
%         
%         d_min(i)= min(d(i,:))
%         
%         %calcola distanza
% %     else
% %        break
% %     end 
% end 


% plot3(x_3,y_3,z_3,'b')
% hold on 
% plot3(x_1,y_1,z_1,'r')
% hold on 
% plot3(x_2,y_2,z_2,'b')
% grid on 
% hold on 
% plot3(w1(1),w1(2),w1(3),'*')
% plot3(w2(1),w2(2),w2(3),'*')
% hold on 
% plot3(w3(1),w3(2),w3(3),'*')
% hold on 
% plot3(r_p(:,1),r_p(:,2),r_p(:,3))

%distanza primo wps e primo punto della pose
% plot3(r_p(:,1),r_p(:,2),r_p(:,3),'r')
% hold on 
% plot3(wps(1,1),wps(1,2),wps(1,3),'+')
% hold on 
% plot3(r_p(1,1),r_p(1,2),r_p(1,3),'*')

% %RETTA FRA LA POSIZIONE DI take off E IL PRIMO WP
% v0=[w1(1)-z(1) w1(2)-z(2) w1(3)-z(3)]
% %equazione parametrica della retta:
% t=linspace(0,2);
% 
% x_1=z(1)+v0(1)*t
% y_1=z(2)+v0(2)*t
% z_1=z(3)+v0(3)*t
% 
% %RETTA FRA PRIMO E SECONDO WPS
% v1=[w2(1)-w1(1) w2(2)-w1(2) w2(3)-w1(3)]
% %equazione parametrica della retta:
% 
% x_2=w1(1)+v1(1)*t
% y_2=w1(2)+v1(2)*t
% z_2=w1(3)+v1(3)*t
% 
% %RETTA FRA SECONDO E TERZO WPS
% v2=[w3(1)-w2(1) w3(2)-w2(2) w3(3)-w2(3)]
% %equazione parametrica della retta:
% 
% x_3=w2(1)+v2(1)*t
% y_3=w2(2)+v2(2)*t
% z_3=w2(3)+v2(3)*t
% 
% %me le scrivo tutte in un vettore
% a=[x_1;
%    x_2;
%    x_3]
% b=[y_1;
%    y_2;
%    y_3]
% c=[z_1;
%    z_2;
%    z_3]
%calcola distanza della 'pose' dalla 'traiettoria';