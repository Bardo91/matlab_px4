% %%  retta passante per 2 punti nello spazio
% z=[0 0 2];
% a=[3 3 1];
% b=[10 15 15];
% c=[11 10 26];
% 
% %punto distnza
% p=[11.13 17.67 22]
% 
% %calcolo vettore direzione della retta BC
% v=[c(1)-b(1) c(2)-b(2) c(3)-(3)]
% 
% %equazione parametrica della retta:
% t=linspace(0,2);
% 
% x=b(1)+v(1)*t
% y=b(2)+v(2)*t
% z=b(3)+v(3)*t
% 
% %distanza punto detta
% 
% d=sqrt((p(1)-x).^2+(p(2)-y).^2+(p(3)-z).^2)
% 
% %trova il minimo di d
% d_min= min(d)

function [d_min]= distanza( w1,w2,w3,z,r_p,n_wps)

% aggiungere validazione input

% costruzione delle rette passanti per i wps

%RETTA FRA LA POSIZIONE DI take off E IL PRIMO WP
v0=[w1(1)-z(1) w1(2)-z(2) w1(3)-z(3)]
%equazione parametrica della retta:
t=linspace(0,2);

x_1=z(1)+v0(1)*t
y_1=z(2)+v0(2)*t
z_1=z(3)+v0(3)*t

%RETTA FRA PRIMO E SECONDO WPS
v1=[w2(1)-w1(1) w2(2)-w1(2) w2(3)-w1(3)]
%equazione parametrica della retta:

x_2=w1(1)+v1(1)*t
y_2=w1(2)+v1(2)*t
z_2=w1(3)+v1(3)*t

%RETTA FRA SECONDO E TERZO WPS
v2=[w3(1)-w2(1) w3(2)-w2(2) w3(3)-w2(3)]
%equazione parametrica della retta:

x_3=w2(1)+v2(1)*t
y_3=w2(2)+v2(2)*t
z_3=w2(3)+v2(3)*t

%me le scrivo tutte in un vettore
a=[x_1;
   x_2;
   x_3]
b=[y_1;
   y_2;
   y_3]
c=[z_1;
   z_2;
   z_3]
%calcola distanza della 'pose' dalla 'traiettoria';
[r c]=size(r_p)
%definisci un vettore di waypoints
wps=[w1;w2;w3];
% d_min=zeros(r,1);
d_min=[]
    
for i=1:r
    
    if r_p(i,3) < wps(1,3)
        
        d(i,:)=sqrt((r_p(i,1)-x_1).^2+(r_p(i,2)-y_1).^2+(r_p(i,3)- z_1).^2)
       
        %trova il minimo di d
        
        d_min(i)= min(d(i))
        
        %calcola distanza
    else
       break
    end 
end 
end 


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



