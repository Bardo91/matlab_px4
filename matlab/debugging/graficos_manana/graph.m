%% Evaluate error

% load('robot_error_y.mat')
% load('robot_error_x.mat')
% load('robot_error_z.mat')
figure
subplot(3,1,1)
plot(x(1:1300,:))
grid on 
title('axis x error')
subplot(3,1,2)
plot(y(1:1300,:))
grid on 
title('axis y error')
hold on 
subplot(3,1,3)
plot(z(1:1300,:))
grid on 
title('axis z error')

% % ERROR FROM FIRST PLANNED TRJECTORY
% figure(2)
% 
% % load('robot_inic_traj.mat')
% % load('robot_inic_traj_y.mat')
% % load('robot_inic_traj_z.mat')
% % 
% subplot(3,1,1)
% plot(abs(x(1:352,1)-x_t))
% title('abs(x_traj - x')
% subplot(3,1,2)
% plot(abs(y(1:352)-y_t))
% title('abs(y_traj - y')
% hold on 
% subplot(3,1,3)
% plot(abs(z(1:352)-z_t))
% title('abs(z_traj -z ')

 %%USING GET
openfig('manana.fig')

ax=gca;
lines=get(ax,'Children')
data_line.X=zeros(979,1)
data_line.Y=zeros(979,1)
data_line.Z=zeros(979,1)

for i=1:length(lines)
   data_line(i).X=get(lines(i),'XData')
   data_line(i).Y=get(lines(i),'YData')
   data_line(i).Z=get(lines(i),'ZData')
end

% X=data_line.X;
% Y=data_line.Y;
% Z=data_line.Z;
% plot3(X,Y,Z,'.')
% 
% hold on 

%%SUBPLOT TRAIETTORIA UNO
subplot(3,1,1)
plot(x_t)
hold on 
plot(250,x_t(250),'o')
hold on 
plot(i,data_line.X,'.')
grid on 

subplot(3,1,2)
plot(y_t)
hold on
plot(250,y_t(250),'o')
hold on
plot(i,data_line.Y,'.')
grid on

subplot(3,1,3)
plot(z_t)
hold on
plot(250,z_t(250),'o')
hold on
plot(i,data_line.Z,'.')
grid on 

% %%plotta punti sulla traiettoria per valutare errore
% plot3(x_t,y_t,z_t,'b')
% grid on 
% hold on 
% plot3(x_t(300),y_t(300),z_t(300),'o')


