%%CALCOLO ERRORE PUNTO PUNTO DALLA TRAIETTORIA ORIGINALE

%LOAD Inicial trajectory --> mi fornisce i vettori x_t,y_t,z_t

load('robot_inic_traj.mat')
load('robot_inic_traj_y.mat')
load('robot_inic_traj_z.mat')

%LOAD il vettore della real pose

openfig('manana.fig')

ax=gca;
lines=get(ax,'Children')
% data_line.X=zeros(979,1)
% data_line.Y=zeros(979,1)
% data_line.Z=zeros(979,1)

Z=zeros(length(lines),1);
Y=zeros(length(lines),1);
X=zeros(length(lines),1);

for i=1:length(lines)
   data_line(i).X=get(lines(i),'XData')
   data_line(i).Y=get(lines(i),'YData')
   data_line(i).Z=get(lines(i),'ZData')
   Z(i)=data_line(i).Z;
   X(i)=data_line(i).X;
   Y(i)=data_line(i).X;
end

%%confronta le alezze z e se ne trovi di uguali, calcola l'errore.
%%utilizzo funzione find

figure

for i = 1: length(z_t)
  indices =  find (round(Z,2) == round(z_t(i),2));
      if isempty(indices) == 1
         continue 
     end 
  a = indices(end)
  a1=x_t(i)
  a2=X(a)
  b1=y_t(i)
  b2=Y(a)
  error = sqrt((x_t(i)-X(a))^2 + (y_t(i)-Y(a))^2);
  plot(i,error,'.')
  hold on
end 