%% Distance
%initialize variables
% Waypoints
% n_wps=3;

load('wp1.mat')
load('wp2.mat')
load('wp3.mat')
% Real Pose 
load('in_x.mat')
load('in_y.mat')
load('in_z.mat')

% %-------------------------------------------------------------------------
% %use the locìgical not operator in xr to locate the zeros;
% %xr_new es el vector de los indices degli elementi not 0; 
% NO NECESITAS ESTA OPERACION SIEMPRE (implement if buckle)
% xr_new=find(~i_x);  % i_x è la variabile che sta salvata nel file  di nome 'in_x.mat')
% n=xr_new(1,1);
% 
% x_traj=xr(1:n-1); %trova size di x_traj
% y_traj=yr(1:n-1);
% z_traj=zr(1:n-1);
%--------------------------------------------------------------------------
%definisci matrice di punti della rel traj

% real_traj=[x_traj y_traj z_traj];
real_traj=[i_x i_y i_z];

%punto in cui arriva dopo il take off
z0=[0 0 1]; %[0 0 1] nella prova per la tesi 

nwps=[z0;
      w_1;
      w_2;
      w_3];
      
[r , ~]=size(nwps);
%-------------------------------------------------------------------------
%definisco le direzioni 
% v0=[w1(1)-z0(1) w1(2)-z0(2) w1(3)-z0(3)];
% v1=[w2(1)-w1(1) w2(2)-w1(2) w2(3)-w1(3)];
% v2=[w3(1)-w2(1) w3(2)-w2(2) w3(3)-w2(3)];
% 
% dir_recta=[v0;
%            v1;
%            v2];
% load('diretta.mat')   
%-------------------------------------------------------------------------      
%cALCOLA IL NUMERO DI WPS
n_wps=r-1;

%calcola il numero di punti nella real pose x_traj;
[n_traj, c]=size(i_x);

%definisci il vettore delle distanze minime
% d_min=zeros(n_traj,1);

% t=linspace(0,1);
%-------------------------------------------------------------------------
%PLOTTA FIGURA
%plot3(x_traj,y_traj,z_traj,'b')
plot3(i_x, i_y, i_z,'b')

hold on 
plot3(w_1(1),w_1(2),w_1(3),'*');
hold on 
plot3(w_2(1),w_2(2),w_2(3),'*');
hold on 
plot3(w_3(1),w_3(2),w_3(3),'*');
%-------------------------------------------------------------------------
k=1;
d=zeros(k,1);
point=zeros(k,3);

for i=1:n_wps
    %equazione parametrica della retta conoscendo il primo punto e la
    %direzione
%     syms t
%     line=nwps(i,:)+t*dir_recta(i,:);    
    a(i,:) = (nwps(i+1,:) - nwps(i,:));
    %plotta retta
    syms ts
    line=nwps(i,:)+ts*a(i,:);  
    
    hold on 
    t=linspace(0,1);
    x=nwps(i,1)+t*a(i,1);
    y=nwps(i,2)+t*a(i,2);
    z=nwps(i,3)+t*a(i,3);
    plot3(x,y,z,'r')
    grid on
    hold on 
    for j=k:50:n_traj  %prima estaba i
        %Calcola proiezione ortogonale
        %Calcola piano passante per P(j) y ortogonale alla direzione i_esima
        syms x_p y_p z_p
        P=[x_p y_p z_p];
        %definisco il piano passante per P-P(j) [punto della traiettoria
        %reale]  y la normale al piano vect(W2-W1);
        realdot = @(u, v) u*transpose(v); %solo risultati reali;
%        planefunction=realdot(dir_recta(1,:),P-real_traj(j,:)); %plane_fun=0
         planefunction=realdot(a(i,:),P-real_traj(j,:)); %plane_fun=0
        % ora vedo per quale punto della retta paramentrizzata con t è
        % soddisfattal'equazione del piano, cioè qual è il punto in comune 
        % fra la retta P(i)P(i+1) ed il piano ortogonale ad AB che ho fatto prima;
        %line replace P nella equazione del piano;
        newfunction = subs(planefunction, P, line); 
        t0 = solve(newfunction); % valore del paramentro t per cui PFUN=0;
        
        % una volta trovato t0 posso calcolare il punto intersezione fra il piano e la direzione V(i)
        % andando a valutare che punto sulla retta corrisponde a t0.
        
        if t0>=0 && t0<=1
            point(k,:) = subs(line, ts, t0);
            hold on
            xp=point(:,1);
            yp=point(:,2);
            zp=point(:,3);
            plot3(xp,yp,zp,'o')
         %t0 replace t in line equation
        % controprova: subs(planefunction, P, point);
        
%           if nwps(i,1)<= point(k,1) && point(k,1) <= nwps(i+1,1) && ...
%             nwps(i,2)<= point(k,2) && point(k,2) <= nwps(i+1,2) && ... 
%             nwps(i,3)<= point(k,3) && point(k,3) <= nwps(i+1,3)
             if 1
        %cross product
        
%         C_product=cross((nwps(i+1,:)-nwps(i,:)),(nwps(i,:)-real_traj(j,:)));
%         % distancia entres nwps(i+1) y nwps(i)
%         D_recta= norm(nwps(i+1,:)-nwps(i,:));
%         % distancia total
%         d(k,1)=norm(C_product)/D_recta;
          hold on
          plot3(real_traj(j,1),real_traj(j,2),real_traj(j,3),'+');
          hold on 
          d(k,1)=sqrt((point(k,1)-real_traj(j,1))^2 + (point(k,2)-real_traj(j,2))^2 + (point(k,3)-real_traj(j,3))^2);
          
        k=k+1;
        else 
            break
           
             end 
        end 
        
    end 
end 

%nota: dal momento che tieni il punto di intersezione, puoi anche
%direttamente trocarti la distanza fra p(j) e p(inters), il risultato è
%uguale;

% %%plotta linee
% t=linspace(0,1);
% x=nwps(i,1)+t*dir_recta(i,1);
% y=nwps(i,2)+t*dir_recta(i,2);
% z=nwps(i,2)+t*dir_recta(i,3);
% plot3(x,y,z,'b')
%
% %con questi valori funziona:
% x= nwps(1,1)+t*(nwps(2,1)-nwps(1,1));
% y= nwps(1,2)+t*(nwps(2,2)-nwps(1,2));
% z= nwps(1,3)+t*(nwps(2,3)-nwps(1,3));

