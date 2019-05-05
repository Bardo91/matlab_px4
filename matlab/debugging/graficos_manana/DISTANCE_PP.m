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
    for j=k:200:n_traj  %prima estaba i
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
        point(k,:) = subs(line, ts, t0); %t0 replace t in line equation
        % controprova: subs(planefunction, P, point);
        
%           if nwps(i,1)<= point(k,1) && point(k,1) <= nwps(i+1,1) && ...
%             nwps(i,2)<= point(k,2) && point(k,2) <= nwps(i+1,2) && ... 
%             nwps(i,3)<= point(k,3) && point(k,3) <= nwps(i+1,3)
             if 1
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
hold on
xp=point(:,1);
yp=point(:,2);
zp=point(:,3);
plot3(xp,yp,zp,'o')
