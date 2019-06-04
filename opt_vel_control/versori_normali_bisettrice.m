%calcola direzione della velocità
% wps= [Position.pose; Position.Pose; Position.Pose ..];

% waypoints=[ wps(1).X wps(2).X wps(3).X wps(4).X wps(5).X;
%            wps(1).Y wps(2).Y wps(3).Y wps(4).Y wps(5).Y;
%            wps(1).Z wps(2).Z wps(3).Z wps(4).Z wps(5).Z];

%tiene conto dello state_start 

ws =[3.0840 1.0 3.0 3.3;10.4464 5.0 9.0 10.0;0.9356 2.0 6.0 9.4] %[ xxxxx, yyyyy,zzzzz]

[a , n_wps]= size(ws);

versore_ort = zeros(3,n_wps-1) 

for i= 2: n_wps-1

    dir_recta1= ([ws(1,i) ws(2,i) ws(3,i)] - [ws(1,i-1) ws(2,i-1) ws(3,i-1)])
    dir_recta2= ([ws(1,i+1) ws(2,i+1) ws(3,i+1)] - [ws(1,i) ws(2,i) ws(3,i)])

    modulo_vett_1= norm(dir_recta1)
    modulo_vett_2= norm(dir_recta2)

    %calcola versore recta_1,2
    versore_recta1= dir_recta1/modulo_vett_1
    versore_recta2= dir_recta1/modulo_vett_2

    %calcola vettore direzione della retta ortogonale alla bisettrice

    vettore_ort= versore_recta2 + versore_recta1
    versore_ort(:,i-1)= vettore_ort/ norm(vettore_ort)


end 
