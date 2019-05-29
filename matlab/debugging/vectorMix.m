clear all

a = ones(10,1);
b = ones(10,1)*2;
c = ones(10,1)*3;
d = ones(10,1)*4;
f = ones(10,1)*5;
g = ones(10,1)*6;

% for i=1:length(a)
%     if(i==1)
%         res(i) = a(i);
%         res(i+1) = b(i);
%         res(i+2) = c(i);
%     else
%         res(i*3) = a(i);
%         res(i*3) = b(i);
%         res(i*4) = c(i);
%     end
%     
% end
% length(res)
% res

res(1:6:length(a)*6) = a;
res(2:6:length(a)*6) = b;
res(3:6:length(a)*6) = c;

res(4:6:length(a)*6) = d;
res(5:6:length(a)*6) = f;
res(6:6:length(a)*6) = g;