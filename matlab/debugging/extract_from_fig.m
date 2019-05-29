%% % OPEN FIGURE using FINDOBJ

%%USING GET
openfig('example.fig')

ax=gca;
lines=get(ax,'Children')
for i=1:length(lines)
   data_line(i).X=get(lines(i),'XData')
   data_line(i).Y=get(lines(i),'YData')
   data_line(i).Z=get(lines(i),'ZData')
end


figure
% axes
% hold on
 plot3(data_line.X,data_line.Y,data_line.Z,'.')
hold on 

% %% OPEN FIGURE 2
% openfig('exemple_compare')
% ax1=gca;
% lines2=get(ax1,'children')
% for i=1:length(lines2)
%    data_line2(i).X=get(lines2(i),'XData')
%    data_line2(i).Y=get(lines2(i),'YData')
%    data_line2(i).Z=get(lines2(i),'ZData')
% end
% plot3(data_line2.X,data_line2.Y,data_line2.Z,'.')
