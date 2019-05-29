
% Create a sample Figure
example_fig=figure
t1=0:.1:2*pi;
y1=cos(t1);
t2=0:.1:4*pi;
y2=sin(t2);
plot(t1,y1,'r',t2,y2,'b')
% Save the Figure
savefig(example_fig,'prova.fig')
% Close the figure
close(example_fig)
% Open the Figure
openfig('prova.fig')

% Dot Notation
% Get the axes handle
ax=gca;
% Get the axes Children handle
lines=ax.Children
% Loop over the Children to extract the data
for i=1:length(lines)
   data_line(i).X=lines(i).XData
   data_line(i).Y=lines(i).YData
   data_line(i).Z=lines(i).ZData
end

% Test the code

% Open a new figure
figure
% Create an axes
axes
hold on
% Loop over the extracted data to plot them
for i=1:length(data_line)
   plot(data_line(i).X,data_line(i).Y)
end

% Same approach, using "get" instead of the Dot Notation
close gcf

openfig('prova.fig')

% Using "get"
ax=gca;
lines=get(ax,'Children')
for i=1:length(lines)
   data_line(i).X=get(lines(i),'XData')
   data_line(i).Y=get(lines(i),'YData')
   data_line(i).Z=get(lines(i),'ZData')
end

figure
axes
hold on
for i=1:length(data_line)
   plot(data_line(i).X,data_line(i).Y)
end
