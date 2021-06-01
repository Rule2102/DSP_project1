% Script for modifying existing matlab figures into a standardized publication format.
% Ivan Petric, 12.3.2021.
% Script uses pre-existing library (addpath('lib')). You can modify Plot.m
% script in the library for some default formating that it uses, or you can
% use lib's commands in this script directly (preferable).
% Important commands for publication are: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% k_scaling -> defines how big will the figure be on your screen. It scales
% the figure's default width and height.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% width -> default is 3.5 * k_scaling. 3.5 is in inches -> width of latex
% column.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% k_width_height = 2 -> width/height ratio (1:2 used here)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% movegui('south') -> nothing special, just moves figure down on the screen
% so that it doesnt get outside of monitor space.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plt.FontSize = 12*k_scaling -> font size (choose same pt as in Latex (12
% is standard)). This will change axis and legend font. Fonts of text boxes
% are not modified by this script, you must do it manually before export.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In the end, there is input to whether you finished fine-tuning the figure
% (change colors, move text boxes and legends, change final details). When
% you input anything other than 0 -> script exports figure in required
% format and saves modified .fig file as well (in the directory of the
% script).

clear
close all


file_here = 1;      %% if .fig file is in the same folder as the script - bypass file_location



addpath('lib');

% load previously generated fig file
file_name = 'MSDUstepHIL';
figFile = strcat(file_name,'.fig');
file_location = 'C:\Users\Ivan\GIT\';

if file_here == 1
    read_name = strcat(figFile)
    
else
    read_name = strcat(file_location,figFile)
end

plt = Plot(read_name);

% change properties
% plt.XLabel = 'Time, t (ms)'; % xlabel
% plt.YLabel = 'Voltage, V (V)'; %ylabel
% plt.YTick = [-10, 0, 10]; %[tick1, tick2, .. ]
% plt.XLim = [0, 80]; % [min, max]
% plt.YLim = [-11, 11]; % [min, max]

plt.Interpreter = 'latex';

% plt.Colors = { % three colors for three data set
%     [ 1,      0,       0]
%     [ 0.25,   0.25,    0.25]
%     [ 0,      0,       1]
%     };

plt.LineWidth = [2, 2, 2]; % three line widths
plt.LineStyle = {'-', '-', '-'}; % three line styles

% plt.Markers = {'o', '', 's'};
% plt.MarkerSpacing = [15, 15, 15];
% plt.Legend = {'\theta = 0^o', '\theta = 45^o', '\theta = 90^o'}; % legends

k_scaling = 3;              %IEEE colum is 3.5 inches. This parameter scales it so that you can see it as a bigger picture on your monitor
width = 3.5 * k_scaling;        
k_width_height = 2;         % 2 for IEEE FIGURE. 4 IF YOU WANT TO MAKE SUBFIGURE
height = width / k_width_height;

plt.FontSize = 12*k_scaling;    %% whatever is your font in latex
plt.BoxDim = [width, height];   %[width, height].


movegui('south');

% Save? comment the following line if you do not want to save
saveFile = strcat(file_name,'.png');
finished=input('finished fine-tuning? (0 - no, else - yes) (do not change fonts or box size)');

if finished ~= 0
    plt.export(saveFile);
    plt.export(figFile);   %% if you want to add some fine touch to the figure.
end
