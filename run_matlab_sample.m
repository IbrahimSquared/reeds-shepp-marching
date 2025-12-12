clc; clear;

% script that reads the output of RS_marching executable and visualizes the distance map, theta map, and path
% can use expfig to save high quality figures
% same visualization style as in the paper

addpath(genpath('/home/ibrahim/projects/reeds-shepp-marching/misc/'))
% save console log, copies of output files, and primitives.json to destDir
destDir = '/home/ibrahim/projects/reeds-shepp-marching/test';

% make sure destination exists before logging
if ~exist(destDir, 'dir')
    mkdir(destDir);
end

% start logging the MATLAB console to a file in destDir
diaryFile = fullfile(destDir, 'matlab_console.log');
diary(diaryFile);   % everything printed from here on goes into this file

executablePath = 'sudo /home/ibrahim/projects/reeds-shepp-marching/RS_marching';
status = system(executablePath);
% system([executablePath, ' &']);
% [status, cmdout] = system([executablePath, ' &']);
% if status ~= 0
%     error('Failed to start the executable: %s', cmdout);
% end
if status ~= 0
    error('Error running the executable');
else 
    disp('Executable ran successfully');
end

filename_distanceMap = "output/distance.txt";
T_distanceMap = readtable(filename_distanceMap,'Delimiter',' ');
distanceMap = T_distanceMap.Variables;
distanceMap(distanceMap >= 50000) = inf;
distanceMap(isinf(distanceMap)) = 0;

filename_thetas = "output/thetas_.txt";
T_thetas = readtable(filename_thetas,'Delimiter',' ');
thetas = T_thetas.Variables;

filename_cameFrom = "output/cameFrom.txt";
T_cameFrom = readtable(filename_cameFrom,'Delimiter',' ');
cameFrom = T_cameFrom.Variables;

filename_origin = "output/origin.txt";
T_origin = readtable(filename_origin, 'Delimiter',' ');
origin = T_origin.Variables;

filename_lightSources = "output/lightSources.txt";
T_lightSources = readtable(filename_lightSources, 'Delimiter',' ');
lightSources = T_lightSources.Variables;

filename_occupancy = "output/occupancy.txt";
T_occupancy = readtable(filename_occupancy, 'Delimiter',' ');
occupancy = T_occupancy.Variables;

[nx, ny] = size(distanceMap);
minimum = min(min(distanceMap));
maximum = max(max(distanceMap));
distanceMap(distanceMap == 0) = maximum;
z = 1.05;


% Read the JSON file
filename = 'primitives.json';  % replace with your filename
fileID = fopen(filename);
rawData = fread(fileID, inf);
strData = char(rawData');
fclose(fileID);
jsonData = jsondecode(strData);

figure(1)
set(gcf, 'Name', 'Distance Map')
clf
% subplot(1,2,1)
% d = 50000;
% distanceMap(isinf(distanceMap)) = 0;
% distanceMap(distanceMap>d) = 1e5;
mesh(distanceMap,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(jet)
if (minimum ~= maximum)
    caxis([minimum maximum])
end
cmap = colormap;
cmap(1,:) = [0 0 0];
% cmap(1,:) = [1 0 0];
cmap(end,:) = [0 0 0];
colormap(cmap)

view(0,90)
axis equal
axis([1 ny 1 nx])
% axis([360 550 340 460])
hold on

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

ax = gca;
ax.Position = [ax.Position(1), ax.Position(2)+0.01, ax.Position(3), ax.Position(4)];
% Add a colorbar
cb = colorbar;
cb.FontWeight = 'bold';
cb.FontSize = 14;
%     % Get the current position of the colorbar
%     cbPosition = cb.Position;
%     % Adjust the colorbar position to move it slightly to the left
%     cb.Position = [cbPosition(1), cbPosition(2), cbPosition(3), cbPosition(4)];
%     % Adjust the plot position to avoid overlap
contour3(distanceMap+0.05, linspace(minimum, maximum, 40), 'LineWidth', 3,...
        'EdgeColor', 'k');
grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca, 'ztick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

% distanceMap(isinf(distanceMap)) = 0;
% minimum = min(min(distanceMap));
% maximum = max(max(distanceMap));
% contour3(distanceMap+0.05, linspace(minimum, maximum, 80), 'LineWidth', 3,...
%         'EdgeColor', 'k');

maximum = max(max(distanceMap));
for i = 1:size(lightSources, 1)
    if i == 1 || i == 2
      pt_ = lightSources(i,:);
      pt_ = [pt_(1)+1, pt_(2)+1, pt_(3)+pi/2];
      if i == 1
        color = 'green';
      else
        color = 'red';
      end
%       plot3(pt_(1), pt_(2), maximum+10,'o',...
%           'MarkerFaceColor','white', 'MarkerEdgeColor',color,...
%           'MarkerSize', 10, 'LineWidth', 2)
      plot3(pt_(1), pt_(2), maximum+10,'o',...
          'MarkerFaceColor','white', 'MarkerEdgeColor', color, ...
          'MarkerSize', 15, 'LineWidth', 3)
      % plot arror for the light source direction given by pt_(3)
      L = 40;
      plot3([pt_(1) pt_(1)+L*cos(pt_(3))], ...
            [pt_(2) pt_(2)+L*sin(pt_(3))], [maximum+10 maximum+10], ...
            color, 'LineWidth', 3)
    end
end

% return

% figure(5)
figure(2)
set(gcf, 'Name', 'Occupancy Map')
clf
mesh(occupancy,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(gray)
% colormap(jet)
view(0,90)
axis equal
axis([1 ny 1 nx])
% axis([360 550 340 460])
hold on
% return

% the exec outputs two paths: path_1.txt and path_2.txt
% those are the two possible RS paths between start and goal
% they're obtained by reversing orientations, computing the path, 
% then reversing the primitives

% filename_thetas = "output/path_1.txt";
% T_path_1 = readtable(filename_thetas,'Delimiter',' ');
% path_1 = T_path_1.Variables;
% filename_thetas = "output/path_2.txt";
% T_path_2 = readtable(filename_thetas,'Delimiter',' ');
% path_2 = T_path_2.Variables;
% z_path = 1.05*ones(1,size(path_1,1));
% plot3(path_1(:,1), path_1(:,2), z_path, '-',...
%     'color', 'red', 'LineWidth', 4);
% z_path = 1.05*ones(1,size(path_2,1));
% plot3(path_2(:,1), path_2(:,2), z_path, '-',...
%     'color', 'blue', 'LineWidth', 4);

% Convert JSON data to MATLAB cell
pathSegObj = struct;
pathSegObj.MotionTypes = [jsonData.motionTypes(:)];
pathSegObj.MotionLengths = [jsonData.motionLengths(:)];
pathSegObj.MotionDirections = [jsonData.motionDirections(:)];
pathSegObj.MinTurningRadius = jsonData.MinTurningRadius;
pathSegObj.waypoints = [jsonData.waypoints(:)];
x1 = pathSegObj.waypoints(1).x;
y1 = pathSegObj.waypoints(1).y;
theta1 = pathSegObj.waypoints(1).theta;
step = 1e-3;
full_black_path = plot_RS_path(pathSegObj,theta1,x1,y1,1,0,0,z,step);
z_path = 1.05*ones(1,size(full_black_path,1));
plot3(full_black_path(:,1), full_black_path(:,2), z_path, '-',...
    'color', 'magenta', 'LineWidth', 4);

for i = 1:size(pathSegObj.waypoints, 1)
    if i == 1 || i == size(pathSegObj.waypoints, 1)
      pt_ = pathSegObj.waypoints(i);
      if i == 1
        pt_ = [pt_.x, pt_.y, pt_.theta];
        color = 'green';
      else
        pt_ = [pt_.x, pt_.y+1, pt_.theta];
        color = 'red';
      end
      plot3(pt_(1), pt_(2), z,'o',...
          'MarkerFaceColor','white', 'MarkerEdgeColor', color, ...
          'MarkerSize', 20, 'LineWidth', 3)
      % plot arror for the light source direction given by pt_(3)
      L = 40;
      plot3([pt_(1) pt_(1)+L*cos(pt_(3))], ...
            [pt_(2) pt_(2)+L*sin(pt_(3))], [z z], ...
            color, 'LineWidth', 3)
    end
end

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca, 'ztick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

% addpath(genpath('~/expfig/'))
% export_fig('samples/test_waffle_3.png', '-r300', '-transparent', '-png')
% export_fig werkgj -r300 -png -transparent
% export_fig('output/visibilityField.pdf', '-transparent', '-pdf', '-painters', '-nocrop', '-pdf', '-q101')
% expfig('output/visibilityField.pdf')

% Copy everything from output/ into destDir
% (contents only, not the 'output' folder itself)
copyfile(fullfile('output', '*'), destDir);

% Copy primitives.json into the same directory
copyfile('primitives.json', fullfile(destDir, 'primitives.json'));

% stop logging the console
diary off;

return
figure(3)
set(gcf, 'Name', 'cameFrom')
clf
mesh(cameFrom,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(jet)
colormap(lines)
view(0,90)
axis equal
axis([1 ny 1 nx])
hold on
grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));

return

thetas = thetas * 180/pi;
thetas(thetas >= 50000) = inf;
thetas(isinf(thetas)) = 0;
minimum = min(min(thetas));
maximum = max(max(thetas));
thetas(thetas == 0) = maximum;

figure(4)
set(gcf, 'Name', 'Theta Map')
clf
mesh(thetas,'FaceLighting','phong','FaceColor','interp',...
    'AmbientStrength',1.0, 'EdgeColor', 'interp','FaceAlpha','1.0');
colormap(jet)
caxis([minimum maximum])
cmap = colormap;
cmap(1,:) = [0 0 0];
% cmap(1,:) = [1 0 0];
cmap(end,:) = [0 0 0];
colormap(cmap)

view(0,90)
axis equal
axis([1 ny 1 nx])
% axis([360 550 340 460])
hold on

grid off
set(gca, 'xtick', [-1e6 1e6]);
set(gca, 'ytick', [-1e6 1e6]);
set(gca,'LooseInset',get(gca,'TightInset'));
ax = gca;
ax.Position = [ax.Position(1), ax.Position(2)+0.01, ax.Position(3), ax.Position(4)];
% Add a colorbar
cb = colorbar;
cb.FontWeight = 'bold';
cb.FontSize = 14;
%     % Get the current position of the colorbar
%     cbPosition = cb.Position;
%     % Adjust the colorbar position to move it slightly to the left
%     cb.Position = [cbPosition(1), cbPosition(2), cbPosition(3), cbPosition(4)];
%     % Adjust the plot position to avoid overlap
contour3(thetas+0.05, linspace(minimum, maximum, 40), 'LineWidth', 3,...
        'EdgeColor', 'k');
hold off

% addpath(genpath('~/expfig/'))
% export_fig('samples/test_waffle_3.png', '-r300', '-transparent', '-png')
