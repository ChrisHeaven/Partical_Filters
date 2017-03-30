clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0; 65,0; 65,45; 40,45; 40,65; 111,65; 111,110; 0,110];  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

botSim.setBotPos([29, 38]);
botSim.setBotAng(pi/5 * 4);

botSim.drawMap();
botSim.drawBot(3); % draws robot with direction indicator with a length 3
%the length of the directin indicator does not matter, it just makes it
%easy to see where the robot is pointing
botSim.setSensorNoise(1);
botSim.setScanConfig(botSim.generateScanConfig(16));
[distance crossingPoint]  = botSim.ultraScan(); %perfoms simulated ultrasound scan
dis = round(distance)
better_angle = pre_turn(dis)
%botSim.turn(better_angle);

botSim.drawScanConfig(); %draws current scan configuration
botSim.drawBot(3);
scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3); %draws crossingpoints
% 
% botSim.setSensorNoise(1);
% clf; axis equal; hold on; botSim.drawMap();  %resets drawing area
% botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
% botSim.drawBot(3);
% [distance crossingPoint]  = botSim.ultraScan()
% scatter(crossingPoint(:,1),crossingPoint(:,2),'marker','o','lineWidth',3);