clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0; 65,0; 65,45; 40,45; 40,65; 110,65; 110,110; 0,110];;  %default map
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.

botSim.setBotPos([20 20]);
botSim.setBotAng(pi/2);

botSim.drawMap();
botSim.drawBot(3); % draws robot with direction indicator with a length 3
%the length of the directin indicator does not matter, it just makes it
%easy to see where the robot is pointing
botSim.setSensorNoise(2);
botSim.setScanConfig(botSim.generateScanConfig(4));
[distance crossingPoint]  = botSim.ultraScan() %perfoms simulated ultrasound scan
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