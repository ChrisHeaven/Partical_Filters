clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
%map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
map=[0,0; 65,0; 65,45; 40,45; 40,65; 111,65; 111,110; 0,110];
startPositions =  [50,20; 30,20; 50,70 ]; %These will change
targetPositions = [80,80; 100,20; 230,70]; %These will change
i = 1;

% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;

botSim.setSensorNoise(2);
botSim.setTurningNoise(1/180);

%botSim.randomPose(10); %puts the robot in a random position at least 10cm away from a wall
target = botSim.getRndPtInMap(10);  %gets random target.
botSim.setBotPos(startPositions(i,:));
target = targetPositions(i,:);

tic %starts timer

%your localisation function is called here.
returnedBot = localise(botSim,map,target); %Where the magic happens

resultsTime = toc %stops timer

%calculated how far away your robot is from the target.
resultsDis =  distance(target, returnedBot.getBotPos())