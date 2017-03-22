COM_CloseNXT all %prepares workspace
h=COM_OpenNXT(); %look for USB devices
COM_SetDefaultNXT(h); %sets default handle

%% Definiton
global TS;      %Touch Sensor
global USS;     %UltraSound Sensor
global mS;      %motor of Sensor
global mR;      %Right motor
global mL;      %Left motor

%global cD;      %Angle of Sensor
%global dir;     %Direction of the Sensor

%dir = 1;        %Initialize direction set to right

TS=SENSOR_1;
USS=SENSOR_2;

mL =NXTMotor('C');
mR =NXTMotor('B');
mS =NXTMotor('A');

%% Testing
 %turn(180,-1);
 move_direct(200,1)