COM_CloseNXT all %prepares workspace
h=COM_OpenNXT(); %look for USB devices
COM_SetDefaultNXT(h); %sets default handle

%% Definiton
global TS;      %Touch Sensor
global USS;     %UltraSound Sensor
global mS;      %motor of Sensor
global mR;      %Right motor
global mL;      %Left motor
global sensorT;

%global cD;      %Angle of Sensor
%global dir;     %Direction of the Sensor

%dir = 1;        %Initialize direction set to right

TS=SENSOR_1;
USS=SENSOR_2;

mL =NXTMotor('C');
mR =NXTMotor('B');
mS =NXTMotor('A');

%% Testing
% sensorT = -1;
% m = ultra_scan(360)
% OpenUltrasonic(USS);
%     m = ultra_scan(4)
%     m = ultra_scan(4);
% move_direct(5,1);
% 
% speedUp(5,1);
%  pause(0.1);
% for i = 1 :9
%     sensorT = mod(i ,2);
%      m(i) = GetUltrasonic(USS);
%      turn_sensor(5,1);
%    m = ultra_scan(4)
%     n(:,i) = m;
% end
% % n
% m
% pause(0.1);
% m = GetUltrasonic(USS)
% CloseSensor(USS);
   turn(-360,-1);
%   move_direct(0,1)

% mS.Power = 25 ;
% mS.TachoLimit = 360 / 10 * 9;
% mS.Stop('brake');
% mS.SendToNXT();
% mS.WaitFor();
% 
% mS.Power = -50 ;
% mS.TachoLimit = 360;
% mS.SendToNXT();
% mS.WaitFor();


%  frequency = { 130.81, 138.59, 138.59, 146.83,
%          155.56, 155.56, 164.81, 174.61, 185.0, 185.0, 196.0,
%          207.65, 207.65, 220.0, 233.08, 233.08, 246.94, 261.63,
%          277.18, 277.18, 293.66, 311.13, 311.13, 329.63, 349.23,
%          369.99, 369.99, 392.0, 415.3, 415.3, 440.0, 466.16, 466.16,
%          493.88, 523.25, 554.37, 554.37, 587.33, 622.25, 622.25,
%          659.26, 698.46, 739.99, 739.99, 783.99, 830.61, 830.61,
%          880.0, 932.33, 932.33, 987.77, 1046.5 }
NXT_PlayTone(440, 500);



%    degree = 60;
%     direction = 1;
%     if degree > 180;
%         degree = 360 - degree;
%         direction = 1;
%     end
%     wheel=13.6;  %The length of the wheel 
%     length=12.25; %The length between wheels 15.4 12.4 10
%      
%     mL.ActionAtTachoLimit = 'brake';
%     mR.ActionAtTachoLimit = 'brake';
%     
%     n=round(length * pi/wheel*degree);
%    
%     mL.Power = 30*direction;
%     mR.Power = -30*direction;
%     
%     mL.TachoLimit = 0;
%     mR.TachoLimit = 0;
%     
%     mL.SendToNXT();
%     mR.SendToNXT();
%     
%     mL.WaitFor();
%     mR.WaitFor();