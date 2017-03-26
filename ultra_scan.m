function [bot_scan] = ultra_scan(resolution)
global sensorT;
sensorT = sensorT * -1;
if sensorT == 1
   dir = 1;
else
   dir = -1;
end    
%% turn after scan
    pause(0.1);
    bot_scan(:,1) =  GetUltrasonic(SENSOR_2); 
    n = round(360/resolution);
	for i = 2:resolution
		turn_sensor( n ,dir);
%         pause(0.03);
        dis = GetUltrasonic(SENSOR_2);
        if dis  == -1;
            bot_scan(i, :) = GetUltrasonic(SENSOR_2); 
        else
            bot_scan(i, :) = dis;
        end
    end
    if dir == -1
        bot_scan = flipud(bot_scan);
    end    
   %turn_sensor(n, 1);
   % turn_sensor(360,-1);
   %turn_sensor_back(resolution);
%    
%%    scan while turning
%     mS = NXTMotor('A'); 
%     mS.Power = 25 ;
%     mS.TachoLimit = 360 / resolution * (resolution - 1);
%     mS.Stop('brake');
%    
%     
%     %pause(0.05);
%     mS.SendToNXT();
% 	for i = 1:resolution
%         bot_scan(i, :) = GetUltrasonic(SENSOR_2)
%         pause(0.1);
%         
%     end
%     mS.WaitFor();
%    %turn_sensor(n, 1);
%    %turn_sensor(360,-1);
%    turn_sensor_back(resolution);
end