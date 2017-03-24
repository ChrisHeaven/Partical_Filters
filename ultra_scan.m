function [bot_scan] = ultra_scan(resolution)
%% turn after scan
    pause(0.1);
    bot_scan(:,1) =  GetUltrasonic(SENSOR_2); 
    n = round(360/resolution);
	for i = 2:resolution
		turn_sensor( n , 1);
%         pause(0.03);
       bot_scan(i, :) = GetUltrasonic(SENSOR_2);      
    end
   %turn_sensor(n, 1);
   % turn_sensor(360,-1);
   turn_sensor_back(resolution);
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