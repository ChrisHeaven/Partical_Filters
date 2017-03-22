function [bot_scan] = ultra_scan(resolution)
    global USS;
    pause(0.05);
    bot_scan(:,1) =  GetUltrasonic(USS); 
	for i = 2:resolution
        n = round(360/resolution);
		turn_sensor( n , 1);
        pause(0.05);
        bot_scan(i, :) = GetUltrasonic(USS);      
    end
    turn_sensor(n, 1);
   % turn_sensor(360,-1);
   turn_sensor_back();
end