function result = turn(degree,direction) %direction 1:right -1:left
    %d=4.4 %cm
    global mL;
    global mR;
    
    if degree > 180;
        degree = 360 - degree;
        direction = 1;
    end

    wheel = 13.6;  %The length of the wheel 
    length = 12.25; %The length between wheels 15.4 12.4 10
     
    mL.ActionAtTachoLimit = 'brake';
    mR.ActionAtTachoLimit = 'brake';
    
    n=round(length * pi / wheel * degree);
   
    mL.Power = 20 * direction;
    mR.Power = -20 * direction;
    
    mL.TachoLimit = n;
    mR.TachoLimit = n;
    
    mL.SendToNXT();
    mR.SendToNXT();
    
    mL.WaitFor();
    mR.WaitFor();
end