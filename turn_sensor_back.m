function current_degree = turn_sensor()   % direction 1:turn right, -1:turn left
    global mS;
 
    mS.Power = -30;
    mS.TachoLimit = 360;
    mS.Stop('brake');
    mS.SendToNXT();
    mS.WaitFor();
end