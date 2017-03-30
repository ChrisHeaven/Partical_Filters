function current_degree = turn_sensor(degree, direction)   % direction 1:turn right, -1:turn left
    global mS
   
    mS.Power = 30 * direction;
    mS.TachoLimit = degree;
    mS.Stop('brake');
    mS.SendToNXT();
    mS.WaitFor();
end