function current_degree = turn_sensor(degree)   % direction 1:turn right, -1:turn left

    mS = NXTMotor('A');
    n = 360 / degree * (degree - 1) + 5;
    mS.Power = -100;
    mS.TachoLimit = n;
    mS.Stop('brake');
    mS.SendToNXT();
    mS.WaitFor();

end