function flag = speedUp(distance,dirction) %ditance in cm; dirction 1:forward -1: backward
    %d=4.4 %cm
%     mL=NXTMotor('C');
%     mR=NXTMotor('B');

    global mL;
    global mR;
    if distance ~= 0
        MaxTurn = 100;
        NumberTurn = 0;
        wheel=13.82;
    
        mL.ActionAtTachoLimit = 'Brake';
        mR.ActionAtTachoLimit = 'Brake';
    
        n=round(distance/wheel*360);
    
        mL.Power = 100*dirction;
        mR.Power = 100*dirction;
    
%     mL.TachoLimit = MaxTurn;
%     mR.TachoLimit = MaxTurn;
        mL.TachoLimit = n;
        mR.TachoLimit = n;    
   
%     NumberTurn = n / MaxTurn;
    
        mL.SendToNXT();
        mR.SendToNXT();  
     
        mL.WaitFor();
        mR.WaitFor();
    
%     for i=1: NumberTurn;   
%         mL.SendToNXT();
%         mR.SendToNXT();
%         mL.WaitFor();
%         mR.WaitFor();
%         j = colision_det()
%         if  j;
%             flag=-1;
%             break;
%         end
%      end  
    end
end