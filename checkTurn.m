function flag = checkTurn(turning)   % direction 1:turn right, -1:turn left
flag = 0;
len = size(turning);
LL = len(2);
if LL >= 2
    if abs(turning(LL)) == 180 && abs(turning(LL - 1)) == 180
        flag = 1;
    end
end

end