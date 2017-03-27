function x = solveF(a,b,c,scans)
degree = 360 / scans ;
flag = 0;
len = 0;
error = 0;
while flag == 0
    len = len + 0.001;
    error = (asin(len / c) - asin(len / b)) * 180 / pi;
    if abs(error - degree) < 0.01
        flag = 1;
    end
    if len > b
        disp('FAILED')
        break;
    end
end
x = acos(len/c) * 180 /pi;
end
