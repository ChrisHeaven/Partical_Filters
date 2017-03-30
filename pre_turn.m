function better_angle = pre_turn(pre_scan)
[scan_, number] = sort(pre_scan);
better_angle = 0;
for i = 1:16
    diff1 = abs(pre_scan(mod(number(i), 16) + 1) - scan_(i));
    index = number(i) - 1;
    if index == 0
        index = 16;
    end
    diff2 = abs(pre_scan(index) - scan_(i));
    
    if diff1 < diff2
        min_diff = diff1;
        max_diff = diff2;
        flag = 1;
    else
        min_diff = diff2;
        max_diff = diff1;
        flag = 2;
    end
    
    if min_diff <= 3 && max_diff <= 8
        if min_diff <= 2 && flag == 1
            better_angle = (number(i) - 1) * 360 / 16 + 360/32;
        else
            if min_diff <= 2 && flag == 2
                better_angle = (number(i) - 1) * 360 / 16 - 360/32;
            else
                if min_diff > 2
                    better_angle = (number(i) - 1) * 360 / 16;
                end
            end
        end
        break;
    end
end