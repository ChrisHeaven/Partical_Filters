function [path] = final_path(move)
flag = 0;
move_size = size(move, 1);
count = 0;

for i = 1:move_size
    if abs(move(i, 1)) == 180 && i ~= 1
        count = count + 1;
        if flag == 0 && i > 1
            previous_move = move(i-1, 2);
            previous_turn = move(i-1, 1);
            sum_move = previous_move;
            first = i - 1;
            flag = 1;
        end
        
        if mod(count, 2) == 0
            sum_move = sum_move + move(i, 2);
        else
            sum_move = sum_move - move(i, 2);
        end

    else
        if count ~= 0
            if sum_move > 0
                sum_turn = previous_turn;
            else
                sum_turn = previous_turn + 180;
                sum_move = sum_move*(-1);
            end
            route(first, 1) = sum_turn;
            route(first, 2) = sum_move;
        end
        flag = 0;
        count = 0;

        route(i, 1) = move(i, 1);
        route(i, 2) = move(i, 2);
    end
end
path(:,1) = route(:, 1);
path(:,2) = route(:, 2);
end