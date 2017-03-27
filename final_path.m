function [path] = final_path(move)
flag = 0;
move_size = size(move, 1);
count = 0;
%special = 0;
%index = find(abs(move(:, 1)) == 180);
% if index(1) == 1
%     first = 0;
% else
%     first = index(1) - 1;
% end
%route = zeros(size(move, 1), 2);

for i = 1:move_size
    if abs(move(i, 1)) == 180 && i ~= 1
        count = count + 1;
        if flag == 0 && i > 1
            previous_move = move(i-1, 2);
            previous_turn = move(i-1, 1);
            sum_move = previous_move;
            first = i - 1;
            flag = 1;
%         else
%             if flag == 0 && i == 1
%                 previous_move = 0;
%                 previous_turn = 0;
%                 sum_move = previous_move;
%                 first = 0;
%                 flag = 1;
%             end
        end
        
        if mod(count, 2) == 0
            sum_move = sum_move + move(i, 2);
        else
            sum_move = sum_move - move(i, 2);
        end
        
        if sum_move > 0
            sum_turn = previous_turn;
        else
            sum_turn = previous_turn + 180;
            sum_move = sum_move*(-1);
        end
% 
%         if first == 0
%             special = 1;
%             backup_sum_turn = sum_turn;
%             backup_sum_move = sum_move;
%         else
            route(first, 1) = sum_turn;
            route(first, 2) = sum_move;
%         end

    else
        flag = 0;
        count = 0;
%         route_(size(route, 1) + 1, 1) = move(i, 1);
%         route_(size(route, 1) + 1, 2) = move(i, 2);
        route(i, 1) = move(i, 1);
        route(i, 2) = move(i, 2);
    end
end

% if special == 1
% %     route_(:, 1) = [backup_sum_turn; route(:, 1)];
% %     route_(:, 2) = [backup_sum_move; route(:, 2)];
% %     route = route_;
%     for i = 2:(size(route, 1) + 1)
%         route_(i, 1) = route(i-1, 1);
%         route_(i, 2) = route(i-1, 2);
%     end
%     route_(1, 1) = backup_sum_turn;
%     route_(1, 2) = backup_sum_move;
%     route = route_;
% end
path(:,1) = route(:, 1);
path(:,2) = route(:, 2);

end


% if count > 1
%     if sum_move > 0
%         sum_turn = previous_turn;
%     else
%         sum_turn = previous_turn + 180;
%         sum_move = sum_move*(-1);
%     end
    
%     index = find(abs(move(:, 1)) == 180);
%     counter = 1;
%     for i = 1:move_size
%         if i < first 
%             path(i, :) = move(i, :); 
%         else
%             if i > first + size(index)
%                 path(first + counter, :) = move(i, :);
%                 counter = counter + 1;
%             end
%         end
%     end
%     if first == 0
%         path_(:, 1) = [sum_turn; path(:, 1)];
%         path_(:, 2) = [sum_move; path(:, 2)];
%         path = path_;
%     else
%         path(first, 1) = sum_turn;
%         path(first, 2) = sum_move;
%     end
% else
%     path = move;
% end