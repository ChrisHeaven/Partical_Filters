function [botSim] = localise(botSim,map,target)
% This function returns botSim, and accepts, botSim, a map and a target.
% LOCALISE Template localisation function

%% setup code
% you can modify the map to take account of your robots configuration space
modifiedMap = map; % you need to do this modification yourself
botSim.setMap(modifiedMap);

% find the centre of the map
mid_x = (max(map(:, 1)) + min(map(:, 1))) / 2;
mid_y = (max(map(:, 2)) + min(map(:, 2))) / 2;
scans = 4;

% generate some random particles inside the map
num = 600; % number of particles
particles(num, 1) = BotSim; % how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  % each particle should use the same map as the botSim object
    particles(i).randomPose(0); % spawn the particles in random locations
    particles(i).setScanConfig(generateScanConfig(particles(i), scans));
end

% move three steps to open area before ultrascan
% steps = 1;
% while (steps <= 3)
%     botSim.setScanConfig(generateScanConfig(botSim, scans));
%     botScan = botSim.ultraScan(); % get a scan from the real robot.
%     if rand() < 0.99 % most of the time move in the maximum direction
%         [max_distance, max_index] = max(botScan); % find maximum possible distance
%         turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
%         move = max_distance * 0.5; 
%     else % some of the time move in a random direction
%         index = randi(scans); 
%         turn = (index - 1) * 2 * pi/scans;
%         move= botScan(index) * 0.3;
%     end
%     
%     botSim.turn(turn); % turn the real robot.  
%     botSim.move(move); % move the real robot. These movements are recorded for marking 
% 
%     if botSim.debug()
%         hold off; % the drawMap() function will clear the drawing when hold is off
%         botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
%         botSim.drawBot(30,'g'); % draw robot with line length 30 and green
%         drawnow;
%     end
%     steps = steps + 1;
%     pause(0.3);
% end

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; % the filter has not converged yet

for i = 1:num
    past_score(i, 1) = 1/num;
end

botSim.setScanConfig(generateScanConfig(botSim, 16));
pre_scan = botSim.ultraScan();
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
better_angle

botSim.turn(better_angle);
    if botSim.debug()
        hold off; % the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); % draw robot with line length 30 and green
        drawnow;
    end

while(converged == 0 && n < maxNumOfIterations) %particle filter loop
    n = n+1; % increment the current number of iterations
    %get a scan from the real robot.
%     border = [15; 15; 15; 15];
%     for i = 1:size(botScan)
%         botScan(i) = botScan(i) - 15;
%     end
%     if n == 1
%         botSim.setScanConfig(generateScanConfig(botSim, 8));
%         botScan = botSim.ultraScan();
% %         [backup_distance, backup_index] = max(botScan); 
%         botSim.setScanConfig(generateScanConfig(botSim, scans));
%         botScan = botSim.ultraScan();
%     else
        botSim.setScanConfig(generateScanConfig(botSim, scans));
        botScan = botSim.ultraScan();
%     end 
    %% Write code for updating your particles scans
    for i = 1:num
        %past_score(i, 1) = 1/num;
        if particles(i).insideMap() == 1
            particales_scan(:, i) = particles(i).ultraScan();

    %% Write code for scoring your particles    
            for j = 1:scans
                weight = circshift(particales_scan(:, i), j); 
                score(i, j) = 10 / sqrt(sum((weight - botScan).^2));
            end
        else
            score(i, 1) = 0;
        end
        [score(i, 1), max_angle] = max(score(i, :));
        particle_angle = particles(i).getBotAng() - max_angle * 2 * pi/scans;
        particles(i).setBotAng(mod(particle_angle, 2 * pi)); % give the particle the best orientation
    end
    
    sum_score = sum(score);
  
    for i = 1:num
        score(i, 1) = score(i, 1) / sum_score(:, 1);
        score(i, 1) = 0.3*past_score(i, 1) + 0.7*score(i, 1);
        past_score(i, 1) = score(i, 1);
    end


    
    [weight, index] = sort(score, 'descend');
    best_particle = index(1);
    
    for i = 1:num / 5
        score_half(i, 1) = weight(i)^2;
    end
    sum_score_half = sum(score_half);
    
    %% Write code for resampling your particles
    particles_pos = zeros(2, num / 5);
    
    for i = 1:num / 5
        particles_pos(:, i) = getBotPos(particles(index(i)));
    end
    
    count = 1;
    
    for i = 1:num / 5
            particles_num(i, :) = fix(weight(index(i))^2 * num * (1 / sum_score_half));
            new_pos_x = (particles_pos(1, i) - 2.5 + rand(1, particles_num(i, :)) * 5);
            new_pos_y = (particles_pos(2, i) - 2.5 + rand(1, particles_num(i, :)) * 5);
        
        for h = 1:particles_num(i, :)
            particles(count).setBotPos([new_pos_x(h) new_pos_y(h)]);
            count = count + 1;
        end
    end
    
    if count <= num
        for k = count:num
            particles(k).randomPose(0);
        end
    end
    
    %% Write code to check for convergence  
    for i = 1:num
        particles_pos(:, i) = getBotPos(particles(i));
    end

    % one way of estimate the position of real robot    
    sort_x = sort(particles_pos(1, :), 'ascend');
    sort_y = sort(particles_pos(2, :), 'ascend');

    count_3 = 0;
    for i = fix(45 * num/100):fix(55 * num/100)
        count_3 = count_3 + 1;
        x_(1, count_3) = sort_x(i);
        y_(1, count_3) = sort_y(i);
    end
    estimate_x_2 = mean(x_(1, :));
    estimate_y_2 = mean(y_(1, :));

    particles_pos_(:, 1) = getBotPos(particles(best_particle));
    estimate_x_4 = particles_pos_(1, 1);
    estimate_y_4 = particles_pos_(2, 1);

    diff_dis = norm([estimate_x_2, estimate_y_2] - [estimate_x_4, estimate_y_4])

    % another way of estimate the position of real robot
    %for i = 1:num / 5
    %    particles_pos_(:, i) = getBotPos(particles(i));
    %end
    %estimate_x_4 = mean(particles_pos_(1, :));
    %estimate_y_4 = mean(particles_pos_(2, :));

    %x_y = getBotPos(botSim)
    %dis_3 = norm([x_y(1), x_y(2)] - [estimate_x_2, estimate_y_2])
    %dis_4 = norm([x_y(1), x_y(2)] - [estimate_x_4, estimate_y_4])
    
    if mid_x < 66
        convergence_threshold = mid_x / 100;
    else
        convergence_threshold = mid_x / 180;
    end

    stdev_x = std(x_(1, :))
    stdev_y = std(y_(1, :))

    numberofMovingStep1 = n;

    if (stdev_x < convergence_threshold || stdev_y < convergence_threshold) && diff_dis < 5
        break;       % particles has converged so break out of while loop immediately before any movement
    end
    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness) 
    mutation_rate = 0.01;
    
    for i = 1:mutation_rate * num
        particles(randi(num)).randomPose(0);
    end 
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    %turn = 0.5;
    %move = 2;
    
    [backup_distance, backup_index] = max(botScan); 
        
    backup_turn = (backup_index - 1) * 2 * 180 / scans;
    backup_move = backup_distance * 0.3;
    
    danger = 0;
    if botScan(1) + botScan(3) > botScan(2) + botScan(4) && botScan(1) + botScan(3) > 105 && botScan(1) + botScan(3) < 115
        if botScan(2) + botScan(4) > 38 && botScan(2) + botScan(4) < 47
            botScan(2) = 33;
            botScan(4) = 33;
            danger = 1;
        end
    else
        if botScan(2) + botScan(4) > botScan(1) + botScan(3) && botScan(2) + botScan(4) > 105 && botScan(2) + botScan(4) < 115
            if botScan(1) + botScan(3) > 38 && botScan(1) + botScan(3) < 47
                botScan(1) = 33;
                botScan(3) = 33;
                danger = 1;
            end
        end
    end
    
    
    %%Solving hitting wall problem
    [min_distance, min_index] = min (botScan);
    if min_distance < 31 % 22 * 3 ^ (1/2)
        %botScans = botScan;
        botScan
        increase_number = floor(scans / 4);
        for i = 1 : increase_number
            flag = min_index + i;
            if flag > scans
                flag = flag - scans;
            end 
            botScan(flag) = 0 + 2*i;
            flag = min_index - i;
            if flag < 1
                flag = flag + scans; 
            end 
            botScan(flag) = 0 + 2*i;
        end
        botScan(min_index) = 0;
        
    %     if rand() < 0.96 % prefer to move in the maximum direction
    %         [max_distance, max_index] = max(botScan); 
    %         turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
    %         move = max_distance * 0.6 * rand(); % move a random amount of the max distance, but never the entire distance
    %     else % some of the time move in a random direction
    %         index = randi(scans); 
    %         turn = (index - 1) * 2 * pi/scans;
    %         %move = botScan(index) * 0.3;
    %         move = 0;
    %     end
    % else
    %     if rand() < 0.76 % prefer to move in the maximum direction
    %         [max_distance, max_index] = max(botScan); 
    %         turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
    %         move = max_distance * 0.3 * rand(); % move a random amount of the max distance, but never the entire distance
    %     else % some of the time move in a random direction
    %         index = randi(scans); 
    %         turn = (index - 1) * 2 * pi/scans;
    %         %move = botScan(index) * 0.3;
    %         move = 0;
    %     end
    end
    botScan

    
    if danger == 1
        [max_distance, max_index] = max(botScan); 
        turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
        move = max_distance * 0.6 * rand();
    else
        if rand() < 0.76 % prefer to move in the maximum direction
            [max_distance, max_index] = max(botScan); 
            turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
            move = max_distance * 0.2 * rand(); % move a random amount of the max distance, but never the entire distance
        else % some of the time move in a random direction
            index = randi(scans); 
            turn = (index - 1) * 2 * pi/scans;
            %move = botScan(index) * 0.3;
            move = 0;
        end
    end
    
    turned(n)  = turn;
    if checkTurn(turned)
        turn = backup_turn;
        move = backup_move;
    end

    botSim.turn(turn); % turn the real robot.  
    botSim.move(move); % move the real robot. These movements are recorded for marking 
    for i =1:num % for all the particles. 
        particles(i).turn(turn); % turn the particle in the same way as the real robot
        particles(i).move(move); % move the particle in the same way as the real robot
        if particles(i).insideMap() == 0
           particles(i).randomPose(0);
        end
    end
    
    %% Drawing
    % only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; % the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); % draw robot with line length 30 and green
%         for i =1:num
%            particles(i).drawBot(3); %draw particle with line length 3 and default color
%         end
        drawnow;
    end
end

%% Initialise the map
limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax - limsMin; % dimension of the map
res = 5; % sampling resouloution in cm
iterators = dims/res;
iterators = ceil(iterators) + [1 1]; % to counteract 1 based indexing
mapArray = zeros(iterators); % preallocate for speed
target_array_x = round((target(2) - limsMin(2)) / res) + 1;
target_array_y = round((target(1) - limsMin(1)) / res) + 1;

%hold off;
for i = 1:iterators(2)
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1] * res;
        mapArray(i,j) = botSim.pointInsideMap(testPos);
        if mapArray(i,j)
            %plot(testPos(1),testPos(2),'o');%inside map
            x_gap = abs(i - target_array_x);
            y_gap = abs(j - target_array_y);
            if 0 <= x_gap && x_gap <= 1 && 0 <= y_gap && y_gap <= 1
                mapArray(i,j) = 10 + 10;
            end
        end
    end
end

count = 1;
finished = 0;
while (finished == 0)
    for i = 1:iterators(2)
        for j = 1:iterators(1)
            if mapArray(i,j) == 10 + 10 * count
                for h = -1:1
                    for k = -1:1
                        if i + h > 0 && j + k > 0 && i + h <= iterators(2) && j + k <= iterators(1) && mapArray((i + h), (j + k)) == 1
                            mapArray((i + h), (j + k)) = 10 + 10 * (count + 1);
                        end
                    end
                end
            end
        end
    end

    count = count + 1;
    one = 0;
    
    for i = 1:iterators(2)
        for j = 1:iterators(1)
            if mapArray(i, j) == 1
                one = one + 1;
            end
        end
    end
    
    if one == 0
        finished = 1;
    end
end
%mapArray

%% Increase the value of the map at the border of map
border_map = zeros(iterators(2), iterators(1));
for i = 1:iterators(2)
    for j = 1:iterators(1)
        zero_num = 0;
        if mapArray(i,j) ~= 0
            for h = -1:1
                for k = -1:1
                    if i + h > 0 && j + k > 0 && i + h <= iterators(2) + 1 && j + k <= iterators(1) && mapArray((i + h), (j + k)) == 0
                        zero_num = zero_num + 1;
                        border_map(i, j) = 1;
                    end
                end
            end
        end
        mapArray(i,j) = mapArray(i,j) + zero_num * 45;
        if i == 1 && mapArray(1, j) ~= 0
            mapArray(1, j) = mapArray(1, j) + 3 * 45;
        end
    end
end

sub_border_map = zeros(iterators(2), iterators(1));
for i = 1:iterators(2)
    for j = 1:iterators(1)
        one_num = 0;
        if mapArray(i,j) ~= 0
            for h = -1:1
                for k = -1:1
                    if i + h > 0 && j + k > 0 && i + h <= iterators(2) && j + k <= iterators(1) && border_map((i + h), (j + k)) == 1
                        one_num = one_num + 1;
                        sub_border_map(i, j) = 1;
                    end
                end
            end
        end
        mapArray(i,j) = mapArray(i,j) + one_num * 35;
    end
end

sub_border_map_2 = zeros(iterators(2), iterators(1));
for i = 1:iterators(2)
    for j = 1:iterators(1)
        one_num = 0;
        if mapArray(i,j) ~= 0
            for h = -1:1
                for k = -1:1
                    if i + h > 0 && j + k > 0 && i + h <= iterators(2) && j + k <= iterators(1) && sub_border_map((i + h), (j + k)) == 1
                        one_num = one_num + 1;
                        sub_border_map_2(i, j) = 1;
                    end
                end
            end
        end
        mapArray(i,j) = mapArray(i,j) + one_num * 25;
    end
end

for i = 1:iterators(2)
    for j = 1:iterators(1)
        one_num = 0;
        if mapArray(i,j) ~= 0
            for h = -1:1
                for k = -1:1
                    if i + h > 0 && j + k > 0 && i + h <= iterators(2) && j + k <= iterators(1) && sub_border_map_2((i + h), (j + k)) == 1
                        one_num = one_num + 1;
                    end
                end
            end
        end
        mapArray(i,j) = mapArray(i,j) + one_num * 15;
    end
end

current_pos_x = round((estimate_y_2 - limsMin(2))/ res) + 1
current_pos_y = round((estimate_x_2 - limsMin(1))/ res) + 1
%mapArray(current_pos_x, current_pos_y) = max(max(mapArray)) + 10;
hold off;
plot(round(estimate_x_2 / res) * res, round(estimate_y_2 / res) * res, '*');
hold on;
if botSim.debug()
    %hold off; % the drawMap() function will clear the drawing when hold is off
    botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
    botSim.drawBot(30,'g'); % draw robot with line length 30 and green
%     for i =1:num
%       particles(i).drawBot(3); %draw particle with line length 3 and default color
%     end
    drawnow;
end
plot(target(1), target(2), '*');


for i = 1:iterators(2)
    for j = 1:iterators(1)
        % testPos = limsMin + [j-1 i-1] * res;
        % mapArray(i,j) = botSim.pointInsideMap(testPos);
        if mapArray(i,j)
            %plot(testPos(1),testPos(2),'o');%inside map
            x_gap = abs(i - target_array_x);
            y_gap = abs(j - target_array_y);
            if 0 <= x_gap && x_gap <= 1 && 0 <= y_gap && y_gap <= 1
                mapArray(i,j) = 10 + 10;
            end
        end
    end
end
mapArray(target_array_x, target_array_y) = 10; % give target the minimum value of 10
mapArray

arrived = 0; % whether arrive at target or not 

%% Set a particle at the position of real robot with 0 degree and take a new ultrascan
% while (finished == 0)
%     current_scans = 4;
%     particles(300).setBotPos([estimate_x_2 estimate_y_2]);
%     particles(300).setBotAng(0);
%     particles(300).setScanConfig(generateScanConfig(particles(300), current_scans));
%     dis = particles(300).ultraScan();
%     botSim.setScanConfig(generateScanConfig(botSim, current_scans));
%     current_dis = botSim.ultraScan();
    
%     %% Calculate the score of every direction of the real robot to calibrate it with the 0-degree particle
%     for j = 1:current_scans
%         real_dis = circshift(current_dis, j);
%         sqrt(sum((dis - real_dis).^2))
%         scores(1, j) = 10 / sqrt(sum((dis - real_dis).^2));
%     end
    
%     compare_score = sort(scores, 'descend');
%     if abs(10/compare_score(1) - 10/compare_score(2)) < 13
%         botSim.setScanConfig(generateScanConfig(botSim, scans));
%         botScan = botSim.ultraScan(); % get a scan from the real robot.
           
%         [max_distance, max_index] = max(botScan); % find maximum possible distance
%         turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
%         move = round(max_distance * 0.7) - mod(round(max_distance * 0.7), 5); 
%         botSim.turn(turn); % turn the real robot.  
%         botSim.move(move); % move the real robot. These movements are recorded for marking 
        
%         if botSim.debug()
%             hold off; % the drawMap() function will clear the drawing when hold is off
%             botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
%             botSim.drawBot(30,'g'); % draw robot with line length 30 and green
%             drawnow;
%         end
%         pause(0.13);
%     else
%         finished = 1;
%     end
% end

current_scans = 12;
particles(300).setBotPos([estimate_x_2 estimate_y_2]);
particles(300).setBotAng(0);
particles(300).setScanConfig(generateScanConfig(particles(300), current_scans));
dis = particles(300).ultraScan();
botSim.setScanConfig(generateScanConfig(botSim, current_scans));
current_dis = botSim.ultraScan();

%% Calculate the score of every direction of the real robot to calibrate it with the 0-degree particle
for j = 1:current_scans
    real_dis = circshift(current_dis, j);
    % sqrt(sum((dis - real_dis).^2))
    scores(1, j) = 10 / sqrt(sum((dis - real_dis).^2));
end

[scores(1, 1), angle_gap] = max(scores(1, :));
bot_angle = - angle_gap * 2 * pi/current_scans;
botSim.turn(bot_angle); % turn the robot to an angle which is close to 0 degree

%% Path plan
n = 0;
moving = 0; % distance of moving
turning = 0; % degree of this turning
heading = 0; % the heading of car
movingD = 0; % the turning data send to car
visited = zeros(iterators(2), iterators(1));

while (arrived == 0)
    n = n+1;
    min_dis = max(max(mapArray)) + 10;
    for i = -1:1
        for j = -1:1
            if current_pos_x + i > 0 && current_pos_x + i <= iterators(2) && current_pos_y + j > 0 && current_pos_y + j <= iterators(1)
                if mapArray(current_pos_x + i, current_pos_y + j) 
                    if i == 0 && j == 0
                            continue;
                    else
                        if mapArray(current_pos_x + i, current_pos_y + j) < min_dis % robot move from big value towards small value
                            min_dis = mapArray(current_pos_x + i, current_pos_y + j);
                            %target_dis = abs(current_pos_x + i - target_array_x) + abs(current_pos_y + j - target_array_y);
                            next_pos_x = current_pos_x + i;
                            next_pos_y = current_pos_y + j;
                        else
                            if mapArray(current_pos_x + i, current_pos_y + j) == min_dis && visited(current_pos_x, current_pos_y) ~= 1 && abs(i) + abs(j) == 1%rand() < 0.5
                                min_dis = mapArray(current_pos_x + i, current_pos_y + j);
                                %target_dis = abs(current_pos_x + i - target_array_x) + abs(current_pos_y + j - target_array_y);
                                next_pos_x = current_pos_x + i;
                                next_pos_y = current_pos_y + j;
                            end
                        end
                    end
                end
            end
        end
    end
    min_dis;
    visited(current_pos_x, current_pos_y) = 1;
    
    degree = (next_pos_x - current_pos_x) / (next_pos_y - current_pos_y);
    if degree == 1 || degree == -1
        if next_pos_x - current_pos_x > 0 && next_pos_y - current_pos_y > 0
            botSim.turn(atan(degree)); % turn the real robot  
            botSim.move(7.071); % move the real robot
            botSim.turn(-atan(degree)); % turn back the real robot
            
            moving =  7.701;
            turning = atan(degree);
            movingD = turning - heading;
            heading = turning;
%             if turning != heading
%                 movingD = turning - heading;
%             else
%                 movingD = 0;
%             end
        else
            if next_pos_x - current_pos_x < 0 && next_pos_y - current_pos_y < 0
                botSim.turn(atan(degree) + pi); 
                botSim.move(7.071); 
                botSim.turn(-atan(degree) - pi); 
                
                moving =  7.701;
                turning = atan(degree) + pi;
                movingD = turning - heading;
                heading = turning;
            else
                if next_pos_x - current_pos_x > 0 && next_pos_y - current_pos_y < 0
                    botSim.turn(atan(degree) + pi); 
                    botSim.move(7.071);
                    botSim.turn(-atan(degree) - pi);  
                    
                    moving =  7.701;
                    turning = atan(degree) + pi;
                    movingD = turning - heading;
                    heading = turning;
                else
                    botSim.turn(atan(degree)); 
                    botSim.move(7.071); 
                    botSim.turn(-atan(degree)); 
                    
                    moving =  7.701;
                    turning = atan(degree);
                    movingD = turning - heading;
                    heading = turning;
                end
            end
        end
    else
        if degree == 0
            if next_pos_y - current_pos_y < 0
                botSim.turn(atan(degree) + pi); 
                botSim.move(5); 
                botSim.turn(-atan(degree) - pi); 
                
                moving = 5;
                turning = atan(degree) + pi;
                movingD = turning - heading;
                heading = turning;
            else
                botSim.turn(atan(degree)); 
                botSim.move(5); 
                botSim.turn(-atan(degree));  
                
                moving = 5;
                turning = atan(degree);
                movingD = turning - heading;
                heading = turning;
            end
        else
            botSim.turn(atan(degree));   
            botSim.move(5); 
            botSim.turn(-atan(degree)); 
            
            moving = 5;
            turning = atan(degree);
            movingD = turning - heading;
            heading = turning;
        end
    end
    veMove(n,1) = moving;
    veMove(n, 2)= movingD / pi * 180;
%     veMove
    
    
    if botSim.debug()
        hold on; % the drawMap() function will not clear the drawing when hold is off
        botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(3,'g'); % draw robot with line length 30 and green
        drawnow;
    end

    %max_score = 0;
    %candidate_pos_x = next_pos_x;
    %candidate_pos_y = next_pos_y;
    %botSim.setScanConfig(generateScanConfig(botSim, 60));
    %current_scan = botSim.ultraScan();
    %for i = -1:1
    %    for j = -1:1
    %        if mapArray(candidate_pos_x + i, candidate_pos_y + j)
    %            particles(300).setBotPos([((candidate_pos_y + j - 1) * 5) ((candidate_pos_x + i - 1) * 5)]);
    %            particles(300).setBotAng(0)
    %            particles(300).setScanConfig(generateScanConfig(particles(300), 60));
    %            dis = particles(300).ultraScan();
    %            new_score = 10 / sqrt(sum((dis - current_scan).^2));
    %            if new_score > max_score
    %                max_score = new_score;
    %                next_pos_x = candidate_pos_x + i;
    %                next_pos_y = candidate_pos_y + j;
    %            end
    %        end
    %    end
    %end

    current_pos_x = next_pos_x;
    current_pos_y = next_pos_y;

    if min_dis == 10
        arrived = 1; % arrive at the target
        
        numberofMovingStep1
        veMove(:,2);
        veMove1 = evaluatePath(veMove(:,2),veMove(:,1))
        
        final_move = final_path(veMove1);
%         final(:, 1) = final_move(find(final_move(:, 1) ~= 0));
%         final(:, 2) = final_move(find(final_move(:, 2) ~= 0), 2);
%         final
        final = evaluatePath(final_move(:,1), final_move(:,2))
%         zero_ = [0];
%         [c ind]=setdiff(final_move(:, 1), zero_);
%         [c ind]=setdiff(final_move(:, 2), zero_);
%         final_move = final_move(sort(ind))
        
        Extratime = numberofMovingStep1 * 5 + 4 + size(final, 1) * 2
    end
end
