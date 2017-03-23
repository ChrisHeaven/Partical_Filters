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
scans = 30;

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
%         move = max_distance * 0.3; 
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
%     pause(1);
% end

%% Localisation code
maxNumOfIterations = 30;
n = 0;
converged =0; % the filter has not converged yet

while(converged == 0 && n < maxNumOfIterations) %particle filter loop
    n = n+1; % increment the current number of iterations
    botSim.setScanConfig(generateScanConfig(botSim, scans));
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    %% Write code for updating your particles scans
    for i = 1:num
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
    end
    
    [weight, index] = sort(score, 'descend');
    
    for i = 1:num / 2
        score_half(i, 1) = weight(i);
    end
    sum_score_half = sum(score_half);
    
    %% Write code for resampling your particles
    particles_pos = zeros(2, num / 2);
    
    for i = 1:num / 2
        particles_pos(:, i) = getBotPos(particles(index(i)));
    end
    
    count = 1;
    
    for i = 1:num / 2
            particles_num(i, :) = fix(weight(index(i)) * num * (1 / sum_score_half));
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
        convergence_threshold = mid_x / 160;
    else
        convergence_threshold = mid_x / 180;
    end

    stdev_x = std(x_(1, :));
    stdev_y = std(y_(1, :));
    
    if stdev_x < convergence_threshold && stdev_y < convergence_threshold
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
    
    %%Solving hitting wall problem
    [min_distance, min_index] = min (botScan);
    if min_distance < 38.105 % 22 * 3 ^ (1/2)
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
    end
    botScan
    
    if rand() < 0.96 % prefer to move in the maximum direction
        [max_distance, max_index] = max(botScan); 
        turn = (max_index - 1) * 2 * pi / scans; % orientate towards the max distance
        move = max_distance * 0.2 * rand(); % move a random amount of the max distance, but never the entire distance
    else % some of the time move in a random direction
        index = randi(scans); 
        turn = (index - 1) * 2 * pi/scans;
        move= botScan(index) * 0.2;
    end
    
    botSim.turn(turn); % turn the real robot.  
    botSim.move(move); % move the real robot. These movements are recorded for marking 
    for i =1:num % for all the particles. 
        particles(i).turn(turn); % turn the particle in the same way as the real robot
        particles(i).move(move); % move the particle in the same way as the real robot
        %if particles(i).insideMap() == 0
        %    particles(i).randomPose(0);
        %end
    end
    
    %% Drawing
    % only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; % the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); % drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); % draw robot with line length 30 and green
        %for i =1:num
        %    particles(i).drawBot(3); %draw particle with line length 3 and default color
        %end
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

hold on;
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
        mapArray(i,j) = mapArray(i,j) + zero_num * 15;
        if i == 1 && mapArray(1, j) ~= 0
            mapArray(1, j) = mapArray(1, j) + 3 * 15;
        end
    end
end

for i = 1:iterators(2)
    for j = 1:iterators(1)
        one_num = 0;
        if mapArray(i,j) ~= 0
            for h = -1:1
                for k = -1:1
                    if i + h > 0 && j + k > 0 && i + h <= iterators(2) && j + k <= iterators(1) && border_map((i + h), (j + k)) == 1
                        one_num = one_num + 1;
                    end
                end
            end
        end
        mapArray(i,j) = mapArray(i,j) + one_num * 10;
    end
end

current_pos_x = round((estimate_y_2 - limsMin(2))/ res) + 1
current_pos_y = round((estimate_x_2 - limsMin(1))/ res) + 1
%mapArray(current_pos_x, current_pos_y) = max(max(mapArray)) + 10;

plot(round(estimate_x_2 / res) * res, round(estimate_y_2 / res) * res, '*');
plot(target(1), target(2), '*');

mapArray(target_array_x, target_array_y) = 10; % give target the minimum value of 10
mapArray

arrived = 0; % whether arrive at target or not 

%% Set a particle at the position of real robot with 0 degree and take a new ultrascan
current_scans = 360;
particles(300).setBotPos([estimate_x_2 estimate_y_2]);
particles(300).setBotAng(0);
particles(300).setScanConfig(generateScanConfig(particles(300), current_scans));
dis = particles(300).ultraScan();
botSim.setScanConfig(generateScanConfig(botSim, current_scans));
current_dis = botSim.ultraScan();

%% Calculate the score of every direction of the real robot to calibrate it with the 0-degree particle
for j = 1:current_scans
    real_dis = circshift(current_dis, j);
    scores(1, j) = 10 / sqrt(sum((dis - real_dis).^2));
end

[scores(1, 1), angle_gap] = max(scores(1, :));
bot_angle = - angle_gap * 2 * pi/current_scans;
botSim.turn(bot_angle); % turn the robot to an angle which is close to 0 degree

%% Path plan
while (arrived == 0)
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
                            next_pos_x = current_pos_x + i;
                            next_pos_y = current_pos_y + j;
                        else
                            if mapArray(current_pos_x + i, current_pos_y + j) == min_dis && rand() < 0.5
                                min_dis = mapArray(current_pos_x + i, current_pos_y + j);
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

    degree = (next_pos_x - current_pos_x) / (next_pos_y - current_pos_y);
    if degree == 1 || degree == -1
        if next_pos_x - current_pos_x > 0 && next_pos_y - current_pos_y > 0
            botSim.turn(atan(degree)); % turn the real robot  
            botSim.move(7.071); % move the real robot
            botSim.turn(-atan(degree)); % turn back the real robot
        else
            if next_pos_x - current_pos_x < 0 && next_pos_y - current_pos_y < 0
                botSim.turn(atan(degree) + pi); 
                botSim.move(7.071); 
                botSim.turn(-atan(degree) - pi);  
            else
                if next_pos_x - current_pos_x > 0 && next_pos_y - current_pos_y < 0
                    botSim.turn(atan(degree) + pi); 
                    botSim.move(7.071);
                    botSim.turn(-atan(degree) - pi);  
                else
                    botSim.turn(atan(degree)); 
                    botSim.move(7.071); 
                    botSim.turn(-atan(degree)); 
                end
            end
        end
    else
        if degree == 0
            if next_pos_y - current_pos_y < 0
                botSim.turn(atan(degree) + pi); 
                botSim.move(5); 
                botSim.turn(-atan(degree) - pi); 
            else
                botSim.turn(atan(degree)); 
                botSim.move(5); 
                botSim.turn(-atan(degree));  
            end
        else
            botSim.turn(atan(degree));   
            botSim.move(5); 
            botSim.turn(-atan(degree)); 
        end
    end

    if botSim.debug()
        hold on; % the drawMap() function will clear the drawing when hold is off
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
    end
end