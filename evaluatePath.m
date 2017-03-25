function [path] = evaluatePath(turning, moving)   % direction 1:turn right, -1:turn left
heading = find(turning ~= 0);
Size = size(heading);
counter = Size(1)
move = 0;

for i = 1 : counter 
  
   path(i,1) = turning(heading(i));
   if i <= (counter - 1)
       startpoint = heading(i);
       endpoint = heading(i + 1);
       sumL = endpoint - startpoint - 1;
       for j = 0 : sumL
           move = move + moving(startpoint + j);
       end
   else 
       len = size(turning);
       L = len(1);
       for j = heading(i) : L
           move = move + moving(j);
       end
   end
   path(i , 2) = move;
   move = 0;
end    
end