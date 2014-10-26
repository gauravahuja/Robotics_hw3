function room = hw3(serPort)
  width = 6;
  robotDiameter = 0.4;
  room = [-width:0.4:width];
  room = repmat(0, size(room,2), size(room,2));
  f = figure('position', [0, 0, size(room,2)*robotDiameter, size(room,2)*robotDiameter]);

  % Mark the current position as 0 at the center of the grid
  row = ceil(size(room,1)/2);
  col = row;
  room(row, col) = 0;
  axis([-6, 6, -6, 6]);
  imshow(room, [0 255]);
  grid on;
  hold on;
  
  [A,B] = find(room == 0);
  rand_ind = randi([1 size(A,1)]);
  target_y = A(rand_ind);
  target_x = B(rand_ind);
  curr_x = col;
  curr_y = row;
  orientation = [1 0];
  
  % new algorithm
  
  while(size(A)~=0)
       fprintf('curr = (%d, %d), target = (%d, %d)\n', curr_x, curr_y, target_x, target_y);
       fprintf('Calculating path\n');
       p = shortestPath(curr_x, curr_y, target_x, target_y, room, size(room,1));
       fprintf('Path Calculated\n');
       if(size(p,1) == 0)
            fprintf('Target is unreachable\n')
            room(target_y, target_x) = 125;
       else
           i = 1;
           while(room(target_y,target_x) == 0) 
             tix = p(i,1);
             tiy = p(i,2);
             fprintf('curr = (%d, %d), intermediate target = (%d, %d)\n', curr_x, curr_y, tix, tiy);

             % if hits an obstacle, move back and return true
             [curr_x, curr_y, orientation, hit] = move(serPort, curr_x,curr_y,tix, tiy, orientation);
             if(hit == 1)
                 room(tiy, tix) = 125; 
                 break;
             else
                 room(tiy, tix) = 255;
                 i = i+1;
             end
             imshow(room, [0 255]);
             hold on;
           end
       end
     
     [A,B] = find(room == 0);
     rand_ind = randi([1 size(A,1)]);
     target_y = A(rand_ind);
     target_x = B(rand_ind);
  end
  fprintf('END\n');
end

function [curr_x, curr_y, orientation, hit] = move(serPort, x, y, tx, ty, orientation)
       diffx = x-tx;
       diffy = y-ty;
       
       angle = -(diffx * orientation(2) + diffy * orientation(1)) * pi/2;
       
       if angle == pi/2
               fprintf('Moving 90cw\n');
               rotate90cw(serPort);
       elseif angle == -pi/2
               fprintf('Moving 90ccw\n');
               rotate90ccw(serPort);
       else if orientation ~= [diffx diffy] 
                fprintf('Moving 180cw\n');
               rotate180cw(serPort);
            end
            
       end
       distanceFwd = diffx+diffy;
       fprintf('Moving fwd\n');
       [hit, distanceMoved] = moveForward(serPort, abs(distanceFwd)*0.4);
       orientation = [diffx -diffy];
       if hit
           fprintf('Moving back\n');
           moveForward(serPort, -distanceMoved);
           curr_x = x;
           curr_y = y;
       else
           curr_x = tx;
           curr_y = ty;
       end
end

function rotate90cw(serPort)
       ang = 0;
       AngleSensorRoomba(serPort);
       pause(0.1);
       
       SetFwdVelAngVelCreate(serPort, 0, -0.5);
       while(abs(abs(ang)-pi/2) > 0.1)
           ang = ang + AngleSensorRoomba(serPort);
           pause(0.1);
       end
end


function rotate180cw(serPort)
       ang = 0;
       AngleSensorRoomba(serPort);
       pause(0.1);
       SetFwdVelAngVelCreate(serPort, 0, -0.5);
       
       while(abs(abs(ang)-pi/2) > 0.1)
           ang = ang + AngleSensorRoomba(serPort);
           pause(0.1);
       end
end

function rotate90ccw(serPort)
       ang = 0;
       AngleSensorRoomba(serPort);
       pause(0.1);
       SetFwdVelAngVelCreate(serPort, 0, 0.5);
       
       while(abs(abs(ang)-pi/2) > 0.1)
           ang = ang + AngleSensorRoomba(serPort);
           pause(0.1);
       end
end

function [hit, distanceMoved] = moveForward(serPort, distance)
       dist = 0;
       DistanceSensorRoomba(serPort);
       pause(0.1);
       SetFwdVelAngVelCreate(serPort, 0.2*sign(distance), 0);
      
       hit = 0;
       while abs(dist- abs(distance)) > 0.1 & hit == 0
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, WheelDropCastor, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        pause(0.1);
        bumped = BumpRight||BumpLeft||BumpFront;
        
        
        dist = dist + DistanceSensorRoomba(serPort);
        pause(0.1);
        
        if bumped 
            hit = 1;
        end
       end
       SetFwdVelAngVelCreate(serPort, 0, 0);
       distanceMoved = dist;
end
