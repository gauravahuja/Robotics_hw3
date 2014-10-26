function room = hw3(serPort)
  width = Constants.room_width;
  robotDiameter = Constants.robot_diameter;

  room = [-width:robotDiameter:width];
  room = repmat(Constants.unexplored_cell, size(room,2), size(room,2));

  disp(size(room));
  
  % Mark the current position as 0 at the center of the grid
  row = ceil(size(room,1)/2);
  col = row;
  room(row, col) = Constants.empty_cell;
  f = figure;
  axis([-width width -width width]);
  imagesc(room);
  colorbar;
 
  
  [A,B] = find(room == Constants.unexplored_cell);
  
  curr_x = col;
  curr_y = row;
  orientation = [1 0];
  
  % new algorithm
  
  while(size(A)~=0)
       rand_ind = randi([1 size(A,1)]);
       target_y = A(rand_ind);
       target_x = B(rand_ind);
       fprintf('curr = (%d, %d), target = (%d, %d)\n', curr_x, curr_y, target_x, target_y);
       room(target_y, target_x) = Constants.target_cell;
       
       figure(f), imagesc(room);
       
       p = shortestPath(curr_x, curr_y, target_x, target_y, room, size(room,1));
       
       if(size(p,1) == 0)
            fprintf('Target is unreachable\n')
            room(target_y, target_x) = Constants.obstacle_cell;
       else
           i = 1;
           while(room(target_y,target_x) == Constants.target_cell) 
             tix = p(i,1);
             tiy = p(i,2);
             fprintf('curr = (%d, %d), intermediate target = (%d, %d), target = (%d, %d)\n', curr_x, curr_y, tix, tiy, target_x, target_y);
             
             
             
             % if hits an obstacle, move back and return true
             [curr_x, curr_y, orientation, hit_left, hit_front, hit_right] = move(serPort, curr_x,curr_y, tix, tiy, orientation);
             if(hit_front == 1)
                 room(tiy, tix) = Constants.obstacle_cell; 
                 break;
             elseif(hit_right == 1)
                break;
             elseif(hit_left == 1)
                break;
             else
                 room(tiy, tix) = Constants.empty_cell;
                 i = i+1;
             end
             figure(f) ,imagesc(room);
             
           end
           if (room(target_y,target_x) == Constants.target_cell)
                room(target_y,target_x) = Constants.unexplored_cell;
           end
       end
     [A,B] = find(room == Constants.unexplored_cell);
  end
  figure(f), imagesc(room);
  fprintf('Explored All\n');
end

function [curr_x, curr_y, orientation, hit_left, hit_front, hit_right] = move(serPort, x, y, tx, ty, orientation)
       diffx = tx-x;
       diffy = ty-y;
       
       angle = -(diffx * orientation(2) + diffy * orientation(1)) * pi/2;
       
       fprintf('Orientation = [%d %d], Diff= [%d %d], Angle = %f\n', orientation(1), orientation(2), diffx, -diffy, angle);
       
       if angle < 0
               fprintf('Moving 90cw\n');
               rotate90cw(serPort);
       elseif angle > 0
               fprintf('Moving 90ccw\n');
               rotate90ccw(serPort);
       
       else 
            if ~isequal(orientation, [diffx -diffy]) 
                fprintf('Moving 180cw\n');
                rotate180cw(serPort);
            end
       end

       distanceFwd = diffx+diffy;
       fprintf('Moving fwd\n');
       [hit_left, hit_front, hit_right, distanceMoved] = moveForward(serPort, abs(distanceFwd)*Constants.robot_diameter);
       fprintf('Moved: %f\n', distanceMoved);
       orientation = [diffx -diffy];
       if hit_left | hit_front | hit_right
           fprintf('Moving back %f m\n', distanceMoved);
           distanceMoved = moveBackward(serPort, -distanceMoved);
           fprintf('Moved: %f\n', distanceMoved);
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
       pause(Constants.time_delay);
       
       SetFwdVelAngVelCreate(serPort, 0, -Constants.angular_speed);
       pause(Constants.time_delay);

       while(ang < pi/2)
           ang = ang + abs(AngleSensorRoomba(serPort))
           pause(Constants.time_delay);
       end
       ang = ang+ abs(AngleSensorRoomba(serPort));
       pause(Constants.time_delay);
       fprintf('Rotated cw %f\n', ang*180/pi);
end


function rotate180cw(serPort)
       ang = 0;
       AngleSensorRoomba(serPort);
       pause(Constants.time_delay);
       
       SetFwdVelAngVelCreate(serPort, 0, -Constants.angular_speed);
       pause(Constants.time_delay);
       
       while(ang < pi)
           ang = ang + abs(AngleSensorRoomba(serPort))
           pause(Constants.time_delay);
       end
       ang = ang+ abs(AngleSensorRoomba(serPort));
       pause(Constants.time_delay);
       fprintf('Rotated cw %f\n', ang*180/pi);
end

function rotate90ccw(serPort)
       ang = 0;
       AngleSensorRoomba(serPort);
       pause(Constants.time_delay);
       
       SetFwdVelAngVelCreate(serPort, 0, Constants.angular_speed);
       pause(Constants.time_delay);
       
       while(ang < pi/2)
           ang = ang + abs(AngleSensorRoomba(serPort))
           pause(Constants.time_delay);
       end
       ang = ang+ abs(AngleSensorRoomba(serPort));
       pause(Constants.time_delay);
       fprintf('Rotated ccw %f\n', ang*180/pi);
end

function distanceMoved = moveBackward(serPort, distance)
       dist = 0;
       DistanceSensorRoomba(serPort);
       pause(Constants.time_delay);

       SetFwdVelAngVelCreate(serPort, -Constants.forward_speed, 0);
       pause(Constants.time_delay);
       while dist < abs(distance)
        dist = dist + abs(DistanceSensorRoomba(serPort));
        pause(Constants.time_delay);
       end
       
       SetFwdVelAngVelCreate(serPort, 0, 0);
       pause(Constants.time_delay);       
       distanceMoved = dist + abs(DistanceSensorRoomba(serPort));
       pause(Constants.time_delay);
       distanceMoved = sign(distance)*distanceMoved;
       
end

function [hit_left, hit_front, hit_right, distanceMoved] = moveForward(serPort, distance)
       dist = 0;
       DistanceSensorRoomba(serPort);
       pause(Constants.time_delay);

       SetFwdVelAngVelCreate(serPort, Constants.forward_speed, 0);
       pause(Constants.time_delay);
      
       bumped = 0;
       hit_left=0;
       hit_front=0;
       hit_right=0;
       while dist < abs(distance) & bumped == 0
        
        [hit_right, hit_left, WheelDropRight, WheelDropLeft, WheelDropCastor, hit_front] = BumpsWheelDropsSensorsRoomba(serPort);
        pause(Constants.time_delay);
        bumped = hit_right|hit_left|hit_front;
        
        dist = dist + abs(DistanceSensorRoomba(serPort));
        pause(Constants.time_delay);
       end
       
       SetFwdVelAngVelCreate(serPort, 0, 0);
       pause(Constants.time_delay);       
       distanceMoved = dist + abs(DistanceSensorRoomba(serPort));
       pause(Constants.time_delay);
       distanceMoved = sign(distance)*distanceMoved;
end
