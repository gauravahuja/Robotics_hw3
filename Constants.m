classdef Constants
    properties(Constant)
        room_width = 3;
        robot_diameter = 0.4;
        time_delay = 0.01;
        time_delay_sim = 0.001
        angle_threshold = 0.05;
        distance_threshold = 0.01;
        obstacle_cell = 125;
        unexplored_cell = 0;
        target_cell = 50;
        current_cell = 10;
        empty_cell = 255;
        forward_speed = 0.4;
        angular_speed =pi/6;
    end
end
