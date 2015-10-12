function bug2_algorithm(serPort)
    eps_x = 0.2; 
    eps_y = 0.02; 
    curr_position = position(0, 0, 0);
    met_goal = 0;
    while ~met_goal 
        [l, f, r, w, curr_position, met_goal] = seek_goal(serPort, curr_position, eps_x);
        if ~met_goal
            [curr_position, stop] = move_around_obstacle(serPort, curr_position, eps_y);
            if(stop == 1)
               display('Closed obstacle.');
               break;
            end
        end
    end
    display('Reached destination.');
end

function [new_position, stop] = move_around_obstacle(serPort, curr_position, eps_y)
    distance_traveled = 0;
    back_on_line = 0;
    stop = 0;
    % turn left until parallel to first wall of obstacle
    [l, f, r, w, curr_position, distance_traveled] = ...
        turn_left_until_parallel(serPort, curr_position, distance_traveled);
    
    start = [curr_position(1,4), curr_position(2,4)];
    
    while back_on_line == 0
        % handle turn
        [curr_position, distance_traveled, same_pos] = ...
            handle_turn(serPort, l, f, r, w, curr_position, distance_traveled, start)
        if(same_pos == 1)
            stop=1;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(0.1);
            break;
        end
        % move forward against the wall
        [l, f, r, w, curr_position, distance_traveled, back_on_line, same_pos] = ...
            follow_straight_wall(serPort, eps_y, curr_position, distance_traveled, start);
        back_on_line
        if(same_pos == 1)
            stop = 1;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(0.1);
            break;
        end
    end  
    
    if back_on_line
        curr_position = turn_back_onto_line(serPort, curr_position, acos(curr_position(1,1)));
        pause(0.05);
    end
    
    new_position = curr_position;
end

function [new_position] = turn_back_onto_line(serPort, curr_position, theta) 
    [AngleR] = AngleSensorRoomba(serPort); 
    angle = theta * 180/pi;
    turnAngle(serPort, 0.05, angle);
    SetFwdVelRadiusRoomba(serPort, 0, inf);
    pause(0.1); 
    [AngleR]=AngleSensorRoomba(serPort); 
    curr_position = curr_position * position(AngleR, 0, 0);
    new_position = curr_position; 
    display('We are turning back onto the line.')
    pause(0.2);          
end

function [new_position, distance_traveled, same_pos] = ...
    handle_turn(serPort, l, f, r, w, curr_position, distance_traveled, start)
    new_position = curr_position;
    start_x = start(1,1);
    start_y = start(1,2);
    same_pos = 0;
    
    if(abs(curr_position(1,4) - start_x) < 0.3 && abs(curr_position(2,4) - start_y) < 0.3 && distance_traveled > 0.5)
        same_pos = 1;
    else
    
        if (r == 1 && f == 0 && l == 0 && w == 1)
             [AngleR] = AngleSensorRoomba(serPort); 
             turnAngle(serPort, 0.2, 5);
             pause(0.1); 
             [AngleR]=AngleSensorRoomba(serPort); 
             new_position = curr_position * position(AngleR, 0, 0);
        end

        if (f == 1 || l == 1) && w == 1
            display ('turning left'); 
            [l, f, r, w, new_position, distance_traveled] = ...
                turn_left_until_parallel(serPort, curr_position, distance_traveled); 
        end

        if w == 0
            display ('turning right'); 
            [l, f, r, w, new_position, distance_traveled] = ...
                turn_right_until_wall(serPort, curr_position, distance_traveled);
        end
    end
end

function [l, f, r, w, new_position, distance_traveled, back_on_line, same_pos] = ...
    follow_straight_wall(serPort, eps_y, curr_position, distance_traveled, start)
    w=1;
    [Distance] = DistanceSensorRoomba(serPort);
    [AngleR] = AngleSensorRoomba(serPort); 
    new_position = curr_position; 
    y = new_position(2,4);
    x = new_position(1,4);
    back_on_line = 0;
    start_x = start(1,1);
    start_y = start(1,2);
    same_pos = 0;

    while 1
        if (abs(y) < eps_y) && x > 0 %&& (distance_traveled > 1)
            SetFwdVelRadiusRoomba(serPort, 0.1, inf);
            pause(0.1);
            back_on_line = 1;
            pause(0.1);
            l=0;
            f=0;
            r=0;
            break;
        end
        
        SetFwdVelRadiusRoomba(serPort, 0.05, inf);      % Move Forward
        pause(0.1); 
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
        
        w = WallSensorReadRoomba(serPort);
        if l || r || f || (w == 0)
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(.5)
            [Distance] = DistanceSensorRoomba(serPort); 
            [AngleR]=AngleSensorRoomba(serPort); 
            distance_traveled = distance_traveled + [Distance]
            new_position = new_position * position(AngleR,Distance, 0);
            break;
        end
        
        [Distance] = DistanceSensorRoomba(serPort); 
        [AngleR] = AngleSensorRoomba(serPort); 
        distance_traveled = distance_traveled + [Distance]
        new_position = new_position * position(AngleR, Distance, 0)
        
        if(abs(new_position(1,4) - start_x) < 0.3 && abs(new_position(2,4) - start_y) < 0.3 && distance_traveled > 0.5 )
            same_pos = 1;
            break;
        end
        
        y = new_position(2,4);
        x = new_position(1,4);
        scatter(x, y)
    end    
end

function [l, f, r, w, new_position, met_goal] = seek_goal(serPort, curr_position, eps_x)
    [Distance] = DistanceSensorRoomba(serPort);
    [AngleR] = AngleSensorRoomba(serPort);
    met_goal = 0;
    new_position = curr_position; 
    x = new_position(1,4); 
    
    while (abs(4-x) > eps_x)     
        SetFwdVelRadiusRoomba(serPort, 0.05, inf);      % Move Forward
        pause(0.2); 
        [BumpRight, BumpLeft, WheelDropRight, WheelDropLeft, ...
            WheelDropCastor, BumpFront] = ...
            BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
        
        wall_sensor = WallSensorReadRoomba(serPort);
        
        if BumpLeft || BumpFront || BumpRight
            l = BumpLeft;
            f = BumpFront;
            r = BumpRight; 
            w = wall_sensor;
            SetFwdVelRadiusRoomba(serPort, 0, inf);
            pause(.5)
            [Distance] = DistanceSensorRoomba(serPort); 
            [AngleR]=AngleSensorRoomba(serPort); 

            new_position = new_position * position(AngleR, Distance, 0); 
            break;
        end
        
        [Distance] = DistanceSensorRoomba(serPort); 
        [AngleR]=AngleSensorRoomba(serPort); 
        new_position = new_position * position(AngleR,Distance, 0)
        x = new_position(1,4);
            c = new_position(1,4);
            d = new_position(2,4);
            scatter(c, d)
    end
    
    if (abs(4-x) < eps_x)
        met_goal = 1;
        display('I met my goal.');
        SetFwdVelRadiusRoomba(serPort, 0, inf);
        pause(0.5); 
        l = 0;
        r = 0;
        f = 0;
        w = 0;
    end
end

function [l, f, r, w, new_position, distance_traveled] = ...
    turn_left_until_parallel(serPort, curr_position, distance_traveled)

    [AngleR]=AngleSensorRoomba(serPort);
    [Distance] = DistanceSensorRoomba(serPort);
    w = WallSensorReadRoomba(serPort); 
    new_position = curr_position;
    [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
        BumpsWheelDropsSensorsRoomba(serPort);

    while l == 1 || f == 1 || r == 1
        SetFwdVelRadiusRoomba(serPort, .05, 0.1);%.1
        pause(0.01);
        SetFwdVelRadiusRoomba(serPort, 0, inf); 
        %pause(0.1);
        [AngleR]=AngleSensorRoomba(serPort); 
        [Distance] = DistanceSensorRoomba(serPort);
        distance_traveled = distance_traveled + [Distance];
        new_position = new_position * position(AngleR, Distance, 0);
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort); % Read Bumpers
        w = WallSensorReadRoomba(serPort);
        c = new_position(1,4);
        d = new_position(2,4);
        scatter(c, d)
    end
end

function [l, f, r, w, new_position, distance_traveled] = ...
    turn_right_until_wall(serPort, curr_position, distance_traveled)

    [AngleR]=AngleSensorRoomba(serPort);
    [Distance] = DistanceSensorRoomba(serPort);
    w = WallSensorReadRoomba(serPort);
    new_position = curr_position;

    [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
        BumpsWheelDropsSensorsRoomba(serPort);

    while w == 0
        SetFwdVelRadiusRoomba(serPort, .05, -0.1); %-.1
        pause(.01);
        SetFwdVelRadiusRoomba(serPort, 0, inf);

        [AngleR] = AngleSensorRoomba(serPort); 
        [Distance] = DistanceSensorRoomba(serPort);
        distance_traveled = distance_traveled + [Distance];
        new_position = new_position * position(AngleR, Distance, 0);
            c = new_position(1,4);
            d = new_position(2,4);
            scatter(c, d)
        w = WallSensorReadRoomba(serPort); 
        [r, l, WheelDropRight, WheelDropLeft, WheelDropCastor, f] = ...
            BumpsWheelDropsSensorsRoomba(serPort);
    end
    SetFwdVelRadiusRoomba(serPort, 0, inf);
end

function a = position(theta, x,y)
    a = eye(4);
    a(1,1) = cos(theta);
    a(1,2) = -sin(theta);
    a(2,1) = sin(theta);
    a(2,2) = cos(theta); 
    a(1,4) = x;
    a(2,4) = y;
end
