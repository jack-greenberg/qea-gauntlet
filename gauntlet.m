function gauntlet()
    global bump_sub pubvel stopMessage
    
    init_NEATO();
    
    placeNeato(0, 0, 1, 0);
    pause(2); % Wait for NEATO to fall
    
    positions = [0, 0, 0]; % Keeps track of position over time (x, y, theta)
    
    % Create symbolic map for calculating gradient
    syms xx yy
    symbolic_map = 0;
    
    % Create numeric map for plotting contours and vector fields
    numeric_map  = 0;
    XX = -2:.1:3;
    YY = -4:.1:1;
    [XX, YY] = meshgrid(XX, YY);
    
    % Constants for gradient descent
    lambda = 0.025;
    delta = 1.1;
    
    f = figure;
    p = [];
    figure(f); hold on; xlim([-2 3]); ylim([-4 1]); axis equal;
    
    % Main movement loop
    % % Each loop, the NEATO will
    % % 1. SCAN
    % % 2. MAP
    % % 3. MOVE
    % Will exit as soon as the NEATO bumps into something
    while (1)
        %%% SCAN %%%
        [r, theta] = LIDAR_scan();
        
        [x, y] = translate_to_G(r, theta, positions(end,3), positions(end,1:2));
        X = [x, y];
        
        p = [p; plot(x, y, 'ks')];
        
        %%% MAP %%%
        while size(X, 1) > 3
            % Line will contain the best end points for the line
            % Circle will contain the center and the radius of the circle
            % Removes the inliers from X
            [line, circle, X] = RANSAC(X, 1000, .025);
            disp(size(X));
            
            if ~isempty(line) % If RANSAC gave us a line
                symbolic_map = symbolic_map + createLineSource(xx, yy, line);
                numeric_map  = numeric_map  + createLineSource(XX, YY, line);
                
                p = [p; plot(line(:,1), line(:,2), '-rs', 'LineWidth', 2)];
            end
            
            if ~isempty(circle) % If RANSAC gave us a circle
                symbolic_map = symbolic_map + createCircleSink(xx, yy, circle(1:2), circle(3));
                numeric_map  = numeric_map  + createCircleSink(XX, YY, circle(1:2), circle(3));
                
                c = viscircles(circle(1:2), circle(3), 'Color', 'green', 'LineWidth', 2);
                p = [p; c];
            end
            drawnow
        end
        
        
        [G1, G2] = gradient(numeric_map);
        quiver(XX, YY, G1, G2);
%         contour(XX, YY, numeric_map, 'ShowText', 'on');
        drawnow
        
        return;
        
        %%% MOVE
        current_position = positions(end,1:2);
        current_angle = positions(end,3);
        
        g = gradient(symbolic_map, [xx, yy]);
        
        current_gradient = transpose(-1*double(subs(g, [xx, yy], {current_position(1,1), current_position(1,2)})));
        
        next_position = current_position + lambda.*current_gradient;
        
        v = next_position - current_position;
        
        % Causes the NEATO to rotate to the new angle
        new_angle = atan2(v(2), v(1));
        rotate(new_angle - current_angle);
        
        % Causes the NEATO to travel to the next point
        travel_distance = norm(v);
        travel(travel_distance);
        
        % If the NEATO has bumped into anything, we should exit
        bumpMessage = receive(bump_sub);
        if any(bumpMessage.Data)
            send(pubvel, stopMessage);
            break;
        end

        % Update the list of positions, and the lambda
        positions = [positions; next_position, new_angle];
        lambda = lambda*delta;
        
        % Reset the numeric and symbolic maps
        symbolic_map = 0; numeric_map = 0;
        delete(p);
    end
    
    hold off
    destroy()
    disp("Done");
end

function term = createCircleSink(x, y, P, r)
    term = 0;
    for theta = 0:0.1*pi:2*pi
        a = r*cos(theta) + P(1);
        b = r*sin(theta) + P(2);

        term = term + log(sqrt((x - a).^2 + (y - b).^2));
    end
end

function term = createLineSource(x, y, endpoints)
    term = 0;
    
    length = norm(endpoints(1,:) - endpoints(2,:));
    spacing = linspace(0, 1, ceil(length * 10));
    
    P1 = endpoints(1,:); P2 = endpoints(2,:);
    
    for n=1:size(spacing, 2)
        point = P1 + spacing(n)*(P2 - P1);
        term = term - log(sqrt((x - point(1,1)).^2 + (y - point(1,2)).^2));
    end
end

function init_NEATO()
    % Initializes the NEATO
    %   Uses rosinit(), sets up LIDAR scanner, wheel velocity
    %   controller, etc.
    global scanner pubvel stopMessage message sub_states sub_encoders bump_sub

    if (~ros.internal.Global.isNodeActive)
        rosshutdown()
        rosinit('localhost', 'NodeHost', '172.17.0.1')
    end

    % Subscribe to LIDAR scanner
    scanner = rossubscriber('/scan');

    % Create STOP message
    pubvel = rospublisher('raw_vel');
    stopMessage = rosmessage(pubvel);
    stopMessage.Data = [0 0];

    % Immediately stop the NEATO
    send(pubvel, stopMessage);

    % Create a message for moving the NEATO
    message = rosmessage(pubvel);
    
    sub_states = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
    
    sub_encoders = rossubscriber('/encoders');
    
    bump_sub = rossubscriber('/bump');
end

function [r, theta] = LIDAR_scan()
    global scanner
    % Scans surroundings using LIDAR scanner and returns
    % r, theta where r is the distance of the object
    % and theta is the angle in radians

    scan_message = receive(scanner);
    r = scan_message.Ranges(1:end-1);
    theta = deg2rad([0:359]');
    r = r(r~=0);
    theta = theta(r~=0);
end

function travel(distance)
    global pubvel message stopMessage bump_sub
    
    wheel_speed = .1;

    start = rostime('now');

    message.Data = [wheel_speed, wheel_speed];
    send(pubvel, message);

    while (1)
        current = rostime('now');
        elapsed = current - start;
        
        bumpMessage = receive(bump_sub);
        if any(bumpMessage.Data)
            send(pubvel, stopMessage);
            break;
        end


        if elapsed.seconds >= abs(distance/wheel_speed)
            send(pubvel, stopMessage);
            break
        end
    end
end

function rotate(theta)
    global pubvel message stopMessage

    wheel_speed = .1;
    wheel_base = 0.235;

    message.Data = [-1*sign(theta)*wheel_speed, sign(theta)*wheel_speed];
    send(pubvel, message);

    start = rostime('now');
    while (1)
        current = rostime('now');
        elapsed = current - start;

        if elapsed.seconds >= abs(theta/(2*wheel_speed / wheel_base))
            send(pubvel, stopMessage);
            break
        end
    end
end

function destroy()
    rosshutdown();
end

function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end

function [x, y] = translate_to_G(r, theta, phi, pos)
    x = r.*cos(theta + phi) - 0.084 .* cos(phi) + pos(1);
    y = r.*sin(theta + phi) - 0.084 .* sin(phi) + pos(2);
end