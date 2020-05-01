function gauntlet()
    global scanner pubvel stopMessage message

    init_NEATO();
    
    placeNeato(0, 0, 1, 0);
    pause(2);
    
    all_points = [];
    
    f = figure;
    
    map = 0;
    
    syms xx yy
    symmap = 0;
    
    XX = -2:.08:3;
    YY = -4:.08:1;
    [XX, YY] = meshgrid(XX, YY);
    
    i = 0;
    while (1)
        i = i + 1;
        if i == 2
            break;
        end
        
        %%% 1. SCAN
        [r, theta] = LIDAR_scan();
        [x, y] = pol2cart(theta, r);
        X = [x, y];
        
        all_points = [all_points; round(x, 2), round(y, 2)];

        figure(f)
        plot(x, y, 'ks'); hold on; xlim([-2 3]); ylim([-4 1]); axis equal;
        p = [];
        
        %%% 2. MAP
        while size(X, 1) > 2
            disp(size(X));
            [bestInlierType, bestInlierSet, X, m, b, bestEndPoints, C, R] = RANSAC(X, 1000, .02);
            
            if strcmp(bestInlierType, 'None')
                continue
            end
            
            if strcmp(bestInlierType, 'line')
                if isempty(bestEndPoints)
                    continue
                end
                p = [p; plot(bestEndPoints(:,1), bestEndPoints(:,2), '-rs', 'LineWidth', 2)];
                p = [p; plot(bestInlierSet(:,1), bestInlierSet(:,2), 'rs')];

                map = map - log(sqrt((XX - bestEndPoints(1,1)).^2 + (YY - bestEndPoints(1,2)).^2));
                map = map - log(sqrt((XX - bestEndPoints(2,1)).^2 + (YY - bestEndPoints(2,2)).^2));

                symmap = symmap - log(sqrt((xx - bestEndPoints(1,1)).^2 + (yy - bestEndPoints(1,2)).^2));
                symmap = symmap - log(sqrt((xx - bestEndPoints(2,1)).^2 + (yy - bestEndPoints(2,2)).^2));
            else
                p = [p; plot(bestInlierSet(:,1), bestInlierSet(:,2), 's')];
                c = viscircles([C(1) C(2)], R, 'Color', 'green');
                p = [p; c];
                
                map = map + log(sqrt((XX - C(1)).^2 + (YY - C(2)).^2));

                symmap = symmap + log(sqrt((xx - C(1)).^2 + (yy - C(2)).^2));
            end
        end
        
        hold off
    end
%     
%     

%     
%     syms xx yy;
%     
%     XX = -2:.08:3;
%     YY = -4:.08:1;
%     [XX, YY] = meshgrid(XX, YY);
%     
%     all_points = [round(x, 2), round(y, 2)];
%     
%     map = 0;
%     symmap = 0;
%     
%     close all;
%     f = figure;
%     plot(x, y, 'ks'); hold on; xlim([-2 3]); ylim([-4 1]); axis equal;
%     
%     while size(X, 1) > 2
%         [fitline_coefs,bestInlierSet,X,bestEndPoints] = RANSAC(X, 100, .01);
%         
%         if any(isnan(bestEndPoints))
%             break
%         end
%         
%         p = plot(bestEndPoints(:,1), bestEndPoints(:,2), '-rs', 'LineWidth', 2);
%         p = [p; plot(bestInlierSet(:,1), bestInlierSet(:,2), 'rs')];
%         
%         m = fitline_coefs(1); b = fitline_coefs(2);
%         
%         map = map - log(sqrt((XX - bestEndPoints(1,1)).^2 + (YY - bestEndPoints(1,2)).^2));
%         map = map - log(sqrt((XX - bestEndPoints(2,1)).^2 + (YY - bestEndPoints(2,2)).^2));
%         
%         symmap = symmap - log(sqrt((xx - bestEndPoints(1,1)).^2 + (yy - bestEndPoints(1,2)).^2));
%         symmap = symmap - log(sqrt((xx - bestEndPoints(2,1)).^2 + (yy - bestEndPoints(2,2)).^2));
%     end
%     
%     g = gradient(symmap, [xx, yy]);
%     
%     current_gradient = double(subs(g, [xx, yy], {0, 0}));
%     
%     new_angle = atan2(current_gradient(2), current_gradient(1));
%     
%     rotate(new_angle);
    
% %     contour(XX, YY, map, 'ShowText', 'on');
% %     
% %     XX = -2:.5:3;
% %     YY = -4:.5:1;
% %     [XX, YY] = meshgrid(XX, YY);
% % 
% %     G1 = subs(g(1), [xx, yy], {XX, YY});
% %     G2 = subs(g(2), [xx, yy], {XX, YY});
% %     
% %     quiver(XX, YY, G1, G2, 'LineWidth', 1.5);
    
    hold off
    destroy()
end

function n = make_peak(P)
    n = -log(sqrt((xx - P(1)).^2 + (yy - P(2)).^2));
end

function n = make_valley(P)
    syms xx yy
    n = log(sqrt((xx - P(1)).^2 + (yy - P(2)).^2));
    disp(n);
end

function init_NEATO()
    % Initializes the NEATO
    %   Uses rosinit(), sets up LIDAR scanner, wheel velocity
    %   controller, etc.
    global scanner pubvel stopMessage message

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
    global pubvel message stopMessage
    
    wheel_speed = .2;

    start = rostime('now');

    message.Data = [wheel_speed, wheel_speed];
    send(pubvel, message);

    while (1)
        current = rostime('now');
        elapsed = current - start;

        if elapsed.seconds >= abs(distance/wheel_speed)
            send(pubvel, stopMessage);
            break
        end
    end
end

function rotate(theta)
    global pubvel message stopMessage
    
    wheel_speed = .2;
    wheel_base = 0.235;

    message.Data = [wheel_speed, sign(theta) * wheel_speed];
    send(pubvel, message);

    start = rostime('now');
    while (1)
        current = rostime('now');
        elapsed = current - start;
        
        disp(elapsed.seconds)

        if elapsed.seconds >= abs(theta/(2*wheel_speed / wheel_base))
            send(pubvel, stopMessage);
            break
        end
    end
end

function destroy()
    rosshutdown();
end

function p = plot_circle(r, x, y)
%     theta = 0:pi/50:2*pi;
%     xunit = r * cos(theta) + x;
%     yunit = r * sin(theta) + y;
%     p = plot(xunit, yunit, '-s');
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