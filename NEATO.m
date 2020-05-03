classdef NEATO < handle
    %NEATO Class object for controlling NEATOs
    %   Detailed explanation goes here
    
    properties
        pubvel
        scanner
        stopMessage
        message
        wheel_speed
    end
    
    methods(Static)
        function obj = NEATO
            % Initializes the NEATO
            %   Uses rosinit(), sets up LIDAR scanner, wheel velocity
            %   controller, etc.
            
            if (~ros.internal.Global.isNodeActive)
                rosshutdown()
                rosinit('localhost', 'NodeHost', '172.17.0.1')
            end

            % Subscribe to LIDAR scanner
            obj.scanner = rossubscriber('/scan');

            % Create STOP message
            obj.pubvel = rospublisher('raw_vel');
            obj.stopMessage = rosmessage(obj.pubvel);
            obj.stopMessage.Data = [0 0];

            % Immediately stop the NEATO
            send(obj.pubvel, obj.stopMessage);

            % Create a message for moving the NEATO
            obj.message = rosmessage(obj.pubvel);
            
            obj.wheel_speed = .2;
        end
        
        function [r, theta] = LIDAR_scan(obj)
            % Scans surroundings using LIDAR scanner and returns
            % r, theta where r is the distance of the object
            % and theta is the angle in radians

            scan_message = receive(obj.scanner);
            r = scan_message.Ranges(1:end-1);
            theta = deg2rad([0:359]');
            r = r(r~=0);
            theta = theta(r~=0);
        end
        
        function travel(obj, distance)
            start = rostime('now');
            
            obj.message = [obj.wheel_speed, obj.wheel_speed];
            send(obj.pubvel, obj.message);
            
            while (1)
                current = rostime('now');
                elapsed = current - start;
                
                if elapsed.seconds >= distance/obj.wheel_speed
                    break
                end
            end
        end
        
        function destroy()
            rosshutdown();
        end
    end
end
