classdef NEATO
    %NEATO Class object for controlling NEATOs
    %   Detailed explanation goes here
    
    properties
        pubvel
        scanner
        stopMessage
        message
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
        end
        
        function [r, theta] = LIDAR_scan(obj)
            disp(obj.scanner)
            scan_message = receive(obj.scanner);
            r = scan_message.Ranges(1:end-1);
            theta = deg2rad([0:359]');
        end
        
        function destroy()
            rosshutdown();
        end
    end
end

