% Identifies one feature of a set of points
function [line, circle, outliers] = RANSAC(X, n, d)
    disp(size(X));
    
    line = []; circle = [];
    
    % Current best set of inliers
    bestInlierSet = [];
    bestInlierSetIndex = [];
    bestInlierType = 'none';
    
    for i=1:n
        % Get 3 random points
        candidates = datasample(X, 3, 'Replace', false);
        P1 = candidates(1,:);
        P2 = candidates(2,:);
        P3 = candidates(3,:);
        
        [line_inliers1, biggestGap1] = lineDetector(X, d, P1, P2);
        [line_inliers2, biggestGap2] = lineDetector(X, d, P1, P3);
        [line_inliers3, biggestGap3] = lineDetector(X, d, P2, P3);
        [circle_inliers, xc, yc, r, biggestGapC] = circleDetector(X, d, P1, P2, P3);
        
        if sum(line_inliers1) > size(bestInlierSet, 1) && biggestGap1 < .2 && sum(line_inliers1) > 10 % If the first line is better than our current best
            bestInlierSet = X(line_inliers1,:);
            bestInlierSetIndex = line_inliers1;
            bestInlierType = 'line';
            
            V = P2 - P1;
            V_hat = V./norm(V);
            distances = X - P2;
            projectedCoordinate = distances(line_inliers1, :)*V_hat';
            bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*V_hat + repmat(candidates(2, :), [2, 1]);
        
            if isempty(bestEndPoints)
                bestInlierSet = [];
                bestInlierType = '';
                bestInlierSetIndex = [];
                continue
            end
%         elseif sum(line_inliers2) > size(bestInlierSet, 1) && biggestGap2 < .2 % If second line is better
%             bestInlierSet = X(line_inliers2,:);
%             bestInlierSetIndex = line_inliers2;
%             bestInlierType = 'line';
%             
%             V = P3 - P1;
%             V_hat = V./norm(V);
%             distances = X - P3;
%             projectedCoordinate = distances(line_inliers2, :)*V_hat';
%             bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*V_hat + repmat(candidates(2, :), [2, 1]);
%             
%         elseif sum(line_inliers3) > size(bestInlierSet, 1) && biggestGap3 < .2 % If third line is better
%             bestInlierSet = X(line_inliers3,:);
%             bestInlierSetIndex = line_inliers3;
%             bestInlierType = 'line';
%             
%             V = P3 - P2;
%             V_hat = V./norm(V);
%             distances = X - P3;
%             projectedCoordinate = distances(line_inliers3, :)*V_hat';
%             bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*V_hat + repmat(candidates(2, :), [2, 1]);
            
        elseif sum(circle_inliers) > size(bestInlierSet, 1) && biggestGapC < 0.01 && sum(circle_inliers) > 3 % If circle is better
            inlier_points = X(circle_inliers,:);
            continue_flag = 0;
            for j=1:size(inlier_points, 1)*10
                arc_candidates = datasample(inlier_points, 3, 'Replace', false);
                C1 = arc_candidates(1,:);
                C2 = arc_candidates(2,:);
                C3 = arc_candidates(3,:);

                [new_circle_inliers, new_xc, new_yc, ~, ~] = circleDetector(inlier_points, d/2, C1, C2, C3);

                center_offset = sqrt((xc-new_xc)^2 + (yc-new_yc)^2);
                if center_offset > d || (sum(new_circle_inliers) ~= sum(circle_inliers))
                    continue_flag = 1;
                    break
                end
            end
            
            if continue_flag == 1
                continue
            end
            
            if r > .275 || r < .225
                continue
            end
            
            bestInlierSet = X(circle_inliers,:);
            bestInlierSetIndex = circle_inliers;
            bestInlierType = 'circle';

            center = [xc, yc];
            bestCircle = [center, r];
        else
            % Nothing was better than our current best
            continue
        end
    end
    
    outliers = X(~bestInlierSetIndex,:);
    
    if strcmp(bestInlierType, 'line')
        line = bestEndPoints;
    elseif strcmp(bestInlierType, 'circle')
        circle = bestCircle;
    else
        return
    end
end

function [line_inliers, biggestGap] = lineDetector(X, d, P1, P2)
    % Vector between the two points
    V = P2 - P1;
    
    % Unit vector between the two points
    V_hat = V./norm(V);
    
    % Orthogonal vector
    O = [-V(2) V(1)];
    
    % Orthogonal unit vector
    O_hat = O./norm(O);
    
    % Calculate perpendicular distances to the line
    distances = X - P2;
    perpendicular_distances = distances * O_hat';
    
    % Get the inliers (less than the threshold d)
    line_inliers = abs(perpendicular_distances) < d;
    
    % Calculate the biggest gap in the inliers
    biggestGap = max(diff(sort(distances(line_inliers,:)*V_hat')));
end

function [circle_inliers, xc, yc, r, biggestGap] = circleDetector(X, d, P1, P2, P3)
    % Get x's and y's of the three sampled points
    P = [P1; P2; P3];
    Px = P(:,1);
    Py = P(:,2);

    % Linear systems of equations to get the equation of the circle
    A = [Px, Py, ones(size(Px))];
    b = -Px.^2 - Py.^2;
    w = A \ b;

    % Center and radius of the circle
    xc = -w(1)/2;
    yc = -w(2)/2;
    r = sqrt(xc.^2 + yc.^2 - w(3));

    % Get the distance from every point to the circle
    distance_to_circle = abs(sqrt((X(:,1) - xc).^2 + (X(:,2) - yc).^2) - r);
    circle_inliers = (distance_to_circle < d);
    
    % If there are fewer than 10 inliers, it's probably not a circle
    if sum(circle_inliers) < 10
        biggestGap = 0;
        return
    end

    % List of points that are inliers
    inlier_points = X(circle_inliers);

    % Find biggest gap %
    % Choose a point on the arc as 0 degrees
    arc_point = datasample(inlier_points, 1);
    
    angles = [];
    for i=1:size(inlier_points, 1)
        angle = dot(inlier_points(i,:), arc_point) ./ norm(inlier_points(i,:)) * norm(arc_point);
        angles = [angles; angle];
    end
    positions = angles.*r;
    biggestGap = max(diff(sort(positions)));
end