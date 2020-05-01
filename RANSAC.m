function [bestInlierType, bestInlierSet, bestOutlierSet, m, b, bestEndPoints, C, R] = RANSAC(X, n, d)
    bestInlierSet = [];
    bestOutlierSet = [];
    bestInlierType = 'line';
    
    C = zeros(0,2); R = 0;
    m = 0; b = 0; bestEndPoints = zeros(0,2);

    for i=1:n
        candidates = datasample(X, 3, 'Replace', false);
        P1 = candidates(1,:);
        P2 = candidates(2,:);
        P3 = candidates(3,:);
        
        [line_inliers, biggestGap] = lineDetector(X, d, P1, P2);
        
        if sum(line_inliers) > size(bestInlierSet, 1) && biggestGap < .2 && sum(line_inliers) > 10
            bestInlierType = 'line';
            bestInlierSet=X(line_inliers,:);
            bestOutlierSet = X(~line_inliers,:);
            V = P2 - P1;
            V_hat = V./norm(V);
            distances = X - P2;
            projectedCoordinate = distances(line_inliers, :)*V_hat';
            bestEndPoints = [min(projectedCoordinate); max(projectedCoordinate)]*V_hat + repmat(candidates(2, :), [2, 1]);
        
            if isempty(bestEndPoints)
                m = NaN;
                b = NaN;
                bestEndPoints = [NaN,NaN;NaN,NaN];
                continue
            end
        end
        
        [circle_inliers, xc, yc, r, biggestGap] = circleDetector(X, d, P1, P2, P3);
        
        if biggestGap > 0.01
            continue
        end

        if sum(circle_inliers) > size(bestInlierSet, 1) && sum(circle_inliers) > 1
            inlier_points = X(circle_inliers,:);
            continue_flag = 0;
            for i=1:size(inlier_points, 1)
                arc_candidates = datasample(inlier_points, 3, 'Replace', false);
                C1 = arc_candidates(1,:);
                C2 = arc_candidates(2,:);
                C3 = arc_candidates(3,:);

                [new_circle_inliers, new_xc, new_yc, new_r, new_biggestGap] = circleDetector(inlier_points, d/2, C1, C2, C3);

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
            
            %%% THEN IT'S PROBABLY A CIRCLE
            bestInlierType = 'circle';
            bestInlierSet = X(circle_inliers,:);
            bestOutlierSet = X(~circle_inliers,:);
            
            C = [xc, yc];
            R = r;
        end
        
    end
%     hold off

%     
%     if strcmp(bestInlierType, 'line')
%         m = diff(bestEndPoints(:,2))/diff(bestEndPoints(:,1));
%         b = bestEndPoints(1,2)-m*bestEndPoints(1,1);
%         fitline_coefs=[m b];
%     end
end

function [line_inliers, biggestGap] = lineDetector(X, d, P1, P2)
    V = P2 - P1;
    V_hat = V./norm(V);
    O = [-V(2) V(1)];
    O_hat = O./norm(O);
    distances = X - P2;
    perpendicular_distances = distances * O_hat';
    line_inliers = abs(perpendicular_distances) < d;
    biggestGap = max(diff(sort(distances(line_inliers,:)*V_hat')));
end

function [circle_inliers, xc, yc, r, biggestGap] = circleDetector(X, d, P1, P2, P3)
    P = [P1; P2; P3];
    Px = P(:,1);
    Py = P(:,2);

    A = [Px, Py, ones(size(Px))];
    b = -Px.^2 - Py.^2;
    w = A \ b;

    xc = -w(1)/2;
    yc = -w(2)/2;
    r = sqrt(xc.^2 + yc.^2 - w(3));

    distance_to_circle = abs(sqrt((X(:,1) - xc).^2 + (X(:,2) - yc).^2) - r);
    circle_inliers = (distance_to_circle < d);
    
    if sum(circle_inliers) < 10
        biggestGap = 0;
        return
    end

    inlier_points = X(circle_inliers);

    arc_point = datasample(inlier_points, 1);
    
    angles = [];
    for i=1:size(inlier_points, 1)
        angle = dot(inlier_points(i,:), arc_point) ./ norm(inlier_points(i,:)) * norm(arc_point);
        angles = [angles; angle];
    end
    positions = angles.*r;
    biggestGap = max(diff(sort(positions)));
end