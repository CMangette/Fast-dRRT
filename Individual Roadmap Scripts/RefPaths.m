function refPath = RefPaths(xi,xf,intention)
persistent right;
persistent straight;
persistent left;

% Initialize persistent variables
if(isempty(right) || isempty(straight) || isempty(left))
    right = 1;
    straight = 2;
    left = 3;
end

numPaths = 0;
switch(intention)
    case right
        numPaths = 2;
    case left
        numPaths = 2;
    case straight
        numPaths = 3;
end

refPath = cell(1);

if(round(xf(3)-xi(3),3) == 0)
    
    rotationAngle = xi(3);
    R = [cos(rotationAngle) -sin(rotationAngle);
         sin(rotationAngle) cos(rotationAngle)];
    
    x = linspace(xi(1),xf(1),100);
    y = linspace(xi(2),xf(2),100);
    theta = xi(3)*ones(1,length(x));
    k = zeros(1,length(x));
    L = norm(xf(1:2)-xi(1:2));
    s = linspace(0,L,length(x));
    
    refPath{1} = [x;y;theta;k;s];
    refPath{2} = [4*R*ones(2,100) + [x;y];theta;k;s];
else
    
    % Otherwise, Compute base paths as cubic splines
    
   
    % Project Base curve control points
    
    intersection_point = FindIntersectionPoint(xi,xf);
    ctrlPts = GenerateControlPoints(xi,xf,intersection_point);
    
    % Initial Path Generation
    refPath{1} = SplineCurve(ctrlPts);
    
    L = sum(refPath{1}(5,:));
    refPath{1}(5,:) = linspace(0,L,length(refPath{1}(1,:)));
    R_i = [cos(xi(3)) -sin(xi(3)) 0 0;
           sin(xi(3)) cos(xi(3)) 0 0;
           0 0 0 0;
           0 0 0 0];
    
    R_f = [cos(xf(3)) -sin(xf(3)) 0 0;
           sin(xf(3)) cos(xf(3)) 0 0;
           0 0 0 0;
           0 0 0 0];
    offset = zeros(4,1);
    if(intention == right)
        offset(2) = 4;
    else
        offset(2) = -4;
    end
    
    xi_offset = xi' + R_i*offset;
    xf_offset = xf' + R_f*offset;
    
    intersection_point_offset = FindIntersectionPoint(xi_offset,xf_offset);
    ctrlPts_offset = GenerateControlPoints(xi_offset,xf_offset,intersection_point_offset);
    refPath{2} = SplineCurve(ctrlPts_offset);
    
    L = sum(refPath{2}(5,:));
    refPath{2}(5,:) = linspace(0,L,length(refPath{2}(1,:)));
end
end

function pts = FindIntersectionPoint(xi,xf)
    Ti = [cos(xi(3));sin(xi(3))];
    Tf = [cos(xf(3));sin(xf(3))];
    
    pts = zeros(1,2);
    if(round(Ti(1),3) == 0)
        pts(1) = xi(1);
        pts(2) = xf(2);
    else
        pts(1) = xf(1);
        pts(2) = xi(2);
    end

end

function ctrlPts = GenerateControlPoints(xi,xf,intersection_point)
    ctrlPts1 = [linspace(xi(1),intersection_point(1),5);
                linspace(xi(2),intersection_point(2),5)];
    
    ctrlPts2 = [linspace(intersection_point(1),xf(1),5);
                linspace(intersection_point(2),xf(2),5)];
            
    ctrlPts2(:,1) = [];
                
    ctrlPts = [ctrlPts1 ctrlPts2];
end

function refPath = SplineCurve(ctrlPts)
    
    numCurves = ceil(length(ctrlPts)/3);
    refPath = [];
    for i = 1:numCurves
        
        idx1 = 3*i - 2;
        idx2 = idx1 + 2;
        
        refPath = [refPath bezierCurve2(ctrlPts(:,(idx1:idx2)),0.05)];
    end
    
end