function [B] = bezierCurve2(ctrlPts,res)

% This function generates a Bezier Curve to smooth curve between control
% points
i = 1;
Q = [ctrlPts(:,1)' - 2*ctrlPts(:,2)' + ctrlPts(:,3)';
                -2*ctrlPts(:,1)' + 2*ctrlPts(:,2)';
                ctrlPts(:,1)'];
k = zeros(1,length([0:res:1]));
B = zeros(2,length([0:res:1]));
theta = zeros(1,length([0:res:1]));
s = zeros(1,length([0:res:1]));
for t = 0:res:1
    B(:,i) = ([t.^2 t 1]*Q)';
    derivative = ([2*t 1]*Q(1:2,:))';
    ddB = (2*Q(1,:))';
    
    theta(i) = atan2(derivative(2),derivative(1));
    k(i) = det([derivative ddB])/(norm(derivative).^3);
    
    if(i == 1)
        s(i) = 0;
    else
        s(i) = norm(B(:,i)-B(:,i-1)) + s(i-1);
    end
    i = i + 1;
end

B = [B;theta;k;s];