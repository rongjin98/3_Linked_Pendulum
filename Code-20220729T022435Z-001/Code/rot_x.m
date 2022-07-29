function transformation = rot_x(theta)
% This function serves the purpose of ouputting a transformation matrix
% about the indicate axis
transformation = [1          0           0 0;
                  0 cos(theta) -sin(theta) 0; 
                  0 sin(theta)  cos(theta) 0; 
                  0          0           0 1];
end

