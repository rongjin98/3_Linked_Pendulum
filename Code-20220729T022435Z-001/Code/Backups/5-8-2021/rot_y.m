function transformation = rot_y(theta)
% This function serves the purpose of ouputting a transformation matrix
% about the indicate axis
transformation = [cos(theta) 0 sin(theta) 0;
                  0          1 0          0; 
                 -sin(theta) 0 cos(theta) 0; 
                  0          0 0          1];
end
