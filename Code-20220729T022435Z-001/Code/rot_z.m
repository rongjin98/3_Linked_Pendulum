function transformation = rot_z(theta)
% This function serves the purpose of ouputting a transformation matrix
% about the indicate axis
transformation = [cos(theta) -sin(theta)  0 0;
                  sin(theta)  cos(theta)  0 0; 
                          0            0  1 0; 
                          0            0  0 1];
end
