function transformation = dhTransform(alpha,a,theta,d)
% Given the specified dh parameters output a homogeneous transofmration
transformation = (rot_x(alpha)*tran_x(a)*rot_z(theta)*tran_z(d));

end

