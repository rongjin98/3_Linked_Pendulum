function q = ik(x,y)
% Calculate the joint angle of the arm for a 2R manipulator
A = 1;
B = 1;
C = sqrt(x^2+y^2);
phi = atan2(y,x);

cosbeta = ((B^2-A^2-C^2)/(-2*A*C));
sinbeta = sqrt(1-cosbeta^2);
beta = atan2(sinbeta,cosbeta)

cosgamma = ((C^2-A^2-B^2)/(-2*A*B));
singamma = sqrt(1-cosgamma^2);
gamma = atan2(singamma,cosgamma);

q(1) = phi + beta;
q(2) = pi+gamma;
end

