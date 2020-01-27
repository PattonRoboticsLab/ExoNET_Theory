% ***********************************************************************************
% [jac] = jacobian(phi,L)
% where phi is a [2x1] instance of the joint angles
% ***********************************************************************************

function [jac] = jacobian(phi,L)

jac = [-L(1)*sind(phi(1))-L(2)*sind(phi(1)+phi(2)), -L(2)*sind(phi(1)+phi(2)); ...
        L(1)*cosd(phi(1))+L(2)*cosd(phi(1)+phi(2)),  L(2)*cosd(phi(1)+phi(2))];

end