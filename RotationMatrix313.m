%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: RotationMatrix313.m
%Created: 1/20/26
function DCM = RotationMatrix313(attitude313)
% Inputs:
%   attitude313 – 3×1 vector of 3-1-3 Euler angles [alpha; beta; gamma],
%                 corresponding to rotations about the z-, x-, and z-axes,
%                 respectively
%
% Outputs:
%   DCM – Direction Cosine Matrix representing the transformation from the
%         inertial North-East-Down (NED) frame to the body frame
%
% Methodology:
%   The Direction Cosine Matrix is constructed using a 3-1-3 Euler rotation
%   sequence. Individual rotation matrices are defined for rotations about
%   the z-axis, x-axis, and z-axis. These matrices are multiplied in reverse
%   order of the Euler rotation sequence to form the overall passive
%   transformation matrix from the inertial frame to the body frame. 

alpha = attitude313(1);
beta  = attitude313(2);
gamma = attitude313(3);

% z-axis rotation (first rotation)
r3_alpha = [ cos(alpha)  sin(alpha) 0;
            -sin(alpha)  cos(alpha) 0;
             0           0          1];

% x-axis rotation (second rotation)
r1 = [1 0 0;
      0 cos(beta)  sin(beta);
      0 -sin(beta) cos(beta)];

% z-axis rotation (third rotation)
r3_gamma = [ cos(gamma)  sin(gamma) 0;
            -sin(gamma)  cos(gamma) 0;
             0           0          1];

% Sequence 3-1-3
DCM = r3_gamma * r1 * r3_alpha;  

end
