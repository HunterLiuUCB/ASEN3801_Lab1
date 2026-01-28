function DCM = RotationMatrix321(attitude321)
% Inputs:
%   attitude321 – 3×1 vector of 3-2-1 Euler angles [alpha; beta; gamma],
%                 corresponding to roll, pitch, and yaw, respectively
%
% Outputs:
%   DCM – Direction Cosine Matrix representing the transformation from the
%         inertial North-East-Down (NED) frame to the body frame
%
% Methodology:
%   The Direction Cosine Matrix is constructed using a 3-2-1 Euler rotation
%   sequence. Individual rotation matrices are formed for rotations about
%   the body x-axis (roll), y-axis (pitch), and z-axis (yaw). These matrices
%   are then multiplied together in reverse order of the rotation sequence
%   to obtain the overall transformation matrix from the inertial frame to
%   the body frame. 
alpha = attitude321(1);
beta = attitude321(2);
gamma = attitude321(3);
% x-axis rotation
r1 = [1 0 0; 0 cos(alpha) sin(alpha); 0 -sin(alpha) cos(alpha)];
% y-axis rotation
r2 = [cos(beta) 0 -sin(beta); 0 1 0; sin(beta) 0 cos(beta)];
% z-axis rotation
r3 = [cos(gamma) sin(gamma) 0; -sin(gamma) cos(gamma) 0; 0 0 1];

% Sequence 3-2-1
DCM = r1 * r2* r3; % computes DCM in reverse order of the sequence
end
