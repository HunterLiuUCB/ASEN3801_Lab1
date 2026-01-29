%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: EulerAngles313.m
%Created: 1/20/26
function attitude313 = EulerAngles313(DCM)
% Inputs:
%   DCM - Direction Cosine Matrix representing the transformation from the
%         inertial North-East-Down (NED) frame to the body frame
%
% Outputs:
%   attitude313 – 3x1 vector of 3-1-3 Euler angles [alpha; beta; gamma],
%                 corresponding to rotations about the z-, x-, and z-axes,
%                 respectively
%
% Methodology:
%   The 3-1-3 Euler angles are extracted by equating the base form of
%   the 3-1-3 Direction Cosine Matrix with the given DCM. The second Euler
%   angle (beta) is obtained from the (3,3) element of the matrix using the
%   arccosine function. The first (alpha) and third (gamma) angles are then
%   computed using inverse trigonometric relationships involving ratios of
%   off-diagonal elements. The atan2 function is used to preserve quadrant
%   information. 

beta = acos(DCM(3,3));

alpha = atan2(DCM(1,3), -DCM(2,3));
gamma = atan2(DCM(3,1),  DCM(3,2));

attitude313 = [alpha; beta; gamma];
end
