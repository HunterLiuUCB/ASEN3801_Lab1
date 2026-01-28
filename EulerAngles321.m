function attitude321 = EulerAngles321(DCM)
% Inputs:
%   DCM – Direction Cosine Matrix representing the transformation from the
%         inertial North-East-Down (NED) frame to the body frame
%
% Outputs:
%   attitude321 – 3×1 vector of 3-2-1 Euler angles [alpha; beta; gamma],
%                 corresponding to roll, pitch, and yaw, respectively
%
% Methodology:
%   The 3-2-1 Euler angles are extracted directly from the Direction Cosine
%   Matrix by equating the analytical form of the 3-2-1 DCM with the given
%   matrix. Specific elements of the DCM are used to solve for each Euler
%   angle using inverse trigonometric relationships. The pitch angle (beta)
%   is obtained from the (1,3) element of the DCM using the arcsine function,
%   while the roll (alpha) and yaw (gamma) angles are computed using the
%   arctangent of appropriate element ratios to preserve quadrant
%   information. 
alpha = atan(DCM(2,3)/DCM(3,3));
beta = -asin(DCM(1,3));
gamma = atan(DCM(1,2)/DCM(1,1));
attitude321 = [alpha;beta;gamma];
end
