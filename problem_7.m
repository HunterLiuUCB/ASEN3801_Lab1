% Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
% Course number: ASEN 3801
% File name: problem_7.m
% Created: 1/27/26

function tar_rel_pos_body = problem_7(tar_rel_pos, av_att)
% Inputs:
%   tar_rel_pos - 3xn matrix of the target position relative to the
%                 aerospace vehicle expressed in the inertial (NED) frame,
%                 where each column is [dx; dy; dz] at a time step
%   av_att      – 3xn matrix of the aerospace vehicle 3-2-1 Euler angles
%                 (rad) relative to the inertial (E/NED) frame. Rows are
%                 [roll; pitch; yaw] and columns correspond to time steps.
%
% Outputs:
%   tar_rel_pos_body – 3xn matrix of the target position relative to the
%                      aerospace vehicle expressed in the aerospace vehicle
%                      body frame, where each column is the relative
%                      position resolved in the body axes at that time step
%
% Methodology:
%   This function converts the relative position vector from the inertial
%   frame to the aerospace vehicle body frame at each time step. For each
%   column of the input data, the aerospace vehicle's 3-2-1 Euler angles are
%   used to compute the Direction Cosine Matrix (DCM) from the inertial
%   (NED) frame to the body frame using RotationMatrix321. The inertial
%   relative position vector is then premultiplied by this DCM to obtain the
%   same physical vector resolved in body coordinates. The results are
%   stored across the full time history.

tar_rel_pos_body = zeros(size(tar_rel_pos));

% Iterate through each time step (each column)
for i = 1:size(tar_rel_pos, 2)
    % DCM from inertial frame to AV body frame at time i
    DCM_RelBody = RotationMatrix321(av_att(1:3, i));

    % Convert relative position from inertial to body frame
    tar_rel_pos_body(1:3, i) = DCM_RelBody * tar_rel_pos(1:3, i);
end

end
