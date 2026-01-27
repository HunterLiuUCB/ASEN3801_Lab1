%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: problem_7.m
%Created: 1/27/26
function [tar_rel_pos_body] = problem_7(tar_rel_pos,av_att)
% Inputs: 
%
% Outputs: 
%
% Methodology: 


tar_rel_pos_body = zeros(size(tar_rel_pos));
% function that iterates for every iteration of the position
for i=1:length(tar_rel_pos)
%Calculates the DCM of the AV at every attitude orientation its in
DCM_RelBody = RotationMatrix321(av_att(1:3,i));
% Converts the relative position from inertial ot body frame
tar_rel_pos_body(1:3,i) = DCM_RelBody * tar_rel_pos(1:3,i);
end
end