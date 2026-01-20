%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: LoadASPENData.m
%Created: 1/20/26
function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
% Inputs: 
%
% Outputs: 
%
% Methodology: 
data = DataConditioning(filename);
t_vec = (1:length(data));
pos_av_aspen = data(:, 11:13)';
att_av_aspen = data(:,8:10)';
pos_tar_aspen = data(:,2:4)';
att_tar_aspen = data(:,5:7)';
[pos_av_class, att_av_class, pos_tar_class, att_tar_class] = ConvertASPENData(pos_av_aspen, att_av_aspen,  pos_tar_aspen, att_tar_aspen);
av_pos_inert =  pos_av_class / 1000;
av_att = att_av_class;
tar_pos_inert = pos_tar_class / 1000;
tar_att = att_tar_class;
end