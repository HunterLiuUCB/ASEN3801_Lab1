% Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
% Course number: ASEN 3801
% File name: LoadASPENData.m
% Created: 1/20/26

function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att,pos_av_aspen] = LoadASPENData(filename)
% Inputs: filename of a .csv file with data from the motion capture system in the ASPEN lab
%
% Outputs: t_vec = a 1xn time vector (s), n = total number of frames from the dataset
%          av_pos_inert = 3xn matrix of position vectors for the vehicle in Frame E
%          av_att = 3xn matrix of attitude vectors listing the 3-2-1 Euler angles in radians for the vehicle relative to Frame E
%          tar_pos_inert = 3xn matrix of position vectors for the target in Frame E
%          tar_att = 3xn matrix of attitude vectors listing the 3-2-1 Euler angles in radians for the target relative to Frame E
%
% Methodology: This function reads in one large data file, conditioning it using a DataConditioning function.
% It then splits this data up accordingly into each required output vector/matrix. First, it
% just splits up the data according to the indices of the data subsets
% within the larger data set. Then, it uses the given ConvertASPENData
% function to convert from the data file's format to the final outputted
% vectors and matrices.

data = DataConditioning(filename); % reading in and conditioning data file
t_vec = (1:length(data))/100; % creating the time vector, adjusting for frame rate
pos_av_aspen = data(:, 11:13)'; % extracting vehicle position vectors
att_av_aspen = data(:,8:10)'; % extracting vehicle Euler angles
pos_tar_aspen = data(:,5:7)'; % extracting target position vectors
att_tar_aspen = data(:,2:4)'; % extracting target Euler anlges

% converting outputs into appropriate form using the given ConvertASPENData
% function
[pos_av_class, att_av_class, pos_tar_class, att_tar_class] = ConvertASPENData(pos_av_aspen, att_av_aspen,  pos_tar_aspen, att_tar_aspen);

av_pos_inert =  pos_av_class / 1000; % converting mm to m
av_att = att_av_class;
tar_pos_inert = pos_tar_class / 1000; % converting mm to m
tar_att = att_tar_class;
end
