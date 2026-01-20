%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: main_Lab2.m
%Created: 1/20/26

clear
clc
close all

%% Problem 1
filename = '3801_Sec001_Test1.csv';
[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename);

attitude321 = [60 * pi/180;60* pi/180;60 * pi/180];
attitude313 = [60 * pi/180;60* pi/180;60 * pi/180];
DCM = RotationMatrix321(attitude321);
DCM2 = RotationMatrix313(attitude313);