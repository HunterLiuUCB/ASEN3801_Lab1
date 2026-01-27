function [tar_attitude_313,av_attitude_313] = problem_5(CSV_filename)
%PROBLEM_5 is used to calculate the 3-1-3 euler angles of both the target
%and the aerospace vehicle. The CSV_file is ran through multiple functions
%to get the attitude of each of the bodys as a function of time then
%finally passed into the EulerAngles313.m function which will yield
%alpha,beta,gamma at a given time for each vehicle.

[t_vec, ~, av_att, ~, tar_att] = LoadASPENData(CSV_filename); % av_att and tar_att are 3xn matrices
% Where the first the first row is roll (alpha) second row is pitch (beta)
% third row is yaw (gamma)
av_attitude_313 = zeros(size(av_att)); % Initializing 3xn vectors for storage
tar_attitude_313 = av_attitude_313; % Alpha is first row, Beta is second, gamme is third for eahc column
for i = 1:length(t_vec)
[DCM_av] = RotationMatrix321(av_att(:,i)); % DCM for aerospace vehicle design
[DCM_tar]= RotationMatrix321(tar_att(:,i));

av_attitude_313(:,i) = EulerAngles313(DCM_av);
tar_attitude_313(:,i) = EulerAngles313(DCM_tar);


end
% Converting to degrees from radians
av_attitude_313 = rad2deg(av_attitude_313);
tar_attitude_313 = rad2deg(tar_attitude_313);
%Plots

figure();
sgtitle('Problem 5: Aerospace Vechicle & Target Attitude vs time ')
subplot(3, 1, 1);
hold on
plot(t_vec, av_attitude_313(1,:),'b');
plot(t_vec,tar_attitude_313(1,:),'r');
hold off
xlabel('Time (s)');
ylabel('\alpha (deg)');
title('\alpha vs Time');
subplot(3, 1, 2);
hold on
plot(t_vec, av_attitude_313(2,:),'b');
plot(t_vec,tar_attitude_313(2,:),'r');
hold off
xlabel('Time (s)');
ylabel('\beta(deg)');
title('\beta vs Time');
subplot(3, 1, 3);
hold on
plot(t_vec, av_attitude_313(3,:),'b');
plot(t_vec,tar_attitude_313(3,:),'r');
hold off
xlabel('Time (s)');
ylabel('\gamma (deg)');
title('\gamma vs Time');




end