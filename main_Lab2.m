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

% attitude321 = [60 * pi/180;60* pi/180;60 * pi/180];
% attitude313 = [60 * pi/180;60* pi/180;60 * pi/180];
% DCM = RotationMatrix321(attitude321);
% DCM2 = RotationMatrix313(attitude313);
% attitude321_test = EulerAngles321(DCM);
% attitude313_test = EulerAngles313(DCM);

%Problem 3
figure(1)
%Plots each axis of the inertial position of the aerospace vehicle
plot3(av_pos_inert(1,:), av_pos_inert(2,:), av_pos_inert(3,:), 'b');
hold on
grid on
%Plots each axis of the inertial position of the target vehicle
plot3(tar_pos_inert(1,:), tar_pos_inert(2,:), tar_pos_inert(3,:), 'r--');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Problem 3: 3D Plot of Inertial Position');
legend('Aerospace Vehicle Position','Target Position')
axis equal;

%% Problem 4: x, y, z position vs time
figure(2);
% 3x1 Subplot
subplot(3, 1, 1);
sgtitle("Problem 4: Position Vector vs Time");
hold on;
%Labeling
title("X Position vs Time");
xlabel("Time (s)");
ylabel("Position (m)");
grid on;
%Plots
plot(t_vec, tar_pos_inert(1,:),'r'); % target position, x
plot(t_vec, av_pos_inert(1,:), 'b'); % vehicle position, x
lgd = legend("Target", "Aerospace Vehicle", 'Location','southwest');
hold off;
subplot(3, 1, 2);
hold on;
%Labeling
title("Y Position vs Time");
xlabel("Time (s)");
ylabel("Position (m)");
grid on;
%plots
plot(t_vec, tar_pos_inert(2, :), 'r'); %target position, y
plot(t_vec, av_pos_inert(2,:), 'b'); %vehicle position, y
lgd = legend("Target", "Aerospace Vehicle", 'Location','southwest');
hold off;
subplot(3, 1, 3);
hold on;
%labeling
title("Z Position vs Time");
xlabel("Time (s)");
ylabel("Position (m)");
grid on;
%Plots
plot(t_vec, tar_pos_inert(3, :), 'r'); %target position, z
plot(t_vec, av_pos_inert(3,:), 'b'); %vehicle position, z
lgd = legend("Target", "Aerospace Vehicle", 'Location','southwest');
hold off;
%% P4: Euler Angles (alpha, beta, gamma) vs time
figure(3);
%3x1 subplot
subplot(3,1,1);
sgtitle("Problem 4: 3-2-1 Euler Angles vs Time");
hold on;
%labeling
title("\alpha vs Time");
xlabel("Time(s)");
ylabel("Angle (degrees)");
grid on;
%plots
plot(t_vec, rad2deg(tar_att(1, :)), 'r'); % target alpha
plot(t_vec, rad2deg(av_att(1, :)), 'b'); %vehicle alpha
lgd = legend("Target", "Aerospace Vehicle", 'Location','northwest');
hold off;
subplot(3,1,2);
hold on;
%labeling
title("\beta vs Time");
xlabel("Time(s)");
ylabel("Angle (degrees)");
grid on;
%plots
plot(t_vec, rad2deg(tar_att(2, :)), 'r'); %target beta
plot(t_vec, rad2deg(av_att(2, :)), 'b'); %vehicle beta
lgd = legend("Target", "Aerospace Vehicle", 'Location','northwest');
hold off;
subplot(3,1,3);
hold on;
%labeling
title("\gamma vs Time");
xlabel("Time(s)");
ylabel("Angle (degrees)");
grid on;
%plots
plot(t_vec, rad2deg(tar_att(3, :)), 'r'); %target gamma
plot(t_vec, rad2deg(av_att(3, :)), 'b'); % vehicle gamma
lgd = legend("Target", "Aerospace Vehicle", 'Location','northwest');
hold off;
% Problem 5
[tar_attitude_313,av_attitude_313] = problem_5(filename);

% Problem 6 
%Finds inertial relative position of target by taking difference between
%target and the AV.
tar_rel_pos = tar_pos_inert - av_pos_inert;

%Subplotted figure showing each component of position
figure(4);
sgtitle('Problem 6: Position of Target Relative to Aerospace Vehicle in Inertial Frame')
subplot(3, 1, 1);
plot(t_vec,tar_rel_pos(1,:), 'b');
xlabel('Time (s)');
ylabel('X Position (m)');
title('Relative Position - X Component');
grid on;
subplot(3, 1, 2);
plot(t_vec,tar_rel_pos(2,:), 'g');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Relative Position - Y Component');
grid on;
subplot(3, 1, 3);
plot(t_vec,tar_rel_pos(3,:), 'r');
xlabel('Time (s)');
ylabel('Z Position (m)');
title('Relative Position - Z Component');
grid on;

%Problem 7
% Calling problem 7 function to find the relative position to the AV in
% body frame
[tar_rel_pos_body] = problem_7(tar_rel_pos,av_att);

% Plotting each component of the position
figure(5);
sgtitle('Problem 7: Position of Target Relative to Aerospace Vehicle in Body Frame')
subplot(3, 1, 1);
plot(t_vec,tar_rel_pos_body(1,:), 'b');
xlabel('Time (s)');
ylabel('X Position (m)');
title('Relative Position - X Component');
grid on;
subplot(3, 1, 2);
plot(t_vec,tar_rel_pos_body(2,:), 'g');
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Relative Position - Y Component');
grid on;
subplot(3, 1, 3);
plot(t_vec,tar_rel_pos_body(3,:), 'r');
xlabel('Time (s)');
ylabel('Z Position(m)');
title('Relative Position - Z Component');
grid on;

% Exporting figures as PNG files
figure(1);
saveas(gcf, 'Problem3.png');

figure(2);
saveas(gcf, 'Problem4a.png');

figure(3);
saveas(gcf, 'Problem4b.png');

figure(4);
saveas(gcf, 'Problem6.png');

figure(5);
saveas(gcf, 'Problem7.png');