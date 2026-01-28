function [tar_attitude_313, av_attitude_313] = problem_5(CSV_filename)
% Inputs:
%   CSV_filename – string name of the ASPEN motion-capture .csv file
%                  containing time-history data for the aerospace vehicle
%                  and target
%
% Outputs:
%   tar_attitude_313 –-3xn matrix of the target's 3-1-3 Euler angles (deg)
%                      over time. Rows correspond to [alpha; beta; gamma]
%                      and columns correspond to time steps.
%   av_attitude_313  - 3xn matrix of the aerospace vehicle's 3-1-3 Euler
%                      angles (deg) over time. Rows correspond to
%                      [alpha; beta; gamma] and columns correspond to time.
%
% Methodology:
%   This function loads the vehicle and target 3-2-1 Euler angle attitudes
%   from the provided CSV file using LoadASPENData. For each time step, the
%   3-2-1 Euler angles are converted into a Direction Cosine Matrix (DCM)
%   using RotationMatrix321 (DCM from inertial NED frame to the body frame).
%   The resulting DCM is then converted into 3-1-3 Euler angles using
%   EulerAngles313. The computed 3-1-3 Euler angles are stored for both the
%   target and aerospace vehicle across the full time history, converted
%   from radians to degrees, and plotted versus time. The final figure is
%   saved as Problem5.png.

[t_vec, ~, av_att, ~, tar_att] = LoadASPENData(CSV_filename); % av_att and tar_att are 3xn matrices
% Where the first row is roll (alpha), second row is pitch (beta),
% third row is yaw (gamma)

av_attitude_313 = zeros(size(av_att)); % Initializing 3xn vectors for storage
tar_attitude_313 = av_attitude_313;    % Alpha is first row, Beta second, Gamma third

for i = 1:length(t_vec)
    DCM_av  = RotationMatrix321(av_att(:,i));  % DCM for aerospace vehicle
    DCM_tar = RotationMatrix321(tar_att(:,i)); % DCM for target

    av_attitude_313(:,i)  = EulerAngles313(DCM_av);
    tar_attitude_313(:,i) = EulerAngles313(DCM_tar);
end

% Converting to degrees from radians
av_attitude_313  = rad2deg(av_attitude_313);
tar_attitude_313 = rad2deg(tar_attitude_313);

% Plots
figure(6);
sgtitle('Problem 5: Aerospace Vehicle & Target Attitude vs Time')

subplot(3, 1, 1); hold on
plot(t_vec, av_attitude_313(1,:),'b');
plot(t_vec, tar_attitude_313(1,:),'r');
hold off; grid on
xlabel('Time (s)'); ylabel('\alpha (deg)');
title('\alpha vs Time');
legend("Aerospace Vehicle", "Target", 'Location','northwest');

subplot(3, 1, 2); hold on
plot(t_vec, av_attitude_313(2,:),'b');
plot(t_vec, tar_attitude_313(2,:),'r');
hold off; grid on
xlabel('Time (s)'); ylabel('\beta (deg)');
title('\beta vs Time');
legend("Aerospace Vehicle", "Target", 'Location','northwest');

subplot(3, 1, 3); hold on
plot(t_vec, av_attitude_313(3,:),'b');
plot(t_vec, tar_attitude_313(3,:),'r');
hold off; grid on
xlabel('Time (s)'); ylabel('\gamma (deg)');
title('\gamma vs Time');
legend("Aerospace Vehicle", "Target", 'Location','northwest');

saveas(gcf,'Problem5.png')
end
