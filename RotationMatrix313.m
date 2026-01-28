function DCM = RotationMatrix313(attitude313)
% Inputs: 
%
% Outputs: 
%
% Methodology: 
alpha = attitude313(1);
beta = attitude313(2);
gamma = attitude313(3);
% z-axis rotation
r1 = [1 0 0; 0 cos(alpha) sin(alpha); 0 -sin(alpha) cos(alpha)];
% x-axis rotation
r2 = [cos(beta) 0 -sin(beta); 0 1 0; sin(beta) 0 cos(beta)];
% z'-axis rotation
r3 = [cos(gamma) sin(gamma) 0; -sin(gamma) cos(gamma) 0; 0 0 1];

% Sequence 3-1-3
DCM = r1 * r2* r3; % computes DCM in reverse order of the sequence
end
