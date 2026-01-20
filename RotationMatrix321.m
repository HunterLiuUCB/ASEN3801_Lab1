function DCM = RotationMatrix321(attitude321)
% Inputs: 
%
% Outputs: 
%
% Methodology: 
alpha = attitude321(1);
beta = attitude321(2);
gamma = attitude321(3);
% x-axis rotation
r1 = [1 0 0; 0 cos(alpha) sin(alpha); 0 -sin(alpha) cos(alpha)];
% y-axis rotation
r2 = [cos(beta) 0 -sin(beta); 0 1 0; sin(beta) 0 cos(beta)];
% z-axis rotation
r3 = [cos(gamma) sin(gamma) 0; -sin(gamma) cos(gamma) 0; 0 0 1];

% Sequence 3-2-1
DCM = r1 * r2* r3; % computes DCM in reverse order of the sequence
end