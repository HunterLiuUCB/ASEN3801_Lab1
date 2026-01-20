function attitude321 = EulerAngles321(DCM)
% Inputs: 
%
% Outputs: 
%
% Methodology: 
alpha = atan(DCM(2,3)/DCM(3,3));
beta = -asin(DCM(1,3));
gamma = atan(DCM(1,2)/DCM(1,1));
attitude321 = [alpha;beta;gamma];
end