%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: DataConditioning.m
%Created: 1/20/26
function [CSV_Final] = DataConditioning(CSV_File)
% Inputs: 
%
% Outputs: 
%
% Methodology: 

% Read the input CSV file
data = readmatrix(CSV_File);
data = rmmissing(data);
data(:, 2) = []; % Remove the second column from the matrix
CSV_Final = data; % Placeholder for further processing
end