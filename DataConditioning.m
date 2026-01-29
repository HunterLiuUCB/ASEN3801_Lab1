%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: DataConditioning.m
%Created: 1/20/26
function [CSV_Final] = DataConditioning(CSV_File)
% Inputs: CSV_File - data file of '.csv' type for conditioning and analysis
%
% Outputs: CSV_Final - Conditioned matrix containing all data with all
% headers, NaN values, and extraneous data removed.
%
% Methodology: Converts the csv file to a matrix. This turns all of the
% wording to NaN values which are then removed. Final the 2nd column is
% removed as it is not needed in the analysis

% Read the input CSV file
data = readmatrix(CSV_File);
data = rmmissing(data); %Remove all NaN values and wording from the matrix
data(:, 2) = []; % Remove the second column from the matrix
CSV_Final = data; % Placeholder for further processing
end