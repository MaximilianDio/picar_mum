function [time, values] = get_oscillatordata(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [TIME, VALUES] = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the data as column
%  vectors.
%
%  [TIME, VALUES] = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  [time, values] = importfile("D:\tek0001CH1.csv", [22, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 05-Nov-2019 13:46:44

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [22, Inf];
end

%% Setup the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 2);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "values"];
opts.VariableTypes = ["double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
time = tbl.time;
values = tbl.values;
end