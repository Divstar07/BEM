%{
% function textAf: Reads in a text file containing airfoil data and returns
Cl and Cd for any given angle of attack.

NOTE: To use this function, the airfoil data must be from xfoil or have 12
lines preceeding the first data entry

Arguments:
- Re: Chord based reynolds number of provided data
- alpha: Angle of attack
- filename: Airfoil data file name
Outputs:
- Cl: Lift coefficient
- Cd: Drag coefficient
%}
function [Cl, Cd] = textAf(alpha, filename)

% Store af data in array
dat = importdata(filename, ' ', 12);

% Find row of alpha, and search for corresponding Cl and Cd
alphaNearest = interp1(dat.data(:, 1), dat.data(:,1), alpha, 'nearest')