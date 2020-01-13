function [points]= getLine(startpoint,endpoint)
%        [points]= getLine(startpoint,endpoint)
%
%   INPUT
%       startpoint      .....   [3x1]   begin of line
%       endpoint        .....   [3x1]   end of line
%
%   OUTPUT
%       pionts          .....   [:x3]   matrix with all points


normalvector = (endpoint - startpoint)/norm((endpoint - startpoint));

ss = 0:0.02:norm((endpoint - startpoint));

points = startpoint' + ss' * normalvector';