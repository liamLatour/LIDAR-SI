function cart = pol2car(pol, pos, clamp, resolution)
%% Variables
%
%   'pol' should be : [distance angle; distance angle; ...]
%   'pos' should be : [X Y; X Y; ...]
%   'clamp' should be : [min max]

if ~exist('pos','var')
    pos = zeros(size(pol, 1), 2);
end
if ~exist('clamp','var')
    clamp = -1;
end
if ~exist('resolution','var')
    resolution = 100000;
end

tempArray = zeros(size(pol, 1), 2);

for i = 1:size(pol, 1)
    X = cos(pol(i, 2))*pol(i, 1) + pos(i, 1);
    Y = sin(pol(i, 2))*pol(i, 1) + pos(i, 2);
    newA = [round(X*resolution), round(Y*resolution)]/resolution;
    if clamp ~= -1
        newA = min(max(newA, clamp(1)), clamp(2));
    end
    
    tempArray(i, :) = newA;
end

cart = tempArray;

end