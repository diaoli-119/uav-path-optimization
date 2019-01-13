lengthGuess = 1;
widthGuess = 1;
heightGuess = 1;

x0 = [lengthGuess widthGuess heightGuess];

xopt = fmincon(@objective,x0,[],[],[],[],[],[],@constraint,[])
 
volumeOpt = calcVolume(xopt)

surfaceAreaOpt = calcSurface(xopt)

function volume = calcVolume(x)
    length = x(1);
    width = x(2);
    height = x(3);
    volume = length * width * height;
end

function surfaceArea = calcSurface(x)
    length = x(1);
    width = x(2);
    height = x(3);
    surfaceArea = 2 * length * width + 2 * length * height + 2 * height * width;
end

function obj = objective(x)
    obj = -calcVolume(x);
end

function [c, ceq] = constraint(x)
    c = calcSurface(x) - 10;
    ceq = [];
end