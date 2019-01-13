function [xtRet,ytRet] = tangentPoint(Xcur, Ycur, Xo, Yo, radius, width, flag)	%(xtRet, ytRet) coordination of tangentPoint, (Xcur, Ycur) current point, 
																		%(Xo, Yo) coordination of center of obstacle

syms xt yt Xtp Ytp;
%Equation one: square of distance between center of obstacle and current point equals square of distance between tangentPoint and current 
%point plus square of sumation of obstacle and UAV wing span. Take coordination of current point (Pcur): (Xcur, Ycur)
%(Xo - Xcur).^2 + (Yo - Ycur).^2 == (xt - Xcur).^2 + (yt - Ycur).^2 + (radius + width).^2;

%Equation two: tangentPoint (xt, yt) is on obstacle 
%(xt - Xo).^2 + (yt - Yo).^2 == (radius + width).^2]

%Combine Equation one and two to work out coordination of tangentPoint: (Xtp, Ytp)
vars = [xt yt];
eqns = [(Xo - Xcur).^2 + (Yo - Ycur).^2 == (xt - Xcur).^2 + (yt - Ycur).^2 + (radius + width).^2,
		(xt - Xo).^2 + (yt - Yo).^2 == (radius + width).^2];
s = solve(eqns, vars);
double(s.xt);
double(s.yt);
%Select an intersected point based on flag
if ( 1 == flag)
    xtRet = min(double(s.xt));   %select the above intersected point
    ytRet = max(double(s.yt));
else
    xtRet = max(double(s.xt));   %select the above intersected point
    ytRet = min(double(s.yt));
end

%Convert to string
%xtStr = sprintf("%f", xt);
%ytStr = sprintf("%f", yt);

%Convert to numeric number
%xtRet = str2num(xtStr)
%ytRet = str2num(ytStr)

%Calculate equation of intersected line
%(y - yp)/(Ycur - yp) == (x - xp)/(Xcur - xp);

%syms xp yp
%vars = [xp yp];
%eqns = [xp^2 - 2*5*xp + 5^2 + yp^2 - 2*5*yp + 5^2 == (1 + 1)^2, (5 - 1)^2 + (5 - 1)^2 == (xp - 1)^2 + (yp - 1)^2 + (1 + 1)^2];