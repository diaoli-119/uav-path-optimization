%Return y coordination of line which has the same x coordination as enter of obstacle
function y = estimateCoordinateonline(Xcur,Ycur,k,x)

%(y - Ycur) = k(x -Xcur); Coordinations of current point and k are available, so equation of line can be worked out. 
%Take x of center of obstacle into equation of line to estimate y
y = k.*x - k.*Xcur + Ycur;