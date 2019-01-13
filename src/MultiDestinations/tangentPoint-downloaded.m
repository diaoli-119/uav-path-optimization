function [xtRet,ytRet] = tangentPoint(Xinput, Yinput, Xobs, Yobs, r, w, flag)	%(xtRet, ytRet) coordination of tangentPoint, (Xcur, Ycur) current point, 
																				%(Xobs, Yobs) coordination of center of obstacle

x0=Xobs; y0=Yobs; %Coordination of the center of obstacle

r0=r+w;	%Radius of obstacle

x1=Xinput; y1=Yinput; %Coordination of point which is outside of the obstacle

k1=(y0*x0+y1*x1-y0*x1-y1*x0+(r0^2*(-2*y0*y1-2*x0*x1+y1^2+y0^2+x0^2-r0^2+x1^2))^(1/2))/(-r0^2+x0^2-2*x0*x1+x1^2)

k2= (y0*x0+y1*x1-y0*x1-y1*x0-(r0^2*(-2*y0*y1-2*x0*x1+y1^2+y0^2+x0^2-r0^2+x1^2))^(1/2))/(-r0^2+x0^2-2*x0*x1+x1^2)

x_1=(-k1*y1+x0+k1^2*x1+y0*k1)/(1+k1^2)

y_1 =-(-y1-k1*x0-y0*k1^2+k1*x1)/(1+k1^2)

x_2=(-k2*y1+x0+k2^2*x1+y0*k2)/(1+k2^2)

y_2 =-(-y1-k2*x0-y0*k2^2+k2*x1)/(1+k2^2)

if ( 1 == flag)
    ytRet = max(y_1,y_2);   %select the above intersected point
    xtRet = min(x_1,x_2);
else
    ytRet = min(y_1,y_2);   %select the below intersected point
    xtRet = max(x_1,x_2);
end