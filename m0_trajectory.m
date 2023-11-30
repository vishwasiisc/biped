function [x,y,z] = m0_trajectory(t1,y0,l,xh,zh,pause)
T=pause;
y=y0*(1-(4*t1/T)+(2*(t1^2)/T^2));
x=xh;
sth=y/l;
cth=sqrt(1-sth^2);
th=atan(sth/cth);
z=zh+cos(th)*l;

end

