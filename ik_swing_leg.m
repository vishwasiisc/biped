function [th1,th2] = ik_swing_leg(xa,za,xh,zh,l1,l2)
dx=xh-xa;
dz=za-zh;
%dx=-dx;

%dx=xa-xh;
%dz=za-zh;

cth2=(norm(dx^2+dz^2-l1^2-l2^2)/(2*l1*l2));
sth2=sqrt(norm(1-cth2^2));
th2=atan2(sth2,cth2);

th1=atan(dx/dz)+atan(l2*sin(th2)/(l1+l2*cos(th2)));


end

