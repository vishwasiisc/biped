function [th1,th2]=ik_support_leg(xh,x0_ankle,step_length,l1,l2)

th2=0;
cx=x0_ankle+step_length/2;
dx=cx-xh;
phi=acos(dx/(l1+l2));
th1=pi-phi;

end
