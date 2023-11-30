function [x,z]=trajectory_hip(x0_ankle,step_length,vhs,vhe,t0,t,step_time,l1,l2)
D=step_length;
T=step_time;
t=t-t0;
a=x0_ankle+step_length/4;
b=vhs;
c=(3*D-2*(vhe*T+2*vhs*T))/(2*T^2);%
d=(T*(vhs+vhe)-D)/(T^3);%vh/(T^2)-D/(T^3);%((vh_start+vh_end)*step_time-step_length)/(step_time^3);


x=a+b*t+c*(t^2)+d*(t^3);
z=norm(sqrt((l1+l2)^2-(x-(x0_ankle+step_length/2))^2));
%z=b*(t);
end