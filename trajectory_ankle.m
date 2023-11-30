function [x,z]=trajectory_ankle(x0,step_length,max_height,t0,t,step_time)


h=max_height;
t=t-t0;
xf=step_length;
x=(3*xf*(t^2)/(step_time^2))-(2*xf*(t^3))/(step_time^3);
z=4*h*(x/step_length-(x^2/(step_length^2)));
x=x+x0;
%z=4*h*(t/step_time-(t/step_time)^2);
%{
T=step_time;
a1=8*h/step_time;
a2=-16*h/(T^2);
a3=8*h/(T^3);

z=a1*t+a2*(t^2)+a3*(t^3);

%}
end