%biped_main
clear;
clc;

%biped configuration
l1=0.3556; l2=0.3556; l3=0.2032; l4=1; l5=0.254; %l1=0.3556
th1=0; th2=0; th3=0; th4=0; th5=0;
%m1=0.1; m2=0.1; m3=0.1; m4=0.1; m5=0.11; m0=1;


x_ankle_i=1;
step_length=0.35;  %0.5; 0.35
step_time=0.25;   %0.8; 0.25
pause=0;  %0.1
h=0.05; %max step height
vhs=0.05;%step_length/(2*step_time); % hip velocity
vhe=0.0;
dt=0.008;
n=9;

%xr_ankle_plt=zeros(step_time/dt,1);
%zr_ankle_plt=zeros(step_time/dt,1);
%t_plt=zeros(step_time/dt,1);



%plotting trajectory
size=n*(step_time+pause)/dt;
size=ceil(size);

%time
t_plt=zeros(size,1);

%rightleg

xr_ankle_plt=zeros(size,1);
yr_ankle_plt=0.08*ones(size,1);
zr_ankle_plt=zeros(size,1);

x_hip_plt=zeros(size,1);
z_hip_plt=zeros(size,1);

xr_knee_plt=zeros(size,1);
yr_knee_plt=0.08*ones(size,1);
zr_knee_plt=zeros(size,1);

th1_plt=zeros(size,1);
th2_plt=zeros(size,1);


%left leg

xl_ankle_plt=zeros(size,1);
yl_ankle_plt=-0.08*ones(size,1);
zl_ankle_plt=zeros(size,1);

xl_knee_plt=zeros(size,1);
yl_knee_plt=-0.08*ones(size,1);
zl_knee_plt=zeros(size,1);

th3_plt=zeros(size,1);
th4_plt=zeros(size,1);

%m0_mass
m0_x=zeros(size,1);
m0_y=zeros(size,1);
m0_z=zeros(size,1);



%variables for live animation
%right
plt_leg_rx=zeros(3,1);
plt_leg_rz=zeros(3,1);
plt_leg_ry=0.08*ones(3,1);

plt_foot_rx=zeros(3,1);
plt_foot_ry=0.08*ones(3,1);
plt_foot_rz=ones(3,1);

%left
plt_leg_lx=zeros(3,1);
plt_leg_lz=zeros(3,1);
plt_leg_ly=-0.08*ones(3,1);

plt_foot_lx=zeros(3,1);
plt_foot_ly=-0.08*ones(3,1);
plt_foot_lz=zeros(3,1);

%hip
hip_x=ones(2,1);
hip_y=ones(2,1);
hip_z=ones(2,1);

%support_knee_x=ones(step_time/dt,1);
%support_knee_x=support_knee_x*(xr_ankle_i+step_length/2);

for k=0:n-1

    x_ankle_i=k*step_length/2;

    


    for i=0:(step_time+pause)/dt


        t=i*dt; % local time starts form zero every step

        if t>=step_time
            t1=t-step_time;
            if rem(k,2)==0
                y0=0.08;
            else
                y0=-0.08;
            end

            %[xm0,ym0,zm0]=m0_trajectory(t1,y0,l3,xh,zh,pause);
            %m0_x(i+1)=xm0;
            %m0_y(i+1)=ym0;
            %m0_z(i+1)=zm0;
            %{
            %do nothing works
            x_ankle_swing=0;
            z_ankle_swing=0;
            x_ankle_support=0;
            z_ankle_support=0;
            %}
            
        else

            %swing leg
            [x_ankle_swing,z_ankle_swing]=trajectory_ankle(x_ankle_i,step_length,h,0,t,step_time);
            [x_hip,z_hip]=trajectory_hip(x_ankle_i,step_length,vhs,vhe,0,t,step_time,l1,l2);
            [th1,th2]=ik_swing_leg(x_ankle_swing,z_ankle_swing,x_hip,z_hip,l1,l2);
            
            %support leg
            [x_ankle_support,z_ankle_support]=trajectory_ankle_support(x_ankle_i,step_length);
            [th3,th4]=ik_support_leg(x_hip,x_ankle_i,step_length,l1,l2);
        end
    
        z_knee_swing=z_hip-l1*cos(th1);
        x_knee_swing=x_hip+l1*sin(th1);

        x_knee_support=x_hip-l2*cos(th3);
        z_knee_support=z_hip-l2*sin(th3);

        i=k*(step_time+pause)/dt + i;
        i=ceil(i);

        if rem(k,2)==0
    
            %right leg
            xr_ankle_plt(i+1)=x_ankle_swing;
            zr_ankle_plt(i+1)=z_ankle_swing;
    
            xr_knee_plt(i+1)=x_knee_swing;
            zr_knee_plt(i+1)=z_knee_swing;
        
            th1_plt(i+1)=th1;
            th2_plt(i+1)=th2;
    
            x_hip_plt(i+1)=x_hip;
            z_hip_plt(i+1)=z_hip;

            %left leg
            xl_ankle_plt(i+1)=x_ankle_support;
            zl_ankle_plt(i+1)=z_ankle_support;
    
            xl_knee_plt(i+1)=x_knee_support;
            zl_knee_plt(i+1)=z_knee_support;

            %m0_mass
            %m0_x(i+1)=;
            %m0_y(i+1)=;
            %m0_z(i+1)=;

        else
            %right leg
            xr_ankle_plt(i+1)=x_ankle_support;
            zr_ankle_plt(i+1)=z_ankle_support;
    
            xr_knee_plt(i+1)=x_knee_support;
            zr_knee_plt(i+1)=z_knee_support;
        
            th1_plt(i+1)=th1;
            th2_plt(i+1)=th2;
    
            x_hip_plt(i+1)=x_hip;
            z_hip_plt(i+1)=z_hip;

            %left leg
            xl_ankle_plt(i+1)=x_ankle_swing;
            zl_ankle_plt(i+1)=z_ankle_swing;
    
            xl_knee_plt(i+1)=x_knee_swing;
            zl_knee_plt(i+1)=z_knee_swing;
      

        end

        %time
        t_plt(i+1)=i*dt;


    end
    hold on
    %plot(xr_ankle_plt,zr_ankle_plt,'b.','LineWidth',2)
    %plot(x_hip_plt,z_hip_plt,'r.','LineWidth',2);
    %plot(xr_knee_plt,zr_knee_plt,'k.','LineWidth',2)
    %plot(t_plt,z_hip_plt)
    %plot(t_plt,x_hip_plt)
    %plot(t_plt,rad2deg(th1_plt));

    %plot(xl_ankle_plt,zl_ankle_plt,'kx','LineWidth',2)
    %plot(xl_knee_plt,zl_knee_plt,'g.','LineWidth',2)

    grid on
end


%plotting biped
%
%
for j=1:length(t_plt)

        clf
        hold on
        plot3(xr_ankle_plt,yr_ankle_plt,zr_ankle_plt,'m','LineWidth',1.5);
        %plot3(x_hip_plt,z_hip_plt,'r.','LineWidth',2);
        plot3(xr_knee_plt,yr_knee_plt,zr_knee_plt,'k','LineWidth',1.5);
        grid on
    
    
        %right leg
        plt_leg_rx(1)=x_hip_plt(j);
        plt_leg_rx(2)=xr_knee_plt(j);
        plt_leg_rx(3)=xr_ankle_plt(j);
    
        plt_leg_rz(1)=z_hip_plt(j);
        plt_leg_rz(2)=zr_knee_plt(j);
        plt_leg_rz(3)=zr_ankle_plt(j);

        %right foot
        plt_foot_rx(2)=xr_ankle_plt(j);
        plt_foot_rx(1)=plt_foot_rx(2)+0.06;
        plt_foot_rx(3)=plt_foot_rx(2)-0.02;

        plt_foot_rz=zr_ankle_plt(j)*ones(3,1);
        


        %left leg
        plt_leg_lx(1)=xl_ankle_plt(j);
        plt_leg_lx(2)=xl_knee_plt(j);
        plt_leg_lx(3)=x_hip_plt(j);%
    
        plt_leg_lz(1)=zl_ankle_plt(j);
        plt_leg_lz(2)=zl_knee_plt(j);
        plt_leg_lz(3)=z_hip_plt(j);

        %left foot
        plt_foot_lx(2)=xl_ankle_plt(j);
        plt_foot_lx(1)=plt_foot_lx(2)+0.06;
        plt_foot_lx(3)=plt_foot_lx(2)-0.02;

        plt_foot_lz=zl_ankle_plt(j)*ones(3,1);

        %hip
        hip_x(1)=x_hip_plt(j);
        hip_x(2)=x_hip_plt(j);

        hip_y(1)=0.08;
        hip_y(2)=-0.08;

        hip_z(1)=z_hip_plt(j);
        hip_z(2)=z_hip_plt(j);
    
    
    
    %plotting legs
        plot3(plt_leg_rx,plt_leg_ry,plt_leg_rz,'b','LineWidth',1.5)
        plot3(plt_leg_lx,plt_leg_ly,plt_leg_lz,'r','LineWidth',2)

        %plottig foot
        plot3(plt_foot_rx,plt_foot_ry,plt_foot_rz,'k','LineWidth',2)
        plot3(plt_foot_lx,plt_foot_ly,plt_foot_lz,'k','LineWidth',2)
        
        %plottin hip
        plot3(hip_x,hip_y,hip_z,'k','LineWidth',8)


        xlim([0,n*step_length/2+1]);
        ylim([-0.5,0.5]);
        zlim([0,0.8]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view([0,-1,0]);
        %view([1,-1,1]);
        %view(14.7,23.5); %(4.2,[21,40])
    
    
        drawnow
        %animation(j)=getframe;
        

 end
%}
%{
mywriter=VideoWriter('biped_3d_10cm_0Pause','MPEG-4');
mywriter.FrameRate=50;

open(mywriter);
writeVideo(mywriter,animation);
close(mywriter)
%}


    %support_x=ones(2,1);
    %support_z=ones(2,1);

%{
f=figure;
hold on
grid on
for k=0:step_time/dt

support_x(1)=x_ankle_i+step_length/2;
support_x(2)=x_hip_plt(k+1);

support_z(1)=0;
support_z(2)=z_hip_plt(k+1);

plot(support_x,support_z,'g','LineWidth',1.5);




end

%}
%plot(x_hip_plt,z_hip_plt,'r','LineWidth',2);
%xlim([0,1])
