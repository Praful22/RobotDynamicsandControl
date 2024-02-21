% Praful Sigdel
% AME 556: HW 2 Matlab Code for 1cii.
% 

function HW2_projectile1cii()
% simulation of a ball thrown in 2D space which is affected by drag force

% system states: X = [x;y;dx;dy];
% control input: u = F;
% v0 = [1;2] m/s 
% c = 0.5

clc; clear all; close all;
global params;

m = 0.5 ; g = 9.81 ; c = 0.5;
params.m=m;
params.g =g;
params.c =c;

x0=[0;0;1;2]; % x0 is the intial state of the system 

tspan=[0; 2]; % simulation time

[t,x]=ode45(@sys_dynamics,tspan,x0);

anim(t,x,1/24); % animate the system and save the simulation video

% recreate control inputs
 for i=1:length(t)
     u(:,i)=controller(t(i),x(i,:)');
 end
 
 % plot the simulation data
figure; 
plot(t,x); 
legend('x','y','dx','dy'); 
grid on;
xlabel('Time(s)');
ylabel('States');
title('System States');

figure; 
plot(t,u); 
grid on;
xlabel('Time(s)'); 
ylabel('control(u)'); 
legend('u'); title('control input');

end

function dx=sys_dynamics(t,x)
global params;

u=controller(t,x);

ds = x(3);

ddsx = -params.c/params.m * norm(x(3:4)) * x(3);

dy = x(4);

ddy = -params.g - params.c/params.m * norm(x(3:4)) * x(4);

dx = [ds;dy;ddsx;ddy];

end

function u=controller(t,x)

global params

u = 0; % you can put your controller here

end

function anim(t,x,ts)
[te,xe]=even_sample(t,x,1/ts);

figure(1);

axes1 = axes;

%save as a video
spwriter = VideoWriter('video_HW2_demo1cii.mp4','MPEG-4');
set(spwriter, 'FrameRate', 1/ts,'Quality',100);
open(spwriter);

fig1 = figure(1);

figure_x_limits = [-15 15];
figure_y_limits = [-15 15];

axes1 = axes;
set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
set(axes1,'Position',[0 0 1 1]);
set(axes1,'Color','w');
grid on;

for k = 1:length(te)
    drawone(axes1, xe(k,:)');
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
    drawnow;
    pause(ts);
    frame = getframe(gcf);
    writeVideo(spwriter, frame);
end

end

function [Et, Ex] = even_sample(t, x, Fs)
%       CONVERTS A RANDOMLY SAMPLED SIGNAL SET INTO AN EVENLY SAMPLED
%       SIGNAL SET (by interpolation)
% Obtain the process related parameters
N = size(x, 2);    % number of signals to be interpolated
M = size(t, 1);    % Number of samples provided
t0 = t(1,1);       % Initial time
tf = t(M,1);       % Final time
EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with
                   % the specified sampling frequency
Et = linspace(t0, tf, EM)';

% Using linear interpolation (used to be cubic spline interpolation)
% and re-sample each signal to obtain the evenly sampled forms
for s = 1:N
  Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1)); 
end
end

function drawone(parent, x)
% draw the robot at the current frame
global params
tem = get(parent,'Children');
delete(tem);
s = x(1);
y = x(2); 
p = [s;
      y] ;
     
plot(p(1),p(2),'ro');
grid on;
end