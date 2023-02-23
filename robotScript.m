%MOBILE_ROBOTSIM Script to run a simulation of a two wheeled car, and plot the results
%
%
% This script requires the file mobile_robot.xls to be in the same folder
%
% The diameter of the car is set in the variable d
%%
%% Define parameters and inputs
%%

% Width of the car
b=.1;


%code can be run to either take input from default values as follows:
%{
% Sampling time in seconds
t = 0:.01:10;
% velocityLeft is the velocity of the left wheel at the sampling times
velocityLeft=[];
velocityLeft(find(t>=0 & t<3)) = .1;
velocityLeft(find(t>=3 & t<7)) = .2;
velocityLeft(find(t>=7 & t<=10)) = 0;
% velocityRight is the velocity of the right wheel at the sampling times
velocityRight=[];
velocityRight(find(t>=0 & t<3)) = 0;
velocityRight(find(t>=3 & t<7)) = .2;
velocityRight(find(t>=7 & t<=10)) = .1;
%}
%Or code can be run by taking in input from the Results array variable
%which can be extracted from the Arduino:

t = 10^-3.*Results(:,1);
velocityLeft = Results(:,2);
velocityRight = Results(:,3);



% redefine these as timeseries objects
velocityLeft = timeseries(velocityLeft,t);
velocityLeft.Name = 'Left Wheel Velocity';
velocityLeft.TimeInfo.Units = 'Seconds';
velocityRight = timeseries(velocityRight,t);
velocityRight.Name = 'Right Wheel Velocity';
velocityRight.TimeInfo.Units = 'Seconds';


%%
%% Run Simulation
%%
% simulate simulink file named mobile_robot.slx
out=sim('mobile_robot');
% get the to workspace block variable Position from the results
Position = out.Position;
% open the simulink block diagram so that publish will include it
% the block diagram should be closed when the script is published in order
% for it to appear
open_system('mobile_robot')
%%
%% Plot Results
%%
%%
% In the following figure, the input signals are plotted as a time series
%
figure(1)
clf
subplot(2,1,1)
plot(velocityLeft)
subplot(2,1,2)
plot(velocityRight)
%%
% In the following figure, the position of the robot is plotted as a time series
%
figure(2)
clf
plot(Position)
legend('x','y','phi')

%%
% In the following figure, the robot position is plotted as x vs y
%
figure(3)
clf
plot(Position.Data(:,1),Position.Data(:,2))
xlabel('x position (m)')
ylabel('y position (m)')
axis equal