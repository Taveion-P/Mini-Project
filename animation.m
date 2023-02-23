% define data to animate
% this will move a rectangle with fixed width and length
% around to specified positions and orientations

%code can be run to either take input from robotScript:
time = Position.Time;
xposition = Position.Data(:,1);
yposition = Position.Data(:,2);
phi = Position.Data(:,3);

%Or can take inputs directly from the Arduino Program:
%{
time = Results(:,1);
xposition = Results(:,4);
yposition = Results(:,5);
phi = Results(:,6);
%}

% Define shape verticies first element is x position, second element is y
% position
r_width = .5;
r_length = 1;
v1 = [-r_length/2; -r_width/2];
v2 = [-r_length/2; r_width/2];
v3 = [r_length/2 ; r_width/2];
v4 = [r_length/2 ; -r_width/2];
figure
for i=1:length(time),
% move shape by moving verticies
% define rotation matrix
T = [cos(phi(i)) -sin(phi(i));sin(phi(i)) cos(phi(i))];
% define center position
pos = [xposition(i);yposition(i)];
% find position of current vertices: each position is multiplied by the rotation
%matrix, and added to the current position
v_c = T*[v1 v2 v3 v4]+pos*ones(1,4);
v = [-.5 -.5 0 0 -.25 -.25 .25 .25 0 0 .5 .5 0 0 -.25 -.25 .25 .25 0 0;
-.4 .4 .4 .45 .45 .55 .55 .45 .45 .4 .3 -.3 -.4 -.45 -.45 -.55 -.55 -.45 -.45 -.4];

v_c = T*v + pos;

% draw shape
fill(v_c(1,:),v_c(2,:),'y')
axis([-10 10 -10 10]) % set axis to have specified x and y limits
% (type 'help axis' for more info)
% make sure matlab draws the figure now
drawnow
% if not last drawing, wait
if i<length(time),
pause(time(i+1)-time(i))
end;
end;