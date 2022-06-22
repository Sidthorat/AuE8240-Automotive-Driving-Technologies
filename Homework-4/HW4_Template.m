% HW4: Pure Pursuit control for lane switching
clear all;
yd = 2;  		    % desired path
x0 = 0;  		    % initial x
y0 = 1;  		    % initial y = 1,2
theta0 = 0;  		% initial theta
N = 1000;  		    % calculate 1000 steps
x = zeros(1,N); 	% creat x vecotor with all 0 elements
y = zeros(1,N); 	% creat y vecotor with all 0 elements
theta = zeros(1,N); % creat theta vecotor with all 0 elements
phi = zeros(1,N); 	% creat phi vecotor with all 0 elements
x(1) = x0; 		    % record the first x value
y(1) = y0; 		    % record the first y value
theta(1) = theta0; 	% record the first theta value
T = 0.01 ;    		% sampling time
v = 1.2 ;    		% speed
k = 1.1 ;    		% control gain (need to tune this)
L = 1.0;    		% vehicle baseline

for i=1:1:N-1
    ld = k*v;                             		 % calculate lookaead distance
    alpha = asin((yd-y(i))/ld)-theta(i);         % calculate alpha angle
    phi(i) = atan(2*L*sin(alpha)/ld);  		     % calculate steering angle phi
    x(i+1) = x(i)+v*cos(theta(i))*T;             % calculate next x based on current control input
    y(i+1) = y(i)+v*sin(theta(i))*T;             % calculate next y based on current control input
    theta(i+1) = theta(i) + (v*tan(phi(i))/L)*T; % calculate next theta based on current control input
end

figure(1)
subplot(3,1,1)
plot(y-yd);    		% plot lane switching errors
grid on;
subplot(3,1,2)
plot(phi);     		% plot steering angles
subplot(3,1,3)
plot(theta)    		% plot vehicle orientations
figure(2)
plot(x,y);
max(y-yd)           % overshoot
plot(y-yd)          %tracking error after 6 seconds
ylim([-0.02 0.02])
xlim([550 1000])
