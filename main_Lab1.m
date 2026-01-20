%Contributors: Hunter Liu, Parker Himes, Jessa Wanninger, Andrew Yates
%Course number: ASEN 3801
%File name: main_Lab1.m
%Created: 1/13/26
clear
clc
close all
function diff_eqs = diff_funcs(t,s)
 % s(1) = w;
 % s(2) = x;
 % s(3) = y;
 % s(4) = z;
 % Differential equations defined as array of variable s
 diff_eqs = [-9*s(1) + s(3); 4*s(1)*s(2)*s(3) - s(2)^2; 2*s(1)-s(2)-2*s(4);s(2)*s(3)-s(3)^2-3*s(4)^3];
end

% Inputs: 
%
% Outputs: 
%
% Methodology: 

%%Problem 1A
tol = 1e-8; %Tolerance for Problem 1A
opt = odeset('RelTol',tol,'AbsTol',tol); %Setting relative and absolute tolerances
time_span = [0 20];
initial_conds = [1 2 3 4]; %Initial conditions for w, x, y, z
%ODE45 function that solves each equation and outputs a matrix for s 1-4
[t,s] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);

% Creating a subplotted figure for each of the ODE solutions vs time
figure();
sgtitle('Problem 1A: Solution of ODEs vs Time')
subplot(4, 1, 1);
plot(t, s(:, 1));
xlabel('Time (n.d.)');
ylabel('w (n.d.)');
title('w vs Time');
subplot(4, 1, 2);
plot(t, s(:, 2));
xlabel('Time (n.d.)');
ylabel('x (n.d.)');
title('x vs Time');
subplot(4, 1, 3);
plot(t, s(:, 3));
xlabel('Time (n.d.)');
ylabel('y (n.d.)');
title('y vs Time');
subplot(4, 1, 4);
plot(t, s(:, 4));
xlabel('Time (n.d.)');
ylabel('z (n.d.)');
title('z vs Time');

fig=gcf;
exportgraphics(fig, 'Problem1A_ODESol.png');
%% Problem 1b
rel_tol = 1e-12; %Relative tolerance for difference calculations
opt = odeset('RelTol',rel_tol,'AbsTol',rel_tol); 
[t,s_ref] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);

tol = [1e-2 1e-4 1e-6 1e-8 1e-10]; %Array of tolerances to test
tol_table = zeros(width(s),length(tol));
%Nested for loop that calculates solutions for each tolerance, finds the
%absolute difference of the solutions and the defined relative tolerance
%and then inputs them into a table accordingly
for i = 1:length(tol)
    opt = odeset('RelTol',tol(i),'AbsTol',tol(i));
    [t,s_b] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);
   for j = 1:width(s)
   tol_table(j,i) = abs(s_b(end,j)-s_ref(end,j));
   end
end

%% Problem 2
Boulder_altitude = 1600; % meters
dt = -10; % Difference in temperature from standard Boulder Temp (since its winter time)
const = constfunc(Boulder_altitude,dt);

win_vel = [0,0,0]';
initial_position = [0,0,0]';
initial_velo = [0,20,-20]'; % Velocity in the NED frame, [north,east,down], so upward velocity is negative
x_0 = [initial_position;initial_velo];
% Create the options structure referencing the event function for the event
opts = odeset('RelTol',1e-8,'Abstol',1e-8,'Events',@GroundEvent);

% Solving the system of equations
span_t = [0, 100]; % Time in seconds, went large time since solver will stop at event

[t,x] = ode45(@(t,x)objectEOM(t,x,win_vel,const),span_t,x_0,opts);

% Creating a structure to organize state variables
state.c.p_n = x(:,1); % c refers to problem 2c
state.c.p_e = x(:,2);
state.c.p_d = x(:,3);
state.c.v_n = x(:,4);
state.c.v_e = x(:,5);
state.c.v_d = x(:,6);
state.c.alt = -state.c.p_d; % flipping down direction to be negative i.e. altitude

%% Plots
figure();
plot3(state.c.p_e,state.c.p_n,state.c.alt);









function xdot = objectEOM(t,x,win_vel,const)
% Declaring variables from the statevector x
p_n = x(1); % Position of the sphere in the north direction
p_e = x(2); % Position of the sphere in the east direction
p_d = x(3); % Position of the sphere in the down direction
v_n = x(4); % Velocity of the sphere in the north direction
v_e = x(5); % Velocity of the sphere in the east direction
v_d = x(6); % Velocity of the sphere in the down direction

velo = [v_n,v_e,v_d]';
air_rel_velo = velo-win_vel; % Air Relative Velocity Vector
air_speed = norm(air_rel_velo); % Magnitude of Air Relative Velocity Vector
unit_air_rel_velo = air_rel_velo ./ air_speed;


    


% Drag Force
D = 0.5 * const.rho * air_speed .^ 2 * const.A * const.Cd;

% Drag Force direction
% If statement is because of division by zero


if air_rel_velo < 0.00001
    f_D = [ 0; 0; 0];
else
    f_D = -D .* unit_air_rel_velo;
end

% Gravity
f_g = [0; 0; const.m * const.g]; % Note positive z direction since N-E-D frame

% Total forces and derivative state vector

accel = (f_g + f_D) / const.m;

p_dot = velo;
v_dot = accel;
xdot = [p_dot; v_dot];
end


function [value, isterminal, direction] = GroundEvent(t,x)
% This function is used to trigger an event in the ode45 function, it is
% passed into the ode45 function as an input. So when p_d (whitch is the
% position of the sphere in the down direction) goes from negative to
% positive (which in the N-E-D frame this means it goes from in the air to
% the ground).
p_d = x(3);


if t < 1e-6
    value = 1; % Keeps initial position from being 0 and intially triggering the event
else
    value = p_d;
end

isterminal = 1;    % Stop integration at event (if 0 then just record and keep integrating)
direction  = +1;   % Trigger only on negative -> positive crossing
end




function const = constfunc(altitude,dt)

const.rho = stdatmo(altitude,dt); % Air Density in m^3/kg
const.Cd = 0.6; % Given from problem
const.diameter = 2 / 1000; % meters
const.A = pi * (const.diameter/2)^2; % Area in meters
const.m = 50 / 1000; % Mass in kilograms
const.g = 9.81; % Acceleration due to gravity (m/s^2)








end