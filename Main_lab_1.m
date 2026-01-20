clear;
clc;

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