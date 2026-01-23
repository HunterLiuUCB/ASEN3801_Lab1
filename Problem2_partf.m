clear;
clc;
close all;

%% Set up
%Constants
Boulder_altitude = 1600; % meters
dt = -10; % Difference in temperature from standard Boulder Temp (since its winter time)
const = constfunc(Boulder_altitude,dt); %Constant function with mass = 0.05kg
constf = constffunc(Boulder_altitude, dt); %Constant function with varying mass values

%initial state vector
initial_position = [0,0,0]';
initial_velo = [0,20,-20]'; % Velocity in the NED frame, [north,east,down], so upward velocity is negative
x_0 = [initial_position;initial_velo];
span_t = [0, 100]; % Time in seconds, went large time since solver will stop at event  

% Create the options structure referencing the event function for the event
opts = odeset('RelTol',1e-8,'Abstol',1e-8,'Events',@GroundEvent);

%Part f- specific
%Kinetic Energy calculation using initial state vector velocities
kinetic_0 = 0.5 * const.m .* (initial_velo.^2);



%% Effect of mass on distance

%wind velocity vector variation
win_vel = [0, 0, 0;
           40, 0, 0;
           0, 40, 0;
           0, 0, 40;
           40, 40, 0;
           0, 40, 40;
           40, 0, 40;
           40, 40, 40
           -40, -40, -40];

for i = 1:length(win_vel)
     figure(i);

    for j = 1:10
       
    hold on;
    % win_vel(i, :) = win_vel(i, :)'; % 0 wind speed
    
    %Get constants out of structure for EOM function
    conf = [constf.rho(j), constf.Cd(j), constf.diameter(j), constf.A(j), constf.m(j), constf.g(j)];

    %Calculate velocity for each specific mass value to keep KE const
    vf_0 = sqrt((2 .* kinetic_0) ./ conf(5)) .* sign(initial_velo);
    
    %Part f initial state vector 
    xf_0 = [initial_position; vf_0];

    %EOM function
    [t_f, x_f] = ode45(@(t,x)objectEOM(t,x,win_vel(i, :)',conf),span_t,xf_0,opts);

    %state values
    state.f1.p_n = x_f(:,1); % f1 refers to problem 2, varying mass
    state.f1.p_e = x_f(:,2);
    state.f1.p_d = x_f(:,3);
    state.f1.v_n = x_f(:,4);
    state.f1.v_e = x_f(:,5);
    state.f1.v_d = x_f(:,6);
    state.f1.alt = -state.f1.p_d; % flipping down direction to be negative i.e. altitude

    %Get distance values to calculate distance from origin later
    values(j).distance = [state.f1.p_e, state.f1.p_n, state.f1.alt];
    land_distance1(j) = sqrt((state.f1.p_e(end)).^2 + (state.f1.p_n(end)).^2);

    %Save mass value to variable for legend
    massval(j) = conf(5);
    win_names(i) = num2str(win_vel(i, 1))+ " m/s N, " + num2str(win_vel(i, 2)) + " m/s E, "+ num2str(win_vel(i, 3)) + " m/s D";

    %Plot
    part_f_1 = plot3(state.f1.p_e, state.f1.p_n, state.f1.alt);
    hold off;
    end
    
    %Analyze and compare distance from origin in table
    massnames =  split(num2str(massval)) + " kg"; %Converting to string
    mass_table(i).data = table(land_distance1', massnames);

    view(3);
    grid on;
    legend(massnames, 'Location','northwest', 'Orientation','horizontal', 'NumColumns',2);
    xlabel("North (x) (m)");
    ylabel("East (y) (m)");
    ylim([-0.5 0.5]);
    zlabel("Altitude (z) (m)");
    zlim([0 25]);
    title("Effect of Mass on Distance with Wind = " + win_names(i));
    hold off;

    % print(figure(i),'-r300','-dpng');
end

%% Functions

%EOM function
function xdot = objectEOM(t,x,win_vel,const)

% Declaring variables from the statevector x
p_n = x(1); % Position of the sphere in the north direction
p_e = x(2); % Position of the sphere in the east direction
p_d = x(3); % Position of the sphere in the down direction
v_n = x(4); % Velocity of the sphere in the north direction
v_e = x(5); % Velocity of the sphere in the east direction
v_d = x(6); % Velocity of the sphere in the down direction


    rho = const(1);
    Cd = const(2);
    diameter = const(3);
    A = const(4);
    m = const(5);
    g = const(6);


velo = [v_n,v_e,v_d]';
air_rel_velo = velo - win_vel; % Air Relative Velocity Vector
air_speed = norm(air_rel_velo); % Magnitude of Air Relative Velocity Vector
unit_air_rel_velo = air_rel_velo ./ air_speed;

% Drag Force
D = 0.5 * rho * air_speed .^ 2 * A * Cd;

% Drag Force direction
% If statement is because of division by zero


if air_rel_velo < 0.00001
    f_D = [ 0; 0; 0];
else
    f_D = -D .* unit_air_rel_velo;
end

% Gravity
f_g = [0; 0; m * g]; % Note positive z direction since N-E-D frame

% Total forces and derivative state vector

accel = (f_g + f_D) / m;

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

%Constant functions
function const = constfunc(altitude, dt)

const.rho = stdatmo(altitude,dt); % Air Density in m^3/kg
const.Cd = 0.6; % Given from problem
const.diameter = 2 / 1000; % meters
const.A = pi * (const.diameter/2)^2; % Area in meters
const.m = 50 / 1000; % Mass in kilograms
const.g = 9.81; % Acceleration due to gravity (m/s^2)

end

function constf = constffunc(altitude, dt)
    len = 11;
    for i = 1:len
        constf.rho = stdatmo(altitude,dt) * ones(1, len); % Air Density in m^3/kg
        constf.Cd = 0.6 * ones(1, len); % Given from problem
        constf.diameter = 2 / 1000 *ones(1, len); % meters
        constf.A = pi .* (constf.diameter/2).^2 .* ones(1, len); % Area in meters
        constf.m = linspace(.05, 1.05, len); % Mass in kilograms
        constf.g = 9.81 .* ones(1, len); % Acceleration due to gravity (m/s^2)
    end
end