%Contributors:
%Course number: ASEN 3801
%File name: main.m
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

win_vel = [0,0,0]'; % N-E-D (1-2-3)
initial_position = [0,0,0]';
initial_velo = [0,20,-20]'; % Velocity in the NED frame, [north,east,down], so upward velocity is negative
x_0 = [initial_position;initial_velo];
% Create the options structure referencing the event function for the event
opts = odeset('RelTol',1e-8,'Abstol',1e-8,'Events',@GroundEvent);

% Solving the system of equations
span_t = [0, 100]; % Time in seconds,large time but, solver will stop at event

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
% Trajectory Plot
figure;
plot3(state.c.p_n,state.c.p_e,state.c.alt,'LineWidth',2);
title('Trajectory');
xlabel('North');
ylabel('East');
zlabel('Altitude');
zlim([0 30]);
grid on;


%% Part D
% Implementing non zero wind speeds into the simulation, note we are only
% changing/adding wind in the north direction.
north_winds = -60:1:60; % Average wind speed in boulder is ~4.5 m/s
wind_vecs = zeros([3,length(north_winds)]);

for i = 1 : length(north_winds)
   wind_vecs(1,i) = north_winds(i); 

end
% Structures to store answers after iterations
times_d = struct();
states_d = struct();
% Function calls
for i = 1:length(wind_vecs)
% Note these are the same calls as part c, the only change in the wind
% vector
Boulder_altitude = 1600; % meters
dt = -10; % Difference in temperature from standard Boulder Temp (since its winter time)
const = constfunc(Boulder_altitude,dt);


initial_position = [0,0,0]'; % Same as part c
initial_velo = [0,20,-20]'; % Same as pa
x_0 = [initial_position;initial_velo];
% Create the options structure referencing the event function for the event
opts = odeset('RelTol',1e-8,'Abstol',1e-8,'Events',@GroundEvent);

% Solving the system of equations
span_t = [0, 5]; % Time in seconds,large time but, solver will stop at event

[t_run,x_run] = ode45(@(t,x)objectEOM(t,x,wind_vecs(:,i),const),span_t,x_0,opts);
w = north_winds(i); % Creating naming scheme for the fields in the structures
if w < 0
    name = sprintf('wN_m%d',abs(w)); % _m represents negative values (minus)
else
    name = sprintf('wN_%d',w);
end

times_d.(name) = t_run;
states_d.(name) = x_run;






end

% extracting values 
names = fieldnames(states_d);
north_pos_d = struct();
final_north_pos_d = zeros(numel(names),1);
final_east_pos_d = final_north_pos_d;
tot_distance = final_north_pos_d;


for i = 1 : numel(names)
    name = names{i}; % Converting names to cell array so it can be indexed
    north_pos_d.(name) = states_d.(name)(:,1);
    final_north_pos_d(i,1) = north_pos_d.(name)(end);
    east_pos_d.(name) = states_d.(name)(:,2); % This line and below
    final_east_pos_d(i,1) = east_pos_d.(name)(end);
    tot_distance_d(i,1) = sqrt(final_east_pos_d(i,1).^2  + final_north_pos_d(i,1).^2);
    

 
end

% Plotting North Deflection
figure;
plot(wind_vecs(1,:),final_north_pos_d,'LineWidth',2);
title('North Wind Deflection');
xlabel('Wind Speed (m/s)');
ylabel('North Deflection (m)');
grid on;

% Part 2 Question d

% Since the object starts and ends at the same altitude (z) only east and
% north final locations are considered
figure;
plot(wind_vecs(1,:),tot_distance_d,'LineWidth',2);
xlabel('Wind Speed (m/s)');
ylabel('Total Distance (m)');
title('Total Distance vs Wind Speed');
grid on;

%% Question 2 Part e
% Adding an outer for loop that goes through different altitudes
% For these altitudes I will be using different popular cities with varying altitudes,
% New York, Los Angeles, Boulder, Flagstaff, and then also a higher altiude on
% the Grand Mesa in Grand Junction Colorado

alts = [ 0 % New Orleans
         600 % Michigan (Mount Arvon)
         1600 % Boulder
         2100 % Flagstaff
         3000 % Grand Mesa


    ];
% For these loop I am going to switch to arrays and not structures for
% simplicity in the nested loop. And I am only going to store the north and
% east positions






dt = -10; % temp offset (same for all runs)

%  Results: rows = altitudes, cols = winds
final_north_pos_e = zeros(length(alts), length(north_winds));
final_east_pos_e  = zeros(length(alts), length(north_winds));
tot_distance_e    = zeros(length(alts), length(north_winds));



% Initial conditions 
initial_position = [0;0;0];
initial_velo     = [0;20;-20];
x_0 = [initial_position; initial_velo];

opts   = odeset('RelTol',1e-8,'Abstol',1e-8,'Events',@GroundEvent);
span_t = [0 5];

for j = 1:length(alts)

    % Build constants for this altitude (density changes here)
    const = constfunc(alts(j), dt); % Keeping dt the same (winter time)

    for k = 1:length(north_winds)

        

        [t_run, x_run] = ode45(@(t,x)objectEOM(t,x,wind_vecs(:,k),const), span_t, x_0, opts);



        % Store landing location 
        pn_end_e = x_run(end,1);
        pe_end_e = x_run(end,2);

        final_north_pos_e(j, k) = pn_end_e;
        final_east_pos_e(j, k)  = pe_end_e;
        tot_distance_e(j, k)    = sqrt(final_east_pos_e(j,k).^2 + final_north_pos_e(j,k).^2);
    end
end

% Plots 


% Generating Labels for the Legend
legend_labels = cell(length(alts), 1);

for L = 1:length(alts)
    legend_labels{L} = sprintf('Altitude = %d m', alts(L));
end
figure;
hold on;
for M = 1 : length(alts)
 plot(north_winds,tot_distance_e(M,:),'LineWidth',1);
end
legend(legend_labels);
set(legend,...
    'Position',[0.440961662225135 0.78149484810666 0.152417659425368 0.132558139534884]);
xlabel('Wind Speeds (m/s)');
ylabel('Total Distance (m)');
title('Varying Altitudes: Landing Location vs Wind Speed');


grid on;
hold off;

% Second Plot
min_distance_e = min(tot_distance_e, [], 2);  % min over wind speeds

figure;
plot(alts,min_distance_e,'LineWidth',2);
xlabel('Altitude (m)');
ylabel('Minimum Distance (m)');
title('Minimum Landiing Distance vs Altitude');
grid on;

%% Part f

 masses_f = (10:10:250) / 1000; % Varying masses
 masses_f = sort(masses_f);
 
 KE_0  = 0.5 * const.m * norm(initial_velo)^2;
 % Initializing total distance vector 
 tot_distance_f = zeros(length(masses_f),length(wind_vecs));
 for n = 1:length(masses_f)
     const_f = const;
     const_f.m = masses_f(n); % Updating mass in constant structure

     % Kinetic Energy Section

     v_f_hat = initial_velo ./ norm(initial_velo); % Initial Direction
     v_f_mag = sqrt((2 * KE_0)/const_f.m); % Magnitude being updated with different masses
     v_f_init = v_f_mag * v_f_hat; % Initial velocity vector
     
     % New initial state vector
     x_0_f = [initial_position;v_f_init];

     % Now for Wind Speed
     for o = 1:length(wind_vecs)


         [t_run_f, x_run_f] = ode45(@(t,x)objectEOM(t,x,wind_vecs(:,o),const_f), span_t, x_0_f, opts);


         pn_final_f = x_run_f(end,1); % Final north position
         pe_final_f = x_run_f(end,2); % Final east position

         tot_distance_f(n,o) = sqrt ( pn_final_f.^2 + pe_final_f .^ 2);

     end
 end
% Plots 

% First plot will be difference in distance with no wind
wind_idx = round(0.5 * length(wind_vecs));
figure;
plot(masses_f * 1000,tot_distance_f(:,wind_idx),'LineWidth',2);
xlabel('Mass (g)');
ylabel('Distance Traveled (m)');
title('Distance Traveled vs Mass');
grid on;

% Second Plot w/ wind

    
 % Creating a more spread out mass_value vec
mass_f_plot_2 = [ 30 40 50 60 70 ];
legend_labels_f = cell(length(mass_f_plot_2), 1);
% Find row indices in masses_f corresponding to new plot masses
idx_mass = zeros(size(mass_f_plot_2));
for p = 1:length(mass_f_plot_2)
    [~, idx_mass(p)] = min(abs(masses_f*1000 - mass_f_plot_2(p)));
end



for q = 1:length(mass_f_plot_2)
    legend_labels_f{q} = sprintf('Mass = %d g', mass_f_plot_2(q));
end
figure;
hold on;
for r = 1 : length(mass_f_plot_2)
 plot(north_winds,tot_distance_f(idx_mass(r),:),'LineWidth',1);
end
xlabel('Wind Speed (m/s)');
ylabel('Total Distance Travelled (m)');
title('Varying Masses: Total Distance vs Mass')
legend(legend_labels_f);
set(legend,...
    'Position',[0.440961662225135 0.78149484810666 0.152417659425368 0.132558139534884]);








 



%% functions for Question 2
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


if air_speed < 0.00001
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
% passed into the ode45 function as an input. So when p_d (which is the
% position of the sphere in the down direction) goes from negative to
% positive (which in the N-E-D frame this means it goes from in the air to
% the ground) the event will trigger ode45 to stop.
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

const.rho = stdatmo(altitude,dt); % Air Density in kg / m^3
const.Cd = 0.6; % Given from problem
const.diameter = 2 / 100; % meters
const.A = pi * (const.diameter/2)^2; % Area in meters
const.m = 50 / 1000; % Mass in kilograms
const.g = 9.81; % Acceleration due to gravity (m/s^2)


end

