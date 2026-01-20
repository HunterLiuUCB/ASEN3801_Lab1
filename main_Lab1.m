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