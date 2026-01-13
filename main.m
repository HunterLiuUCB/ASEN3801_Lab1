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
 diff_eqs = [-9*s(1) + s(3); 4*s(1)*s(2)*s(3) - s(2)^2; 2*s(1)-s(2)-2*s(4);s(2)*s(3)-s(3)^2-3*s(4)^3];
end

tol = 1e-8;
opt = odeset('RelTol',tol,'AbsTol',tol);
time_span = [0 20];
initial_conds = [1 2 3 4]; % Define initial conditions for w, x, y, z
[t,s] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);

figure();
subplot(4, 1, 1);
plot(t, s(:, 1));
xlabel('Time');
ylabel('w');
title('w vs Time');
grid on;

subplot(4, 1, 2);
plot(t, s(:, 2));
xlabel('Time');
ylabel('x');
title('x vs Time');
grid on;

subplot(4, 1, 3);
plot(t, s(:, 3));
xlabel('Time');
ylabel('y');
title('y vs Time');
grid on;

subplot(4, 1, 4);
plot(t, s(:, 4));
xlabel('Time');
ylabel('z');
title('z vs Time');
grid on;

%% Problem 1b
reltol = 1e-12;
opt = odeset('RelTol',reltol,'AbsTol',reltol);
[t,s_ref] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);
tol = [1e-2 1e-4 1e-6 1e-8 1e-10];
tol_table = zeros(width(s),length(tol));
for i = 1:length(tol)
    opt = odeset('RelTol',tol(i),'AbsTol',tol(i));
    [t,s_b] = ode45(@(t,s) diff_funcs(t,s), time_span, initial_conds,opt);
   for j = 1:width(s)
   tol_table(j,i) = abs(s_b(end,j)-s_ref(end,j));
   end
end