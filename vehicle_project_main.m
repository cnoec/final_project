clear all
close all
clc

path        =       pwd;
addpath('functions');
addpath('model');
addpath('mat_data');
addpath('steps');
addpath('functions\unc_optimization');
addpath('functions\con_optimization');
addpath('functions\track');
addpath('functions\cost_function');

%% Initialization

run('initial_guess_setting');

fprintf('\n *********************************************************************** \n');
fprintf('\n Initialization is complete! \n');

%% Step 1 Optimization

run('step1_main.m');
fprintf('\n *********************************************************************** \n');
fprintf('\n Step 1 is complete! \n');

%% Step 1 Plots

figure('Name','Step1_results')
subplot 211
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black');grid on;axis equal;
    title('Initial Guess vs Optimal Solution Found at Step1');
hold on
plot(xi_in_step1(1,:), xi_in_step1(2,:), '.blue');
hold off

subplot 212
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on, axis equal
hold on
plot(xi_step1(1,:), xi_step1(2,:), '.r');
hold off

%% Step 2 Optimization

run('step2_main.m');
fprintf('\n *********************************************************************** \n');
fprintf('\n Step 2 is complete! \n');

%% Step 2 Plots

figure('Name','Step2_results')
subplot 211
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black');grid on;axis equal;
    title('Optimal Solution Found at Step1 vs Optimal Solution Found at Step2');
hold on
plot(xi_in_step2(1,:), xi_in_step2(2,:), '.blue');
hold off

subplot 212
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on, axis equal
hold on
plot(xi_step2(1,:), xi_step2(2,:), '.r');
hold off

%% Step 3 Optimization

run('step3_main.m');

fprintf('\n *********************************************************************** \n');
fprintf('\n Step 3 is complete! \n');
fprintf('\n *********************************************************************** \n');


%% Step 3 Plots

figure('Name','Step3_results')
subplot 211
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black');grid on;axis equal;
    title('Optimal Solution Found at Step2 vs Optimal Solution');
hold on
plot(xi_in_step3(1,:), xi_in_step3(2,:), '.blue');
hold off

subplot 212
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
    outerBoundary(:,2),'black'),grid on, axis equal
hold on
plot(xi_step3(1,:), xi_step3(2,:), '.r');
hold off

figure('Name','Optimal_Solution')
plot(innerBoundary(:,1),innerBoundary(:,2),'black',outerBoundary(:,1),...
     outerBoundary(:,2),'black');grid on;xlabel('m');ylabel('m');
axis equal
hold on
plot(xi_step3(1,:),xi_step3(2,:))

figure('Name','Torque_and_speed')
subplot 211
plot(t_vec(1:end-1),torque);grid;title('Torque');xlabel('time [s]');ylabel('Torque [Nm]');
subplot 212
plot(t_vec,xi_step3(3,:));grid;title('Speed');xlabel('time [s]');ylabel('Speed [m/s]');

savefig;

%% Analysis

%edo

%%