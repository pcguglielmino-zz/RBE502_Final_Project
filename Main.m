%% Place for the project code
clc
close all
clear
%% Parameters required to define the manipulators

I1=10; I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

param = [m1, m2, I1, I2, l1, l2, r1, r2, g];

p = 4; % Number of robots

%% Defining the initial and final conditions
% Naming convention:
% qi_n, i is the robot number, 1 to p. n is the joint number, 1 or 2.
% state space eqns given as [ qi_1; dqi_1; qi_2; dqi_2]

q1 = [ 0.3; 0; 0.1; 1];
q2 = [0.2; 1; 0.2; 0];     %random initial conditions position 
q3 = [ -0.1; 0; 0.5; 1];
q4 = [0.1; 0.2; 0.1; 0.5];

q(:,:,1) = q1;
q(:,:,2) = q2;
q(:,:,3) = q3;
q(:,:,4) = q4;

% Generate trajectory
tf = 10;

% q = [q_1, v_1, q_2, v_2]
q0 = [0.3, 0, 0.3, 0];
qf = [1, 0, 0.5, 0];

% Trajectory matrices for joints
% generate_a_matrix(t0, tf, q0, v0, acc0, qf, vf, accf)
a1 = generate_a_matrix(0, tf, q0(1), q0(2), 0, qf(1), qf(2), 0);
a2 = generate_a_matrix(0, tf, q0(3), q0(4), 0, qf(3), qf(4), 0);


%% Simulate robots

x0 = q;

% options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x) odeSynchronization(t, x, param, a1, a2, p), [0 tf], x0);

%% Plotting the results
syms t
vec_t = [1; t; t^2; t^3; t^4; t^5]; % quintic polynomials
q_d = [a1'*vec_t; a2'*vec_t];

figure('Name','Theta_1 following a quintic trajectory');
plot(T, X(:,1), 'r-', T, X(:,5), 'g-', T, X(:,9), 'b-', T, X(:,13), 'y-');
hold on
fplot(q_d(1),[0 tf],'k--o');
grid on
legend({'State trajectory: Robot 1', 'State trajectory: Robot 2', 'State trajectory: Robot 3', 'State trajectory: Robot 4', 'Desired state trajectory'},'Location','northwest')

figure('Name','Theta_2 following a quintic trajectory');
plot(T, X(:,3), 'r-', T, X(:,7), 'g-', T, X(:,11), 'b-', T, X(:,15), 'y-');
hold on
fplot(q_d(2),[0 tf],'k--o');
grid on
legend({'State trajectory: Robot 1', 'State trajectory: Robot 2', 'State trajectory: Robot 3', 'State trajectory: Robot 4', 'Desired state trajectory'},'Location','northwest')


%% Matrix variables

V = 5 * eye(2);
K1 = 8 * eye(2);
K2 = 2 * eye(2);
zero = zeros(2,2);


L = [K1, -K2, zero, -K2;
    -K2, K1, -K2, zero;
    zero, -K2, K1, -K2;
    -K2, zero, -K2, K1];

one = 1/sqrt(p)*[eye(2),eye(2),eye(2),eye(2)]';

D1 = one'*L*one;



%% Functions to generate trajectory matrix

function a = generate_a_matrix(t0, tf, q0, v0, acc0, qf, vf, accf)
    t_matrix = [1, t0, t0^2,   t0^3,    t0^4,    t0^5;
                0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4;
                0,  0,    2,   6*t0, 12*t0^2, 20*t0^3;
                1, tf, tf^2,   tf^3,    tf^4,    tf^5;
                0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4;
                0,  0,    2,   6*tf, 12*tf^2, 20*tf^3];
    initial_matrix = [q0;v0;acc0;qf;vf;accf];
    a = t_matrix\initial_matrix;
end



























