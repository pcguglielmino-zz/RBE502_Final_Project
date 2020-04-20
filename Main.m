%% Place for the project code
clc
close all
clear
%% Parameters required to define the manipulators

I1=10; I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% % we compute the parameters in the dynamic model
% a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
% b = m2*l1*r2;
% d = I2+ m2*r2^2;

param = [m1, m2, I1, I2, l1, l2, r1, r2, g];

% symx= sym('X',[4,1]); 
% 
% M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
%     d+b*cos(symx(2)), d];
% C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
% G = [m1*g*r1*cos(symx(1))+m2*g*(l1*cos(symx(1))+r2*cos(symx(1)+symx(2)));
%     m2*g*r2*cos(symx(1)+symx(2))];
% 
% X = sym('X',[4,1]);
% M = subs(M, symx, X);
% C = subs(C, symx, X);
% G = subs(G, symx, X);
% invM = inv(M);

p = 3; % Number of robots

%% Defining the initial and final conditions
% Naming convention:
% qi_n, i is the robot number, 1 to p. n is the joint number, 1 or 2.
% state space eqns given as [ qi_1; dqi_1; qi_2; dqi_2]

q1 = [ 0.3; 0; 0.1; 1];
q2 = [0.2; 1; 0.2; 0];     %random initial conditions position 
q3 = [ -0.1; 0; 0.5; 1];

q(:,:,1) = q1;
q(:,:,2) = q2;
q(:,:,3) = q3;


% Generate trajectory
tf = 10;
q0 = [0.3, 0, 0.3, 0];
qf = [1, 0, 1, 0];

% Trajectory matrices for joints
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
plot(T, X(:,1), 'r-', T, X(:,3), 'g-', T, X(:,5), 'b-');
hold on
fplot(q_d(1),[0 tf],'k--o');
grid on
legend({'State trajectory: Robot 1', 'State trajectory: Robot 2', 'State trajectory: Robot 2', 'Desired state trajectory'},'Location','southwest')

figure('Name','Theta_2 following a quintic trajectory');
plot(T, X(:,2), 'r-', T, X(:,4), 'g-', T, X(:,6), 'b-');
hold on
fplot(q_d(2),[0 tf],'k--o');
grid on
legend({'State trajectory: Robot 1', 'State trajectory: Robot 2', 'State trajectory: Robot 3', 'Desired state trajectory'},'Location','southwest')


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

  
