%% Place for the project code
%% Parameters required to define the manipulators

I1=10;  I2 = 10; m1=5; r1=.5; m2=5; r2=.5; l1=1; l2=1;
g=9.8;

% we compute the parameters in the dynamic model
a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
b = m2*l1*r2;
d = I2+ m2*r2^2;

symx= sym('X',[4,1]); 

M = [a+2*b*cos(symx(2)), d+b*cos(symx(2));
    d+b*cos(symx(2)), d];
C = [-b*sin(symx(2))*symx(4), -b*sin(symx(2))*(symx(3)+symx(4)); b*sin(symx(2))*symx(3),0];
G = [m1*g*r1*cos(symx(1))+m2*g*(l1*cos(symx(1))+r2*cos(symx(1)+symx(2)));
    m2*g*r2*cos(symx(1)+symx(2))];

X = sym('X',[4,1]);
M = subs(M, symx, X);
C = subs(C, symx, X);
G = subs(G, symx, X);
invM = inv(M);

%% Defining the initial and final conditions

   % Considering there are four robots 
   % state space eqns given as [ q_1; q_1d; q_2; q_2d]

r_1 = [ 0.3; 0; 0.1; 1];
r_2 = [0.2; 1; 0.2; 0];     %random initial conditions
r_3 = [ -0.1; 0; 0.5; 1];
%r_4 = [ -0.2; 1; 0.4; 0.5];


r_1d = [ 0.5; 2; 0.4; 1];
r_2d = [ 0.5; 2; 0.4; 1];  %desired conditions
r_3d = [ 0.5; 2; 0.4; 1];
%r_4d = [ 0.5; 2; 0.4; 1];

to = 0; tf = 10;

k_1 = [1 0 0 0; 0 2 0 0; 0 0 1 0; 0 0 0 2];  %feedback gain k1
k_2 = [2 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 1];  %coupling gain k2
%% Synchronization Control Law

Tau = M*(v_1-v_1d) + C*(r_1-r_1d) + G + ...  
      k_1*(r_1-r_1d) - k_2*(r_3-r_3d) - k_2*(r_2-r_2d)
  
