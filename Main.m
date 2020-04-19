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

% number of robots considered as 3 : p = 3

%% Defining the initial and final conditions

   % Considering there are three robots 
   % state space eqns given as [ q_1; q_1d; q_2; q_2d]

r_1 = [ 0.3; 0; 0.1; 1];
r_2 = [0.2; 1; 0.2; 0];     %random initial conditions position 
r_3 = [ -0.1; 0; 0.5; 1];


r_1d = [ 0.5; 2; 0.4; 1];
r_2d = [ 0.5; 2; 0.4; 1];  %desired conditions position 
r_3d = [ 0.5; 2; 0.4; 1];

A = [1 0 0; 0 1 0; 0 0 1]  %positive diagonal matrix

r_1(1,1) = r_1d(3,1) - A*(qi_1 - r_1d(1,1)) 

to = 0; tf = 10;

k_1 = [3 0 0 ; 0 4 0 ; 0 0 3];  %feedback gain k1
k_2 = [1 0 0 ; 0 1 0 ; 0 0 1];  %coupling gain k2

%% Position Error

double qi_1 
double qi_2
double qi_3

A = [1 0 0 ; 0 1 0 ; 0 0 1];

r_1(1,1) = r_1d(3,1) - A*(qi_1 - r_1d(1,1)) 
r_2(1,1) = r_2d(3,1) - A*(qi_2 - r_2d(1,1))    % qir dot = q d dot - A( qi - qd)
r_3(1,1) = r_3d(3,1) - A*(qi_3 - r_3d(1,1)) 

%% Synchronization Error

s_1 = qi_1 - r_1(1,1)
s_2 = qi_2 - r_2(1,1)         % si = qi dot - qir
s_3 = qi_3 - r_3(1,1)
%% Closedd Loop Dynamics

double s_1d
doubel s_2d
double s_3d


Tau_1 = M*s_1d + C*s_1 + k_1*s_1d - k_2*(s_2+s_3) 
Tau_2 = M*s_2d + C*s_2 + k_1*s_2d - k_2*(s_1+s_3)  
Tau_3 = M*s_3d + C*s_3+ k_1*s_3d - k_2*(s_1+s_2) 

  
