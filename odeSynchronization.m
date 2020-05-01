function [dx] = odeSynchronization(t, x, param, a1, a2, p)
%odeSynchronization Function to control the synchronizing of 2 dof
%manipulators

    % x is an in the form of 
    % [[q1_1, dq1_1, q1_2, dq1_2, q2_1, dq2_1, q2_2, dq2_2],...];

    % convert param to system values
    m1 = param(1);
    m2 = param(2);
    I1 = param(3);
    I2 = param(4);
    l1 = param(5);
    l2 = param(6);
    r1 = param(7);
    r2 = param(8);
    g = param(9);

    a = I1+I2+m1*r1^2+ m2*(l1^2+ r2^2);
    b = m2*l1*r2;
    d = I2+ m2*r2^2;
    
    % Generate trajectory
    vec_t = [1; t; t^2; t^3; t^4; t^5]; % quintic polynomials
    

    % compute the velocity and acceleration for both theta 1 and theta2.
    a1_vel = [a1(2), 2*a1(3), 3*a1(4), 4*a1(5), 5*a1(6), 0];
    a1_acc = [2*a1(3), 6*a1(4), 12*a1(5), 20*a1(6), 0, 0];
    a2_vel = [a2(2), 2*a2(3), 3*a2(4), 4*a2(5), 5*a2(6), 0];
    a2_acc = [2*a2(3), 6*a2(4), 12*a2(5), 20*a2(6), 0, 0];

    % compute the desired trajectory (assuming 5th order polynomials for trajectories)
    q_d = [a1'*vec_t; a2'*vec_t];
    dq_d =[a1_vel*vec_t; a2_vel* vec_t];
    ddq_d =[a1_acc*vec_t; a2_acc* vec_t];
    
    
    % x is an array in the form of 
    % [[q1_1, dq1_1, q1_2, dq1_2, q2_1, dq2_1, q2_2, dq2_2],...];
    % s = dq - dq_r = dq - (dq_d - V*(q - q_d))
    
    V = 10 * eye(2);
    K1 = 100 * eye(2);
    K2 = 4 * eye(2);
    
    for i = 1:p
        
        % state in form [q_1, dq_1, q_2, dq_2]'
        
        state_minus = [x(wrap_index(p, i-1) + 1);
                       x(wrap_index(p, i-1) + 2);
                       x(wrap_index(p, i-1) + 3);
                       x(wrap_index(p, i-1) + 4)];
        
        state = [x(wrap_index(p, i) + 1);
                 x(wrap_index(p, i) + 2);
                 x(wrap_index(p, i) + 3);
                 x(wrap_index(p, i) + 4)];
        
        state_plus = [x(wrap_index(p, i+1) + 1);
                     x(wrap_index(p, i+1) + 2);
                     x(wrap_index(p, i+1) + 3);
                     x(wrap_index(p, i+1) + 4)];
        
        % the actual dynamic model of the system:
        Mmat = [a+2*b*cos(state(3)), d+b*cos(state(3));  d+b*cos(state(3)), d];
        Cmat = [-b*sin(state(3))*state(4), -b*sin(state(3))*(state(2)+state(4)); b*sin(state(3))*state(2),0];
        Gmat =  [m1*g*r1*cos(state(1))+m2*g*(l1*cos(state(1))+r2*cos(state(1)+state(3)));
            m2*g*r2*cos(state(1)+state(3))];
%         invM = inv(Mmat);
        invMC = Mmat\Cmat;
        
        s_minus = [state_minus(2); state_minus(4)] - (dq_d - V*([state_minus(1); state_minus(3)] - q_d));
        s = [state(2); state(4)] - (dq_d - V*([state(1); state(3)] - q_d));
        s_plus = [state_plus(2); state_plus(4)] - (dq_d - V*([state_plus(1); state_plus(3)] - q_d));
    
        % dq_r = dq_d - V*(q-q_d)
        dq_r_minus = dq_d - V * ([state_minus(1); state_minus(3)] - q_d);
        dq_r = dq_d - V * ([state(1); state(3)] - q_d);
        dq_r_plus = dq_d - V * ([state_plus(1); state_plus(3)] - q_d);
        
        % ddq_r = ddq_d - V*(dq - dq_d)
        ddq_r = ddq_d - V * ([state(2); state(4)] - dq_d);

        % Finding ddq using equation (8) and inverse dynamics
        tau = Mmat*ddq_r + Cmat*dq_r + Gmat - K1*([state(2); state(4)] - dq_r) ...
                + K2*([state_minus(2); state_minus(4)] - dq_r_minus) ...
                + K2*([state_plus(2); state_plus(4)] - dq_r_plus);
        
        ddq = Mmat\tau - invMC * [state(2); state(4)] - Mmat\Gmat;
        
        
        % Finding ddq by solving the closed loop dynamics in equation (10)
        % this gives the same result as above
        ddq_1 = Mmat\(-(Cmat * s) - (K1 * s) + (K2 * s_minus) + (K2 * s_plus)) + ddq_r;
        
        dx(wrap_index(p, i) + 1) = state(2);
        dx(wrap_index(p, i) + 2) = ddq(1);
        dx(wrap_index(p, i) + 3) = state(4);
        dx(wrap_index(p, i) + 4) = ddq(2);
    end
    dx = dx';
end

function out = wrap_index(p, val)
    if val < 1
        out = p;

    elseif (1 <= val) && (val <= p)
        out = val;

    elseif val > p
        out = 1;
    end
    
    out = ((out-1)*4);
end

