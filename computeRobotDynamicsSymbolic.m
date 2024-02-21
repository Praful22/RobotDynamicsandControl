%% Praful Sigdel
% AME 556: Robot Dynamics and Control
% Problem 2 b

function ddq = computeRobotDynamicsSymbolic()

% Symbolic variables
syms x theta dx dtheta q dq real;
syms M m L g real;

q = [x; theta];
dq = [dx; dtheta];

% Kinetic Energy
T = 0.5 * (M + m) * dx^2 + 0.5 * m * L^2 * dtheta^2 + m * L * dx * dtheta * cos(theta);

% Potential Energy
U = -m*g*L*cos(theta);

% Lagrangian
L = simplify(T - U);

%control input
tau = [0;0];

fq_dq = (jacobian(L,dq))';

D_q = jacobian(fq_dq,dq);

Cq_dq = jacobian(fq_dq,q)*dq - (jacobian(T,q))';

g_q = (jacobian(U,q))';


Nq_dq = Cq_dq + g_q;

 
ddq = (D_q)\(tau - Nq_dq);

end
