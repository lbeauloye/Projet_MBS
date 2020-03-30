function [t,y] = Template_main_for_student_MBS_2020(param_in)

% Project Template (MECA2802 - 2020)

if nargin == 0
    param_in = 1; % If needed (input parameter(s))
end

global data % Global structure that contains all the data (data.m, data.d, data.g, ...)

%[data] = load_data; % Loading of the data from the data file (up to you!)

% Initial conditions for the time simulation(q and qdot at t = 0 sec)
% Example for a 3 dof MBS

syms m1 m2 f11 f12 f21 f22 f13 f23 l11 l12 l21 l22 l13 l23 I111 I121 I131 I211 I221 I231 I311 I321 I331;
syms I112 I122 I132 I212 I222 I232 I312 I322 I332;
syms d231 d232 d233 d221 d222 d223;
data.N = 2;
data.inbody = [0, 1];
data.g = [0; 0; 9.81];
data.joint_type = ['R2', 'T3'];
data.d = sym(zeros(3,data.N+1, data.N+1));
data.d(:,2,3) = [0 0 d233]; % [0,0,0.5]; pour l'ex
data.d(:,2,2) = [0 0 d223]; % [0,0,0.4]; pour l'ex
data.m = [m1; m2]; %[5, 2];
data.fext = [f11, f12; f21, f22; f13, f23];
%data.fext = zeros(3,2); % pour l'ex
data.lext = [l11, l12; l21, l22; l13, l23];
%data.lext= zeros(3,2); % pour l'ex
data.I(:,:,1) = sym(zeros(3,3));
data.I(2,2,1) = I221; % 0.1 pour l'ex
data.I(:,:,2) = sym(zeros(3,3));
data.q = [1; 0.2]; % funny values
data.qd = [0.0; 0.0]; % ...
data.qdd = [0.0; 0.0];


% TESTS A enlever pour integrer
syms q1 q2 q3 qd1 qd2 qd3;
q = [q1, q2, q3];
qd = [qd1, qd2, qd3];

%[M, c] = dirdyn(q, qd, data)
[M, c] = dirdyn2(q, qd, data, t)

subs(M, [m1, m2, d223, d233, q2, I221], [5, 2, 0.4, 0.5, 0.2, 0.1]),
subs(c, [m1, m2, d223, d233, q2, q1, f12, f11, f23, l21, l22, qd1, qd2],  [5, 2, 0.4, 0.5, 0.2, 1, 0, 0, 0, 0, 0, 0, 0])

% Variable substitution for an order-1 integrator (ode45)

y0 = [data.q data.qd];

% Time integration
% MBS model to be programmed in the external function :
% yd = Template_fprime_for_student_MBS_2020(t,y)

tspan = [0 5];
[t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0);

% Plot of results ...

figure(1)
subplot(2,1,1)
plot(t, y(:,1));grid on;title('q T3');hold on;
subplot(2,1,2)
plot(t, y(:,2));grid on;title('qd T3');hold on;

% % Happy end !
% 
% load handel
% sound(y,Fs)

%



