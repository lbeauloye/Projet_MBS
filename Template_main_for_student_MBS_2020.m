function [t,y] = Template_main_for_student_MBS_2020(param_in)

% Project Template (MECA2802 - 2020)

if nargin == 0
    param_in = 1; % If needed (input parameter(s))
end

global data % Global structure that contains all the data (data.m, data.d, data.g, ...)

%[data] = load_data; % Loading of the data from the data file (up to you!)

% Initial conditions for the time simulation(q and qdot at t = 0 sec)
% Example for a 3 dof MBS

syms m1 m2 f11 f12 f21 f22 f13 f23 l11 l12 l21 l22 l13 l23 I11 I12 I13 I21 I22 I23 I31 I32 I33;
data.N = 2;
data.inbody = [0, 1];
data.g = [0; 0; -9.81];
data.joint_type = ["R2", "T3"];
data.m = [m1; m2];
data.fext = [f11, f12; f21, f22; f13, f23];
%data.fext = zeros(3,2);
data.lext = [l11, l12; l21, l22; l13, l23];
%data.lext= zeros(3,2);
data.I(:,:,1) = [I11, I12, I13; I21, I22, I23; I31, I32, I33];
% data.I(:,:,1) = zeros(3,3);
% data.I(2,2,1) = 0.1;
% data.I(:,:,2) = zeros(3,3);
data.I(:,:,2) = [I11, I12, I13; I21, I22, I23; I31, I32, I33];
data.q = [1; 0.2]; % funny values
data.qd = [0.0; 0.0]; % ...

syms q1 q2 q3 qd1 qd2 qd3;
q = [q1, q2, q3];
qd = [qd1, qd2, qd3];

[M, c] = dirdyn(q, qd, data)
% Variable substitution for an order-1 integrator (ode45)

y0 = [data.q; data.qd];

% Time integration
% MBS model to be programmed in the external function :
% yd = Template_fprime_for_student_MBS_2020(t,y)

%tspan = [0 5];
[t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0);

% Plot of results ...

figure(1)
subplot(2,1,1)
plot(t, -y(:,1));grid on;title('q T3');hold on;
subplot(2,1,2)
plot(t, -y(:,2));grid on;title('qd T3');hold on;

% Happy end !

load handel
sound(y,Fs)

%



