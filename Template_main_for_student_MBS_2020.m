function [time] = Template_main_for_student_MBS_2020(param_in)

% Project Template (MECA2802 - 2020)
format long
if nargin == 0
    param_in = 1; % If needed (input parameter(s))
end

test = input('Enter a number: ');
global data i % Global structure that contains all the data (data.m, data.d, data.g, ...)

%[data] = load_data; % Loading of the data from the data file (up to you!)

% Initial conditions for the time simulation(q and qdot at t = 0 sec)
% Example for a 3 dof MBS




syms q1 q2 q3 q4 qd1 qd2 qd3 qd4;


switch test 
    case 1
        
        data.N = 1;
        data.inbody = [0];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R2"];
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,1) = [0,0,0.2];
        data.m = [5];
        data.fext = zeros(3,3); % pour l'ex
        data.lext= zeros(3,3); % pour l'ex
        data.I(:,:,1) = zeros(3,3);
        data.I(2,2,1) = 0.1;
        data.q = [1.0];
        data.qd = [0.0]; % ...
        data.qdd = [0.0];
        
        data.q_symb = [q1];
        data.qd_symb = [qd1]; % ...
        tmax = 10;
        result = load('/Users/LB/Documents/MBProjects/Test_1_body/resultsR/dirdyn_q.res');
        
    case 2 
        
        data.N = 2;
        data.inbody = [0, 1];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R2","R2"];
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,2) = [0,0,0.4];
        data.d(:,1,1) = [0,0,0.2];
        data.d(:,2,2) = [0,0,0.2];
        data.m = [5, 5];
        data.fext = zeros(3,3); % pour l'ex
        data.lext= zeros(3,3); % pour l'ex
        data.I(:,:,1) = zeros(3,3);
        data.I(2,2,1) = 0.1;
        data.I(:,:,2) = zeros(3,3);
        data.I(2,2,2) = 0.1;
        data.q = [1.0; 0.0]; % 
        data.qd = [0.0; 0.0]; % ...
        data.qdd = [0.0; 0.0];
        
        data.q_symb = [q1; q2];
        data.qd_symb = [qd1; qd2]; % ...
        tmax = 10;
        result = load('/Users/LB/Documents/MBProjects/Test_2_body_M/resultsR/simdirq.res');
    case 3 
        data.N = 3;
        data.inbody = [0, 1, 2];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R2","R2", "R2"];
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,2) = [0,0,0.4];
        data.d(:,1,1) = [0,0,0.2];
        data.d(:,2,2) = [0,0,0.2];
        data.d(:,2,3) = [0,0,0.4];
        data.d(:,3,3) = [0,0,0.2];
        data.m = [5, 5, 5];
        data.fext = zeros(3,3); % pour l'ex
        data.lext= zeros(3,3); % pour l'ex
        data.I(:,:,1) = zeros(3,3);
        data.I(2,2,1) = 0.1;
        data.I(:,:,2) = zeros(3,3);
        data.I(2,2,2) = 0.1;
        data.I(:,:,3) = zeros(3,3);
        data.I(2,2,3) = 0.1;
        data.q = [1.0; 0.0; 0.0]; % 
        data.qd = [0.0; 0.0; 0.0]; % ...
        data.qdd = [0.0; 0.0; 0.0];
        
        data.q_symb = [q1; q2; q3];
        data.qd_symb = [qd1; qd2; qd3];
        tmax = 10;
        result = load('/Users/LB/Documents/MBProjects/Test_3_body_M/resultsR/simdirq.res');
    case 4
        
        data.N = 4;
        data.inbody = [0, 1, 2, 3];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R2","R2", "R2", "R2"];
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,2) = [0,0,0.4];
        data.d(:,1,1) = [0,0,0.2];
        data.d(:,2,2) = [0,0,0.2];
        data.d(:,2,3) = [0,0,0.4];
        data.d(:,3,3) = [0,0,0.2];
        data.d(:,3,4) = [0,0,0.4];
        data.d(:,4,4) = [0,0,0.2];
        data.m = [5, 5, 5, 5];
        data.fext = zeros(3,4); % pour l'ex
        data.lext= zeros(3,4); % pour l'ex
        data.I(:,:,1) = zeros(3,3);
        data.I(2,2,1) = 0.1;
        data.I(:,:,2) = zeros(3,3);
        data.I(2,2,2) = 0.1;
        data.I(:,:,3) = zeros(3,3);
        data.I(2,2,3) = 0.1;
        data.I(:,:,4) = zeros(3,3);
        data.I(2,2,4) = 0.1;
        data.q = [1.0; 0.0; 0.0; 0.0]; % 
        data.qd = [0.0; 0.0; 0.0; 0.0]; % ...
        data.qdd = [0.0; 0.0; 0.0; 0.0];
        
        data.q_symb = [q1; q2; q3; q4];
        data.qd_symb = [qd1; qd2; qd3; qd4];
        tmax = 10;
        result = load('/Users/LB/Documents/MBProjects/Test_4_body_M/resultsR/simdirq.res');
        
    otherwise 
        
        data.N = 2;
        data.Nu = 1;
        data.ind_u = 2;
        data.ind_c = 1;
        data.inbody = [0, 1];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R2","T3"];
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,2) = [0,0,0.5];
        data.d(:,1,1) = [0,0,0.4];
        data.m = [5, 2];
        data.fext = zeros(3,3);
        data.lext= zeros(3,3); % pour l'ex
        data.I(:,:,1) = zeros(3,3);
        data.I(2,2,1) = 0.1;
        data.I(:,:,2) = zeros(3,3);
        data.q_symb = [q1; q2];
        data.qd_symb = [qd1; qd2];
        data.q = [1.0; 0.2]; % funny values
        data.qd = [0.0; 0.0]; % ...
        data.qdd = [0.0; 0.0];
        tmax = 5;
        result = load('/Users/LB/Documents/Job/mbsysc/ExampleProjects/TutorialProjects/modellingFeatures/1_Bodies_and_Joints/resultsR/dirdyn_q.res');
        
end


data.dirdyn = 3;
[data.M, data.c] =  dirdyn_symb(data.q_symb, data.qd_symb, data);
%data.c(1,0)
% Variable substitution for an order-1 integrator (ode45)
% 
y0 = [data.q(data.ind_u)' data.qd(data.ind_u)'];

% Time integration
% MBS model to be programmed in the external function :
% yd = Template_fprime_for_student_MBS_2020(t,y)
i = 1;

tspan = [0:0.01:tmax];
data.lambda = zeros(length(tspan), 1);
data.t = zeros(length(tspan), 1);
tic;
 options = odeset('RelTol',1e-6);
[t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0, options);
time(1) = toc;
% 
% data.dirdyn = 2;
% tic;
% [t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0, options);
% time(2) = toc;
% % 
% data.dirdyn = 3;
% tic;
% [t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0, options);
% time(3) = toc;
% Plot of results ...
% 

figure;
subplot(2,1,1)
plot(t, y(:,1));grid on;title('q R2');hold on;
subplot(2,1,2)
plot(data.t, data.lambda);grid on;title('q R2 Robotran');hold on;
xlim([0 tmax]);

% % % Happy end !
% 
% load handel;
% sound(x, fe);




