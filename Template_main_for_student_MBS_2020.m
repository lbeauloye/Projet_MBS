function Template_main_for_student_MBS_2020(param_in)
 
% Project Template (MECA2802 - 2020)
format long
if nargin == 0
    param_in = 1; % If needed (input parameter(s))
end
 
%test = input('Enter a number: ');
test = 5;
global data i % Global structure that contains all the data (data.m, data.d, data.g, ...)
 
%[data] = load_data; % Loading of the data from the data file (up to you!)
 
% Initial conditions for the time simulation(q and qdot at t = 0 sec)
% Example for a 3 dof MBS
 
 
 
%% Geometry, definition of the generalized coordinates and parameters of the system
 
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9;
 
 
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
    
    case 5 %% Manege Validation parameters
        
        data.N = 9;
        data.Nu = 4;
        data.ind_u = [3 4 7 8];
        data.ind_c = [1 2 5 6 9];
        data.inbody = [0, 1, 2, 3, 4, 1, 6, 7, 8];
        data.g = [0; 0; 9.81];
        data.joint_type = ["R3","T1","R1","R2","T3","T1","R1","R2","T3"];
        % Geometry
        data.d = zeros(3,data.N, data.N);
        data.d(:,1,1) = [0,0,-2.5];
        data.d(:,1,2) = [-0.5,0,-5.0];
        data.d(:,2,2) = [-1.5,0,0];
        data.d(:,2,3) = [-3.0,0,0.15];
        data.d(:,3,3) = [0,0,0];
        data.d(:,3,4) = [0,0,0];
        data.d(:,4,4) = [0,0,1.5];
        data.d(:,4,5) = [0,0,3.0];
        data.d(:,5,5) = [0,0,0];
        data.d(:,1,6) = [0.5,0,-5.0];
        data.d(:,6,6) = [1.5,0,0];
        data.d(:,6,7) = [3.0,0,0.15];
        data.d(:,7,7) = [0,0,0];
        data.d(:,7,8) = [0,0,0];
        data.d(:,8,8) = [0,0,1.5];
        data.d(:,8,9) = [0,0,3.0];
        data.d(:,9,9) = [0,0,0];
        data.m = [1000, 200, 0, 5, 70, 200, 0, 5, 70];
        % Fext
        data.fext = zeros(3,9);
        data.lext= zeros(3,9);
        % Inertial matrices
        data.I(:,:,1) = [2145 0 0; 0 2145 0; 0 0 125];
        data.I(:,:,2) = [10.875 0 0; 0 151 0; 0 0 159];
        data.I(:,:,3) = [0 0 0; 0 0 0; 0 0 0];
        data.I(:,:,4) = [3.75 0 0; 0 3.75 0; 0 0 0];
        data.I(:,:,5) = [0.5^2*data.m(5)*2/5 0 0; 0 0.5^2*data.m(5)*2/5 0; 0 0 0.5^2*data.m(5)*2/5];
        data.I(:,:,6) = [10.875 0 0; 0 151 0; 0 0 159];
        data.I(:,:,7) = [0 0 0; 0 0 0; 0 0 0];
        data.I(:,:,8) = [3.75 0 0; 0 3.75 0; 0 0 0];
        data.I(:,:,9) = [0.5^2*data.m(5)*2/5 0 0; 0 0.5^2*data.m(5)*2/5 0; 0 0 0.5^2*data.m(5)*2/5];
        
        data.q_symb = [q1; q2; q3; q4; q5; q6; q7; q8; q9];
        data.qd_symb = [qd1; qd2; qd3; qd4; qd5; qd6; qd7; qd8; qd9];
        data.q = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0]; 
        data.qd = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0]; 
        data.qdd = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
        tmax = 25;
        result = load('/Users/vermeulenlucas/Documents/MBProjects/Simple_manege/resultsR/dirdyn_q.res');
        
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
 
%% Numeric integration
data.dirdyn = 1;
[data.M, data.c] =  dirdyn_symb(data.q_symb, data.qd_symb, data);
%data.c(1,0)
% Variable substitution for an order-1 integrator (ode45)
% 
y0 = [data.q(data.ind_u)' data.qd(data.ind_u)'];
 
% Time integration
% MBS model to be programmed in the external function :
% yd = Template_fprime_for_student_MBS_2020(t,y)
i = 1;
dt = 0.01;
tspan = 0:dt:tmax;
data.lambda = zeros(length(tspan), length(data.ind_c));
data.t = zeros(length(tspan), 1);
tic;
options = odeset('RelTol',1e-6);
[t, y] = ode45('Template_fprime_for_student_MBS_2020', tspan, y0, options);
time(1) = toc;
fprintf('Time spend on integration : %d [sec] \n',time);
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
 
%% Plots of the results
 
QM = 1; % Indicexs of the joint observed in matlab
QR = 2; % Indice of the joint observed in Robotran
 
figure;
subplot(2,1,1)
 
% q(i) plot on Matlab
% plot(t, y(:,QM)*180/pi,'b','Linewidth',2);grid on;title('q R1','Fontsize',14);hold on;
% xlabel('Time [s]','Fontsize',14)
% ylabel('q [�]','Fontsize',14);
 
% Force/Torque plot on Matlab
plot(data.t(:,1),data.lambda(:,1),'b','Linewidth',2);grid on;title('Torque on R3','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Torque [N.m]','Fontsize',14);
 
subplot(2,1,2)
 
% q(i)  plot on Robotran
% plot(t, result(:,QR)*180/pi,'b','Linewidth',2);grid on;title('q R1 robotran','Fontsize',14);
% xlabel('Time [s]','Fontsize',14)
% ylabel('q [�]','Fontsize',14);
 
% Force/Torque plot on Robotran
plot(result(:,1), result(:,QR),'b','Linewidth',2);grid on;title('Torque on R3 robotran','Fontsize',14);
xlabel('Time [s]','Fontsize',14)
ylabel('Torque [N.m]','Fontsize',14);
 
xlim([0 tmax]);
 
%% Plot 2
figure;
subplot(2,2,1)
plot(t,y(:,1),'b','Linewidth',2);grid on;title('q R1','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [�]','Fontsize',14);
subplot(2,2,2)
plot(t,y(:,2),'b','Linewidth',2);grid on;title('q R2','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [�]','Fontsize',14);
subplot(2,2,3)
plot(t,result(:,4),'b','Linewidth',2);grid on;title('q R1 Robotran','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [�]','Fontsize',14);
subplot(2,2,4)
plot(t,result(:,5),'b','Linewidth',2);grid on;title('q R2 Robotran','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [�]','Fontsize',14);
 
xlim([0 tmax]);
 
%% Error Analysis
 
% Error = zeros(tmax/dt+1,2);
% Error(:,1) = t;
% Error(:,2) = result(:,QR) - y(:,QM);
% figure;
% plot(t, Error(:,2),'b','Linewidth',2);grid on;title('Error on q1','Fontsize',14);hold on;
% xlabel('Time [s]','Fontsize',14)
% ylabel('Error [�]','Fontsize',14);
% 
Error_max = max(abs((result(:,QR)-y(:,QM))));
t_error_max = find(abs((result(:,QR)-y(:,QM)))==Error_max);
Error_max_in_Pourcent = 100*(result(t_error_max,QR)-y(t_error_max,QM))/(result(t_error_max,QR));
fprintf('Error max in Pourcent : %d %% \n', Error_max_in_Pourcent);
 
MSE = 0;
for k = 1:tmax/dt+1
    MSE = MSE+(result(k,QR)-y(k,QM))^2;
end
MSE = MSE/(tmax/dt+1);
fprintf('MSE : %d [rad^2] \n',MSE);
 
 
% % % Happy end !
