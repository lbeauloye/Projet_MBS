function main()
 
% Project (MECA2802 - 2020)
format long


fprintf('Hello \n \nPlease choose a method to compute the mass matrix \n');
fprintf('1 : The one of the instructions \n');
fprintf('2 : The one of the reference book (spliting terms associated to M and c) \n');
fprintf('3 : Symbolically \n');
method = input('Enter a number: ');


fprintf('\n \nDo you want a spring-damper on the joints R1 and R2 ? (see report for further explanations)  \n');
fprintf('0 : No\n');
fprintf('1 : Yes\n');
damper = input('Enter a number: ');

global data i % Global structure that contains all the data (data.m, data.d, data.g, ...)
 
 
 
%% Geometry, definition of the generalized coordinates and parameters of the system
 
syms q1 q2 q3 q4 q5 q6 q7 q8 q9 qd1 qd2 qd3 qd4 qd5 qd6 qd7 qd8 qd9;
 
 
 %% Manege Validation parameters
        
data.N = 9;
data.Nu = 4;
data.ind_u = [3 4 7 8];
data.ind_c = [1 2 5 6 9];
data.inbody = [0, 1, 2, 3, 4, 1, 6, 7, 8];
data.g = [0; 0; 9.81];
data.joint_type = ["R3","T1","R1","R2","T3","T1","R1","R2","T3"];
data.damper = damper;
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
% Inertia matrices
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
 
%% Numeric integration

data.dirdyn = method;
if method == 3
    tic;
    [data.M, data.c] =  dirdyn_symb(data.q_symb, data.qd_symb, data);
    ti = toc;
    fprintf('Time spent on M and c symbolic generation : %d [sec] \n',ti);
end

% Variable substitution for an order-1 integrator (ode45)
 
y0 = [data.q(data.ind_u)' data.qd(data.ind_u)'];
 
% Time integration
% MBS model to be programmed in the external function :
% yd = fprime(t,y)


i = 1;
dt = 0.01;
tspan = 0:dt:tmax;
data.lambda = zeros(length(tspan), length(data.ind_c));
data.t = zeros(length(tspan), 1);
tic;
options = odeset('RelTol',1e-6);
[t, y] = ode45('fprime', tspan, y0, options);
time(1) = toc;
fprintf('Time spent on integration : %d [sec] \n',time);
 
%% Plots of the results

figure;
% Force/Torque plot on Matlab

t_ok = find(data.t ~= 0);
plot(data.t(t_ok),data.lambda(t_ok,1),'b','Linewidth',2);grid on;title('Torque on R3','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Torque [N.m]','Fontsize',14);
 

%% Plot 2
figure;
subplot(2,1,1)
plot(t,y(:,1),'b','Linewidth',2);grid on;title('q R1','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [°]','Fontsize',14);
subplot(2,1,2)
plot(t,y(:,2),'b','Linewidth',2);grid on;title('q R2','Fontsize',14);hold on;
xlabel('Time [s]','Fontsize',14)
ylabel('Angle [°]','Fontsize',14);
 
xlim([0 tmax]);
 
% % % Happy end !

fprintf('Happy end ! \n');
