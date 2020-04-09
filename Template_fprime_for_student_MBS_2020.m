function [yp] = Template_fprime_for_student_MBS_2020(t, y)

global data % global declaration required for the integrator (Matlab "limitation")

% Variable substitution (from Integrator to MBS)

data.q = y(1:data.N);
data.qd = y(data.N+1:end);

% Q vector

%[Q] = Joint_forces(data.q, data.qd, data); % up to you : function 'Joint_forces' to program (if needed)
%size(data.q)

Q = [0; -(100*(data.q(2) - 0.1) + 2*data.qd(2))]; % pour l'ex

% Mass matrix M and c term

[M, c] = dirdyn(data.q, data.qd, data); % up to you : function 'dirdyn to program (NER method) <== MECA2802 :-)

F = Q - c;

% Variable substitution (from  MBS to Integrator)

% yp(1:data.N) = y(data.N+1:end);
% yp(3:4) = (M\F); % solution of linear system ("Ax = b")
% figure(2);
% plot(t, data.q(1), '*'); hold on
% drawnow;

yp = [y(data.N+1:end) ; (M\F)];
