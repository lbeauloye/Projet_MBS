function [yp] = Template_fprime_for_student_MBS_2020(t, y)

global data % global declaration required for the integrator (Matlab "limitation")

% Variable substitution (from Integrator to MBS)

data.q(data.ind_u) = y(1:data.Nu);
data.qd(data.ind_u) = y(data.Nu+1:end);

data.q(data.ind_c) = t;
data.qd(data.ind_c)= 1;
data.qqd(data.ind_c) = 0.0;

% Q vector

%[Q] = Joint_forces(data.q, data.qd, data); % up to you : function 'Joint_forces' to program (if needed)
%size(data.q)

Q = [0; -(100*(data.q(2) - 0.1) + 2*data.qd(2))]; % pour l'ex

%Q = zeros(data.N, 1);%[0;0];

% Mass matrix M and c term

if(data.dirdyn == 1)
    [M, c] = dirdyn(data.q, data.qd, data); % up to you : function 'dirdyn to program (NER method) <== MECA2802 :-)
elseif(data.dirdyn == 2)
    [M, c] = dirdyn_replace(data, data.q, data.qd);
else 
    M = data.M([data.q]);
    c = data.c([data.q', data.qd']);
end
    
F = Q - c;

Fu = F(data.ind_u);
Muu = M(data.ind_u,data.ind_u);
Muc = M(data.ind_u, data.ind_c);
% Variable substitution (from  MBS to Integrator)

% yp(1:data.N) = y(data.N+1:end);
% yp(3:4) = (M\F); % solution of linear system ("Ax = b")
% figure(2);
% plot(t, data.q(1), '*'); hold on
% drawnow;

yp = [y(data.Nu+1:end) ; (Muu\(Fu-Muc*data.qdd(data.ind_c)))];
%yp = [y(data.N+1:end) ; (M\F)];
