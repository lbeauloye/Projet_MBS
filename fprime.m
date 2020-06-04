function [yp] = fprime(t, y)

global data i % global declaration required for the integrator (Matlab "limitation")

% Variable substitution (from Integrator to MBS)

data.q(data.ind_u) = y(1:data.Nu);
data.qd(data.ind_u) = y(data.Nu+1:end);

% Driven variables
data.q(data.ind_c) = 0.0;
data.qd(data.ind_c) = 0.0;
data.qdd(data.ind_c) = 0.0;

if(t<=10)
    data.q(data.ind_c(1)) = 0.1*t^2;
    data.qd(data.ind_c(1)) = 0.1*t;
    data.qdd(data.ind_c(1)) = 0.1;
else
    data.q(data.ind_c(1)) = (0.1*10^2)+(1.0*(t-10));
    data.qd(data.ind_c(1)) = 0.1*10;
    data.qdd(data.ind_c(1)) = 0.0;
end


% Q vector

Q = zeros(data.N, 1);
if data.damper
    Q(4) = -(100*(data.q(4))+1000*data.qd(4)); %% Damping
    Q(3) = -(100*(data.q(3))+1000*data.qd(3)); %% Damping
    Q(7) = -(100*(data.q(7))+1000*data.qd(7)); %% Damping
    Q(8) = -(100*(data.q(8))+1000*data.qd(8)); %% Damping
end

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
Fc = F(data.ind_c);
Muu = M(data.ind_u,data.ind_u);
Muc = M(data.ind_u, data.ind_c);
Mcu = M(data.ind_c, data.ind_u);
Mcc = M(data.ind_c, data.ind_c);

qdd_u = Muu\(Fu-Muc*data.qdd(data.ind_c));


% Lambda computation
data.t(i) = t;
lambda = Mcu*qdd_u + Mcc*data.qdd(data.ind_c) - Fc;
data.lambda(i,:) = lambda';
i = i + 1;


% Variable substitution (from  MBS to Integrator)

yp = [y(data.Nu+1:end) ; qdd_u];
