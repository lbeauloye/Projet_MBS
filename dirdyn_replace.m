function [M, c] = dirdyn_replace(data, q, qdbis)

%%% This function computes M and c according to 
%%% the formalism given in the project instructions


unit = eye(data.N);
c = zeros(data.N, 1);
M = zeros(data.N, data.N);

for k = 1:data.N+1
    w = zeros(3, data.N);
    wdot = zeros(3, data.N);
    beta = zeros(3, 3, data.N);
    alpha = zeros(3, data.N);

    if (k == data.N + 1)
        qdd = zeros(data.N,1);
        qd = qdbis;
        alpha_0 = -data.g;
    else
        qd = zeros(1,data.N);
        qdd = (unit(:,k));
        alpha_0 = zeros(3,1);
end


% loops

for i = 1:data.N
    h = data.inbody(i);
    R_ih = Rot(data,i,q).';
    phi = Phi(data,i);
    psi = Psi(data,i);
    if (h==0)
        w(:,i) = phi*qd(i);
        wdot(:,i) = tilde(w(:,i))*phi*qd(i) + phi*qdd(i);
    else
        w(:,i) = R_ih*w(:,h) + phi*qd(i);
        wdot(:,i) = R_ih*wdot(:,h) + tilde(w(:,i))*phi*qd(i) + phi*qdd(i);
    end
    beta(:,:,i) = tilde(wdot(:,i)) + tilde(w(:,i))*tilde(w(:,i));
    if (h==0)
       alpha(:,i) = R_ih*(alpha_0) + 2*tilde(w(:,i))*psi*qd(i) + psi*qdd(i);
    else
       alpha(:,i) = R_ih*(alpha(:,h) + beta(:,:,h)*(q(h)*Psi(data,h)+data.d(:,h,i))) ...
           + 2*tilde(w(:,i))*psi*qd(i) + psi*qdd(i);
    end
end



%% Backward Dynamics 


% Allocation 
W = zeros(3, data.N);
F = zeros(3, data.N);
L = zeros(3, data.N);

for i = data.N:-1:1
    psi = Psi(data,i);
    W(:,i) = data.m(i)*(alpha(:,i) + beta(:,:,i)*(q(i)*psi+data.d(:,i,i))) ...
        - data.fext(:,i);
    children = find(data.inbody == i);
    F(:,i) = W(:,i);
    L(:,i) = tilde(q(i)*psi+data.d(:,i,i))*W(:,i) - data.lext(:,i) ...
        + data.I(:,:,i)*wdot(:,i) + tilde(w(:,i))*data.I(:,:,i)*w(:,i);
    for j = 1:length(children)
       R_ij = Rot(data,children(j),q);
       F(:,i) = F(:,i) + R_ij*F(:,children(j));
       L(:,i) = L(:,i) + R_ij*L(:,children(j)) ...
           + tilde(q(i)*psi+data.d(:,i,children(j)))*R_ij*F(:,children(j));
    end
    
end


%% Projection 

PHI = zeros(data.N, 1);

for i = 1:data.N
    phi = Phi(data,i);
    psi = Psi(data,i);
    PHI(i) = F(:,i).'*psi + L(:,i).'*phi;
    
end

if (k == data.N + 1)
    c = PHI;
else
    M(:,k) = PHI;
end

end

end

function [R] = Rot(data, i, q)

if(strcmp(data.joint_type(i),'R1'))
    R = [1 0 0; 0 cos(q(i)) -sin(q(i)); 0 sin(q(i)) cos(q(i))];
elseif(strcmp(data.joint_type(i),'R2'))
    R = [cos(q(i)) 0 sin(q(i)); 0 1 0; -sin(q(i)) 0 cos(q(i))];
elseif(strcmp(data.joint_type(i),'R3'))
    R = [cos(q(i)) -sin(q(i)) 0; sin(q(i)) cos(q(i)) 0; 0 0 1];
else 
    R = eye(3,3);
end


end

function [phi] = Phi(data,i)

if(strcmp(data.joint_type(i),'R1'))
    phi = [1; 0; 0];
elseif(strcmp(data.joint_type(i),'R2'))
    phi = [0; 1; 0];
elseif(strcmp(data.joint_type(i),'R3'))
    phi = [0; 0; 1];
else 
    phi = [0; 0; 0];
end

end


function [psi] = Psi(data,i)

if(strcmp(data.joint_type(i),'T1'))
    psi = [1; 0; 0];
elseif(strcmp(data.joint_type(i),'T2'))
    psi = [0; 1; 0];
elseif(strcmp(data.joint_type(i),'T3'))
    psi = [0; 0; 1];
else 
    psi = [0; 0; 0];
end

end

function [u_tilde] = tilde(u)
u_tilde = [0, -u(3), u(2); u(3), 0, -u(1); -u(2), u(1), 0];
end 

function [delta] = delta_kron(m,n)
if(m==n)
    delta = 1;
else 
    delta = 0;
end

end