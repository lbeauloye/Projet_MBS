function [M, c] = dirdyn2(q, qd, data, t)
%% Forward Kinematics

% Allocation
w = sym(zeros(3, data.N));
wdot = sym(zeros(3, data.N));
beta = sym(zeros(3, 3, data.N));
alpha = sym(zeros(3, data.N));
qdd = data.qdd;

% loops

for i = 1:data.N
    h = data.inbody(i);
    R_ih = Rot(data,i,q);
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
       alpha(:,i) = R_ih*(data.g) + 2*tilde(w(:,i))*psi*qd(i) + psi*qdd(i);
    else
       alpha(:,i) = R_ih*(alpha(:,h) + beta(:,:,h)*(q(i)*psi+data.d(:,h,i))) ...
           + 2*tilde(w(:,i))*psi*qd(i) + psi*qdd(i);
    end
end



%% Backward Dynamics 


% Allocation 
W = sym(zeros(3, data.N));
F = sym(zeros(3, data.N));
L = sym(zeros(3, data.N));

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
c = sym(zeros(data.N, 1));
M = sym(zeros(data.N, data.N));
PHI = sym(zeros(data.N, 1));

for i = 1:data.N
    phi = Phi(data,i);
    psi = Psi(data,i);
    PHI(i) = F(:,i)'*psi + L(:,i)'*phi;
    unit = eye(data.N);
    for k = 1:i
        qddk = (unit(:,k));
        qdk = zeros(data.N, 1);
        
    end
    qddc = zeros(data.N, 1);
    qdc = ones(data.N, 1);
    
end
data.qdd = M\(Q-c);

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