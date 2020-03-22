function [M, c] = dirdyn(q, qd, data)

% Forward Kinematics



% Allocation
w = sym(zeros(3, data.N+1));
wdot_c = sym(zeros(3, data.N+1));
beta_c = sym(zeros(3, 3, data.N+1));
alpha_c = sym(zeros(3, data.N+1));
O_M = sym(zeros(3,data.N+1, data.N+1));
A_M =  sym(zeros(3,data.N+1, data.N+1));

% Initial coniditions

alpha_c(:,1) = -data.g;

% loops

for i = 2:data.N+1 
    h = data.inbody(i-1);
    R_ih = Rot(data, i-1, q);
    phi = Phi(data,i-1);
    psi = Psi(data,i-1);
    w(:,i) = R_ih*w(:,h+1) +  phi*qd(i-1);
    w_tilde_i = tilde(w(:,i));
    wdot_c(:,i) = R_ih*wdot_c(:,h+1) + w_tilde_i*phi*qd(i-1);
    wdot_c_tilde_i = tilde(wdot_c(:,i));
    beta_c(:,:,i) = wdot_c_tilde_i + w_tilde_i*w_tilde_i;
    alpha_c(:,i) = R_ih*(alpha_c(:,h+1) + beta_c(:,:,h+1)*(q(i-1)*psi + data.d(:,h+1,i))) ...
        + 2*w_tilde_i*psi*qd(i-1);
    
    for k = 2:i
        O_M(:,i,k) = R_ih*O_M(:,h+1,k) + delta_kron(k-1,i-1)*phi;
        A_M(:,i,k) = R_ih*(A_M(:,h+1,k) + tilde(O_M(:,h+1,k))*(q(i-1)*psi + data.d(:,h+1,i))) ...
            + delta_kron(k-1,i-1)*psi;
    end
    
end


alpha_c = alpha_c(:, 2:end);
w = w(:,2:end);
wdot_c = wdot_c(:,2:end);
beta_c = beta_c(:,:,2:end);
O_M = O_M(:,2:end, 2:end);
A_M = A_M(:,2:end, 2:end);


% Backward Dynamics 


% Allocation 
W_c = sym(zeros(3, data.N));
F_c = sym(zeros(3, data.N));
L_c = sym(zeros(3, data.N));
W_M = sym(zeros(3, data.N, data.N));
F_M = sym(zeros(3, data.N, data.N));
L_M = sym(zeros(3, data.N, data.N));


for i = data.N:-1:1
    
    psi = Psi(data,i);
    W_c(:,i) = data.m(i)*(alpha_c(:,i)+ beta_c(:,:,i)*(q(i)*psi + data.d(:,i+1,i+1))) ...
        - data.fext(:,i);
    children = find(data.inbody == i);
    for j = 1:length(children)
        F_c(:,i) = F_c(:,i) + Rot(data, children(j), q)*F_c(:,children(j));
        L_c(:,i) = L_c(:,i)+ Rot(data, children(j), q)*L_c(:,children(j)) ...
            + tilde(q(i)*psi + data.d(:,i+1, children(j)+1))*Rot(data, children(j), q)*F_c(:,children(j));
    end 
    F_c(:,i) = F_c(:,i) + W_c(:,i);
    L_c(:,i) = L_c(:,i) + tilde(q(i)*psi + data.d(:,i+1,i+1))*W_c(:,i) - data.lext(:,i) + ...
        data.I(:,:,i)*wdot_c(:,i) + tilde(w(:,i))*data.I(:,:,i)*w(:,i);
    
    for k = 1:i
        
        W_M(:,i,k) = data.m(i)*(A_M(:,i,k) + tilde(O_M(:,i,k))*(q(i)*psi + data.d(:,i+1,i+1)));
        for j = 1:length(children)
            F_M(:,i,k) = F_M(:,i,k) + Rot(data, children(j), q)*F_M(:,children(j),k);
            L_M(:,i,k) = L_M(:,i,k) + Rot(data, children(j), q)*L_M(:,children(j),k) ...
                + tilde(q(i)*psi + data.d(:,i+1,children(j)+1))*Rot(data, children(j), q)*F_M(:,children(j),k);
        end
        F_M(:,i,k) = F_M(:,i,k) + W_M(:,i,k);
        L_M(:,i,k) = L_M(:,i,k) + tilde(q(i)*psi + data.d(:,i+1,i+1))*W_M(:,i,k) ...
            + data.I(:,:,i)*O_M(:,i,k);
    end
end


% Projection 
c = sym(zeros(data.N, 1));

M = sym(zeros(data.N, data.N));


for i = 1:data.N
    phi = Phi(data,i);
    psi = Psi(data,i);
    c(i) = psi'*F_c(:,i) + phi'*L_c(:,i);
    for j = 1:i
        M(i,j) = psi'*F_M(:,i,j) + phi'*L_M(:,i,j);
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
delta = (m==n);
end


