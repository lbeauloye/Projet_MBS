function [M, c] = dirdyn(q, qd, data)

% Forward Kinematics
 

% Allocation
w = sym(zeros(3, data.N+1));
wdot_c = sym(zeros(3, data.N+1));
beta_c = sym(zeros(3, 3, data.N+1));
alpha_c = sym(zeros(3, data.N+1));
O_M = sym(zeros(3,data.N+1, data.N+1));
A_M =  sym(zeros(3,data.N+1, data.N+1));
d = zeros(3, data.N+1 , data.N+1);
%d = data.d;

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
    alpha_c(:,i) = R_ih*(alpha_c(:,h+1) + beta_c(:,:,h+1)*(q(i-1)*psi + d(:,h+1, i))) ...
        + 2*w_tilde_i*psi*qd(i-1);
    
    for k = 2:i
        O_M(:,i,k) = R_ih*O_M(:,h+1,k) + delta_kron(k,i)*phi;
        A_M(:,i,k) = R_ih*(A_M(:,h+1,k) + tilde(O_M(:,h+1,k))*(q(i-1)*psi + d(:,h+1,i))) ...
            + delta_kron(k,i)*psi;
    end
    
end


% Backward Dynamics 




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