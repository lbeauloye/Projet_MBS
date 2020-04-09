function [M, c] = dirdyn_symb(q, qd, data)

% Forward Kinematics



% Allocation
w = sym(zeros(3, data.N));
wdot_c = sym(zeros(3, data.N));
beta_c = sym(zeros(3, 3, data.N));
alpha_c = sym(zeros(3, data.N));
O_M = sym(zeros(3,data.N, data.N));
A_M =  sym(zeros(3,data.N, data.N));

% d_ijz = zeros(3,data.N, data.N));
% 
% for i=1:data.N
%     
%     if(strcmp(data.joint_type(i),'T1') || strcmp(data.joint_type(i),'T2') || strcmp(data.joint_type(i),'T3'))
%         d_ijz(:,i,j) = 
% end
% Initial coniditions

%alpha_c_0 = -data.g;

% loops

for i = 1:data.N
    h = data.inbody(i);
    R_ih = Rot(data,i,q).';
    phi = Phi(data,i);
    psi = Psi(data,i);
    if (h==0)
        w(:,i) = phi*qd(i);
        wdot_c(:,i) = tilde(w(:,i))*phi*qd(i);
    else
        w(:,i) = R_ih*w(:,h) + phi*qd(i);
        wdot_c(:,i) = R_ih*wdot_c(:,h) + tilde(w(:,i))*phi*qd(i);
    end
    beta_c(:,:,i) = tilde(wdot_c(:,i)) + tilde(w(:,i))*tilde(w(:,i));
    if (h==0)
       alpha_c(:,i) = R_ih*(-data.g) + 2*tilde(w(:,i))*psi*qd(i);
    else
       alpha_c(:,i) = R_ih*(alpha_c(:,h) + beta_c(:,:,h)*(q(h)*Psi(data,h)+data.d(:,h,i))) ...
           + 2*tilde(w(:,i))*psi*qd(i);
    end
    
    for k = 1:i
        if (h==0)
           O_M(:,i,k) = delta_kron(k,i)*phi;
           A_M(:,i,k) = delta_kron(k,i)*psi;
        else
           O_M(:,i,k) = R_ih*O_M(:,h,k) + delta_kron(k,i)*phi;
           A_M(:,i,k) = R_ih*(A_M(:,h,k) + tilde(O_M(:,h,k))*(q(h)*Psi(data,h)+data.d(:,h,i))) ...
              + delta_kron(k,i)*psi;
        end
% %         
        if(k ~= i)
            O_M(:,k,i) = O_M(:,i,k);
            A_M(:,k,i) = A_M(:,i,k);
        end
            
    end    
end
% 
% O_M
% A_M
% alpha_c
% w
% wdot_c
% beta_c

%alpha_c = alpha_c(:, 2:end);
%w = w(:,2:end);
%wdot_c = wdot_c(:,2:end);
%beta_c = beta_c(:,:,2:end);
%O_M = O_M(:,2:end, 2:end);
%A_M = A_M(:,2:end, 2:end);


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
    W_c(:,i) = data.m(i)*(alpha_c(:,i) + beta_c(:,:,i)*(q(i)*psi+data.d(:,i,i))) ...
        - data.fext(:,i);
    children = find(data.inbody == i);
    F_c(:,i) = W_c(:,i);
    L_c(:,i) = tilde(q(i)*psi+data.d(:,i,i))*W_c(:,i) - data.lext(:,i) ...
        + data.I(:,:,i)*wdot_c(:,i) + tilde(w(:,i))*data.I(:,:,i)*w(:,i);
    for j = 1:length(children)
       R_ij = Rot(data,children(j),q);
       F_c(:,i) = F_c(:,i) + R_ij*F_c(:,children(j));
       L_c(:,i) = L_c(:,i) + R_ij*L_c(:,children(j)) ...
           + tilde(q(i)*psi+data.d(:,i,children(j)))*R_ij*F_c(:,children(j));
    end

    for k = 1:i
%         i
%         k
%         A_M(:,i,k)
%         tilde(O_M(:,i,k))*(q(i)*psi+data.d(:,i,i))
        W_M(:,i,k) = data.m(i) * (A_M(:,i,k)+tilde(O_M(:,i,k))*(q(i)*psi+data.d(:,i,i)));
        F_M(:,i,k) = W_M(:,i,k);
        L_M(:,i,k) = tilde(q(i)*psi+data.d(:,i,i))*W_M(:,i,k) + data.I(:,:,i)*O_M(:,i,k);
        for j = 1:length(children)
            R_ij = Rot(data,children(j),q);
            F_M(:,i,k) = F_M(:,i,k) + R_ij*F_M(:,children(j),k);
            L_M(:,i,k) = L_M(:,i,k) + R_ij*L_M(:,children(j),k) ...
                + tilde(q(i)*psi+data.d(:,i,children(j)))*R_ij*F_M(:,children(j),k);
        end
%         
        if(k ~= i)
            W_M(:,k,i) = W_M(:,i,k);
            F_M(:,k,i) = F_M(:,i,k);
            L_M(:,k,i) = L_M(:,i,k);
        end
%         
    end
end
% 
% W_c
% F_c
% L_c
% L_M
% Projection 
c = sym(zeros(data.N, 1));

M = sym(zeros(data.N, data.N));


for i = 1:data.N
    phi = Phi(data,i);
    psi = Psi(data,i);
    c(i) = psi'*F_c(:,i) + phi'*L_c(:,i);
    for j = 1:i
        M(i,j) = psi'*F_M(:,i,j) + phi'*L_M(:,i,j);
        if(i~=j)
            M(j,i) = M(i,j);
        end
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