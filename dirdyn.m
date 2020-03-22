function [M, c] = dirdyn(q, qd, data)

% Forward Kinematics
 

% Initial coniditions
w = zeros(3, data.N);
wdot_c = zeros(3, data.N);
beta_c = zeros(3, data.N);
alpha_c = zeros(3, data.N);
O_M = zeros(data.N, data.N);
A_M =  zeros(data.N, data.N);

% loops

for i = 1:data.N 
    h = data.inbody(i);
    w(:,i) = Rot(
end





% Backward Dynamics 




end

function [R] = Rot(data, i)

if(data.joint_type(i) == 1)
    R = eye(3,3);
else 
    R = 
end


end