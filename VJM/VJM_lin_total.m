% Calculate robot stiffness matrix in current position
% for all configurations
% Using:
% [K1,K2,K3,K4] = VJM_lin_total(x,y)
% Input: x,y end-effector position
% Output: K1,K2,K3,K4 - 6*6 compliance matrix for each configuration

function [K1] = VJM_lin_total(x,y,z)

% robot params
L=300e-3;
l=100e-3;

% get angles with IK
q = HOWTO(x,y,z);
Tbase = eye(4);
Ttool = eye(4);
t = 0.001*zeros(1,13);

% result matrix = chain1 matrix + chain2 matrix
if  isreal(q(1))&&isreal(temp(2))&&isreal(temp(3))&&isreal(temp(4))...
    &&isfinite(temp(5))&&isfinite(temp(6))&&isfinite(temp(7))&&isfinite(temp(8))...
    &&isfinite(temp(9))&&isfinite(temp(10))&&isfinite(temp(11))&&isfinite(temp(12))
    K1=inv(VJM_lin_1(Tbase,Ttool,q,t,L,l) + VJM_lin_2(Tbase,Ttool,q,t,L,l));
else
    K1=0;
end

q0 = [-temp(2,2) -temp(2,1)];
q1 = temp(2,4)-temp(2,2);
q2 = temp(2,3)-temp(2,1);
q3 = pi*3 - temp(2,1) - temp(2,2) - q1- q2;
q = [-q1 -q2 q3];

if isreal(temp(1,1))&&isreal(temp(2,2))&&isreal(temp(2,3))&&isreal(temp(2,4))...
                &&isfinite(temp(2,1))&&isfinite(temp(2,2))&&isfinite(temp(2,3))&&isfinite(temp(2,4))
            K2=inv(VJM_lin_1(Tbase,Ttool,q0,q,t,L,l,d) + VJM_lin_2(Tbase,Ttool,q0,q,t,L,l,d));
else
    K2=0;
end

q0 = [-temp(3,2) -temp(3,1)];
q2 = temp(3,3)-temp(3,1);
q1 = temp(3,4)-temp(3,2);
q3 = pi*3 - temp(3,1) - temp(3,2) - q1- q2;
q = [-q1 -q2 q3];

if isreal(temp(1,1))&&isreal(temp(2,2))&&isreal(temp(2,3))&&isreal(temp(2,4))...
    &&isfinite(temp(2,1))&&isfinite(temp(2,2))&&isfinite(temp(2,3))&&isfinite(temp(2,4))
K2=inv(VJM_lin_1(Tbase,Ttool,q0,q,t,L,l,d) + VJM_lin_2(Tbase,Ttool,q0,q,t,L,l,d));

else
    K3=0;
end


q0 = [-temp(4,2) -temp(4,1)];
q2 = temp(4,3)-temp(4,1);
q1 = temp(4,4)-temp(4,2);
q3 = pi*3 - temp(4,1) - temp(4,2) - q1- q2;
q = [-q1 -q2 q3];

if isreal(temp(1,1))&&isreal(temp(2,2))&&isreal(temp(2,3))&&isreal(temp(2,4))...
    &&isfinite(temp(2,1))&&isfinite(temp(2,2))&&isfinite(temp(2,3))&&isfinite(temp(2,4))
K2=inv(VJM_lin_1(Tbase,Ttool,q0,q,t,L,l,d) + VJM_lin_2(Tbase,Ttool,q0,q,t,L,l,d));

else
    K4=0;
end
end
