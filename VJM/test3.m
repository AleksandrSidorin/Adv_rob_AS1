% function [Kc,tp] = VJM_nonlin_2(Tbase,Ttool,q0,q,theta,L,l,d,F)
%
L=300e-3;
l=100e-3;
d=300e-3;

temp = HOWTO(-0.15,-0.25,false);
Tbase = eye(4);
Ttool = eye(4);
theta = zeros(1,13);

q0 = [-temp(1,2) -temp(1,1)];
q2 = temp(1,3)-temp(1,1);
q1 = temp(1,4)-temp(1,2);
q3 = pi*3 - temp(1,1) - temp(1,2) - q1- q2;
q = [-q1 -q2 q3];
F = [0;0;0;0;0;0];


k0 = 1e6; % Actuator stif
%LINK1
%material and shape parameters
E_1 = 70 *10e9; %Young's modulus
G_1 = 25.5*10e9; %shear modulus
d_1 = 10*10e-3;
L_1 = L;

%for cylinder
S_1 = pi*d_1^2/4;
Iy_1 = pi*d_1^4/64;
Iz_1 = pi*d_1^4/64;
J_1 = Iy_1 + Iz_1;


k1 = [E_1*S_1/L_1 0                  0                 0           0                 0;
    0           12*E_1*Iz_1/L_1^3  0                 0           0                 6*E_1*Iy_1/L_1^2;
    0           0                  12*E_1*Iy_1/L_1^3 0           -6*E_1*Iy_1/L_1^2 0;
    0           0                  0                 G_1*J_1/L_1 0                 0;
    0           0                  -6*E_1*Iy_1/L_1^2 0           4*E_1*Iy_1/L_1      0;
    0           6*E_1*Iy_1/L_1^2   0                 0           0                 4*E_1*Iz_1/L_1];




%LINK2
%material and shape parameters
E_2 = 70 *10e9; %Young's modulus
G_2 = 25.5*10e9; %shear modulus
d_2 = 10*10e-3;
L_2 = l;

%for cylinder
S_2 = pi*d_2^2/4;
Iy_2 = pi*d_2^4/64;
Iz_2 = pi*d_2^4/64;
J_2 = Iy_2 + Iz_2;


k2 = [E_2*S_2/L_2 0                  0                 0           0                 0;
    0           12*E_2*Iz_2/L_2^3  0                 0           0                 6*E_2*Iy_2/L_2^2;
    0           0                  12*E_2*Iy_2/L_2^3 0           -6*E_2*Iy_2/L_2^2 0;
    0           0                  0                 G_2*J_2/L_2 0                 0;
    0           0                  -6*E_2*Iy_2/L_2^2 0           4*E_2*Iy_2/L_2      0;
    0           6*E_2*Iy_2/L_2^2   0                 0           0                 4*E_2*Iz_2/L_2];





Kt = [k0 zeros(1,12)
    zeros(6,1) k2 zeros(6,6)
    zeros(6,1) zeros(6,6) k1];

    
iter=100; % iterations number
limit = 10;

m = 0;


Fx = [1e-7;0;0;0;0;0];

t=FK_2(Tbase,Ttool,q0,q,theta,L,l,d);

tp =zeros(6,1);
tp(1:3)=t(1:3,4);
eul = tr2eul(t);
tp(6)=eul(3); %%%
tp=[-0.15;-0.25;0;0;0;1.91682170172372];
% a = 1;
%
% Fa = F./a;
%
% for z=1:a
% F=Fa;%.*z;

    
    qx = q;
    tx = theta';
    Fxp = 10e10;
    txp = 10e10;
    qxp = 10e10;
    

    i = 1;

    while (norm(Fx-Fxp) > 1)

        Jq=Jq_2(Tbase,Ttool,q0,qx,tx,L,l,d);
        Jt=Jt_2(Tbase,Ttool,q0,qx,tx,L,l,d);
        fk=FK_2(Tbase,Ttool,q0,qx,tx,L,l,d);
        
        g=zeros(6,1);
        g(1:3)=fk(1:3,4);
        eul = tr2eul(fk);
        g(6)=eul(3); %%%
        
        P=inv([Jt*Kt^-1*Jt' Jq;Jq' zeros(2,2)])*[tp-g+Jq*(qx(2:3)')+Jt*tx;0;0];
%         P=([Jt*Kt^-1*Jt' Jq;Jq' zeros(2,2)])\[tp-g+Jq*(qx(2:3)')+Jt*tx;0;0];
        Fxp = Fx;
        Fx=P(1:6);
        qxp=qx(2:3);
        qx(2:3)=P(7:8);
        txp = tx;
        tx=Kt^-1*Jt'*Fx;
%         tx=Kt\(Jt'*Fx);

        i = i+1;
        if i>iter
             disp('Cant find solution 1 - 2');
            break;
        end
    end
%     Jq=Jq_2(Tbase,Ttool,q0,qx,tx,L,l,d);
%     Jt=Jt_2(Tbase,Ttool,q0,qx,tx,L,l,d);
%     Hqq=Hqq_2(Tbase,Ttool,q0,qx,tx,L,l,d,Fx);
%     Htt=Htt_2(Tbase,Ttool,q0,qx,tx,L,l,d,Fx);
%     Hqt=Hqt_2(Tbase,Ttool,q0,qx,tx,L,l,d,Fx);
%     Htq=Htq_2(Tbase,Ttool,q0,qx,tx,L,l,d,Fx);
%
%     ktf=inv(Kt-Htt);
%     Kc0=inv(Jt*ktf*Jt');
% %     Kcq=-(Hqq+Hqt*ktf*Htq-(Jq'+Hqt*ktf*Jt')*Kc0*(Jq+Jt*ktf*Htq))^-1*(Jq'+Hqt*ktf*Jt')*Kc0;
%     Kcq=-(Hqq+Hqt*ktf*Htq-(Jq'+Hqt*ktf*Jt')*Kc0*(Jq+Jt*ktf*Htq))\(Jq'+Hqt*ktf*Jt')*Kc0;
%     Kc=Kc0-Kc0*(Jq+Jt*ktf*Htq)*Kcq
% %     cond(Kc1)
% %     pinv(Kc1)
% %
% %     Kz=inv([zeros(6,6) Jq Jt;Jq' Hqq Hqt;Jt' Htq Htt-Kt]);
% %     Kc=Kz(1:6,1:6);
% %     cond(Kc)
% %     pinv(Kc)
%
% %     tp = tp + inv(Kc)*(F-Fx);
%     tp = tp + (Kc)\(F-Fx);

% t=tp;
% end
% end
