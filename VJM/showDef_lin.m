% Calculate robot end-effector displacement
%
% Using:
% showDef_lin(F);
% Input: F - force vector (Fx,Fy,Fz,Mx,My,Mz) example: F = [100;00;0;0;0;0];
% Output: none

function showDef_lin_q(x,y)

clear all

L = 300e - 3;
l = 100e - 3;
d = 300e - 3;


for i = 1:length(x)
%     disp(i/length(x)*100)
    for j = 1:length(y)
        % Calc robot stiffnes matrix
        [K1] =VJM_lin_total(x(i)/1000, y(j)/1000, z(k)/1000);
        % Get deflections for all configurations
        dt=K1*F;
        dr=sqrt(dt(1)^2+dt(2)^2);
        z1(i,j,k) = dr;
    end
end

% Draw plot
% c1 = (z1>7e-4)';
% c2 = (z2>7e-4)';
% c3 = (z3>7e-4)';
% c4 = (z4>7e-4)';
figure
HOWTO(0.1,0.2,0.1);
% figure()
% subplot(2,2,1)
% contourf(z1',[0.000001 0.000001 0.0001 0.001 0.01 0.1])
% subplot(2,2,2)
% figure()
% contourf(z2',[0.000001 0.000001 0.0001 0.001 0.01 0.1])
% subplot(2,2,3)
% figure()
contourf(z3',[0.000001 0.000001 0.0001 0.001 0.01 0.1])
% subplot(2,2,4)
% figure()
% contourf(z4',[0.000001 0.000001 0.0001 0.001 0.01 0.1])


end
