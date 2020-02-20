% Inverse kinematics
% Using:
% HOWTO(x,y,plot);
% Input: x,y - desired EE position , plot - if true than draw robot in plot
% Output: Q - 4x4 matrix with joints angles in rad
 
function [Q] = HOWTO(Px,Py,Pz)

[q1,q2,q3,q4] = Inv_k(Px,Py,Pz);
[q5,q6,q7,q8] = Inv_k(Px,Py,Pz);
[q9,q10,q11,q12] = Inv_k(Px,Py,Pz);

[q] = [q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12];

function [d1, Q2,Q3,Q4] = Inv_k(px,py,pz)

a1 =3003e - 3;
a2 =3003e - 3;

c2 = (px^2+py^2-a1^2-a2^2)/(2*a1*a2);
s2 = sqrt(1-c2^2);
s1 = ((a1+a2*c2)*py-a2*s2*px)/(px^2+py^2);
c1 = ((a1+a2*c2)*px+a2*s2*py)/(px^2+py^2);

d1 = pz;
Q3 = atan2(s2,c2);
Q2 = atan2(s1,c1);
Q4 = -Q2-Q3;

end

end
