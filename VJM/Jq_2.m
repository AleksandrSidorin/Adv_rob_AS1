function [ Jq ] = Jq_2(Tbase,Ttool,q0,q,t,L,l,d)
 
T0 = FK_2(Tbase,Ttool,q0,q,t,L,l,d);
T0(1:3,4) = [0;0;0];
T0 = T0';
 
T=Tbase*Tz(l)*Ty(l)*Ty(q(5))*Ty(t(l))*Ry(q(6))*Tx(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Ry(q(7))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Ry(q(8))*Ttool;
 
J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
T=Tbase*Tz(l)*Ty(l)*Ty(q(5))*Ty(t(l))*Ry(q(6))*Tx(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Ry(q(7))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Ry(q(8))*Ttool;
 
J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
T=Tbase*Tz(l)*Ty(l)*Ty(q(5))*Ty(t(l))*Ry(q(6))*Tx(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Ry(q(7))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Ry(q(8))*Ttool;

J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;

T=Tbase*Tz(l)*Ty(l)*Ty(q(5))*Ty(t(l))*Ry(q(6))*Tx(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Ry(q(7))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Ry(q(8))*Ttool;

J4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;


Jq = [J1, J2, J3, J4 ;
end
