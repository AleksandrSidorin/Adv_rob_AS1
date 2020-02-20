function [ Jt ] = Jt_1(Tbase,Ttool,q0,q,t,L,l,d)
 
T0 = FK_1(Tbase,Ttool,q0,q,t,L,l,d);
T0(1:3,4) = [0;0;0];
T0 = T0';
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J4 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;

J5 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J6 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;

J7 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
*Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J8 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J9 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J10 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J11 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J12 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
Td=Tbase*Tx(l)*Ty(l)*Txd(q(l))*Tx(t(l))*Rx(q(2))*Tz(L)*Tx(t(2))*Ty(t(3))*Tz(t(4))...
    *Rx(t(5))*Ry(t(6))*Rz(t(7))*Rx(q(3))*Tx(L)*Tx(t(8))*Ty(t(9))*Tz(t(10))...
    Rx(t(11))*Ry(t(12))*Rz(t(13))*Rx(q(4))*Ttool*T0;
 
J13 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' ;
 
 
Jt = [J1 J2 J3 J4 J5 J6 J7 J8 J9 J10 J11 J12 J13] ;
end
