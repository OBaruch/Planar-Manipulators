%% MANIPULADOR PLANAR DE 3 DOF
clear;
close;
clc;
syms a1 a2 a3 theta1 theta2 theta3

L1=Revolute('a',sym('a1'),'alpha',0,'d',0,'offset',0);
L2=Revolute('a',sym('a2'),'alpha',0,'d',0,'offset',0);
L3=Revolute('a',sym('a3'),'alpha',0,'d',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
qv=[sym('theta1') sym('theta2') sym('theta3')];
T03=simplify(bot.fkine(qv));
T01=bot.A(1,qv);
T02=bot.A(1:2,qv);
txyz=T03(1:3,4);
Jvx=[diff(txyz(1),theta1) diff(txyz(1),theta2) diff(txyz(1),theta3)];
Jvy=[diff(txyz(2),theta1) diff(txyz(2),theta2) diff(txyz(2),theta3)];
Jvz=[diff(txyz(3),theta1) diff(txyz(3),theta2) diff(txyz(3),theta3)];
Jv=[Jvx; Jvy; Jvz];
Jv
% CINEMÁTICA INVERSA
a1=0.35; a2=0.35; a3=0.25;
J=@(q)[-a2*sin(q(1)+q(2))-a1*sin(q(1))-a3*sin(q(1)+q(2)+q(3)),-a2*sin(q(1)+q(2))-a3*sin(q(1)+q(2)+q(3)),-a3*sin(q(1)+q(2)+q(3));
     a2*cos(q(1)+q(2))+a1*cos(q(1))+a3*cos(q(1)+q(2)+q(3)),a2*cos(q(1)+q(2))+a3*cos(q(1)+q(2)+q(3)),a3*cos(q(1)+q(2)+q(3));
    0, 0, 1];
txyz=@(q)[a2*cos(q(1)+q(2))+a1*cos(q(1))+a3*cos(q(1)+q(2)+q(3));
    a2*sin(q(1)+q(2))+a1*sin(q(1))+a3*sin(q(1)+q(2)+q(3)); 0];
L1=Revolute('a',a1,'alpha',0,'d',0,'offset',0);
L2=Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3=Revolute('a',a3,'alpha',0,'d',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
t=0.1;
q=[0 pi/6 pi/3]';
td=[0.6 0.5 0.0]';
K=eye(3);
N=100;
Q=zeros(N,3);
Qp=zeros(N,3);
J=J(q);
for i=1:N
    Ti=bot.fkine(q);
    ti=Ti(1:3,4);
    e=td-ti;
    qp=pinv(J)*(K*e);
    q=q+qp*t;
    Q(i,:)=q';
    Qp(i,:)=qp';
end
disp('x y z aproximado');
q
disp('Comparación de coordenadas deseadas con aproximadas');
t=[td txyz(q)]
bot.plot(Q);
figure;
plot(Q);
title('Posiciones');
legend('q(1)','q(2)','q(3)');
figure;
plot(Qp);
title('Velocidades');
legend('q(1)','q(2)','q(3)');
pause;

%% MANIPULADOR ANTROPOMÓRFICO DE 3 DOF
clear;
close;
clc;
syms a2 a3 alpha1 theta1 theta2 theta3 d1
%Tabla DH
L1=Revolute('a',0,'alpha',sym('alpha1'),'d',sym('d1'),'offset',0);
L2=Revolute('a',sym('a2'),'alpha',0,'d',0,'offset',0);
L3=Revolute('a',sym('a3'),'alpha',0,'d',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
qv=[sym('theta1') sym('theta2') sym('theta3')];
T03=simplify(bot.fkine(qv));
T03=subs(T03,alpha1,pi/2);
T03=simplify(T03);
T01=bot.A(1,qv);
T02=bot.A(1:2,qv);
txyz=T03(1:3,4)
Jvx=[diff(txyz(1),theta1) diff(txyz(1),theta2) diff(txyz(1),theta3)];
Jvy=[diff(txyz(2),theta1) diff(txyz(2),theta2) diff(txyz(2),theta3)];
Jvz=[diff(txyz(3),theta1) diff(txyz(3),theta2) diff(txyz(3),theta3)];
Jv=[Jvx; Jvy; Jvz];
Jv
% CINEMÁTICA INVERSA
alpha1=pi/2; a2=0.3; a3=0.25; d1=0.35;
J=@(q)[-sin(q(1))*(a3*cos(q(2)+q(3))+a2*cos(q(2))),-cos(q(1))*(a3*sin(q(2)+q(3))+a2*sin(q(2))),-a3*sin(q(2)+q(3))*cos(q(1));
    cos(q(1))*(a3*cos(q(2)+q(3))+a2*cos(q(2))),-sin(q(1))*(a3*sin(q(2)+q(3))+a2*sin(q(2))),-a3*sin(q(2)+q(3))*sin(q(1));
    0,a3*cos(q(2)+q(3))+a2*cos(q(2)),a3*cos(q(2)+q(3))];
txyz=@(q)[cos(q(1))*(a3*cos(q(2)+q(3))+a2*cos(q(2)));
    sin(q(1))*(a3*cos(q(2)+q(3))+a2*cos(q(2)));
    d1+a3*sin(q(2)+q(3))+a2*sin(q(2))];
L1=Revolute('a',0,'alpha',alpha1,'d',d1,'offset',0);
L2=Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3=Revolute('a',a3,'alpha',0,'d',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
t=0.1;
q=[pi/6 pi/6 pi/3]';
td=[0.25 0.25 0.5]';
K=eye(3);
N=100;
Q=zeros(N,3);
Qp=zeros(N,3);
J=J(q);
for i=1:N
    Ti=bot.fkine(q);
    ti=Ti(1:3,4);
    e=td-ti;
    qp=pinv(J)*(K*e);
    q=q+qp*t;
    Q(i,:)=q';
    Qp(i,:)=qp';
end
disp('x y z  aproximado');
q
disp('Comparación de coordenadas deseadas con aproximadas');
t=[td txyz(q)]
bot.plot(Q);
figure;
plot(Q);
title('Posiciones');
legend('q(1)','q(2)','q(3)');
figure;
plot(Qp);
title('Velocidades');
legend('q(1)','q(2)','q(3)');
pause;

%% MANIPULADOR CILÍNDRICO DE 3 DOF
clear;
close;
clc;

syms theta1 d1 d2 d3 alpha2
L1=Revolute('a',0,'alpha',0,'d',sym(d1),'offset',0);
L2=Prismatic('a',0,'alpha',sym('alpha2'),'theta',0,'offset',0);
L3=Prismatic('a',0,'alpha',0,'theta',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
qv=[sym('theta1') sym('d2') sym('d3')];
T03=simplify(bot.fkine(qv));
T03=subs(T03,alpha2,pi/2);
T03=simplify(T03);
T01=bot.A(1,qv);
T02=bot.A(1:2,qv);
txyz=T03(1:3,4)
Jvx=[diff(txyz(1),theta1) diff(txyz(1),d2) diff(txyz(1),d3)];
Jvy=[diff(txyz(2),theta1) diff(txyz(2),d2) diff(txyz(2),d3)];
Jvz=[diff(txyz(3),theta1) diff(txyz(3),d2) diff(txyz(3),d3)];
Jv=[Jvx; Jvy; Jvz];
disp('Jacobiana de velocidades lineales');
Jv
% CINEMÁTICA INVERSA
d1=0.35; alpha2=pi/2;
J=@(q)[q(3)*cos(q(1)),0,sin(q(1));
    q(3)*sin(q(1)),0,-cos(q(1));
    0,1,0];
txyz=@(q)[q(3)*sin(q(1));
    -q(3)*cos(q(1));
    d1+q(2)];
L1=Revolute('a',0,'alpha',0,'d',d1,'offset',0);
L2=Prismatic('a',0,'alpha',alpha2,'theta',0,'offset',0);
L3=Prismatic('a',0,'alpha',0,'theta',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
t=0.1;
q=[pi/2 0.8 0.5]';
td=[0.5 0.25 0.8]';
K=eye(3);
N=100;
Q=zeros(N,3);
Qp=zeros(N,3);
J=J(q);
for i=1:N
    Ti=bot.fkine(q);
    ti=Ti(1:3,4);
    e=td-ti;
    qp=pinv(J)*(K*e);
    q=q+qp*t;
    Q(i,:)=q';
    Qp(i,:)=qp';
end
disp('x y z aproximado');
q
disp('Comparación de coordenadas deseadas con aproximadas');
t=[td txyz(q)]
bot.plot(Q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
figure;
plot(Q);
title('Posiciones');
legend('q(1)','q(2)','q(3)');
figure;
plot(Qp);
title('Velocidades');
legend('q(1)','q(2)','q(3)');
pause;

%% MANIPULADOR ESFÉRICO DE 3 DOF
clear;
close;
clc;

syms theta1 theta2 d1 d3 alpha1 alpha2
L1=Revolute('a',0,'alpha',sym('alpha1'),'d',sym('d1'),'offset',0);
L2=Revolute('a',0,'alpha',sym('alpha2'),'d',0,'offset',0);
L3=Prismatic('a',0,'alpha',0,'theta',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
qv=[sym('theta1') sym('theta2') sym('d3')];
T03=simplify(bot.fkine(qv));
T03=subs(T03,alpha1,pi/2);
T03=subs(T03,alpha2,-pi/2);
T03=simplify(T03);
T01=bot.A(1,qv);
T02=bot.A(1:2,qv);
txyz=T03(1:3,4)
Jvx=[diff(txyz(1),theta1) diff(txyz(1),theta2) diff(txyz(1),d3)];
Jvy=[diff(txyz(2),theta1) diff(txyz(2),theta2) diff(txyz(2),d3)];
Jvz=[diff(txyz(3),theta1) diff(txyz(3),theta2) diff(txyz(3),d3)];
Jv=[Jvx; Jvy; Jvz];
disp('Jacobiana de velocidades lineales');
Jv
%% CINEMÁTICA INVERSA
alpha1=pi/2; d1=0.35; alpha2=-pi/2;
J=@(q)[q(3)*sin(q(1))*sin(q(2)),-q(3)*cos(q(1))*cos(q(2)),-cos(q(1))*sin(q(2));
    -q(3)*cos(q(1))*sin(q(2)),-q(3)*cos(q(2))*sin(q(1)),-sin(q(1))*sin(q(2));
    0,-q(3)*sin(q(2)),cos(q(2))];
txyz=@(q)[-q(3)*cos(q(1))*sin(q(2));
    -q(3)*sin(q(1))*sin(q(2));
    d1+q(3)*cos(q(2))];
L1=Revolute('a',0,'alpha',alpha1,'d',d1,'offset',0);
L2=Revolute('a',0,'alpha',alpha2,'d',0,'offset',0);
L3=Prismatic('a',0,'alpha',0,'theta',0,'offset',0);
bot=SerialLink([L1 L2 L3],'name','T800');
t=0.1;
q=[pi pi/2 0.45]';
td=[0.5 0.25 0.5]';
K=eye(3);
N=100;
Q=zeros(N,3);
Qp=zeros(N,3);
J=J(q);
for i=1:N
    Ti=bot.fkine(q);
    ti=Ti(1:3,4);
    e=td-ti;
    qp=pinv(J)*(K*e);
    q=q+qp*t;
    Q(i,:)=q';
    Qp(i,:)=qp';
end
disp('x y z aproximado');
q
disp('Comparación de coordenadas deseadas con aproximadas');
t=[td txyz(q)]
bot.plot(Q+0.0001,'workspace',[-1 1 -1 1 -1 1]);
figure;
plot(Q);
title('Posiciones');
legend('q(1)','q(2)','q(3)');
figure;
plot(Qp);
title('Velocidades');
legend('q(1)','q(2)','q(3)');