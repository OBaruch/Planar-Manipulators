clear all
close all
clc
%% OMAR BARUCH MORON LOPEZ 
%% Manipulador planar de 3 DOF.
theta_1 = sym('theta_1');
theta_2 = sym('theta_2');
theta_3 = sym('theta_3');
a1 = sym('a1');
a2 = sym('a2');
a3 = sym('a3');
L1 = Revolute('a',a1,'alpha',0,'d',0,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',a3,'alpha',0,'d',0,'offset',0);%%L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 theta_3];%bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
T01=bot.A(1,[theta_1 theta_2 theta_3]);
T02=bot.A(1:2,[theta_1 theta_2 theta_3]);
T03=simplify(bot.fkine(q));
%%Armar matriz jacobiana
%Matriz de velocidades lineales
[x,y,z] = transl(T03) ;
JVx=[diff(x,theta_1) diff(x,theta_2) diff(x,theta_3)];
JVy=[diff(y,theta_1) diff(y,theta_2) diff(y,theta_3)];
JVz=[diff(z,theta_1) diff(z,theta_2) diff(z,theta_3)];
Jv=[JVx; JVy; JVz];
%%Matriz de velovidades Angulares
z0=[0;0;1];
[R,~] = tr2rt(T01);
z1=R(1:3,3);
[R,~] = tr2rt(T02);
z2=R(1:3,3);
    Jw=[z0 z1 z2];
disp('Jacobiana')
    J=[Jv; Jw]
pause();

%% Manipulador antropomórfico de 3 DOF.
clear all
close all
clc
theta_1 = sym('theta_1');
theta_2 = sym('theta_2');
theta_3 = sym('theta_3');
a2 = sym('a2');
a3 = sym('a3');
al1= sym('al1');
d1 = sym('d1');
L1 = Revolute('a',0,'alpha',al1,'d',d1,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',a3,'alpha',0,'d',0,'offset',0);%%L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 theta_3];%bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
T01=bot.A(1,[theta_1 theta_2 theta_3]);
T02=bot.A(1:2,[theta_1 theta_2 theta_3]);
T03=simplify(bot.fkine(q));
%%Armar matriz jacobiana
%Matriz de velocidades lineales
[x,y,z] = transl(T03) ;
JVx=[diff(x,theta_1) diff(x,theta_2) diff(x,theta_3)];
JVy=[diff(y,theta_1) diff(y,theta_2) diff(y,theta_3)];
JVz=[diff(z,theta_1) diff(z,theta_2) diff(z,theta_3)];
Jv=[JVx; JVy; JVz];
%%Matriz de velovidades Angulares
z0=[0;0;1];
[R,~] = tr2rt(T01);
z1=R(1:3,3);
[R,~] = tr2rt(T02);
z2=R(1:3,3);
    Jw=[z0 z1 z2];
disp('Jacobiana')
    J=[Jv; Jw];
    J=subs(J,al1,pi/2)
pause();
%% Manipulador cilíndrico de 3 DOF..
clear all
close all
clc
theta_1 = sym('theta_1');
d2 = sym('d2');
d3 = sym('d3');
al2= sym('al2');
d1 = sym('d1');
L1 = Revolute('a',0,'alpha',0,'d',d1,'offset',0);
L2 = Prismatic('a',0,'alpha',al2,'theta',0,'offset',0);
L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',0);%%L3 = Revolute('a',0,'alpha',0,'d',0,'offset',0);%%L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 d2 d3];%bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
T01=bot.A(1,[theta_1 d2 d3]);
T02=bot.A(1:2,[theta_1 d2 d3]);
T03=simplify(bot.fkine(q));
%%Armar matriz jacobiana
%Matriz de velocidades lineales
[x,y,z] = transl(T03) ;
JVx=[diff(x,theta_1) diff(x,d2) diff(x,d3)];
JVy=[diff(y,theta_1) diff(y,d2) diff(y,d3)];
JVz=[diff(z,theta_1) diff(z,d2) diff(z,d3)];
Jv=[JVx; JVy; JVz];
%%Matriz de velovidades Angulares
z0=[0;0;1];
z1=[0;0;0];
z2=[0;0;0];
    Jw=[z0 z1 z2];
disp('Jacobiana')
    J=[Jv; Jw];
    J=subs(J,al2,pi/2)
pause();
%% Manipulador esferico de 3 DOF..
clear all
close all
clc
theta_1 = sym('theta_1');
theta_2 = sym('theta_2');
d3 = sym('d3');
al2= sym('al2');
al1= sym('al1');
d1 = sym('d1');
L1 = Revolute('a',0,'alpha',al1,'d',d1,'offset',0);
L2 = Revolute('a',0,'alpha',al2,'d',0,'offset',0);
L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',0);%%L3 = Revolute('a',0,'alpha',0,'d',0,'offset',0);%%L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 d3];%bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
T01=bot.A(1,[theta_1 theta_2 d3]);
T02=bot.A(1:2,[theta_1 theta_2 d3]);
T03=simplify(bot.fkine(q));
%%Armar matriz jacobiana
%Matriz de velocidades lineales
[x,y,z] = transl(T03) ;
JVx=[diff(x,theta_1) diff(x,theta_2) diff(x,d3)];
JVy=[diff(y,theta_1) diff(y,theta_2) diff(y,d3)];
JVz=[diff(z,theta_1) diff(z,theta_2) diff(z,d3)];
Jv=[JVx; JVy; JVz];
%%Matriz de velovidades Angulares
z0=[0;0;1];
[R,~] = tr2rt(T01);
z1=R(1:3,3);
z2=[0;0;0];
    Jw=[z0 z1 z2];
disp('Jacobiana')
    J=[Jv; Jw];
    J=subs(J,al1,pi/2);
    J=subs(J,al2,-pi/2)

