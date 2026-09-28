clear all
close all
clc
%% Manipulador planar de 2 DOF.configuracion 1
tx=.4;
ty=.4;

a1=.35;
a2=.25;
if(tx^2+ty^2-a1^2-a2^2 > 2*a1*a2)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

%CINEMATICA INVERSA (obtener tetas que alcansan la pocicion deseada)
% theta_2=acos(((tx^2)+(ty^2)-(a1^2)-(a2^2))/(2*a1*a2))
theta_2=-acos((tx^2+ty^2-a1^2-a2^2)/(2*a1*a2));
theta_1=atan2((ty),(tx))-(asin((a2*sin(theta_2))/(sqrt((tx^2)+(ty^2)))))

L1 = Revolute('a',a1,'alpha',0,'d',0,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
bot = SerialLink([L1 L2]);
q = [theta_1 theta_2];
bot.plot(q)
disp("Cinematica directa")
T02 = bot.fkine(q) %Cinematica directa
pause();
%% Manipulador planar de 2 DOF. configuracion 2
cla
a1=.35;
a2=.25;
if(tx^2+ty^2-a1^2-a2^2 > 2*a1*a2)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

%CINEMATICA INVERSA (obtener tetas que alcansan la pocicion deseada)
theta_2=acos(((tx^2)+(ty^2)-(a1^2)-(a2^2))/(2*a1*a2))
% theta_2=-acos((tx^2+ty^2-a1^2-a2^2)/(2*a1*a2));
theta_1=atan2((ty),(tx))-(asin((a2*sin(theta_2))/(sqrt((tx^2)+(ty^2)))))

L1 = Revolute('a',a1,'alpha',0,'d',0,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
bot = SerialLink([L1 L2]);
q = [theta_1 theta_2];
bot.plot(q)
disp("Cinematica directa")
T02 = bot.fkine(q) %Cinematica directa
pause();
%% Manipulador antropomórfico de 3 DOF. configuracion 1
tx=.3;
ty=.2;
tz=.35;
cla
a1=0;
a2=.3;
a3=.25;
d1=.35;
if(tx^2+ty^2+(tz-d1)^2-a2^2-a3^2 > 2*a2*a3)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

theta_3=acos((tx^2+ty^2+((tz-d1)^2)-a2^2-a3^2)/(2*a2*a3));
% theta_3=-acos((tx^2+ty^2+(tz-d1)^2-a2^2-a3^2)/(2*a2*a3));
theta_2=atan2((tz-d1),(sqrt(tx^2+ty^2)))-(asin((a3*sin(theta_3))/(sqrt(tx^2+ty^2+((tz-d1)^2)))));
theta_1=atan2(ty,tx);

L1 = Revolute('a',a1,'alpha',pi/2,'d',d1,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);

bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 theta_3];
bot.plot(q)
disp("Cinematica directa")
T03 = bot.fkine(q) %Cinematica directa
pause();
%% Manipulador antropomórfico de 3 DOF. configuracion 2
cla
a1=0;
a2=.3;
a3=.25;
d1=.35;
if(tx^2+ty^2+(tz-d1)^2-a2^2-a3^2 > 2*a2*a3)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

% theta_3=acos((tx^2+ty^2+((tz-d1)^2)-a2^2-a3^2)/(2*a2*a3));
theta_3=-acos((tx^2+ty^2+(tz-d1)^2-a2^2-a3^2)/(2*a2*a3));
theta_2=atan2((tz-d1),(sqrt(tx^2+ty^2)))-(asin((a3*sin(theta_3))/(sqrt(tx^2+ty^2+((tz-d1)^2)))));
theta_1=atan2(ty,tx);

L1 = Revolute('a',a1,'alpha',pi/2,'d',d1,'offset',0);
L2 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);
L3 = Revolute('a',a2,'alpha',0,'d',0,'offset',0);

bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 theta_3];
bot.plot(q)
disp("Cinematica directa")
T03 = bot.fkine(q) %Cinematica directa
pause();
%% Manipulador cilíndrico de 3 DOF.
cla
tx=.5;
ty=.25;
tz=.8;
d1=.35;
d2off=.15;
d3off=.15;


if(tz < d1+d2off || sqrt(tx^2+ty^2) < d3off)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

theta_1=atan2(ty,tx);
d2=tz-d1-d2off;
d3=sqrt(tx^2+ty^2-d3off);


L1 = Revolute('a',0,'alpha',0,'d',d1,'offset',pi/2);
L2 = Prismatic('a',0,'alpha',pi/2,'theta',0,'offset',d2off);
L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 d2 d3];
bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
disp("Cinematica directa")
T03 = bot.fkine(q) %Cinematica directa
pause();

%% Manipulador esférico de 3 DOF.
cla
tx=.5;
ty=.25;
tz=.8;

d3off=.35;
d1=.35;

if(sqrt(tx^2+ty^2+(tz-d1)^2) < d3off)
    disp('Los puntos que quieres alcansar el robot no los puede alcannsar por sus limitaciones fisicas.')
end

theta_1=atan2(ty,tx);
theta_2=atan2((tz-d1),(sqrt(tx^2+ty^2)));
d3=sqrt(tx^2+ty^2+((tz-d1)^2))-d3off;


L1 = Revolute('a',0,'alpha',pi/2,'d',d1,'offset',0);
L2 = Revolute('a',0,'alpha',-pi/2,'d',0,'offset',-pi/2);
L3 = Prismatic('a',0,'alpha',0,'theta',0,'offset',d3off);
bot = SerialLink([L1 L2 L3]);
q = [theta_1 theta_2 d3];
bot.plot(q+0.00000001,'workspace',[-1 1 -1 1 -1 1]);
disp("Cinematica directa")
T03 = bot.fkine(q) %Cinematica directa
pause();







