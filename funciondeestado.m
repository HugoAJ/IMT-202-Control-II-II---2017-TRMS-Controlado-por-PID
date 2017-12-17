syms j1 j2 j3 anglev angleh mt1 mt2 lt1 lt2 p1 p2 g Danglev Dangleh
syms lb mb mcb lcb mms mts mt mm mtr mmr lm lt rts rms mh h

mt1 = mt+mtr+mts+mm+mmr+mms;
mt2 = mb+mcb;
j1 = (((mt/3)+mtr+mts)*(lt^2))+(((mm/3)+mmr+mms)*(lm^2))+((mms/2)*(rms^2))+((mts/2)*(rts^2));
j2 = ((mb/3)*(lb^2))+((mcb/3)*(lcb^2));
j3 = ((mh/3)*(h^2));
lt2 = (((mb/2)*lb)+(mcb*lcb))/mt2;
lt1 = ((((mm/2)+mmr+mms)*lm)-(((mt/2)+mtr+mts)*lt))/mt1;

mt1_1 = subs(mt1,{mt,mtr,mts,mm,mmr,mms},{0.1195,0.09,0.01,0.1195,0.09,0.01});
mt2_1 = subs(mt2,{mb,mcb},{0.017,0.177});
j1_1 = subs(j1,{mt,mtr,mts,lt,mm,mmr,mms,lm,rms,rts},{0.1195,0.09,0.01,0.25,0.1195,0.09,0.01,0.25,0.02,0.02});
j2_1 = subs(j2,{mb,lb,mcb,lcb},{0.017,0.215,0.177,0.16});
j3_1 = subs(j3,{mh,h},{0.0435,0.053});
lt2_1 = subs(lt2,{mb,lb,mcb,lcb,mt2},{0.017,0.215,0.177,0.16,mt2_1});
lt1_1 = subs(lt1,{mm,mmr,mms,lm,mt,mtr,mts,lt,mt1},{0.1195,0.09,0.01,0.25,0.1195,0.09,0.01,0.25,mt1_1});


%m1=j1*(cos(anglev))^2+j2*(sin(anglev))^2+h^2*(mt1+mt2)+j3;
%m2=0;
%m3=0;
%m4=j1+j2;
    
%M=[m1 m2;m3 m4];
    
%c1=2*(j2-j1)*sin(anglev)*cos(anglev)*Danglev;
%c2=h*(mt1*lt1*sin(anglev)-mt2*lt2*cos(anglev));
%c3=(j2-j1)*sin(anglev)*cos(anglev)*Danglev;
%c4=0;
    
%C=[c1 c2;c3 c4];
    
%g1=0;
%g2=g*(mt1*lt1*cos(anglev)+mt2*lt2*sin(anglev));    
%G=[g1;g2];
    

p1=Dangleh*cos(anglev);
p2=Danglev;
%ecuaciones de estado
D2angleh = p1/(j1*(cos(anglev))^2+j2*(sin(anglev))^2+h^2*(mt1+mt2)+j3);
D2anglev = (p2-g*mt1*lt1*cos(anglev)-g*mt2*lt2*sin(anglev))/(j1+j2);
pretty(D2angleh)
pretty(D2anglev)

%variables de control 

x1=anglev;
x2=Danglev;
x3=angleh;
x4=Dangleh;

%funciones de las variables de control
f1=Danglev;
f2=D2anglev;
f3=Dangleh;
f4=D2angleh;

%Matriz de estado por jacobian

J=jacobian([f1,f2,f3,f4],[x1,x2,x3,x4]);

J1 = subs(J,{g,mt,mtr,mts,lt,mm,mmr,mms,lm,rms,rts,mb,lb,mcb,lcb,mt2,mh,h,mt1},{9.81,0.1195,0.09,0.01,0.25,0.1195,0.09,0.01,0.25,0.02,0.02,0.017,0.215,0.177,0.16,mt2_1,0.0435,0.053,mt1_1})

pretty(J1)

%Mx1x2=subs(M,{x1,x2},{anglev,Danglev});
%Mx1x2=subs(M,{x1,x2},{anglev,Danglev});

A=subs(J1,{anglev,angleh,Danglev, Dangleh},{0,0,0,0});
B=[1;0;1;0];
C=[1 0 1 0];
D=[0 0 0 0];
disp('Matriz A.');
pretty(A)
disp('Vector B.');
B
disp('Vector C.');
C
disp('Vector D.');
D

%Rango de la matriz A
Rango_A=rank(A)

%Controlabilidad del sistema
Controlabilidad=ctrb(A,B)

%Rango de la matriz de controlabilidad
Rango_Contr=rank(Controlabilidad)

%Observabilidad del sistema
Observabilidad=obsv(A,C)

%Rango de la matriz de observabilidad
Rango_Obsr=rank(Observabilidad)




%Para Trabajar con LQR
%Q=[[100 0 0 0];
%  [0 1 0 0];
%  [0 0 10 0];
%  [0,0,0,1]];

%R=0.001;

%K =lqr(A,B,Q,R);
