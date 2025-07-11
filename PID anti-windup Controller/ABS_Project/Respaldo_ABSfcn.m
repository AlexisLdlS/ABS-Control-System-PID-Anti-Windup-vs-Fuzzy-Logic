function [pWhl, MEM_out,Vel,Desl,YawRate,LatAcc,Yaw,control,tiempo,Velmax,ddd] = ABSfcn(dT, pMC, WSS, IMU, MEM)
% ABSfcn summary:
%   This is the brake controller's script. Here you shall type the code of
%   your ABS controller. A template is given, you are free to follow it.

% ******************************* INPUTS: ******************************* %
% dT  ----> Controller's sampling time                                    %
% pMC ----> Master Cylinder pressure                                      %
% WSS ----> Wheel Speed Sensors, angular speed of wheels                  %
%           [w_FL w_FR w_RL w_RR]                                         %
% IMU ----> Inertial Measurement Unit                                     %
%           [LonAcc LatAcc YawRate]                                       %
% MEM ----> Memory input. Used to pass values of local variables of this  %
%           function to the next iteration. This input shall be used      %
%           along with the output 'MEM_out'                               %
%           Default: Empty column vector of length 50                     %
% **************************** INPUTS - END ***************************** %

% ******************************** HINT ********************************* %
% Begin by making sure MEM and MEM_out are working properly and you       %
% understand how they work. Check the example below. A wrong use of       %
% these interfaces may cause you some headaches.                          %
%                                                                         %
% Background: the way simulink and the 'MATLAB function' block work does  %
% not allow storage of local variables whithin the script/block.          %
% Therefore, it was necessary to find a way to pass such variables to the %
% next iteration. This is done by connecting an output port that contains %
% all these variables to an input port of the same block. In between      %
% those two ports it is required to connect a 'Unit Delay', which all it  %
% does is: receive whatever value is sent to it in simulation's time      %
% step K, hold it (the value), and then in time step K+1 send it out.     %
% Refer to simulink block 'BrakeController' to see the whole picture.     %
% ***************************** HINT - END ****************************** %

% MEMORY ALLOCATION (INPUT)
ekm1_FL=MEM(2);
ekm1_FR=MEM(3);
ekm1_RL=MEM(4);
ekm1_RR=MEM(5);
ukm1_FL=MEM(6);
ukm1_FR=MEM(7);
ukm1_RL=MEM(8);
ukm1_RR=MEM(9);
vFLm1=MEM(11);
vFRm1=MEM(12);
vRLm1=MEM(13);
vRRm1=MEM(14);
ekm2_FL=MEM(15);
ekm2_FR=MEM(16);
ekm2_RL=MEM(17);
ekm2_RR=MEM(18);
R_FL=MEM(19);
R_FR=MEM(20);
R_RL=MEM(21);
R_RR=MEM(22);
Yawm1=MEM(35);
sm1_FL=MEM(36);
sm1_FR=MEM(37);
sm1_RL=MEM(38);
sm1_RR=MEM(39);
tm1=MEM(40);
VMm1=MEM(41);
dm1=MEM(42);

% CONSTANTS
x=-0.8:0.01:0.2;
y=0:0.01:1;

% SIGNAL PROCESSING

LonAcc=IMU(1,1);
LatAcc=IMU(2,1);
YawRate=IMU(3,1);

w_FL=WSS(1,1);
w_FR=WSS(2,1);
w_RL=WSS(3,1);
w_RR=WSS(4,1);

%V=wr    
R_FL=0.37;
R_FR=0.37;
R_RL=0.37;
R_RR=0.37;
    


%integral LonAcc
v_FL=(0.005*LonAcc)+vFLm1;
v_FR=(0.005*LonAcc)+vFRm1;
v_RL=(0.005*LonAcc)+vRLm1;
v_RR=(0.005*LonAcc)+vRRm1;


Yaw=(0.005*YawRate)+Yawm1;


% CONTROLLER


      if pMC>0 
            %Cálculo de deslizamiento
                s_FL=abs((v_FL-(w_FL*R_FL))/(v_FL)); 
      else
                s_FL=0.01; 
                  
      end
         
      
      if pMC>0 
            %Cálculo de deslizamiento
                s_FR=abs((v_FR-(w_FR*R_FR))/(v_FR)); 
                
      else 
                s_FR=0; 
      end
      

      if pMC>0 
            %Cálculo de deslizamiento 
                s_RL=abs((v_RL-(w_RL*R_RL))/(v_RL));
      else 
                s_RL=0;  
      end
      
      
      if pMC>0 
            %Cálculo de deslizamiento
                s_RR=abs((v_RR-(w_RR*R_RR))/(v_RR));
      else
                s_RR=0;    
      end


            
            %Error deslizamiento
                e_FL=(0.2-s_FL);
                e_FR=(0.2-s_FR);
                e_RL=(0.2-s_RL);
                e_RR=(0.2-s_RR);
                
             if e_FL>0.2
                 e_FL=0.2;
             else
                 if e_FL<-0.8
                     e_FL=-0.8;
                 end
             end
             
          
          %monitoreo activamiento control
          control=1;


% < Super duper badass ABS controller deslizamiento >

    %encontrando posición del error leido dentro del vector de error dado
    n_FL=find(abs (x - (round(e_FL*100)/100)) < 0.005);%2 * eps (e_FL*100)/100);
    n_FR=find(abs (x - (round(e_FR*100)/100)) < 0.005);%2 * eps (e_FR*100)/100);
    n_RL=find(abs (x - (round(e_RL*100)/100)) < 0.005);%2 * eps (e_RL*100)/100);
    n_RR=find(abs (x - (round(e_RR*100)/100)) < 0.005);%2 * eps (e_RR*100)/100);

%FUSIFICANDO PARA FL
    u_FL = Fuzzyc(x, y, n_FL);
% end
            
%FUSIFICANDO PARA FR
    u_FR = Fuzzyc(x, y, n_FR);
% end

%FUSIFICANDO PARA RL
    u_RL = Fuzzyc(x, y, n_RL);
% end

%FUSIFICANDO PARA RR
    u_RR = Fuzzyc(x, y, n_RR);
% end
     
%datos interfaz
    AAA=0.005;

    if pMC>0 && v_FL>0.65   %tolerancia para estabilización del tiempo
        tiempo=tm1+AAA;
        ddd=v_FL*0.005+dm1;
    else
        tiempo=tm1;
        ddd=dm1;
    end
    
    if tiempo==0.005
        Velmax=v_FL;
    else
        Velmax=VMm1;
    end
    


%control de yaw
    
if Yaw<-0.02
    u_RR=0.575*u_RR;
    u_FR=0.575*u_FR;
        
end



%Retorno relación directa PMC
if (v_FL<0.65 || v_FR<0.65 ||  v_RL<0.65 || v_RR<0.65)
    u_FL=1;
    u_FR=1;
    u_RL=1;
    u_RR=1;    
end    

pWhl=[(pMC*u_FL) (pMC*u_FR) (pMC*u_RL) (pMC*u_RR)]';




% MEMORY ALLOCATION (OUTPUT)
MEM_out    = zeros(50,1);      % Declaration (do not delete)

MEM_out(2)=e_FL;
MEM_out(3)=e_FR;
MEM_out(4)=e_RL;
MEM_out(5)=e_RR;
MEM_out(6)=u_FL;
MEM_out(7)=u_FR;
MEM_out(8)=u_RL;
MEM_out(9)=u_RR;
MEM_out(11)=v_FL;
MEM_out(12)=v_FR;
MEM_out(13)=v_RL;
MEM_out(14)=v_RR;
MEM_out(15)=ekm1_FL;
MEM_out(16)=ekm1_FL;
MEM_out(17)=ekm1_FL;
MEM_out(18)=ekm1_FL;
MEM_out(19)=R_FL;
MEM_out(20)=R_FR;
MEM_out(21)=R_RL;
MEM_out(22)=R_RR;
MEM_out(35)=Yaw;
MEM_out(36)=s_FL;
MEM_out(37)=s_FR;
MEM_out(38)=s_RL;
MEM_out(39)=s_RR;
MEM_out(40)=tiempo;
MEM_out(41)=Velmax;
MEM_out(42)=ddd;


%monitoreo variables calculadas
Vel=[v_FL v_FR v_RL v_RR];
Desl=[s_FL s_FR s_RL s_RR];

    function def = Fuzzyc(x, y, ix0)
%funciones de membresía error (entrada)
            %Eng
            a=-0.7; b=-0.4;
            FEng=(x>a&x<=b).*((b-x)/(b-a))+(x<=a).*(1);

            %Enm
            c=-0.65; d=-0.4; e=-0.3;
            FEnm= (x>=c&x<=d).*((x-c)/(d-c))+(x>d&x<=e).*((e-x)/(e-d));

            %Enp
            f=-0.35; g=-0.2; h=0;
            FEnp= (x>=f&x<=g).*((x-f)/(g-f))+(x>g&x<=h).*((h-x)/(h-g));

            %Ec
            i=-0.1; j=0;    k=0.1;
            FEc= (x>=i&x<=j).*((x-i)/(j-i))+(x>j&x<=k).*((k-x)/(k-j));

            %Epp
            l=0;    m=0.1; 
            FEpp=(x>l&x<m).*((x-l)/(m-l))+(x>=m).*(1);

   %funciones de membresía presión (salida)
            %nPmc
            A=0.01;  B=0.02;
            FnPmc=(y>A&y<=B).*((B-y)/(B-A))+(y<=A).*(1);

            %pocaPmc
            C=0; D=0.15;  E=0.25;
            FpPmc= (y>=C&y<=D).*((y-C)/(D-C))+(y>D&y<=E).*((E-y)/(E-D));

            %mPmc
            F=0.25; G=0.4;  H=0.65;
            FmPmc= (y>=F&y<=G).*((y-F)/(G-F))+(y>G&y<=H).*((H-y)/(H-G));

            %casiPmc
            I=0.65; J=0.75;  K=0.95;
            FcPmc= (y>=I&y<=J).*((y-I)/(J-I))+(y>J&y<=K).*((K-y)/(K-J));

            %Pmc
            L=0.9;  M=0.95;
            FPmc=(y>L&y<M).*((y-L)/(M-L))+(y>=M).*(1);

    % FUSIFICACIÓN E INFERENCIA

            %Eng -> nPmc
            u1 = FEng(ix0);
            if u1 > 0
                iFnPmc = FnPmc;
                for z = 1:length(iFnPmc)
                    if iFnPmc(z) > u1
                        iFnPmc(z) = u1;
                    end
                end
            else
                iFnPmc = zeros(1,length(x));
            end

            %Enm -> pPmc
            u2 = FEnm(ix0);
            if u2 > 0
                iFpPmc = FpPmc;
                for z = 1:length(iFpPmc)
                    if iFpPmc(z) > u2
                        iFpPmc(z) = u2;
                    end
                end
            else
                iFpPmc = zeros(1,length(x));
            end

            %Enp -> mPmc
            u3 = FEnp(ix0);
            if u3 > 0
                iFmPmc = FmPmc;
                for z = 1:length(iFmPmc)
                    if iFmPmc(z) > u3
                        iFmPmc(z) = u3;
                    end
                end
            else
                iFmPmc = zeros(1,length(x));
            end

            %Ec -> FcPmc
            u4 = FEc(ix0);
            if u4 > 0
                iFcPmc = FcPmc;
                for z = 1:length(iFcPmc)
                    if iFcPmc(z) > u4
                        iFcPmc(z) = u4;
                    end
                end
            else
                iFcPmc = zeros(1,length(x));
            end

            %Epp -> FPmc
            u5 = FEpp(ix0);
            if u5 > 0
                iFPmc = FPmc;
                for z = 1:length(iFPmc)
                    if iFPmc(z) > u5
                        iFPmc(z) = u5;
                    end
                end
            else
                iFPmc = zeros(1,length(x));
            end

            %UNIÓN 
            union = zeros(1,length(x));
            for z = 1:length(union)
                if iFnPmc(z) > union(z)
                    union(z) = iFnPmc(z);
                end
                if iFpPmc(z) > union(z)
                    union(z) = iFpPmc(z);
                end
                if iFmPmc(z) > union(z)
                    union(z) = iFmPmc(z);
                end
                if iFcPmc(z) > union(z)
                    union(z) = iFcPmc(z);
                end
                if iFPmc(z) > union(z)
                    union(z) = iFPmc(z);
                end
            end

            %DEFUZZIFICACIÓN
            def = sum(y.*union)/sum(union);
    end

end


