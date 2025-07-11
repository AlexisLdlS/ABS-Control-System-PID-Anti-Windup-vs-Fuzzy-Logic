function [pWhl, MEM_out,Vel,Desl,YawRate,LatAcc,Yaw,control,t,Velmax,d] = ABSfcn(dT, pMC, WSS, IMU, MEM)
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
    %control pid deslizamiento
        kp=0.9;
        ti=0.3;
        td=0;
        Ts=dT;


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
                s_FL=0; 
                  
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
                
            %error omega L
          if pMC>0



                E_FL=((v_FL/0.37)-Yaw);
                E_FR=((v_FR/0.37)-Yaw);
                E_RL=((v_RL/0.37)-Yaw);
                E_RR=((v_RR/0.37)-Yaw);
                
                
          else
           
                E_FL=0;
                E_FR=0;
                E_RL=0;
                E_RR=0;
                
          end
              
control=1;


% < Super duper badass ABS controller deslizamiento >
   
   
    
    %Calculo control pid digital
        q0=kp*(1+Ts/(2*ti)+td/Ts);
        q1=-kp*(1-Ts/(2*ti)+(2*td)/Ts);
        q2=(kp*td)/Ts;
    
    %limites máximos
        umax=pMC;  

    
if s_FL==0.2 
        u_FL=1;  
else
        %-----------antiwindup FL
        u_FL= ukm1_FL+q0*e_FL + q1*ekm1_FL + q2*ekm2_FL;
        if (u_FL >= umax)    
            u_FL = umax;
        end
        
end     


if s_FR==0.2 
        u_FR=1;  
else
        %-----------antiwindup FR
        u_FR= ukm1_FR+q0*e_FR + q1*ekm1_FR + q2*ekm2_FR;
        if (u_FR >= umax)    
            u_FR = umax;
        end
        
end     



if s_RL==0.2 
        u_RL=1;  
else
        %-----------antiwindup RL
        u_RL= ukm1_RL+q0*e_RL + q1*ekm1_RL + q2*ekm2_RL;
        if (u_RL >= umax)    
            u_RL = umax;
        end
        
end     
        
if s_RR==0.2 
        u_RR=1;  
else
        %-----------antiwindup RR
        u_RR= ukm1_RR+q0*e_RR + q1*ekm1_RR + q2*ekm2_RR;
        if (u_RR >= umax)    
            u_RR = umax;
        end
        
end     


     
%datos interfaz
    a=0.005;

    if pMC>0 && v_FL>0
        t=tm1+a;
        d=v_FL*0.005+dm1;
    else
        t=tm1;
        d=dm1;
    end
    
    if t==0.005
        Velmax=v_FL;
    else
        Velmax=VMm1;
    end
    
%Velocidad desactivamiento de ABS
if (v_FL<0.5 || v_FR<0.5 ||  v_RL<0.5 || v_RR<0.5)
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
MEM_out(40)=t;
MEM_out(41)=Velmax;
MEM_out(42)=d;




%monitoreo variables calculadas
Vel=[v_FL v_FR v_RL v_RR];
Desl=[s_FL s_FR s_RL s_RR];

end


