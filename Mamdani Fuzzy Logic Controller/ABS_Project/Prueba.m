e_FL=-0.0005; e_FR=0.1957; e_RL=-0.5478 ;e_RR=-0.7987;

x=-0.8:0.01:0.2;
y=0:0.01:1;

    %funciones de membresía error (entrada)

        %Eng
        a=-0.5;b=-0.3; 
        FEng=(x>a&x<=b).*((b-x)/(b-a))+(x<=a).*(1);
        
        %Enm
        c=-0.5; d=-0.3 ;e=-0.1;
        FEnm= (x>=c&x<=d).*((x-c)/(d-c))+(x>d&x<=e).*((e-x)/(e-d));
        
        %Enp
        f=-0.3; g=-0.15; h=0;
        FEnp= (x>=f&x<=g).*((x-f)/(g-f))+(x>g&x<=h).*((h-x)/(h-g));
        
        %Ec
        i=-0.1; j=0;k=0.1;
        FEc= (x>=i&x<=j).*((x-i)/(j-i))+(x>j&x<=k).*((k-x)/(k-j));
        
        %Epp
        l=0;m=0.1; 
        FEpp=(x>l&x<m).*((x-l)/(m-l))+(x>=m).*(1);

   %funciones de membresía presión (salida)
     
        %nPmc
        A=0.1;B=0.15;
        FnPmc=(y>A&y<=B).*((B-y)/(B-A))+(y<=A).*(1);
        
        %pocaPmc
        C=0.25; D=0.3;E=0.35;
        FpPmc= (y>=C&y<=D).*((y-C)/(D-C))+(y>D&y<=E).*((E-y)/(E-D));
        
        %mPmc
        F=0.45; G=0.5;H=0.55;
        FmPmc= (y>=F&y<=G).*((y-F)/(G-F))+(y>G&y<=H).*((H-y)/(H-G));
        
        %casiPmc
        I=0.65; J=0.7;K=0.75;
        FcPmc= (y>=I&y<=J).*((y-I)/(J-I))+(y>J&y<=K).*((K-y)/(K-J));
        
        %Pmc
        L=0.9;M=1; 
        FPmc=(y>L&y<M).*((y-L)/(M-L))+(y>=M).*(1);
        
        
%encontrando posición del error leido dentro del vector de error dado
n_FL=find(abs (x - (round(e_FL*100)/100)) <2 * eps (e_FL*100)/100);
n_FR=find(abs (x - (round(e_FR*100)/100)) <2 * eps (e_FR*100)/100);
n_RL=find(abs (x - (round(e_RL*100)/100)) <2 * eps (e_RL*100)/100);
n_RR=find(abs (x - (round(e_RR*100)/100)) <2 * eps (e_RR*100)/100);



%Fusificando FL
M1_FL=min(FpPmc,FEng(n_FL));
M2_FL=min(FmPmc,FEnm(n_FL));
M3_FL=min(FcPmc,FEnp(n_FL));
M4_FL=min(FPmc,FEc(n_FL));
M5_FL=min(FPmc,FEpp(n_FL));
M_FL=max(M1_FL,(max(M2_FL,max(M3_FL,max(M4_FL,M5_FL)))));

%Fusificando FR
M1_FR=min(FpPmc,FEng(n_FR));
M2_FR=min(FmPmc,FEnm(n_FR));
M3_FR=min(FcPmc,FEnp(n_FR));
M4_FR=min(FPmc,FEc(n_FR));
M5_FR=min(FPmc,FEpp(n_FR));
M_FR=max(M1_FR,(max(M2_FR,max(M3_FR,max(M4_FR,M5_FR)))));

%Fusificando RL
M1_RL=min(FpPmc,FEng(n_RL));
M2_RL=min(FmPmc,FEnm(n_RL));
M3_RL=min(FcPmc,FEnp(n_RL));
M4_RL=min(FPmc,FEc(n_RL));
M5_RL=min(FPmc,FEpp(n_RL));
M_RL=max(M1_RL,(max(M2_RL,max(M3_RL,max(M4_RL,M5_RL)))));

%Fusificando RR
M1_RR=min(FpPmc,FEng(n_RR));
M2_RR=min(FmPmc,FEnm(n_RR));
M3_RR=min(FcPmc,FEnp(n_RR));
M4_RR=min(FPmc,FEc(n_RR));
M5_RR=min(FPmc,FEpp(n_RR));
M_RR=max(M1_RR,(max(M2_RR,max(M3_RR,max(M4_RR,M5_RR)))));


%Defusificando
u_FL=sum(y.*M_FL)/sum(M_FL)
u_FR=sum(y.*M_FR)/sum(M_FR)
u_RL=sum(y.*M_RL)/sum(M_RL)
u_RR=sum(y.*M_RR)/sum(M_RR)

x_FL=defuzz(y,M_FL,'centroid')
x_FL=defuzz(y,M_FR,'centroid')
x_FL=defuzz(y,M_RL,'centroid')
x_FL=defuzz(y,M_RR,'centroid')