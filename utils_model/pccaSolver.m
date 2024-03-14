function [x,lambda,exitflag] = pccaSolver(x_init,l_inter,n_disk,r_disk,inputTendon,p_tendon,cCenter,ab)
%%returns the solved values for [kappa_x] for each subsegment i
%Adapted from the model proposed in  K. P. Ashwin, S. Kanti, and A. Ghosal, “Profile and contact force
%estimation of cable-driven continuum robots in presence of obstacles,”
%Mechanism and Machine Theory, vol. 164, p. 104404, 2021.

fun = @optim_f;
x0 = x_init;
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
options = optimoptions('fmincon','Display','off','Algorithm','interior-point ','TolCon',1e-10,'FinDiffRelStep',1e-5,'TolFun',1e-15);
tic
% profile on -history
[x,fval,exitflag,output,lambda,grad,hessian]  = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,@fnonlin,options);
% profile report 
% profile off;
toc
%% solver

    function [c,ceq] = fnonlin(x)
        var=zeros(1,n_disk*3);
        var(1:3:end)=x;
        [pcoord,ltendon,ptcoord1,ptcoord2]=positionCalc(l_inter,n_disk,var,r_disk,p_tendon);
        dmp=[pcoord ptcoord1, ptcoord2];
        fx=obstacleFunc(cCenter,ab,0,dmp);
        fx(fx>1)=1;
     c =[-fx']; %inequality constraint - forcing points on the robot to lie outside each obstacle
    ceq = [inputTendon-ltendon(1)]; %equality constraint

    end

    function [res] = optim_f(x)
        res=sum((x*l_inter).^2); %objective function minimizing the angle of each subsegment
    end
end