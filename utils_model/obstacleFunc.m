function [fxArr] = obstacleFunc(cCenterArr,ab,plotoption,X,co)
%%returns the value of f(X) for a super ellipsoide 
%ab=[a,b,s] ,where s is the power of the ellipsoide
%draws the above obstacles if plotoption >0

 N=size(ab,1);
 n=size(X,2);
 fxArr=zeros(1,n*N);

     
 if plotoption>0
     for iter=1:1:N
         a=ab(iter,1); b=ab(iter,2);s=ab(iter,3);cCenter=cCenterArr(:,iter);
         theta=0:0.01:2*pi;
         r=a*b./(b^s*cos(theta).^s+a^s*sin(theta).^s).^(1/s);
         patch(r.*cos(theta)+cCenter(1),zeros(size(r)),r.*sin(theta)+cCenter(3),co);hold on;
     end
 else

         for iter=1:1:N         

             a=ab(iter,1); 
             b=ab(iter,2);
             s=ab(iter,3);
             cCenter=cCenterArr(iter,1:3)';
             fxArr((iter-1)*n+1:iter*n)=sum(((X-cCenter)./[a;1;b]).^s)-1;
         end
 end
end