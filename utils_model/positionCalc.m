function [pcoord,l_tendon,ptcoord1,ptcoord2] = positionCalc(l_inter,n_disk,var,r_disk,p_tendon)


% returns the coordinates of each disk for given var
% also the length of the tendon pulling it
%ptcoord : positions of tendon position at each disk
%pcood : position of backbone at each disk
%based on Rao, P., Peyron, Q., Lilge, S. and Burgner-Kahrs, J., 2021. How to model tendon-driven continuum robots and benchmark modelling performance. Frontiers in Robotics and AI, 7, p.630245.
pcoord=zeros(3, n_disk);
ptcoord1=zeros(3, 2*n_disk+1);
ptcoord2=zeros(3, 2*n_disk+1);
ptcoord1(:,1)=p_tendon(1:3,1);
ptcoord2(:,1)=p_tendon(1:3,2);
l_tendon=[0;0];
T_i=eye(4);
for ss_i=1:n_disk
    T_i=T_i*trans_mat1(var,l_inter,ss_i,ss_i-1);
    pt=T_i*p_tendon;
    pcoord(:,ss_i)=T_i(1:3,4);
    ptcoord1(:,ss_i+1)=pt(1:3,1);
    ptcoord2(:,ss_i+1)=pt(1:3,2);
    l_tendon=l_tendon+[sqrt(sum((ptcoord1(:,ss_i+1)-ptcoord1(:,ss_i)).^2));
    sqrt(sum((ptcoord2(:,ss_i+1)-ptcoord2(:,ss_i)).^2))];
    ptcoord1(:,ss_i+n_disk+1)=(ptcoord1(:,ss_i+1)+ptcoord1(:,ss_i))/2;
    ptcoord2(:,ss_i+n_disk+1)=(ptcoord2(:,ss_i+1)+ptcoord2(:,ss_i))/2;

end

end
