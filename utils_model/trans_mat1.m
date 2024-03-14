function [T] = trans_mat1(var,l,q,p)
%%retruns 3D transformation matrix from frame q to p for a segment represented by kappa_x, kappa_y and epsilon (3n parameters)
%based on Rao, P., Peyron, Q., Lilge, S. and Burgner-Kahrs, J., 2021. How to model tendon-driven continuum robots and benchmark modelling performance. Frontiers in Robotics and AI, 7, p.630245.

T=eye(4);
if q<p
    error('Some error in rotation matrix indices')
else
    for iter=p+1:1:q
        beta=var(3*iter-2);
        gamma=var(3*iter-1);
        epsi=var(3*iter);
        k=sqrt(beta^2+gamma^2);
        phi=atan2(gamma,beta);

        if size(l,2)==1 &&size(l,1)==1
            theta=k*l;
        else
            theta=k*l(iter);
        end
        p_i=[(1-cos(theta))*cos(phi)/k;(1-cos(theta))*sin(phi)/k;sin(theta)/k]; %coordinates of disk i
        Rz=[cos(phi) -sin(phi) 0;
            sin(phi) cos(phi) 0;
            0 0 1];
        Ry=[cos(theta) 0 sin(theta);
            0 1 0;
            -sin(theta) 0 cos(theta)];
        Rz2=[cos(epsi-phi) -sin(epsi-phi) 0;
            sin(epsi-phi) cos(epsi-phi) 0;
            0 0 1];
        T=T*[Rz*Ry*Rz2 p_i;
            0 0 0 1];
        %Can be sped up by pre-calculating Rz*Ry*Rz2
    end
end
end
