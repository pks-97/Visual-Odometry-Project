function F_est =myEstimateFundamentalMatrix(a1,b1)
A=ones(8,9);
for i=1:8
    A(i,1)=a1(1,i)*b1(1,i);
    A(i,2)=a1(1,i)*b1(2,i);
    A(i,3)=a1(1,i);
    A(i,4)=b1(1,i)*a1(2,i);
    A(i,5)=a1(2,i)*b1(2,i);
    A(i,6)=a1(2,i);
    A(i,7)=b1(1,i);
    A(i,8)=b1(2,i);
    A(i,9)=1;
end    
[U,D,V]=svd(A);
[~,solColV]=min(diag(D));
F_est=[V(1:3,solColV)';V(4:6,solColV)';V(7:9,solColV)'];
if(rank(F_est) ==3)
    [u,d,v]=svd(F_est);
    d(3,3)=0;
    %e=(d(1,1) + d(2,2))/2;
    %d(1,1)=e;
    %d(2,2)=e;
    F_est=u*d*v';
end