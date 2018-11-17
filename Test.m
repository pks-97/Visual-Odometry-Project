function[d]=Test()
img1=rgb2gray(imread('frame0001.jpg'));
currCameraPose=eye(4,4);
prevCameraPose=eye(4,4);
for i=2:2
    l=int2str(i);
    if(i<10)
        c=strcat('frame000',l,'.jpg');
    elseif(i>=10 && i<=99)
        c=strcat('frame00',l,'.jpg');
    else
        c=strcat('frame0',l,'.jpg');
    end    
    img2=rgb2gray(imread(c));
    [a1,b1]=findCorrespondences(img1,img2);
    [x1,y1]=size(a1);
    [x2,y2]=size(b1);
    maxIterations=20;
    Final_Points=randperm(y1,8);
    F_best=ones(3,3);
    max_inlier_count=0;
    for j=1:maxIterations
        model_points=randperm(y1,8);
        A=ones(3,8);
        B=ones(3,8);
        for k=1:8
            A(:,k)=a1(:,model_points(k));
            B(:,k)=b1(:,model_points(k));
        end
        F_est=myEstimateFundamentalMatrix(A,B);
        inlier_count=0;
        for e=1:y1
            res=ismember(e,model_points);
            if(res==1)
                epipolar=(b1(:,e)')*F_est*(a1(:,e));
                if(abs(epipolar)<=0.01)
                    inlier_count=inlier_count+1;
                end
            end
        end
        if(inlier_count > max_inlier_count)
          max_inlier_count=inlier_count;
          Final_Points=model_points;
          F_best=F_est;
        end
    end
K=[406.952636, 0.000000, 366.184147; 0.000000, 405.671292, 244.705127; 0.000000, 0.000000, 1.000000];    
E_best=K'*F_best*K; 
%we=ones(3,3);
[u,d,v]=svd(E_best);
%d(3,3)=0;
same=(d(1,1)+d(2,2))/2;
d(1,1)=same;
d(2,2)=same;
E_best=u*d*v';
    %we=d
end