img1=rgb2gray(imread('frame0001.jpg'));
x4=[0;0;0;1];
currCameraPose=eye(4,4);
prevCameraPose=eye(4,4);
for i=2:707
    l=int2str(i);
    if(i<10)
        c=strcat('frame000',l,'.jpg');
    elseif(i>=10 && i<=99)
        c=strcat('frame00',l,'.jpg');
    else
        c=strcat('frame0',l,'.jpg');
    end    
    img2=rgb2gray(imread(c));
    [a1,b1,t1,t2]=findCorrespondences(img1,img2);
    [x1,y1]=size(a1);
    [x2,y2]=size(b1);
    maxIterations=10;
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
            if(res==0)
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
F_best=t2'*F_best*t1;
E_best=K'*F_best*K;    
if(rank(E_best)==3)
    [u,d,v]=svd(E_best);
    d(3,3)=0;
    same=(d(1,1) + d(2,2))/2;
    d(1,1)=same;
    d(2,2)=same;
    E_best=u*d*v';
else
    [u,d,v]=svd(E_best);
    same=(d(1,1) + d(2,2))/2;
    d(1,1)=same;
    d(2,2)=same;
    E_best=u*d*v';
end
[R,t]=decomposeEssentialMatrix(E_best,a1,b1,K);
currentcameraPose=[R,t;0,0,0,1];
currCameraPose=prevCameraPose*currentcameraPose;
%x4(1)=x4(1)/x4(4);
%x4(2)=x4(2)/x4(4);
%x4(3)=x4(3)/x4(4);
%x4(4)=x4(4)/x4(4);
%plot3(x4(1),x4(2),x4(3));
%hold on;
%x4=currCameraPose(1:4,4)+x4;
%X(i,1)=x4(1)/x4(4);
%X(i,2)=x4(2)/x4(4);
%X(i,3)=x4(3)/x4(4);
%X(i,4)=x4(4)/x4(4);
%X(i,:)=x4';
%plot3(x4(1),x4(2),x4(3));
CameraTrajectory(1:3,i)=currCameraPose(1:3,4);
prevCameraPose=currCameraPose;
img1=img2;
end
%X(1,1:3)=zeros(1,3);
%X(1,4)=1;
CameraTrajectory(1:3,1)=zeros(1,3);
plot3(CameraTrajectory(1,2:707),CameraTrajectory(2,2:707),CameraTrajectory(3,2:707));
%imshow(img2);
%x4=[0;0;0;1]
%for w=2:707
  %camera_frame1=CameraTrajectory(w)
  %transVector=camera_frame1(1:3,4);
  
  %plot3(CameraTrajectory(1,1:10),CameraTrajectory(2,1:10),CameraTrajectory(3,1:10));
%end