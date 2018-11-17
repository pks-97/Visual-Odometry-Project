function pts_hat = algebraicTriangulation(x1, x2, P1, P2)

cube_img1 = x1;
cube_img2 = x2;

% ========================================================================
% TRIANGULATE POINT USING ALGEBRAIC METHOD
% ========================================================================

% estimated cube points in homogeneous 
pts_hat = zeros(4,size(x1,2));     

nPoints = size(cube_img1,2);
for i = 1: nPoints
    
    % form the normal system
    x1 = cube_img1(1,i);
    y1 = cube_img1(2,i);
    x2 = cube_img2(1,i);
    y2 = cube_img2(2,i);    
    A = [x2*P2(3,1)-P2(1,1) x2*P2(3,2)-P2(1,2) x2*P2(3,3)-P2(1,3) x2*P2(3,4)-P2(1,4);
         y2*P2(3,1)-P2(2,1) y2*P2(3,2)-P2(2,2) y2*P2(3,3)-P2(2,3) y2*P2(3,4)-P2(2,4); 
         x1*P1(3,1)-P1(1,1) x1*P1(3,2)-P1(1,2) x1*P1(3,3)-P1(1,3) x1*P1(3,4)-P1(1,4);
         y1*P1(3,1)-P1(2,1) y1*P1(3,2)-P1(2,2) y1*P1(3,3)-P1(2,3) y1*P1(3,4)-P1(2,4)          
         ];
    
    % solve the system to get [X Y Z W] 
    [U, D, V] = svd(A);
    d = diag(D);    
    [~, solCol] = min(d);       
    pts_hat(:,i) = V(:, solCol);        
        
end