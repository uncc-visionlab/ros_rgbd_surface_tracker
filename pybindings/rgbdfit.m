
f=530;
width=640;
height=480;
cx = (width-1)/2
cy = (height-1)/2
x1d=linspace(0,width-1,width)
y1d=linspace(0,height-1,height)
tanX = (x1d - cx)./f;
tanY = (y1d - cy)./f;
surface = [-0.1 0.2 0.5 -2];
surface = surface./norm(surface(1:3));
normal = surface(1:3);
pt_in_plane = [1 1 1];
pt_in_plane(3) = (-surface(4) - dot(pt_in_plane(1:2),surface(1:2)))/surface(3);

ray_origin = [0 0 0];
dot([pt_in_plane 1],surface)
depth=zeros(height,width);
for x=1:width
    for y=1:height
        ray_dir = [tanX(x),tanY(y),1];
        ray_dir = ray_dir./norm(ray_dir);
        denom = dot(normal, ray_dir);
        t = dot((pt_in_plane - ray_origin), normal) / denom;
        pt = ray_origin + t * ray_dir;
        depth(y,x) = pt(3);
    end
end
imshow(depth,[0,12])

pts3 = zeros(height,width,3);
for x=1:width
    for y=1:height
        offset = y*width + x;
        pts3(y,x,:) = depth(y,x)*[ ((x-1)-cx)/f, ((y-1)-cy)/f, 1];
    end
end
x3 = reshape(pts3(:,:,1),width*height,1);
y3 = reshape(pts3(:,:,2),width*height,1);
z3 = reshape(pts3(:,:,3),width*height,1);
plot3(x3,y3,z3,'o','MarkerSize',0.1);

iZ = 1.0./depth;
hwinsize=2;
%for x=1:width
%    for y=1:height
for x=320:320
    for y=240:240
        minX=max(x-hwinsize,1);
        maxX=min(x+hwinsize,width);
        minY=max(y-hwinsize,1);
        maxY=min(y+hwinsize,height);
        numX = numel(minX:maxX);
        numY = numel(minY:maxY);
        sum_TanX = numY*sum(tanX(minX:maxX));
        %sum_TanXTanY3 = sum(sum(tanY(minY:maxY))*tanX(minX:maxX));
        %sum_TanXTanY2 = sum(sum(tanY(minY:maxY)'*tanX(minX:maxX)));
        sum_TanXTanY = sum(sum((ones(numX,1)*tanY(minY:maxY))'.*(ones(numY,1)*tanX(minX:maxX))));
        sum_TanY = numX*sum(tanY(minY:maxY));
        sum_TanX2 = numY*sum(tanX(minX:maxX).*tanX(minX:maxX));
        sum_TanY2 = numX*sum(tanY(minY:maxY).*tanY(minY:maxY));
        sum_iZ = sum(sum(iZ(minY:maxY,minX:maxX)));
        sum_iZ2 = sum(sum(iZ(minY:maxY,minX:maxX).*iZ(minY:maxY,minX:maxX)));
        sum_TanX_div_iZ = sum(sum((ones(numY,1)*tanX(minX:maxX)).*iZ(minY:maxY,minX:maxX)));
        sum_TanY_div_iZ = sum(sum((ones(numX,1)*tanY(minY:maxY))'.*iZ(minY:maxY,minX:maxX)));
        MtM = [sum_TanX2 sum_TanXTanY sum_TanX sum_TanX_div_iZ;
            sum_TanXTanY sum_TanY2 sum_TanY sum_TanY_div_iZ;
            sum_TanX sum_TanY numX*numY sum_iZ;
            sum_TanX_div_iZ sum_TanY_div_iZ sum_iZ sum_iZ2];
        [vecs, vals] = eig(MtM);
        coeffs = vecs(:,1)./norm(vecs(1:3,1))

        MtM2 = [sum_TanX2 sum_TanXTanY sum_TanX;
            sum_TanXTanY sum_TanY2 sum_TanY;
            sum_TanX sum_TanY numX*numY];
        Mtb = -[sum_TanX_div_iZ; sum_TanY_div_iZ; sum_iZ];
        coeffs2a = inv(MtM2)*Mtb;
        coeffs2(4) = 1/norm(coeffs2a);
        coeffs2(1:3) = coeffs2a(1:3)./norm(coeffs2a);
        
        sum_cX2 = sum(sum(pts3(minY:maxY,minX:maxX,1).*iZ(minY:maxY,minX:maxX).*pts3(minY:maxY,minX:maxX,1).*iZ(minY:maxY,minX:maxX)));
        sum_cXY = sum(sum(pts3(minY:maxY,minX:maxX,1).*iZ(minY:maxY,minX:maxX).*pts3(minY:maxY,minX:maxX,2).*iZ(minY:maxY,minX:maxX)));
        sum_X2 = sum(sum(pts3(minY:maxY,minX:maxX,1).*pts3(minY:maxY,minX:maxX,1)));
        sum_XY = sum(sum(pts3(minY:maxY,minX:maxX,1).*pts3(minY:maxY,minX:maxX,2)));
        sum_XZ = sum(sum(pts3(minY:maxY,minX:maxX,1).*pts3(minY:maxY,minX:maxX,3)));
        sum_X = sum(sum(pts3(minY:maxY,minX:maxX,1)));
        sum_Y2 = sum(sum(pts3(minY:maxY,minX:maxX,2).*pts3(minY:maxY,minX:maxX,2)));
        sum_YZ = sum(sum(pts3(minY:maxY,minX:maxX,2).*pts3(minY:maxY,minX:maxX,3)));
        sum_Z2 = sum(sum(pts3(minY:maxY,minX:maxX,3).*pts3(minY:maxY,minX:maxX,3)));
        sum_Y = sum(sum(pts3(minY:maxY,minX:maxX,2)));
        sum_Z = sum(sum(pts3(minY:maxY,minX:maxX,3)));
        MtM3 = [sum_X2 sum_XY sum_XZ sum_X;
            sum_XY sum_Y2 sum_YZ sum_Y;
            sum_XZ sum_YZ sum_Z2 sum_Z;
            sum_X sum_Y sum_Z numX*numY];
        [vecs3,val3] = eig(MtM3);
        coeffs3 = vecs3(:,1)./norm(vecs3(1:3,1))
    
    end
end

coeffs'*depth(y,x)*[tanX(x); tanY(y); 1; iZ(y,x)]
coeffs2'*depth(y,x)*[tanX(x); tanY(y); 1; iZ(y,x)]
coeffs3'*depth(y,x)*[tanX(x); tanY(y); 1; iZ(y,x)]
