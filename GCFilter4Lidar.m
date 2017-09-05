%% Description
% Y = GCFilter4Lidar(X,N) returns the output of the order N, one dimensional
% curvature filtering of X. 
%
% 文件名：GCFilter4Lidar.m
% 功  能：高斯曲率滤波，适用于单线激光雷达数据。
% 作  者：Jie Li @ 2017-01-04 @NUST
% 邮  箱：JieLi_cn@163.com

%% Syntax
% y = GCFilter4Lidar(x, y)

%% Implement
function y = GCFilter4Lidar(u)
sizu = size(u);
N = sizu(1); %帧数，N必须至少为3
M = sizu(2); %一帧所包含数据的数量
y = u;%直接复制，省去边界处理时再次复制

d = zeros(8,1);
for i = 2:N-1 %从第二帧开始处理
    for j = 2:M-1
        d(1) = (u(i-1,j)+u(i+1,j))*0.5 - u(i,j);
        d(2) = (u(i,j-1)+u(i,j+1))*0.5 - u(i,j);
        d(3) = (u(i-1,j-1)+u(i+1,j+1))*0.5 - u(i,j);
        d(4) = (u(i-1,j+1)+u(i+1,j-1))*0.5 - u(i,j);
        d(5) = u(i-1,j) + u(i,j-1) - u(i-1,j-1) - u(i,j);
        d(6) = u(i-1,j) + u(i,j+1) - u(i-1,j+1) - u(i,j);
        d(7) = u(i,j-1) + u(i+1,j) - u(i+1,j-1) - u(i,j);
        d(8) = u(i,j+1) + u(i+1,j) - u(i+1,j+1) - u(i,j);
        [~,m] = min(abs(d));
        y(i,j) = u(i,j)+ d(m);
    end
end

disp('Done');
% Initialize y with the correct dimension
%y = zeros(siz);

% Reshape x into the right dimension.
%[x, nshifts] = shiftdim(x);

% Call medfilt1D (vector)
% for i = 1:prod(siz(2:end)),
% 	y(:,i) = GCFilter1D(x(:,i),n);
% end

% Convert y to the original shape of x
%y = shiftdim(y, -nshifts);


%-------------------------------------------------------------------
%                       Local Function
%-------------------------------------------------------------------
function y = GCFilter1D(x,n)
%GCFilter1D:  One dimensional curvature filter.
% Inputs:
%   x     - vector
%   n     - order of the filter

nx = length(x);
if rem(n,2)~=1    % n even
    m = n/2;
else
    m = (n-1)/2;
end
X = [zeros(m,1); x; zeros(m,1)];
y = zeros(nx,1);

for i=1:nx
    y(i) = (X(i) + X(i+n-1))/2;
end




