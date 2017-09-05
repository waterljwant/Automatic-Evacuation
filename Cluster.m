
%% 聚类，并给出数字编号
function [Y1, halo,NCLUST, H,index] = Cluster(ladar_data_pole)
n = size(ladar_data_pole);

%%  数据预处理 ：一帧雷达数据转平面直角坐标
X = zeros(n(2),2);
for i=1:n(2)
    if ladar_data_pole(i) > 10000
        ladar_data_pole(i) = 0;
    end
    X(i,1) = ladar_data_pole(i)*cos((225-0.25*(i-1))*pi/180);
    X(i,2) = ladar_data_pole(i)*sin((225-0.25*(i-1))*pi/180);
end

%% 数据预处理 ： 剔除无效点
% 由于雷达数据中存在大量超过测量范围的无效点，将无效点距离强置为0
A1=sum(abs(X'));
index=find(A1==0);
X(index,:)=[];

%X = X(X~=0);

%% 计算距离矩阵
dist_mat = pdist2(X,X);

%% 截断距离 dc, 计算密度 rho
ND = length(X);
rho = zeros(1,ND);
for i=1:ND
    rho(i)=0.;
end
% ----------------------
% Gaussian kernel
% ----------------------
% xx = pdist(X);
% percent=5;
% fprintf('average percentage of neighbours (hard coded): %5.6f\n', percent);
% position=round(length(xx) * percent / 100);
% sda=sort(xx);
% dc=sda(position);

% fprintf('Computing Rho with gaussian kernel of radius: %12.6f\n', dc);
% for i=1:ND-1
%   for j=i+1:ND
%      rho(i)=rho(i)+exp(-(dist_mat(i,j)/dc)*(dist_mat(i,j)/dc));
%      rho(j)=rho(j)+exp(-(dist_mat(i,j)/dc)*(dist_mat(i,j)/dc));
%   end
% end

% ----------------------
% "Cut off" kernel
% ----------------------
for i=1:ND-1
    for j=i+1:ND
        dc = 0.2*min(ladar_data_pole(i),ladar_data_pole(j));
        if dc<10
            dc = 43;% 单位mm
        end
        if (dist(i,j)<dc)
            rho(i)=rho(i)+1.;
            rho(j)=rho(j)+1.;
        end
    end
end

%% 计算距离 delta
maxd=max(max(dist_mat));

[rho_sorted,ordrho]=sort(rho,'descend');
delta(ordrho(1))=-1.;
nneigh(ordrho(1))=0;

for ii=2:ND
    delta(ordrho(ii))=maxd;
    for jj=1:ii-1
        if(dist_mat(ordrho(ii),ordrho(jj))<delta(ordrho(ii)))
            delta(ordrho(ii))=dist_mat(ordrho(ii),ordrho(jj));
            nneigh(ordrho(ii))=ordrho(jj);
        end
    end
end
delta(ordrho(1))=max(delta(:));

%% 绘制决策图 Decision Graph
% figure(3);
% tt=plot(rho(:),delta(:),'o','MarkerSize',5,'MarkerFaceColor','k','MarkerEdgeColor','k');
% title ('Decision Graph','FontSize',15.0)
% xlabel ('\itm','FontSize',10.0)
% ylabel ('\delta','FontSize',12.0)

rhomin=0.1;
deltamin=580;

NCLUST=0;%类别数目
for i=1:ND
    cl(i)=-1;
end
for i=1:ND
    if ( (rho(i)>rhomin) && (delta(i)>deltamin))
        NCLUST=NCLUST+1;
        cl(i)=NCLUST;
        icl(NCLUST)=i;
    end
end
fprintf('NUMBER OF CLUSTERS: %i \n', NCLUST);
disp('Performing assignation')

%assignation
for i=1:ND
    if (cl(ordrho(i))==-1)
        cl(ordrho(i))=cl(nneigh(ordrho(i)));
    end
end
%halo
for i=1:ND
    halo(i)=cl(i);
end
if (NCLUST>1)
    for i=1:NCLUST
        bord_rho(i)=0.;
    end
    for i=1:ND-1
        for j=i+1:ND
            if ((cl(i)~=cl(j))&& ((dist_mat(i,j)<=dc)))
                rho_aver=(rho(i)+rho(j))/2.;
                if (rho_aver>bord_rho(cl(i)))
                    bord_rho(cl(i))=rho_aver;
                end
                if (rho_aver>bord_rho(cl(j)))
                    bord_rho(cl(j))=rho_aver;
                end
            end
        end
    end
    for i=1:ND
        if (rho(i)<bord_rho(cl(i)))
            halo(i)=0;
        end
    end
end
for i=1:NCLUST
    nc=0;
    nh=0;
    for j=1:ND
        if (cl(j)==i)
            nc=nc+1;
        end
        if (halo(j)==i)
            nh=nh+1;
        end
    end
    fprintf('CLUSTER: %i CENTER: %i ELEMENTS: %i CORE: %i HALO: %i \n', i,icl(i),nc,nh,nc-nh);
end

%cmap=colormap;
%cmap=colormap('Lines');
cmap=colormap('Jet');
%cmap=colormap('HSV');

%% 显示聚类中心点
% figure(2)
% for i=1:NCLUST
%     ic=int8((i*64.)/(NCLUST*1.));
%     hold on
%     plot(rho(icl(i)),delta(icl(i)),'o','MarkerSize',8,'MarkerFaceColor',cmap(ic,:),'MarkerEdgeColor',cmap(ic,:));
% end

%% 显示聚类结果
figure(1);
cla();  %清除figure
disp('Performing 2D nonclassical multidimensional scaling')
Y1 = mdscale(dist_mat, 2, 'criterion','metricstress');
plot(Y1(:,1),Y1(:,2),'o','MarkerSize',2,'MarkerFaceColor','k','MarkerEdgeColor','k');
%title ('2D Nonclassical multidimensional scaling','FontSize',15.0)
xlabel ('X (mm)','FontSize',10.0)
ylabel ('Y (mm)','FontSize',10.0)
for i=1:ND
    A(i,1)=0.;
    A(i,2)=0.;
end
EndPoint = zeros(1,NCLUST);
for i=1:NCLUST
    nn=0;
    ic=int8((i*64.)/(NCLUST*1.));
    for j=1:ND
        if (halo(j)==i)
            nn=nn+1;
            A(nn,1)=Y1(j,1);
            A(nn,2)=Y1(j,2);
        end
    end
    hold on
    plot(A(1:nn,1),A(1:nn,2),'o','MarkerSize',2,'MarkerFaceColor',cmap(ic,:),'MarkerEdgeColor',cmap(ic,:));
    %记录每个类的结束点
    if i >1
        EndPoint(i) = EndPoint(i-1)+nn;
    else
        EndPoint(i) = nn;
    end
    H(EndPoint(i)-nn+1:EndPoint(i),1) = A(1:nn,1);
    H(EndPoint(i)-nn+1:EndPoint(i),2) = A(1:nn,2);
    H(EndPoint(i)-nn+1:EndPoint(i),3) = i; 
    
    % 为了将标号位置画的好看，手动调整位置
%     switch i
%         case {1,2,8}
%             tempax = 400;tempay=200;
%             tempbx = 300;tempby=200;
%         case {3,4}
%             tempax = 300;tempay=700;
%             tempbx = 240;tempby=700;
%         case {5,9}
%             tempax = 160;tempay=450;
%             tempbx = 80;tempby=450;
%         case 6
%             tempax = 500;tempay=-100;
%             tempbx = 300;tempby=-100;
%         case {13,14}
%             tempax = 380;tempay=-50;
%             tempbx = 100;tempby=-50;
%         case 10
%             tempax = -100;tempay=700;
%             tempbx = -330;tempby=700;
%         case 11
%             tempax = 180;tempay=600;
%             tempbx = -120;tempby=600;
%         case 12
%             tempax = -400;tempay=300;
%             tempbx = -640;tempby=300;
%         case 15
%             tempax = 0;tempay=-600;
%             tempbx = -200;tempby=-600;
%         otherwise
%             tempax = 160;tempay=360;
%             tempbx = 20;tempby=360;
%     end
%     % 画带圆圈的编号
%     h=scatter(A(1,1)+tempax,A(1,2)+tempay,'k','SizeData',200);
%     text(A(1,1)+tempbx,A(1,2)+tempby,num2str(i));
end
