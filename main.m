clear all;
close all;

ladar_data_pole = load('ladar_data.txt');

%%  雷达数据滤波
% 原始数据，不做滤波处理
%ladar_data_pole = medfilt1(ladar_data_pole,5);% 中值滤波
ladar_data_pole = GCFilter4Lidar(ladar_data_pole);%自适应曲率滤波来改进效果

DBW = 535;
[NF, ND1] = size(ladar_data_pole);
for kk = 1:NF
    %  ------------------------聚类
    [X1,halo, NCLUST, H,index] = Cluster(ladar_data_pole(kk,:));
    
    fig = figure(2);
    cla();  %清除figure
    for i=1:ND1
        Y1(i,1) = ladar_data_pole(kk,i)*cos((225-0.25*(i-1))*pi/180);
        Y1(i,2) = ladar_data_pole(kk,i)*sin((225-0.25*(i-1))*pi/180);
    end
    Y1(index,:) = [];
    cmap=colormap('Jet');
    [~, ND2] = size(halo);
    countDB=0;
    for i=1:NCLUST%每一类进行逐个分析与识别
        nn=0;
        ic=int8((i*64.)/(NCLUST*1.));
        for j=1:ND2
            if (halo(j)==i)
                nn=nn+1;
                A(nn,1)=Y1(j,1);
                A(nn,2)=Y1(j,2);
            end
        end
        %hold on
        %plot(A(1:nn,1),A(1:nn,2),'o','MarkerSize',2,'MarkerFaceColor',cmap(ic,:),'MarkerEdgeColor',cmap(ic,:));
        
        tem_x = A(1:nn,1);
        tem_y = A(1:nn,2);
        hold on
        plot(tem_x, tem_y, '.','color','b');
        
         %-----------------------------------------筛选第一步：个数判断
        pointN = nn;
        if (pointN>15)&&(pointN<260)
            %-----------------------------------------筛选第二步：线性程度
            p = polyfit(tem_x,tem_y,1); %曲线拟合，数学基础为最小二乘法 %多项式拟合，此处为直线拟合 y = ax + b
            % 计算点到直线距离均值方差
            dist = [];
            for j=1:pointN
                dist(j) = abs( p(1)*tem_x(j)-tem_y(j)+p(2) ) / sqrt( p(1)^2 +1);
            end
            meanDist = sum(dist)/pointN;
            msd = sqrt( sum((dist-meanDist).^2)/pointN );
            width = sqrt( (tem_x(1)-tem_x(pointN))^2+(tem_y(1)-tem_y(pointN))^2 );
            if meanDist<50 && msd<20 && width<(DBW+300) && width>(DBW-100)  %距离越远检测的挡板越宽，能达到580多毫米；距离近了更接近挡板宽度，也有560左右
                hold on
                plot(tem_x, tem_y, '.','color','magenta');
                a = p(1);
                b = p(2);
                countDB = countDB + 1;
                %检测到的挡板可能有多个，记录每个挡板候选分段的索引
                IndexDB(countDB) = i % 将候选挡板的分段索引记录下来
                disp(['均值=',num2str(meanDist),'；均方差=',num2str(msd),'；挡板宽度=',num2str(width),'；直线参数=',num2str(a),num2str(b)]);
                
                %-----------------------------------------筛选第二步：线性程度第三步，联合检测
                %挡板的左边缘点
                leftx = tem_x(1);
                lefty = tem_y(1);
                %挡板的右边缘点
                rightx = tem_x(nn);
                righty = tem_y(nn);
                for i3=1:NCLUST
                    nn3=0;
                    for j3=1:ND2
                        if (halo(j3)==i3)
                            nn3=nn3+1;
                            A3(nn3,1)=Y1(j3,1);
                            A3(nn3,2)=Y1(j3,2);
                        end
                    end
                    if nn3==0
                        continue
                    end
                    if any(A3(nn3,1))==0
                        continue
                    end
                    %筛选车轮：个数判断11111111111111111111
                    pointN3 = nn3;
                    if (pointN3<10)||(pointN3>900)%轮胎的扫描点数
                        continue
                    end
                    tem_x3 = A3(1:nn3,1);
                    tem_y3 = A3(1:nn3,2);
                    %计算距离2222222222222222
                    %计算类簇到挡板的距离
                    xj = round(nn3);
                    x1 = tem_x3(xj);
                    y1 = tem_y3(xj);
                    dist_chelun_dangban = 0;
                    if i3< i%车轮在挡板左边,i为车轮的车轮类簇索引
                        dist_chelun_dangban = norm([x1 y1] - [leftx lefty])
                    end
                    %i3==i即j为挡板自身
                    if i3>i %车轮在挡板右边
                        dist_chelun_dangban = norm([x1 y1] - [rightx righty])
                    end
                    if (dist_chelun_dangban >300) && (dist_chelun_dangban<1200)%距离在1.5m--2m之间
                        hold on
                        plot(tem_x3, tem_y3, '*','color','green');
                        i3
                        dist_chelun_dangban
                    end
                end%//第三步，联合检测结束
                
            end%第二步
            
        end%第一步
        
    end
    pause();
end%一帧数据处理结束

