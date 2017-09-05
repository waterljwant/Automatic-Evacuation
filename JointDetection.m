function JointDetection(ladar_data_pole, Clusters, BoardWidth)
DBW = BoardWidth;   %挡板宽

n = size(ladar_data_pole);


%% 
fig = figure(1);
xlabel ('X (mm)','FontSize',10.0)
ylabel ('Y (mm)','FontSize',12.0)
set(gcf, 'position', [200 400 400 280]);
for k=1:n(1)  %
    k
    SPNum = [1]; %各段起始点
    EPNum = []; %各段结束点
    SegTH = 180; %聚类时相邻点距离阈值，单位mm
    cla();  %清除figure
    
    %%  一帧雷达数据转平面直角坐标
    Ladar_dara_x = zeros(1081);
    Ladar_dara_y = zeros(1081);
    
    hold on;
    plot(Ladar_dara_x, Ladar_dara_y, '.');
    
    %%  一帧数据分段
    for i=1:1080
        D = sqrt( (Ladar_dara_x(i)-Ladar_dara_x(i+1))^2 + (Ladar_dara_y(i)-Ladar_dara_y(i+1))^2 );
        if D>SegTH
            EPNum = [EPNum i];      %i为结束点，将i加入到 EPNum 数组中
            SPNum = [SPNum i+1];    %i+1为起始点，将i+1加入到 SPNum 数组中
        end
    end
    EPNum = [EPNum 1081];
            
    %绘制点
    m = size(SPNum);
    plot(0,0,'o');

    %% 直线拟合+找挡板
    countDB=0;
    for i=1:m(2)
        pointN = EPNum(i)-SPNum(i)+1;
        if (pointN>15)&&(pointN<260) %筛选第一步：个数判断
            tem_x = Ladar_dara_x( SPNum(i):EPNum(i) );
            tem_y = Ladar_dara_y( SPNum(i):EPNum(i) );
            p = polyfit(tem_x,tem_y,1); %曲线拟合，数学基础为最小二乘法 %多项式拟合，此处为直线拟合 y = ax + b
            %% 计算点到直线距离均值方差
            dist = [];
            for j=1:pointN
                dist(j) = abs( p(1)*tem_x(j)-tem_y(j)+p(2) ) / sqrt( p(1)^2 +1);
            end
            mean = sum(dist)/pointN;
            msd = sqrt( sum((dist-mean).^2)/pointN );
            width = sqrt( (tem_x(1)-tem_x(pointN))^2+(tem_y(1)-tem_y(pointN))^2 )
%             disp(['均值=',num2str(mean),'；均方差=',num2str(msd),'；宽度=',num2str(width)]);
% 筛选第二步：线性程度
            if mean<50 && msd<10 && width<(DBW+100) && width>(DBW-100)  %距离越远检测的挡板越宽，能达到580多毫米；距离近了更接近挡板宽度，也有560左右
                plot(tem_x, tem_y, '.','color','magenta');
                hold on
                a = p(1); b = p(2);
                countDB = countDB + 1;
                %检测到的挡板可能有多个，记录每个挡板候选分段的索引
                IndexDB(countDB) = i % 将候选挡板的分段索引记录下来
                disp(['均值=',num2str(mean),'；均方差=',num2str(msd),'；挡板宽度=',num2str(width),'；直线参数=',num2str(a),num2str(b)]);
            end
        end  %end 0f pointN>15  
    end
    
    %第三步：联合定位筛选
    %可能会有多个，也可能仅有一个，但是这个为误检         
    %在候选挡板的两侧寻找车轮
    if countDB==0
        continue % 跳到下一帧处理
    end
    %mm = size(IndexDB);%需要在每一帧都将IndexDB重置
    mm = countDB;
    for j = 1:mm
        index_DangBan = IndexDB(j);
        j
        %挡板的左边缘点
        xileft=SPNum(index_DangBan);
        leftx = Ladar_dara_x(xileft);
        lefty = Ladar_dara_y(xileft);
        %挡板的右边缘点
        xiright=EPNum(index_DangBan);
        rightx = Ladar_dara_x(xiright);
        righty = Ladar_dara_y(xiright);
        
        for i=1:m(2)%遍历类簇
            %筛选车轮第一步：个数判断11111111111111111111
            pointN = EPNum(i)-SPNum(i)+1;
            if (pointN>6) & (pointN<400)
                if any(Ladar_dara_x(SPNum(i):EPNum(i)))==0
                    continue
                end
                %第二步：计算距离2222222222222222
                %计算类簇到挡板的距离
                xj = round((SPNum(i)+EPNum(i))/2);
                x1 = Ladar_dara_x(xj);
                y1 = Ladar_dara_y(xj);
                dist_chelun_dangban = 0;
                if i< IndexDB(j)%车轮在挡板左边,i为车轮的车轮类簇索引
                    dist_chelun_dangban = norm([x1 y1] - [leftx lefty])
                end
                %j==i即j为挡板自身
                if i>IndexDB(j) %车轮在挡板右边
                    dist_chelun_dangban = norm([x1 y1] - [rightx righty])
                end
                if (dist_chelun_dangban >300) && (dist_chelun_dangban<1200)%距离在1.5m--2m之间
                    tem_x = Ladar_dara_x( SPNum(i):EPNum(i) );
                    tem_y = Ladar_dara_y( SPNum(i):EPNum(i) );
                    %plot(tem_x, tem_y, '.','color','green');
                    i
                    dist_chelun_dangban
                end
            end
        end
    end
    pause(0.1)
    
end
