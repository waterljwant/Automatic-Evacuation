function JointDetection(ladar_data_pole, Clusters, BoardWidth)
DBW = BoardWidth;   %�����

n = size(ladar_data_pole);


%% 
fig = figure(1);
xlabel ('X (mm)','FontSize',10.0)
ylabel ('Y (mm)','FontSize',12.0)
set(gcf, 'position', [200 400 400 280]);
for k=1:n(1)  %
    k
    SPNum = [1]; %������ʼ��
    EPNum = []; %���ν�����
    SegTH = 180; %����ʱ���ڵ������ֵ����λmm
    cla();  %���figure
    
    %%  һ֡�״�����תƽ��ֱ������
    Ladar_dara_x = zeros(1081);
    Ladar_dara_y = zeros(1081);
    
    hold on;
    plot(Ladar_dara_x, Ladar_dara_y, '.');
    
    %%  һ֡���ݷֶ�
    for i=1:1080
        D = sqrt( (Ladar_dara_x(i)-Ladar_dara_x(i+1))^2 + (Ladar_dara_y(i)-Ladar_dara_y(i+1))^2 );
        if D>SegTH
            EPNum = [EPNum i];      %iΪ�����㣬��i���뵽 EPNum ������
            SPNum = [SPNum i+1];    %i+1Ϊ��ʼ�㣬��i+1���뵽 SPNum ������
        end
    end
    EPNum = [EPNum 1081];
            
    %���Ƶ�
    m = size(SPNum);
    plot(0,0,'o');

    %% ֱ�����+�ҵ���
    countDB=0;
    for i=1:m(2)
        pointN = EPNum(i)-SPNum(i)+1;
        if (pointN>15)&&(pointN<260) %ɸѡ��һ���������ж�
            tem_x = Ladar_dara_x( SPNum(i):EPNum(i) );
            tem_y = Ladar_dara_y( SPNum(i):EPNum(i) );
            p = polyfit(tem_x,tem_y,1); %������ϣ���ѧ����Ϊ��С���˷� %����ʽ��ϣ��˴�Ϊֱ����� y = ax + b
            %% ����㵽ֱ�߾����ֵ����
            dist = [];
            for j=1:pointN
                dist(j) = abs( p(1)*tem_x(j)-tem_y(j)+p(2) ) / sqrt( p(1)^2 +1);
            end
            mean = sum(dist)/pointN;
            msd = sqrt( sum((dist-mean).^2)/pointN );
            width = sqrt( (tem_x(1)-tem_x(pointN))^2+(tem_y(1)-tem_y(pointN))^2 )
%             disp(['��ֵ=',num2str(mean),'��������=',num2str(msd),'�����=',num2str(width)]);
% ɸѡ�ڶ��������Գ̶�
            if mean<50 && msd<10 && width<(DBW+100) && width>(DBW-100)  %����ԽԶ���ĵ���Խ���ܴﵽ580����ף�������˸��ӽ������ȣ�Ҳ��560����
                plot(tem_x, tem_y, '.','color','magenta');
                hold on
                a = p(1); b = p(2);
                countDB = countDB + 1;
                %��⵽�ĵ�������ж������¼ÿ�������ѡ�ֶε�����
                IndexDB(countDB) = i % ����ѡ����ķֶ�������¼����
                disp(['��ֵ=',num2str(mean),'��������=',num2str(msd),'��������=',num2str(width),'��ֱ�߲���=',num2str(a),num2str(b)]);
            end
        end  %end 0f pointN>15  
    end
    
    %�����������϶�λɸѡ
    %���ܻ��ж����Ҳ���ܽ���һ�����������Ϊ���         
    %�ں�ѡ���������Ѱ�ҳ���
    if countDB==0
        continue % ������һ֡����
    end
    %mm = size(IndexDB);%��Ҫ��ÿһ֡����IndexDB����
    mm = countDB;
    for j = 1:mm
        index_DangBan = IndexDB(j);
        j
        %��������Ե��
        xileft=SPNum(index_DangBan);
        leftx = Ladar_dara_x(xileft);
        lefty = Ladar_dara_y(xileft);
        %������ұ�Ե��
        xiright=EPNum(index_DangBan);
        rightx = Ladar_dara_x(xiright);
        righty = Ladar_dara_y(xiright);
        
        for i=1:m(2)%�������
            %ɸѡ���ֵ�һ���������ж�11111111111111111111
            pointN = EPNum(i)-SPNum(i)+1;
            if (pointN>6) & (pointN<400)
                if any(Ladar_dara_x(SPNum(i):EPNum(i)))==0
                    continue
                end
                %�ڶ������������2222222222222222
                %������ص�����ľ���
                xj = round((SPNum(i)+EPNum(i))/2);
                x1 = Ladar_dara_x(xj);
                y1 = Ladar_dara_y(xj);
                dist_chelun_dangban = 0;
                if i< IndexDB(j)%�����ڵ������,iΪ���ֵĳ����������
                    dist_chelun_dangban = norm([x1 y1] - [leftx lefty])
                end
                %j==i��jΪ��������
                if i>IndexDB(j) %�����ڵ����ұ�
                    dist_chelun_dangban = norm([x1 y1] - [rightx righty])
                end
                if (dist_chelun_dangban >300) && (dist_chelun_dangban<1200)%������1.5m--2m֮��
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
