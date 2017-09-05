clear all;
close all;

ladar_data_pole = load('ladar_data.txt');

%%  �״������˲�
% ԭʼ���ݣ������˲�����
%ladar_data_pole = medfilt1(ladar_data_pole,5);% ��ֵ�˲�
ladar_data_pole = GCFilter4Lidar(ladar_data_pole);%����Ӧ�����˲����Ľ�Ч��

DBW = 535;
[NF, ND1] = size(ladar_data_pole);
for kk = 1:NF
    %  ------------------------����
    [X1,halo, NCLUST, H,index] = Cluster(ladar_data_pole(kk,:));
    
    fig = figure(2);
    cla();  %���figure
    for i=1:ND1
        Y1(i,1) = ladar_data_pole(kk,i)*cos((225-0.25*(i-1))*pi/180);
        Y1(i,2) = ladar_data_pole(kk,i)*sin((225-0.25*(i-1))*pi/180);
    end
    Y1(index,:) = [];
    cmap=colormap('Jet');
    [~, ND2] = size(halo);
    countDB=0;
    for i=1:NCLUST%ÿһ��������������ʶ��
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
        
         %-----------------------------------------ɸѡ��һ���������ж�
        pointN = nn;
        if (pointN>15)&&(pointN<260)
            %-----------------------------------------ɸѡ�ڶ��������Գ̶�
            p = polyfit(tem_x,tem_y,1); %������ϣ���ѧ����Ϊ��С���˷� %����ʽ��ϣ��˴�Ϊֱ����� y = ax + b
            % ����㵽ֱ�߾����ֵ����
            dist = [];
            for j=1:pointN
                dist(j) = abs( p(1)*tem_x(j)-tem_y(j)+p(2) ) / sqrt( p(1)^2 +1);
            end
            meanDist = sum(dist)/pointN;
            msd = sqrt( sum((dist-meanDist).^2)/pointN );
            width = sqrt( (tem_x(1)-tem_x(pointN))^2+(tem_y(1)-tem_y(pointN))^2 );
            if meanDist<50 && msd<20 && width<(DBW+300) && width>(DBW-100)  %����ԽԶ���ĵ���Խ���ܴﵽ580����ף�������˸��ӽ������ȣ�Ҳ��560����
                hold on
                plot(tem_x, tem_y, '.','color','magenta');
                a = p(1);
                b = p(2);
                countDB = countDB + 1;
                %��⵽�ĵ�������ж������¼ÿ�������ѡ�ֶε�����
                IndexDB(countDB) = i % ����ѡ����ķֶ�������¼����
                disp(['��ֵ=',num2str(meanDist),'��������=',num2str(msd),'��������=',num2str(width),'��ֱ�߲���=',num2str(a),num2str(b)]);
                
                %-----------------------------------------ɸѡ�ڶ��������Գ̶ȵ����������ϼ��
                %��������Ե��
                leftx = tem_x(1);
                lefty = tem_y(1);
                %������ұ�Ե��
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
                    %ɸѡ���֣������ж�11111111111111111111
                    pointN3 = nn3;
                    if (pointN3<10)||(pointN3>900)%��̥��ɨ�����
                        continue
                    end
                    tem_x3 = A3(1:nn3,1);
                    tem_y3 = A3(1:nn3,2);
                    %�������2222222222222222
                    %������ص�����ľ���
                    xj = round(nn3);
                    x1 = tem_x3(xj);
                    y1 = tem_y3(xj);
                    dist_chelun_dangban = 0;
                    if i3< i%�����ڵ������,iΪ���ֵĳ����������
                        dist_chelun_dangban = norm([x1 y1] - [leftx lefty])
                    end
                    %i3==i��jΪ��������
                    if i3>i %�����ڵ����ұ�
                        dist_chelun_dangban = norm([x1 y1] - [rightx righty])
                    end
                    if (dist_chelun_dangban >300) && (dist_chelun_dangban<1200)%������1.5m--2m֮��
                        hold on
                        plot(tem_x3, tem_y3, '*','color','green');
                        i3
                        dist_chelun_dangban
                    end
                end%//�����������ϼ�����
                
            end%�ڶ���
            
        end%��һ��
        
    end
    pause();
end%һ֡���ݴ������

