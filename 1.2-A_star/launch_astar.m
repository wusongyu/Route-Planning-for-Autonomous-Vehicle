clear;clc;
%%���ò�������·��ͼ����ʼ�㡢Ŀ����
tmp_map = imread('Map_png\HNU-Line-480x780.png');  %��ȡ���õĵ�·��ͼ
n = graythresh(tmp_map);    %���һ�����ʵ���ֵ���Ա�ͼ���ֵ������
imp = ~im2bw(tmp_map,n);    %ͼ���ֵ������
location_c = [721, 259;...   %1 - ���ԺB��
              228, 141;...  %2 - �ݷ�¥
              225, 346;...  %3 - ��ѵB��107
              78, 366;...   %4 - ��ʳ��
              536, 183;...  %5 - ��������
              66, 154;...  %6 - ������㳡
              71, 242];...  %7 - �˶���
start_c = location_c(1,:);    %�����������
dest_c = location_c(4,:);      %����Ŀ�������

%% TEST: BUG ����
%  a = [1 1 1 1 1;
%        1 0 1 1 1;
%        1 1 0 1 1;
%        1 1 1 0 1;
%        1 1 1 1 1];
% imp = logical(~a);
% start_c = [4, 2];
% dest_c = [2, 4];
%% ����A*�㷨�������������·��
[check, map, OptimalPath, Distance,www] = a_star_get (imp, start_c, dest_c);
% End. 
%% ���㷨�����OptimalPathͼ�񻯣����㷨δ�ҵ�����·���������������ʾ��Ϣ
if (~check)
%     h = msgbox('�ɹ��ҵ�����·��','��ʾ','none');
    how_far = Distance/64*100;
    image(1.5, 1.5, map);
    axis image;
    hold on
    plot(start_c(2),start_c(1),'o','color','g','LineWidth',2);
    plot(dest_c(2),dest_c(1),'o','color','c','LineWidth',2);
    plot(OptimalPath(:,2),OptimalPath(:,1),'color','r','LineWidth',2);
    legend('Start','Goal','Path','Location','SouthEast');
else 
   h = msgbox('δ�ҵ�·����������ȷ��','����','warn');
end