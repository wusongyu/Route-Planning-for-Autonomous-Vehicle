clear;clc;
tmp_map = imread('Map_png\HNU-Line-480x780.png');  %读取备好的道路地图
n = graythresh(tmp_map);    %获得一个合适的阈值，以便图像二值化处理
imp = ~im2bw(tmp_map,n);    %图像二值化处理
location_c = [721, 259;...   %1 - 物电院B栋
              228, 141;...  %2 - 逸夫楼
              225, 346;...  %3 - 工训B栋107
              78, 366;...   %4 - 五食堂
              536, 183;...  %5 - 超算中心
              66, 154;...  %6 - 东方红广场
              71, 242];...  %7 - 运动场
start_c = location_c(1,:);    %设置起点坐标
dest_c = location_c(4,:);      %设置目标点坐标
% start_c = [721,259];    %设置起点坐标
% dest_c = [71,242];      %设置目标点坐标
%% TEST: BUG 检验
%  a = [1 1 1 1 1;
%        1 0 1 1 1;
%        1 1 0 1 1;
%        1 1 1 0 1;
%        1 1 1 1 1];
% imp = logical(~a);
% start_c = [4,2];    %设置起点坐标
% dest_c = [2,4];      %设置目标点坐标
%% 调用Dijkstra算法函数
[check, map, OptimalPath, Distance] = dijkstra_get(imp, start_c, dest_c);
%% 构建出从初始点到目标点的最短路径
if (~check)
%     h = msgbox('成功找到最优路径','提示','none');
    how_far = Distance/64*100;
    image(1.5, 1.5, map);
    axis image;
    hold on
    plot(start_c(2),start_c(1),'o','color','g','LineWidth',2);
    plot(dest_c(2),dest_c(1),'o','color','c','LineWidth',2);
    plot(OptimalPath(:,2),OptimalPath(:,1),'color','r','LineWidth',2);
    legend('Start','Goal','Path','Location','SouthEast');
else 
   h = msgbox('未找到路径，请重新确认','错误','warn');
end