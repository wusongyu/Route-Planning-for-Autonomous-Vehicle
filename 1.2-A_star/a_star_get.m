%% A*算法函数
function [check, map, OptimalPath, Distance,current] = a_star_get (imp, start_c, dest_c)
%% 初始化
cmap = [1 1 1; ...% 1 - white - 不可行使区域
        0 0 0; ...% 2 - black - 可行驶车道线
        0 0 1; ...% 3 - blue - 检索过的单元点
        1 1 0; ...% 4 - yellow  - 当前单元点的领域点
        0 1 0; ...% 5 - green - 起始点
        1 0 0; ...% 6 - red - 目标点
	    1 0 1];...% 7 - pink - 路径线
colormap(cmap); %将当前图窗的颜色图设置为 cmap 指定的色图

[nrows, ncols] = size(imp);
map = zeros(nrows,ncols);
map(imp)  = 1;   % 填充不可形式区域单元点
map(~imp) = 2;   % 填充可行使车道线单元点

% OPEN = int8(zeros(nrows, ncols));    %存放 open grid cells；初始化为全0
% OPEN(start_c(1), start_c(2)) = 1;
% CLOSE = int8(zeros(nrows, ncols));  %存放 closed grid cells；初始化为全0
% CLOSE(imp==1) = 1;            % 配置不可行驶单元点至closed matrix

gScore = zeros(nrows, ncols);           % g值矩阵，用于跟踪单元点的实际代价 
fScore = single(inf(nrows, ncols));     % f值矩阵，用于跟踪单元点的估计值(only open list) 
hn = single(zeros(nrows, ncols));       % 启发值矩阵；初始化为全0
hn(start_c(1),start_c(2)) = sqrt( (start_c(1)-dest_c(1)).^2 + (start_c(2)-dest_c(2)).^2 ) ;
fScore(start_c(1),start_c(2)) = hn(start_c(1),start_c(2));

%% 建立启发函数数组
% [col, row] = find(imp==1);
% 
% for k=1:size(col)
%     for j=1:size(row)
%         hn(col(k), rom(j)) = sqrt( (col(k)-dest_c(1))^2 + (rom(j)-dest_c(2))^2);   % 赋值给启发函数值
%     end
% end
%% 
% 生成起始点和目标点的线性索引
start_node = sub2ind(size(map), start_c(1), start_c(2));
dest_node  = sub2ind(size(map), dest_c(1),  dest_c(2));

map(start_node) = 5;    % 填充起始点，5-green
map(dest_node)  = 6;    % 填充目标点，6-red

% For each grid cell this array holds the index of its parent
par_p = zeros(nrows,ncols);
%% 检索循环
while true
    % 选取最短路径节点
    [MFC, current] = min(fScore(:));    % 取第一个最小f值单元点，及其索引current
    
    if ( current == dest_node )
        % Draw current map
        map(start_node) = 5;
        map(dest_node) = 6;
        check = 0;  % 路径搜索成功，返回0
        break;
    elseif (isinf(MFC))
        check = 1;  % 路径搜索失败，返回1
        break;
    end
    
    map(current) = 3;   % Update map  - blue 
%     OPEN(current) = 0;
%     CLOSE(current) = 1;
    fScore(current) = inf; % remove this node from further consideration
    
    [i, j] = ind2sub(size(fScore), current);    % 当前单元点序列值
    % 采用四连接扩展单元点
     neighbor = [i-1, j ;... 
                i+1, j ;... 
                i, j+1 ;... 
                i, j-1 ]; 
    % 过滤超出边界的单元点
    outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) + (neighbor(:,2)<1) + (neighbor(:,2)>ncols );
    locate = find(outRangetest>0);  
    neighbor(locate,:) = [];    
    
    neighborIndex = sub2ind(size(map),neighbor(:,1),neighbor(:,2));     % 建立扩展单元点索引
    
    for k=1:length(neighborIndex)  
        % 过滤障碍点和起始点
        if map(neighborIndex(k))==1 || map(neighborIndex(k))==5 || map(neighborIndex(k)) == 3
            continue;
        end
        tmp_gScore = gScore(current) + 1;  
        if  map(neighborIndex(k))~= 3  % 没被检索过
            map(neighborIndex(k)) = 4; 
        elseif tmp_gScore >= gScore(neighborIndex(k))
             continue;
        end
        gScore(neighborIndex(k)) = tmp_gScore; 
        [aa, bb] = ind2sub(size(fScore), neighborIndex(k));
        hn(neighborIndex(k)) = sqrt( (aa-dest_c(1)).^2 + (bb-dest_c(2)).^2 );
        fScore(neighborIndex(k)) = tmp_gScore + hn(neighborIndex(k));
        par_p(neighborIndex(k)) = current;  
    end 
    
%     for k=1:length(neighborIndex)  
%         % 过滤障碍点和起始点
%         if CLOSE(neighborIndex(k)) == 0
%             tmp_gScore = gScore(current) + 1;  
%             if  OPEN(neighborIndex(k)) == 0  % 没被检索过
%                 OPEN(neighborIndex(k)) = 1; 
%             elseif tmp_gScore >= gScore(neighborIndex(k))
%                  continue
%             end
%             gScore(neighborIndex(k)) = tmp_gScore; 
% %             [aa, bb] = ind2sub(size(fScore), neighborIndex(k));
% %             hn(neighborIndex(k)) = sqrt((aa-dest_c(1).^2+(bb-dest_c(2)).^2));
%             fScore(neighborIndex(k)) = tmp_gScore ;%+ hn(neighborIndex(k));
%             par_p(neighborIndex(k)) = current;  
%         end
%     end 
end
%% 构建路径输出、距离输出
if (check)
    OptimalPath = [];
    Distance = [];
else
    Distance = min(fScore(:));
    OptimalPath(1,:) = [start_c(1), start_c(2)];
    route = [dest_node];
    while (par_p(route(1)) ~= 0)
        route = [par_p(route(1)), route];
    end
    for k = 2:length(route) - 1   
        [kk, jj] = ind2sub(size(fScore), route(k));
        OptimalPath(k,:) = [kk, jj];
    end
end   