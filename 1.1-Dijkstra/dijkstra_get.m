%% Dijkstra算法函数
function [check, map, OptimalPath, Distance] = dijkstra_get (imp, start_p, dest_p)
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

% 生成起始点和目标点的线性索引
start_node = sub2ind(size(map), start_p(1), start_p(2));
dest_node  = sub2ind(size(map), dest_p(1),  dest_p(2));

map(start_node) = 5;    % 填充起始点，5-green
map(dest_node)  = 6;    % 填充目标点，6-red

% 初始化节点的距离矩阵
DistanceFromStart = Inf(nrows,ncols);  % 初始化为正无穷
DistanceFromStart(start_node) = 0;  % 起始点初始化为0
% For each grid cell this array holds the index of its parent
par_p = zeros(nrows,ncols);
%% 检索循环
while true
    % 选取最短路径节点
    [min_dist, current] = min(DistanceFromStart(:));    % 取第一个最小距离单元点，及其索引current
    
    if ( current == dest_node )
        % Draw current map
        map(start_node) = 5;
        map(dest_node) = 6;
        check = 0;  % 路径搜索成功，返回0
        break;
    elseif (isinf(min_dist))
        check = 1;  % 路径搜索失败，返回1
        break;
    end
    
    map(current) = 3;   % Update map      
    DistanceFromStart(current) = Inf; % remove this node from further consideration
       
    [i, j] = ind2sub(size(DistanceFromStart), current);    % 当前单元点序列值
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
    for i=1:length(neighborIndex)  
        % 过滤障碍点、已检索过的点和起始点
        if ((map(neighborIndex(i))~=1) && (map(neighborIndex(i))~=3 && map(neighborIndex(i))~= 5))
            map(neighborIndex(i)) = 4; 
            if (DistanceFromStart(neighborIndex(i)) > min_dist + 1)
                   DistanceFromStart(neighborIndex(i)) = min_dist + 1; 
                   par_p(neighborIndex(i)) = current;   
            end
        end
    end   
end
%% 构建路径输出、距离输出
if (check)
    OptimalPath = [];
    Distance = [];
else
    Distance = min(DistanceFromStart(:));
    OptimalPath(1,:) = [start_p(1), start_p(2)];
    route = [dest_node];
    while (par_p(route(1)) ~= 0)
        route = [par_p(route(1)), route];
    end
    for k = 2:length(route) - 1   
        [kk, jj] = ind2sub(size(DistanceFromStart), route(k));
        OptimalPath(k,:) = [kk, jj];
    end
end   