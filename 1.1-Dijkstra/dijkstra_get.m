%% Dijkstra�㷨����
function [check, map, OptimalPath, Distance] = dijkstra_get (imp, start_p, dest_p)
%% ��ʼ��
cmap = [1 1 1; ...% 1 - white - ������ʹ����
        0 0 0; ...% 2 - black - ����ʻ������
        0 0 1; ...% 3 - blue - �������ĵ�Ԫ��
        1 1 0; ...% 4 - yellow  - ��ǰ��Ԫ��������
        0 1 0; ...% 5 - green - ��ʼ��
        1 0 0; ...% 6 - red - Ŀ���
	    1 0 1];...% 7 - pink - ·����
colormap(cmap); %����ǰͼ������ɫͼ����Ϊ cmap ָ����ɫͼ

[nrows, ncols] = size(imp);
map = zeros(nrows,ncols);
map(imp)  = 1;   % ��䲻����ʽ����Ԫ��
map(~imp) = 2;   % ������ʹ�����ߵ�Ԫ��

% ������ʼ���Ŀ������������
start_node = sub2ind(size(map), start_p(1), start_p(2));
dest_node  = sub2ind(size(map), dest_p(1),  dest_p(2));

map(start_node) = 5;    % �����ʼ�㣬5-green
map(dest_node)  = 6;    % ���Ŀ��㣬6-red

% ��ʼ���ڵ�ľ������
DistanceFromStart = Inf(nrows,ncols);  % ��ʼ��Ϊ������
DistanceFromStart(start_node) = 0;  % ��ʼ���ʼ��Ϊ0
% For each grid cell this array holds the index of its parent
par_p = zeros(nrows,ncols);
%% ����ѭ��
while true
    % ѡȡ���·���ڵ�
    [min_dist, current] = min(DistanceFromStart(:));    % ȡ��һ����С���뵥Ԫ�㣬��������current
    
    if ( current == dest_node )
        % Draw current map
        map(start_node) = 5;
        map(dest_node) = 6;
        check = 0;  % ·�������ɹ�������0
        break;
    elseif (isinf(min_dist))
        check = 1;  % ·������ʧ�ܣ�����1
        break;
    end
    
    map(current) = 3;   % Update map      
    DistanceFromStart(current) = Inf; % remove this node from further consideration
       
    [i, j] = ind2sub(size(DistanceFromStart), current);    % ��ǰ��Ԫ������ֵ
    % ������������չ��Ԫ��
     neighbor = [i-1, j ;... 
                i+1, j ;... 
                i, j+1 ;... 
                i, j-1 ]; 
    % ���˳����߽�ĵ�Ԫ��
    outRangetest = (neighbor(:,1)<1) + (neighbor(:,1)>nrows) + (neighbor(:,2)<1) + (neighbor(:,2)>ncols );
    locate = find(outRangetest>0);  
    neighbor(locate,:) = []; 
    
    neighborIndex = sub2ind(size(map),neighbor(:,1),neighbor(:,2));     % ������չ��Ԫ������
    for i=1:length(neighborIndex)  
        % �����ϰ��㡢�Ѽ������ĵ����ʼ��
        if ((map(neighborIndex(i))~=1) && (map(neighborIndex(i))~=3 && map(neighborIndex(i))~= 5))
            map(neighborIndex(i)) = 4; 
            if (DistanceFromStart(neighborIndex(i)) > min_dist + 1)
                   DistanceFromStart(neighborIndex(i)) = min_dist + 1; 
                   par_p(neighborIndex(i)) = current;   
            end
        end
    end   
end
%% ����·��������������
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