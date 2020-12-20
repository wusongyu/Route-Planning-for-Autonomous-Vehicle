%% A*�㷨����
function [check, map, OptimalPath, Distance,current] = a_star_get (imp, start_c, dest_c)
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

% OPEN = int8(zeros(nrows, ncols));    %��� open grid cells����ʼ��Ϊȫ0
% OPEN(start_c(1), start_c(2)) = 1;
% CLOSE = int8(zeros(nrows, ncols));  %��� closed grid cells����ʼ��Ϊȫ0
% CLOSE(imp==1) = 1;            % ���ò�����ʻ��Ԫ����closed matrix

gScore = zeros(nrows, ncols);           % gֵ�������ڸ��ٵ�Ԫ���ʵ�ʴ��� 
fScore = single(inf(nrows, ncols));     % fֵ�������ڸ��ٵ�Ԫ��Ĺ���ֵ(only open list) 
hn = single(zeros(nrows, ncols));       % ����ֵ���󣻳�ʼ��Ϊȫ0
hn(start_c(1),start_c(2)) = sqrt( (start_c(1)-dest_c(1)).^2 + (start_c(2)-dest_c(2)).^2 ) ;
fScore(start_c(1),start_c(2)) = hn(start_c(1),start_c(2));

%% ����������������
% [col, row] = find(imp==1);
% 
% for k=1:size(col)
%     for j=1:size(row)
%         hn(col(k), rom(j)) = sqrt( (col(k)-dest_c(1))^2 + (rom(j)-dest_c(2))^2);   % ��ֵ����������ֵ
%     end
% end
%% 
% ������ʼ���Ŀ������������
start_node = sub2ind(size(map), start_c(1), start_c(2));
dest_node  = sub2ind(size(map), dest_c(1),  dest_c(2));

map(start_node) = 5;    % �����ʼ�㣬5-green
map(dest_node)  = 6;    % ���Ŀ��㣬6-red

% For each grid cell this array holds the index of its parent
par_p = zeros(nrows,ncols);
%% ����ѭ��
while true
    % ѡȡ���·���ڵ�
    [MFC, current] = min(fScore(:));    % ȡ��һ����Сfֵ��Ԫ�㣬��������current
    
    if ( current == dest_node )
        % Draw current map
        map(start_node) = 5;
        map(dest_node) = 6;
        check = 0;  % ·�������ɹ�������0
        break;
    elseif (isinf(MFC))
        check = 1;  % ·������ʧ�ܣ�����1
        break;
    end
    
    map(current) = 3;   % Update map  - blue 
%     OPEN(current) = 0;
%     CLOSE(current) = 1;
    fScore(current) = inf; % remove this node from further consideration
    
    [i, j] = ind2sub(size(fScore), current);    % ��ǰ��Ԫ������ֵ
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
    
    for k=1:length(neighborIndex)  
        % �����ϰ������ʼ��
        if map(neighborIndex(k))==1 || map(neighborIndex(k))==5 || map(neighborIndex(k)) == 3
            continue;
        end
        tmp_gScore = gScore(current) + 1;  
        if  map(neighborIndex(k))~= 3  % û��������
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
%         % �����ϰ������ʼ��
%         if CLOSE(neighborIndex(k)) == 0
%             tmp_gScore = gScore(current) + 1;  
%             if  OPEN(neighborIndex(k)) == 0  % û��������
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
%% ����·��������������
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