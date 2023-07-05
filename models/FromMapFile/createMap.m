function [Map, map_name] = createMap(path)
% read map data given in text format
% open nodes specified with '.'

map_name = 'map1';

fid = fopen(path);
fgetl(fid);

% Height
tline = fgetl(fid);
H = tline;
H = regexp(H,'\d*','Match');
H = str2num(H{1});

% Width
tline = fgetl(fid);
W = tline;
W = regexp(W,'\d*','Match');
W = str2num(W{1});

%
fgetl(fid);

% mapCell
% read each line from file and put it in mapCell
iLine=1;
mapCell = cell(H,1);
tline = fgetl(fid);
while ischar(tline)
    mapCell{iLine,1} =  tline;
    tline = fgetl(fid);
    iLine = iLine+1;
end
fclose(fid);

% flip mapCell
mapCell = flip(mapCell);

% Map Matrix
% Detect Obstacles and Free Nodes (1:Free, 0:Obstacle)
Map = zeros(H, W);
for iLine=1:H
    for j=1:W
        Map(iLine,j) =  strcmp(mapCell{iLine,1}(j), '.');
    end
end

% % save
% save(name_,'Map', 'map_name');

end