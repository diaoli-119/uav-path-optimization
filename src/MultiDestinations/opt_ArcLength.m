function [arcLen] = arcLength(num)

global destinationNum;  
%global Darc;

%Read curvelinePointsCoordination_n, get pairs of coordinations
  Darc = 0;
  %for num = 1 : destinationNum
    %num = 2;
    %Darc = 0;
    fileName = sprintf("curvelinePointsCoordination_%s%s", int2str(num), ".txt");
    fid = fopen(fileName);

    %Ensure Whether file is opened successfully
    if fid == -1
      %continue;
    end

    coordinates = textscan(fid, '%s%s%s');
    rowsS = size(coordinates{1,1});
    rowsC = rowsS(1,1);             %rows of coordinates
    curvePoint_Coordinations = zeros(rowsC, 2);
    for c = 1 : 3
        curvePoint_Coordinations(:, c) = str2num(char(coordinates{1,c}));
    end

    %Put first and last tangentPoint coordinates of each obstacle into an array
    %nLoop = 0;    %number of loops
    %nRow = 0;     %current rows
    n = 1;
    stop = 1;
    while stop
      pointArray = zeros(2,3);   % 2 rows by 3 columns, two tangentPoint coordinates
      if (10*n) <= rowsC
        pointArray(1,:) = curvePoint_Coordinations(1 + 10*(n-1),:);
        pointArray(2,:) = curvePoint_Coordinations(10*n,:);

        %Get radius
        r = pointArray(1, 3);

        %Calculate the distance by using "inverse trigonometric function Inverse Sine" (calculate arc length using asin())
        segLen = ((pointArray(2,1) - pointArray(1,1))^2+(pointArray(2,2) - pointArray(1,2))^2)^0.5/(2*r);
        Darc = Darc + 2*r*asin(segLen);

        %for seg = 1 : 9   %9 segments consist of 10 points
          %x1 = pointArray(seg, 1);
          %y1 = pointArray(seg, 2);
          %x2 = pointArray(seg + 1, 1);
          %y2 = pointArray(seg + 1, 2);
          %Darc = Darc + 2*r*asin(((x2 - x1)^2+(y2 - y1)^2)^0.5/(2*r));
        %end
        n = n + 1;
      else
        stop = 0;
      end
    end
  %end
  arcLen = Darc;
end