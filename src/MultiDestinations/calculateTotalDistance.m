function [totalDis, Dp2p, approximateLen] = calculateTotalDistance()

global destinationNum;
%global totalDis, Dp2p, Darc;

  %----------Calculate distance of straight paths----------%
  %Read tangentPointsCoordination_n, get pairs of coordinations
  Disp2p = 0;
  for num = 1 : destinationNum
      fileName = sprintf("tangentPointsCoordination_%s%s", int2str(num), ".txt");
      fid = fopen(fileName);

      %Ensure Whether file is opened successfully
      if fid < 0
        continue;
      end

      coordinates = textscan(fid, '%s%s');
      rowsS = size(coordinates{1,1});
      rowsC = rowsS(1,1);             %rows of coordinates
      straightPoint_Coordinations = zeros(rowsC, 2);
      for c = 1 : 2
          straightPoint_Coordinations(:, c) = str2num(char(coordinates{1,c}));
      end
      %Calculate the distance
      for row = 1 : (rowsC - 1)
        Xs1 = straightPoint_Coordinations(row, 1);
        Ys1 = straightPoint_Coordinations(row, 2);
        Xs2 = straightPoint_Coordinations(row + 1, 1);
        Ys2 = straightPoint_Coordinations(row + 1, 2);
        Disp2p = Disp2p + ((Xs2 - Xs1)^2 + (Ys2 - Ys1)^2)^0.5;
      end
  end
  Dp2p = Disp2p;
  %--------------------------------------------------------%

  %----------Calculate approximate distance of curve paths----------%
  %summation of each segment between two consecutive points
  %Read curvelinePointsCoordination_n, get pairs of coordinations
  apxmLen = 0;
  for num = 1 : destinationNum
      fileName = sprintf("curvelinePointsCoordination_%s%s", int2str(num), ".txt");
      fid = fopen(fileName);

      %Ensure Whether file is opened successfully
      if fid == -1
        continue;
      end

      coordinates = textscan(fid, '%s%s%s');
      rowsS = size(coordinates{1,1});
      rowsC = rowsS(1,1);             %rows of coordinates
      curvePoint_Coordinations = zeros(rowsC, 2);
      for c = 1 : 3
          curvePoint_Coordinations(:, c) = str2num(char(coordinates{1,c}));
      end

      %Put every 10 coordinations into an array, because curve line of every obstacle consists of 10 points
      nLoop = 0;    %number of loops
      nRow = 0;     %current rows
      while nRow < rowsC
        pointArray = zeros(10,3);   % 10 rows by 3 columns
        for line = 1 : 10
          pointArray(line,:) = curvePoint_Coordinations(line + (nLoop * 10),:);
          nRow = nRow + 1;
        end

        %Calculate the distance approximately
        for row = 1 : 9
          Xc1 = pointArray(row, 1);
          Yc1 = pointArray(row, 2);
          Xc2 = pointArray(row + 1, 1);
          Yc2 = pointArray(row + 1, 2);
          apxmLen = apxmLen + ((Xc2 - Xc1)^2 + (Yc2 - Yc1)^2)^0.5;
        end
        nLoop = nLoop + 1;
      end
  end
  approximateLen = apxmLen;
  %--------------------------------------------------------%

  totalDis = Dp2p + approximateLen;
end