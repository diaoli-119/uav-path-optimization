function pathPointsCoodns = avoidObstacles(ObsM, uav_wing, multiDes, desNum)	%ObsM: obstacalMatrix, stored the coordinates of all the obstacles order by increasing x

	global Xcur;		%x coordinate of current point
	global Ycur;		%y coordinate of current point
	global Xe;			%x coordinate of end point
	global Ye;			%y coordinate of end point
	global Xtan_1;
	global Ytan_1;
	global Xtan_2;
	global Ytan_2;
	global segments;	%segments of straight line or curve line
	global tangentIndicator;	%Indicate tangent line between current points and tangent point1
														%or between end points and tangent point2
	global firstObstacle;
	global lastObstacle;
	global step;

	%---------------------------------Plot the path based intersected points---------------------------------%
	rowsNColumns = size(ObsM);
	n_obs = rowsNColumns(1,1);	%number of obstacles
	w = uav_wing;

	Po = [0,0];			%coordinates of origin point
	%Pd = [100,100];	%coordinates of destination point
	tangentPoints = [];	%two tangent points of current obstacle

	%Initial line equation
	Xcur = Po(1,1);
	Ycur = Po(1,2);
	%Xe = Pd(1,1);
	%Ye = Pd(1,2);

	%Calculate the distance between center of obstacle and the line
	hold on;
	flag = 0;	%Indicate line above the center of obstacle or below
	linewidth = 1;
	d = 0;
	judge = 0;

	%delete previous txt files
	deletefiles();

	for n_des = 1 : desNum
		Xe = multiDes(n_des,1);
		Ye = multiDes(n_des,2);
		k0 = (Ye - Ycur)/(Xe - Xcur);
		if (k0 > 0)
			firstObstacle = 1;
			lastObstacle = n_obs;
			step = 1;
		else
			firstObstacle = n_obs;
			lastObstacle = 1;
			step = -1;
		end

		%Store start point in the file "tangentPoint_n.txt"
		startPoint = [Xcur, Ycur];
		storeTangentPoints(startPoint, n_des);

		pathPointsCoordnations = [];	%Store path points
		for i_obs = firstObstacle : step : lastObstacle

			%fprintf("This is %d obstacle\n", i_obs);

			%General line equation: x*(Ye - Ycur) + y*(Xcur - Xe) + Xe*Ycur - Xcur*Ye == 0
			A = (Ye - Ycur);
			B = (Xcur - Xe);
			C = Xe*Ycur - Xcur*Ye;

			d = abs(A*ObsM(i_obs,1) + B*ObsM(i_obs,2) + C)/sqrt(A.^2 + B.^2);
			%fprintf("d = %f\n", d);
			%fprintf("radius = %f\n\n", ObsM(i_obs,3));

			if (k0 > 0)
				%Make sure Xcur < X coordinate of current obstacle,otherwise, X coordinate of tangent point2 may be before tangent point1
				judge = (Xcur < ObsM(i_obs,1));
			else
				%Make sure Xcur > X coordinate of current obstacle,otherwise, X coordinate of tangent point2 may be before tangent point1
				judge = (Xcur > ObsM(i_obs,1));
			end

			if (d <= (ObsM(i_obs,3) + uav_wing) && judge)
				%if d <= the radius, we consider the line and the obstacle intersect, so we Calculate the tangent points(2 points)
				%Estimate the line is above the center of the obstacle or below. Take x coordinate of the center of obstacle into line Equation
				%Estimate the yl coordinate on the line > or < yo (ObsM(i,2)) coordinate of the center of obstacle
				%fprintf("Xcur = %f\n", Xcur);
				%fprintf("Ycur = %f\n", Ycur);
				yl = estimateCoordinateOnLine(Xcur, Ycur, k0, ObsM(i_obs,1));
				%fprintf("yl = %f\n", yl);
				%fprintf(" Xo = ObsM(%d, 1) = %f\n", i_obs, ObsM(i_obs,1));
				%fprintf(" Yo = ObsM(%d, 2) = %f\n", i_obs, ObsM(i_obs,2));
				if (yl < ObsM(i_obs,2))
					flag = -1;	%below the center of obstacle
				else
					flag = 1;	%above the center of obstacle
				end
				%fprintf("flag = %f\n", flag);

				%Calculate tangent points (2 points)
				%Point one, between start point and tangent point
				r = ObsM(i_obs,3);
				[Xtan_1, Ytan_1] = tangentPoint(Xcur, Ycur, ObsM(i_obs,1), ObsM(i_obs,2), r, w, flag);
				%[Xtan_1, Ytan_1]

				%Point two, between tangent point and destination point
				[Xtan_2, Ytan_2] = tangentPoint(Xe, Ye, ObsM(i_obs,1), ObsM(i_obs,2), r, w, flag);
				%[Xtan_2, Ytan_2]

				%store the tangent points of current obstacle
				tangentPoints = [Xtan_1, Ytan_1;Xtan_2, Ytan_2];

				%Store tangent points in the file "tangentPoint_n.txt"
				storeTangentPoints(tangentPoints, n_des);

				%Calculate 'segments' equal segments between Xtan_1 and Xtan_2
				segments = 10;
				segX = linspace(Xtan_1, Xtan_2, 10);
				%segX

				%Infer to 'segments' points along Xtan_1 and Xtan_2 based on segX
				curvelinePoints = [];
				i_segX = 0;
				for i_segX = 1 : segments
					curvelinePoints(i_segX,1) = segX(1,i_segX);
					if (1 == flag)
						curvelinePoints(i_segX,2) = ObsM(i_obs,2) + sqrt((r+w).^2 - (segX(1,i_segX) - ObsM(i_obs,1)).^2);
					else
						curvelinePoints(i_segX,2) = ObsM(i_obs,2) - sqrt((r+w).^2 - (segX(1,i_segX) - ObsM(i_obs,1)).^2);
					end
				end

				straightPart = [];
				%Draw line from current point (Xcur, Ycur) to tangent point1 (Xtan_1, Ytan_1)
				straightPart(1,1) = Xcur;
				straightPart(1,2) = Ycur;
				%storePathPoints(straightPart);
				straightPart(2,1) = curvelinePoints(1,1);
				straightPart(2,2) = curvelinePoints(1,2);
				%straightPart
				plot(straightPart(:,1), straightPart(:,2), 'g', 'LineWidth', linewidth);
				%pause(0.1);

				%Draw curve line between tangent point1 and tagent point2
				plot(curvelinePoints(:,1), curvelinePoints(:,2), 'g','LineWidth',linewidth);
				%curvelinePoints
				%pause(0.1);

				%Store the curveline points in file "curvelinePoints—_n.txt"
				storeCurvelinePoints(curvelinePoints,n_des, r);

				%set tangent point2 as current point
				Xcur = Xtan_2;
				Ycur = Ytan_2;

				%Store all the points of straightPart and curvelinePoints in pathPointsCoordnations
				[rowsC,columnsC] = size(curvelinePoints);
				[rowsP,columnsP] = size(pathPointsCoordnations);
				%rowsOfCurvelinePoints = rowsNColumnsOfCurveline(1,1);
				pathPointsCoordnations(rowsP + 1,1) = straightPart(1,1);
				pathPointsCoordnations(rowsP + 1,2) = straightPart(1,2);
				pathPointsCoordnations(rowsP + 2,1) = straightPart(2,1);
				pathPointsCoordnations(rowsP + 2,2) = straightPart(2,2);
				for n = 1 : rowsC - 1
					pathPointsCoordnations(n + rowsP + 2,1) = curvelinePoints(n + 1,1);
					pathPointsCoordnations(n + rowsP + 2,2) = curvelinePoints(n + 1,2);
				end
			else
				%Go to next obstacle
				continue;
			end
		end

		%Store end point in the file "tangentPoint_n.txt"
		endPoint = [Xe, Ye];
		storeTangentPoints(endPoint, n_des);

		%------------ store coordinates of every path in file --------------%
		%pathPointsCoordnations = [pathPointsCoordnations; Xe, Ye];
		%rowsNcolumns = size(pathPointsCoordnations);
		%rows = rowsNcolumns(1, 1);  %rows of pathPointsCoordnations
		%fileName = sprintf("pathPointsCoordinations%s%s", int2str(n_des), ".txt");
		%fid = fopen(fileName, 'w');
		%for i = 1 : rows
		%   fprintf(fid, "%f \t %f\n",pathPointsCoordnations(i,1), pathPointsCoordnations(i,2));
		%end
		%fclose(fid);
		%------------------------------------------------------------%

		%No more obstacles to end point(Xe, Ye). Draw line from tangent point2 (Xtan_2, Ytan_2) to end point in current path
		straightPart(1,1) = Xcur;
		straightPart(1,2) = Ycur;
		straightPart(2,1) = Xe;
		straightPart(2,2) = Ye;
		Xcur = Xe;
		Ycur = Ye;
		%straightPart
		plot(straightPart(:,1), straightPart(:,2), 'g', 'LineWidth', linewidth);
		%pause(0.1);
		%storePathPoints(straightPart);
	end
	hold off
	%--------------------------------------------------------------------------------------------------------%

	%Store all the coordinates of points on path
	%[rowsP,columnsP] = size(pathPointsCoordnations);
	%pathPointsCoordnations(rowsP + 1, 1, n_des) = 200;
	%pathPointsCoordnations(rowsP + 1, 2, n_des) = 200;
	%dest = [200,200];
	%pathPointsCoordnations = [pathPointsCoordnations; dest]
	%pathPointsCoodns = pathPointsCoordnations;
end
