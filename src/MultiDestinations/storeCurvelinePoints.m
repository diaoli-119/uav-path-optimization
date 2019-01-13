function storeCurvelinePoints(curvelinePoints, n_des, radius)

	rowsNcolumns = size(curvelinePoints);
	rows = rowsNcolumns(1, 1);  %rows of sorted pathPointsCoordnations
	fileName = sprintf("curvelinePointsCoordination_%s.txt", int2str(n_des));
	fid = fopen(fileName, 'a');
	for i = 1 : rows
	   fprintf(fid, "%f \t %f \t %f\n",curvelinePoints(i,1), curvelinePoints(i,2), radius);
	end
	fclose(fid);
end
