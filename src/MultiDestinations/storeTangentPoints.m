function storeTangentPoints(tangentPoints, n_des)

	rowsNColumns = size(tangentPoints);
	rows = rowsNColumns(1,1);
	fileName = sprintf("tangentPointsCoordination_%s.txt", int2str(n_des));
	fid = fopen(fileName, 'a');
	for i = 1 : rows
	   fprintf(fid, "%f \t %f\n",tangentPoints(i,1), tangentPoints(i,2));
	end
	fclose(fid);
end
