function [bezier_Len] = bezier_curve_old(Path_Bez)

	%each group contains 10 points, because each arc consists of 10 points
	NumP = 10;
	len = length(Path_Bez);
	groupNum = len/NumP;	
	bezierCurve_Len = 0;
	t = linspace(0,1,1000);
	
	for groupIndex = 1 : groupNum

		pointNum = (groupIndex - 1)*NumP + 1;

		while pointNum <= (NumP * groupIndex)
			P0 = Path_Bez(pointNum,:);
			P1 = Path_Bez(pointNum + 1,:);
			P2 = Path_Bez(pointNum + 2,:);
			p_prev = P0;

		    %calculate distance from final path segment's end point to the final destination
			for i = 2 : length(t)
			    %calculate position
			    %paper "UAV Path-Planning using Bézier Curves and a Receding Horizon Approach", page 4, Equation 1
			    %B(t) = (1−t)^2*P0 + 2(1−t)tP1 + t^2*P2, 0 ≤ t ≤ 1
			    %B'(t) = 2(1−t)(P1 −P0) + 2t(P2 −P1)

			    B(1) = (1-t(i))^2*P0(1,1) + 2*(1-t(i))*t(i)*P1(1,1)+t(i)^2*P2(1,1);
			    B(2) = (1-t(i))^2*P0(1,2) + 2*(1-t(i))*t(i)*P1(1,2)+t(i)^2*P2(1,2);

			    %find distance from previous position to new position
			    %d = norm(p-p_prev);
			    d = ((B(1)-p_prev(1,1))^2+(B(2)-p_prev(1,2))^2)^0.5;
			    
			    %add distance to total length
			    bezierCurve_Len = bezierCurve_Len + d;
			    
			    %change initial position
			    p_prev = B;
			end

			pointNum = pointNum + 2;	%move two positions

			if pointNum == ((groupIndex - 1)*NumP + 9) 	%one more point to the end
			   	bezierCurve_Len = bezierCurve_Len + ((P2(1,1)-Path_Bez((pointNum + 1),1))^2+(P2(1,2)-Path_Bez((pointNum + 1),2))^2)^0.5;
			   	break;
			end
		end
	end
	bezier_Len = bezierCurve_Len;
end