function [bezier_Len] = bezier_curve(Path_Bez)

	%each group contains 10 points, because each arc consists of 10 points
	NumP = 10;
	len = length(Path_Bez);
	groupNum = len/NumP;	
	bezierCurve_Len = 0;
	t = 1;
	
	for groupIndex = 1 : groupNum

		pointNum = (groupIndex - 1)*NumP + 1

		while pointNum < (NumP * groupIndex)
			pointNum
			P0 = Path_Bez(pointNum,:)
			P1 = Path_Bez(pointNum + 1,:)
			P2 = Path_Bez(pointNum + 2,:)

			%L(t_) = ((2*Sqrt[A]*(2*A*t*Sqrt[C + t*(B + A*t)] + B*(-Sqrt[C] + Sqrt[C + t*(B + A*t)])) +  
			%(B^2 - 4*A*C) (Log[B + 2*Sqrt[A]*Sqrt[C]] - Log[B + 2*A*t + 2 Sqrt[A]*Sqrt[C + t*(B + A*t)]])) /(8* A^(3/2)));

			ax = P0(1,1)-2*P1(1,1)+P2(1,1);
			  
			ay = P0(1,2)-2*P1(1,2)+P2(1,2);  
			  
			bx = 2*P1(1,1)-2*P0(1,1);  
			  
			by = 2*P1(1,2)-2*P0(1,2);  
			  
			A = 4*(ax*ax+ay*ay)
			  
			B = 4*(ax*bx+ay*by)
			  
			C = bx*bx+by*by
			  
			temp1 = sqrt(C+t*(B+A*t));  
			  
			temp2 = (2*A*t*temp1+B*(temp1-sqrt(C)));  
			  
			temp3 = log(B+2*sqrt(A)*sqrt(C));  
			  
			temp4 = log(B+2*A*t+2*sqrt(A)*temp1);  
			  
			temp5 = 2*sqrt(A)*temp2;  
			  
			temp6 = (B*B-4*A*C)*(temp3-temp4);  
			  
			bezierCurve_Len = bezierCurve_Len + (temp5+temp6)/(8*power(A,1.5))

			pointNum = pointNum + 2;	%move two positions

			if pointNum == ((groupIndex - 1)*NumP + 9) 	%one more point to the end
			   	bezierCurve_Len = bezierCurve_Len + ((P2(1,1)-Path_Bez((pointNum + 1),1))^2+(P2(1,2)-Path_Bez((pointNum + 1),2))^2)^0.5;
			   	break;
			end
		end
	end
	bezier_Len = bezierCurve_Len
end