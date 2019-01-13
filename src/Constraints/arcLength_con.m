function [c, ceq] = arcLength_Con(x)

	x1 <= 200;	%We set 0 <= x1 <= 200, 0 <= x2 <= 200, all the path is constrained in the coordination.
	x1 > 0;
	x2 <= 200;
	x2 > 0;
end