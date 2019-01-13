function [c, ceq] = final_con(xi)

%global variables
global step_max;
global step_min;
global x0;
global t;
global obs;
global n_obs;
global obs_rad;
global Pmid;
global turn_r; %minimum turn radius
global num_path;
global xf;
global Dynamic_Obstacles;
global uav_ws;
global lr;
c = [];
%--------------maximum/minimum step distance----------%
% step_max > length of line
% 0 > length of line - step_max;
for i = 1 : num_path
    
    l_l = 0;
    
    if i == 1
        p_prev = x0(1,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*x0(1,:) + 2*(1-t(j))*t(j)*xi(1,:)+t(j)^2*xi(2,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
            %change initial position
            p_prev = p;
        end
        
    else
        p_prev = xi(2*i-2,:);
        
        for j = 1 : length(t)
            %calculate position
            p = (1-t(j))^2*xi(2*i-2,:) + 2*(1-t(j))*t(j)*xi(2*i-1,:)+t(j)^2*xi(2*i,:);
            
            %find distance from previous position to new position
            d = norm(p-p_prev);
            
            %add distance to total length
            l_l = l_l + d;
            
            %change initial position
            p_prev = p;
        end
    end
    
    %add constraints
    l_l
    l_l-step_max
    step_min*0.75 - l_l
    c = [c l_l-step_max step_min*0.75 - l_l]
    
end