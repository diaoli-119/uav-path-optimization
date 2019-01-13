function [td tt te] = compare_of()


%compare three different optimization functions
% rng(59);
% n_obs = 55; %number of static obstacles
% obs = rand(n_obs,2)*90+5; %obstacle locations
% rng(4); %for partially random obstacle size
% obs_rad = 3 +  rand(n_obs,1)*3; %obstacle radius
%-------------------------------------------%
%close all;
%import data
path_e = importdata('path_e.txt');
start_e = importdata('start_e.txt');


global n_obs obs obs_rad uav_finite_size uav_ws delta_t;


hold on

%tick labels
set(gca,'XTickLabel','')
set(gca,'YTickLabel','')

%plot landing area
cx = 50;
lr = 15;
xf = [100, 100];

cs = 2*lr/cx;
x = xf(1) - lr : cs : xf(1)+ lr;
y =  (lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %top part of circle
y1 = -(lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %bottom part of circle

plot(x,y,'g--');
plot(x,y1,'g--');

%square axes
axis square

    
        Path_bez = path_e;
        path_start = start_e;
    
    %plot(Path_bez(:,1),Path_bez(:,2),'Color',[0, 0.5, 0]); %plots path of UAV
    
    if uav_finite_size == 1
        for i = 1 : length(path_start)
            x = path_start(i,1) - uav_ws : 0.05 : path_start(i,1)+ uav_ws;
            y =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
            y1 = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle

                plot(x,y,'r');
                plot(x,y1,'r');
   
        end
    end
    
        plot(Path_bez(:,1),Path_bez(:,2),'r'); %
        

%compare total distance traveled by UAV
td = zeros(3,1);
td(1) = evaluate_solution(path_e);

%compare time elapsed for each path
delta_time = 0.5*delta_t;

tt = zeros(3,1);
tt(1) = delta_time*length(path_e);


end
