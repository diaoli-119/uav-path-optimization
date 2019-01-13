function [td] = destinationPath(Path_Bez)

global uav_ws;

path_t = importdata('path_t.txt');
start_t = importdata('start_t.txt');

    hold on
    
    axis square
    
    linewidth = 1;
    
    %plot landing area
    %lr = 15.0; cx = 50;
    
    %cs = 2*lr/cx;
    %x = xf(1) - lr : cs : xf(1)+ lr;
    %y_top =  (lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %top part of circle
    %y_bottom = -(lr^2 - (x - xf(1)).^2).^0.5 + xf(2); %bottom part of circle
    
    %plot(x,y_top,'g--');
    %plot(x,y_bottom,'g--');

    Path_bez = path_t;
    path_start = start_t;
    for i = 1 : length(path_start)
        %x = path_start(i,1) - uav_ws : 0.25 : path_start(i,1)+ uav_ws;
        %y_top =  (uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %top part of circle
        %y_bottom = -(uav_ws^2 - (x - path_start(i,1)).^2).^0.5 + path_start(i,2); %bottom part of circle

        %plot(x,y_top,'r','LineWidth',linewidth);
        %plot(x,y_bottom,'r','LineWidth',linewidth);
    end
        plot(Path_bez(:,1),Path_bez(:,2),'r','LineWidth',linewidth); % r

    xlim([0 120])
    ylim([0 120])
    
    hold off;
