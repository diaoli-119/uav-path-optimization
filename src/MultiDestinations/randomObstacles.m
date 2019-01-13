clear;
clc;
close all;

%--------------Add paths----------%
addpath(genpath('..\Objective_Functions\'));
addpath(genpath('..\Constraints\'));
addpath(genpath('..\ColorPath\'));
addpath(genpath('..\OptimalPathGuesses\'));
addpath(genpath('..\CalculateEnergyUse\'));
addpath(genpath('..\Objective_Functions\'))
%---------------------------------%

%-------global variables----------%
global xf; %final position
global x0; %current starting point
global step_max; %max step distance
global step_min; %minimum step distance
global t;   %parameterization vector t
global Pmid; %needed to match derivatives
global num_path;    %number of segments optimized
global initial;     % to calculate d_l_min
global uav_ws;  %UAV wing span
global rho f W span eo;     %Efficiency prerequisite
global max_speed min_speed;
global D_eta_opt;   %optimized energy use
global temp pos;    %temporary variable, position
global n_obs;   %number of obstacles
global obs;     %positions of obstacles
global obs_co;     %matrix which is used to store coordinate of obstacles
global obs_sorted;  %store sorted coordinates
global scale_obs;           %get rows and columns of obs
global scale_obs_sorted;    %get rows and columns of obs_sorted
global objectiveFunc;       %objective function
global dynamic_Obstacles;   %Dynamic Obstacle flag
global destinationNum;      %number of destinations
global approxDarc;          %approximate length of the arc path
global optimalDarc;     %optimal distance of the arc path
global Dp2p;                %length between two points
%---------------------------------%

%--------------------Generate Obstacles Randomly---------------------%
uav_finite_size = 1;       %1:calculate UAV size; otherwise do not calculate it
dynamic_Obstacles = 0;

%Wing span of UAV
if uav_finite_size == 1
    uav_ws = 1.0; %UAV wing span
else
    uav_ws = 0.001;
end

hold on
axis square

Xmax=200;
Xmin=0;
Ymax=200;
Ymin=0;

%The number of the circles needed to be drawn
Totalnum=40;
obs_co = [];   %matrix which is used to store coordinate of obstacles

%The max and min radius values of a circle
Rmax=6;
Rmin=3;

%Initialization Parameters
Existnum=0; % The numbers of the existing circles
XPlist=[]; % The x position values of the center of existing circles
YPlist=[]; % The y position values of the center of existing circles
Rlist=[]; % The radius of the center of existing circles corresponding center positions

for i = 1 : Totalnum   %zero column 3
    obs_Co(i, 3) = 0;
end

while 1
    if Existnum>=Totalnum
        break;
    end

    % Generate the radius and center position of a circle randomly
    r=rand*(Rmax-Rmin)+Rmin;
    xp=rand*(Xmax-Xmin)+Xmin;
    yp=rand*(Ymax-Ymin)+Xmin;
    % Check whether the standby circle is out of the boundarry
    if xp+r<=Xmax && xp-r>=Xmin && yp+r<=Ymax && yp-r>=Ymin
        if Existnum==0
            plot(xp,yp,'xb');   %staic obstacles' centers
            Existnum= Existnum+1;
            rectangle('Position',[xp-r,yp-r,2*r,2*r],'Curvature',[1,1],'FaceColor','none');
            Rlist(Existnum)=r;
            XPlist(Existnum)=xp;
            YPlist(Existnum)=yp;
        elseif Existnum~=0
            % Check whether the standby circle is in existing circles
             if all((XPlist-xp).^2+(YPlist-yp).^2>((Rlist+r).^2 + 2*uav_ws))  %lest circles tangent
                 plot(xp,yp,'xb');   %staic obstacles' centers
                 Existnum= Existnum+1;
                 rectangle('Position',[xp-r,yp-r,2*r,2*r],'Curvature',[1,1],'FaceColor','none');
                 Rlist(Existnum)=r;
                 XPlist(Existnum)=xp;
                 YPlist(Existnum)=yp;
             end
        end
    end
end
for i = 1 : Existnum
    obs_Co(i, 1) = XPlist(1,i);
    obs_Co(i, 2) = YPlist(1,i);
    obs_Co(i, 3) = Rlist(1,i);
    obs(i, 1) = XPlist(1,i);
    obs(i, 2) = YPlist(1,i);
end
xlim([0 200]);
ylim([0 200]);
hold off
%---------------------------------------------------------------------%

%-----------------sort the coordinates order by increasing x -----------------%
%get the smallest x in obs, store (x,y) in obs_sorted
obs_sorted = [];
scale_obs = size(obs_Co);                  %get rows and columns of obs_Co
while (scale_obs(1,1) > 0)
    scale_obs_sorted = size(obs_sorted);    %get rows and columns of obs_sorted
    if (1 == scale_obs(1,1))                %if obs_Co has only one row left
        obs_sorted(scale_obs_sorted(1,1) + 1,:) = obs_Co(1, :);
        obs_Co(1,:) = [];
        return;
    end

    temp = obs_Co(1,1);  %store the first coordinate temporarily
    pos = 1;
    for i = 2 : scale_obs
        if (temp > obs_Co(i, 1))
            temp = obs_Co(i, 1);   %seek the minimum element in the 1st column
            pos = i;               %record the position of the minimum element
        end
    end
    obs_sorted(scale_obs_sorted(1,1) + 1,:) = obs_Co(pos,:);
    obs_Co(pos,:) = [];
    scale_obs = size(obs_Co);                  %get rows and columns of obs_Co
end
%-----------------------------------------------------------------------------%

%------------ store obstacls coordinates in file --------------%
%rowsNcolumns = size(obs_sorted);
%rows = rowsNcolumns(1, 1);  %rows of sorted obs
%fid = fopen('obstaclecoordinates.txt', 'w');
%for i = 1 : rows
%   fprintf(fid, "%f \t %f\n",obs_sorted(i,1), obs_sorted(i,2));
%end
%fclose(fid);
%----------------------------------------------------------------%

%------------Multidestinations------------%
Md = [200,25; 0,75; 200,125; 0,175; 200,200];         %Multi destinations, 4 destinations and 1 start point
rNc = size(Md);     %row and column of Md
destinationNum = rNc(1,1);      %Number of destinations
%-----------------------------------------%

%--------------Plot UAV path------------%
avoidObstacles(obs_sorted, uav_ws, Md, destinationNum);
%---------------------------------------%

%------------Calculate total distance------------%
[totalDis, Dp2p, approxDarc] = calculateTotalDistance();
%------------------------------------------------%

%------------Calculate total time------------%
max_speed = 15;
min_speed = 10;
Tp2p = Dp2p / max_speed;
Tarc = approxDarc / min_speed;
totalTime = Tp2p + Tarc;
%--------------------------------------------%

%------------Algorithm Options------------%
%UAV efficiency parameter values
rho = 1.225;    %air density
f = .2;         %equivalent parasite area
W = 10;         %weight of aircraft
span = .20;     %span
eo = 0.9;       %Oswald efficiency factor

%transalte UAV information to fit with algorithm
step_max = max_speed; %/2;
step_min = min_speed; %/2;

%parameterization vector t
t = linspace(0,1,10);

%Objective Function selection
optimize_energy_use = 1;    %optimize energy use
optimize_time = 1;          %optimize time. If both are zero, optimize distance
%-----------------------------------------%

%-------------------------final optimization------------------%
if optimize_energy_use == 1
    objectiveFunc = @opt_e;
elseif optimize_time == 1
    objectiveFunc = @opt_t;
else
    objectiveFunc = @bezier_curve;
end

A = [];
b = [];
Aeq = [];
beq = [];
%lb = -10*ones(2*num_path,2);
%ub = 110*ones(2*num_path,2);
lb = [0, 0];
ub = [200, 200];
Pmid = [-min_speed/2,-min_speed/2];

%Get curve points coordinations from curvelinePointsCoordination_n.txt files
bezierCurve_Len = 0;
optimalDarc = 0;
for num = 1 : destinationNum

    fileName = sprintf("curvelinePointsCoordination_%s%s", int2str(num), ".txt");
    fid = fopen(fileName);
    if fid < 0
        continue;
    end

    coordinates = textscan(fid, '%s%s%s');
    rowsS = size(coordinates{1,1});
    rowsC = rowsS(1,1);             %rows of coordinates
    num_path = 3;                   %Receding Horizon Approach (any number really, but 3 is standard)
    Path_point = zeros(rowsC, 2);
    x_final = zeros(rowsC, 2);

    for c = 1 : 2
        Path_point(:, c) = str2num(char(coordinates{1,c}));
    end

    %fmincon calculates minimum value
    %options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',1,'MaxIter',1);
    %options = optimoptions('fmincon','Algorithm','sqp','MaxIter',25);
    %options = optimoptions('fmincon','Algorithm','sqp','MaxFunEvals',500000,'MaxIter',100000);
    %xf = Md(num,:);
    %[x_final(:,:), fval, exitflag, construct] = fmincon(objectiveFunc, Path_point(:,:) , A, b, Aeq, beq, lb, ub, @onepath_cons, options);
    %[x_final(:,:), fval, exitflag, construct] = fmincon(objectiveFunc, Path_point(:,:) , A, b, Aeq, beq, lb, ub);
    %[td, tt, te] = optimization_tt_td_te(Path_point,x_final,optimize_energy_use,optimize_time, D_eta_opt,xf)

    %Bezier_curve calculates minimum value
    bezierCurve_Len = bezier_curve(Path_point(:,:));
    optimalDarc = opt_ArcLength(num);
end

%Calculation of energy
[energy, optEnergy] = optimization_te();
%-------------------------------------------------------------%