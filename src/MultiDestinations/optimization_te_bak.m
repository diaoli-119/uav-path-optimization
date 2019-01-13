function [te, topte] = optimization_te()

    global rho f W span eo;     %Efficiency prerequisite
    global approxDarc Dp2p optimalDarc;

    %-------------------------Efficiency Parameters-------------------%
    %Defined in paper 《Fuel Efficiency of Small Aircraft 》 (2nd column, page 2)
    A = rho*f/(2*W);
    B = 2*W/(rho*span^2*pi*eo);

    %find minimum d_l, and minimum efficiency
    V_possible = [15, 10];

    D_eta = [0,0];
    d_l = [0,0];

    for i = 1 : length(V_possible)

        d_l(i) = A*V_possible(i)^2 + B/V_possible(i)^2; % we want d_l corresponding to the speed

        eta_pos = calc_eff(V_possible(i));

        D_eta(i) =  d_l(i)/eta_pos;
    end
    %-----------------------------------------------------------------%

    %calculate energy use
    Ep2p = d_l(1)*Dp2p/D_eta(1);
    Eapparc = d_l(2)*approxDarc/D_eta(2);
    Eoptarc = d_l(2)*optimalDarc/D_eta(2);
    te = Ep2p + Eapparc;
    topte = Ep2p + Eoptarc;
end