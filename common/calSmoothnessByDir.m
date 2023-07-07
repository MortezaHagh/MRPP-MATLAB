function smoothness = calSmoothnessByDir(sol)

    dn = diff(sol.nodeNumbers);
    stall = dn == 0;

    sol.dirs(stall) = [];

    theta = sol.dirs;
    dTheta = angdiff(theta);
    smoothness = sum(abs(dTheta));

end
