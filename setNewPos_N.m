function UAVs = setNewPos_N(nUAV, Ang, h, Rj)
    
    UAVs = zeros(nUAV, 3);
    phi = pi - ((nUAV-1)/2)*Ang;                    % Angle of first UAV that preserves symmetry
    for iU = 1:nUAV
        UAVs(iU,1) = Rj*cos(phi + (iU-1)*Ang);
        UAVs(iU,2) = Rj*sin(phi + (iU-1)*Ang);
        UAVs(iU,3) = h;
    end

end