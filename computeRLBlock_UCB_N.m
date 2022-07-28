function [WSCEst, WSCN] = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN, angleUAV, hUAV, rjUAV, ...
                                            A, E, dAB, gammaA, gammaJ, c, channelParam, i, choice, alpha, nUAV, boolFade)
    
	%   Choose which action set to work with
    switch choice
        case 1
            WSCEst = WSCEst_Angle;
        case 2
            WSCEst = WSCEst_H;
        case 3
            WSCEst = WSCEst_Rj;
    end
    
    
    %   Upper-Confidence-Bound Action Selection
    if i == 1
        ind = randi(length(WSCEst));
    else
        WSCEst_UCB = WSCEst + c*sqrt( log(i)./WSCN );
        maxInds_UCB = find( WSCEst_UCB == max( WSCEst_UCB ) );              %   Store multiple maximum values indeces
        if(length(maxInds_UCB) > 1)                                         %   Check if there are multiple maximum values
            ind = maxInds_UCB(randi(length(maxInds_UCB)));                 	%   Choose a random greedy action
        else
            ind = maxInds_UCB;                                              %   Choose (single) greedy action with 1-epsilon probability
        end
    end
    
    
    % Update virtual UAVs positions
    [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   %   Get greedy positioning for other action sets (all of them temporarily)
    Angle   = angleUAV(indAng);
    H       = hUAV(indH);
    Rj      = rjUAV(indRj);
    switch choice                                                           %   For the current action set, set current estimate
        case 1
            Angle   = angleUAV(ind);
        case 2
            H       = hUAV(ind);
        case 3
            Rj      = rjUAV(ind);
    end
    %[UAV1, UAV2] = setNewPos(Angle, H, Rj);                                 %   Compute the 3D position of both UAVs for these position values
    UAVs = setNewPos_N(nUAV, Angle, H, Rj);
    
    %   Compute Reward (WSC) of action ind
    %WSC = computeWSC(A, E, UAV1, UAV2, Rj, H, dAB, gammaA, gammaJ, channelParam )/length(E(:));
    WSC = computeWSC_NUAV(A, E, UAVs, dAB, gammaA, gammaJ, channelParam, boolFade )/length(E(:));
    
    %   Action-value updates
    WSCN(ind) = WSCN(ind)+1;                                                %   Update the ocurrences
    if alpha >0
        WSCEst(ind) = WSCEst(ind) + alpha*(WSC-WSCEst(ind));                %   Action value incremental update with fixed step size
    else
        WSCEst(ind) = WSCEst(ind) + (1/WSCN(ind))*(WSC-WSCEst(ind));        %   Action value incremental update
    end
    
    

end