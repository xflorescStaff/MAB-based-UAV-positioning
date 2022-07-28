function [indAng, indH, indRj] = getNewPos(WSCEst_Ang, WSCEst_H, WSCEst_Rj)

    % ANGLE: choose new greedy action
    maxInds = find(WSCEst_Ang==max(WSCEst_Ang));        % Store multiple maximum values indeces
    if(length(maxInds) > 1)                     % Check if there are multiple maximum values
        indAng = maxInds(randi(length(maxInds)));  % Choose a random greedy action
    else
        indAng = maxInds;                          % Choose (single) greedy action with 1-epsilon probability
    end
    
    % HEIGHT: choose new greedy action
    maxInds = find(WSCEst_H==max(WSCEst_H));        % Store multiple maximum values indeces
    if(length(maxInds) > 1)                     % Check if there are multiple maximum values
        indH = maxInds(randi(length(maxInds)));  % Choose a random greedy action
    else
        indH = maxInds;                          % Choose (single) greedy action with 1-epsilon probability
    end
    
    % ORBIT: choose new greedy action
    maxInds = find(WSCEst_Rj==max(WSCEst_Rj));        % Store multiple maximum values indeces
    if(length(maxInds) > 1)                     % Check if there are multiple maximum values
        indRj = maxInds(randi(length(maxInds)));  % Choose a random greedy action
    else
        indRj = maxInds;                          % Choose (single) greedy action with 1-epsilon probability
    end
    
end