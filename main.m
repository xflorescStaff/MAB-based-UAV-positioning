clc, clear, close all

%% Secrecy Improvement Block Scheme

%   Simulation for performance on SIB scheme evaluation.
%
%   A SIB is a Secrecy Improvement Block that is made up of nRLB RL blocks
%   in which nRL Bandits iterations are performed for a given dAB estimate,
%   and a Positioning block where the UAVs are made to assume new
%   positions.
%
%   Positions are based in 3 variables:
%       Rj:     Orbit radius of both UAVs around Alice
%       hj:     Height of both UAVs
%       theta:  Opening angle of UAVs behind Alice
%
%   Optimization method:    Non-static k-Armed Bandits UCB
%
%   Performance Variables
%       E_SIB:  Total energy consumed by the UAV at each SIB. Comprised of
%               Rx energy, positioning energy and ACK Tx energy.
%       S_SIB:  Total secrecy obtained. Basically a time integral of the
%               active WSC level over the SIB time.
%
%   Control Variables
%       sigAB:  Uncertainty in Bob's position.
%       nRLB:   Number of RL blocks in a SIB.
%   
%   Constants
%       nSIB:   Number of SIBs to be simulated. Could be adapted over
%               convergence criteria.
%       E_RX:   Energy needed by UAV to receive positioning information.
%               May be related to Rx power and number of bits for the
%               instruction.
%       E_ACK:  Same as E_RX but for the ACK.
%       P_Mov:  Power of UAV to move at a given speed v_J

%% Multi Varying nRLB
clc, clear, close all

% Environment parameters (Urban)

phi = 9.61;         % Environmental constant
omega = 0.16 ;      % Environmental constant
alpha = 0.3;        % Ground Path Loss exponent
alpha_AG = 0.3;     % Air-to-Ground Path Loss Exponent
ne_LOS = 1.0;       % Air-to-Ground LOS attenuation
ne_NLOS = 20;       % Air-to-Ground NLOS attenuation
KbVal = 5;          % K parameter for Bob's channels
KeVal = 5;          % K parameter for Eve's channels


% Channel parameters

m = 3;              % Number of parallel channels
sigma = 1/sqrt(2);  % Noise std dev of each component (real and imag) of every parallel channel

Rs = 1;


channelParam =  [   phi,...
                    omega,...
                    alpha,...
                    alpha_AG,...
                    ne_LOS,...
                    ne_NLOS,...
                    Rs,...
                    KbVal,...
                    KeVal];

% Positions ***************************************************************

    % Alice
nA = 1;             %   Number of Alices
A = [0,0,0];        %   Position of Alice (zero point)
gammaA = 200;        %   Alice Tx SNR

% -------------------------------------------------------------------------

    % Bob
nB = 1;             %   Number of Bobs
dAB_R = 20;        	%   Real distance between A and B
sigAB = dAB_R/10;          %   Unreliability of B's position
B = [dAB_R,0,0];      %   Position of Bob

% -------------------------------------------------------------------------

    % Eve
nR = 100;           %   Number of radial points
nTheta = 360;       %   Number of angular points
nE = nR*nTheta;     %   Number of Eves

rLow = 0.1;         %   Lowest radius of Eve
rHigh = 50;         %   Highest radius of Eve
thetaLow = 0;       %   Lowest angle of Eve
thetaHigh = 2*pi;   %   Highest radius of Eve

rangeR = linspace(rLow,rHigh,nR);                       % Points in Radial dimension
rangeTheta = linspace(thetaLow,thetaHigh,nTheta);       % Points in Angular dimension

thetat  =  repmat(rangeTheta,1,nR);
rt  = (repmat(rangeR',1,nTheta)).';

E = [rt(:).*cos(thetat(:)), rt(:).*sin(thetat(:)) , zeros(nR*nTheta,1)];             % Eves' position (rectangle coordinates)

% -------------------------------------------------------------------------

    % UAV
Nu = 2;                             %   Number of simultaneous UAVs
nUAV = 2;

nAng = 90;                          %   Angle discretization level (opening angle)  -> Number of Angle Actions
nH = 30;                            %   Height discretization level for both UAVs   -> Number of Height Actions
nRj = 30;                           %   Orbit radius discretization level           -> Number of radius Actions

gammaJ = 0.1*gammaA/nUAV;                 %   UAVs Jamming SNR

angleUAV    = linspace(0,2*pi/(nUAV-1),nAng);       %   Possible angle actions (opening angles)
hUAV        = linspace(100,200,nH);                    %   Possible height actions
rjUAV       = linspace(1,rHigh,nRj);                %   Possible orbit radius actions

% -------------------------------------------------------------------------

% *************************************************************************


%   UAV technical parameters [dummy values, research due]

E_RX = 0.1;                 % Energy it takes for UAV to receive positional information [dummy value].
E_ACK = 0.1;                % Energy it takes for UAV to send ACK to Alice [dummy value].
P_Mov = 80;                 % Power it takes UAV to maneuver [dummy value].
v_J = 2;                    % Speed at which UAV moves [dummy value].

%   SIB k-Armed Bandits
nSIB = 20;                  % Number of SIBs to be considered

nRLB_V = [5 10 20];
NnRLB = length(nRLB_V);

nRL = 1;                    % Number of RL steps per estimate
epsilon = 0;                % Epsilon for epsilon-greedy choice
initWSC = 0;                % Optimistic initial action values
c = 0.3;                    % Exploration parameter for UCB
alpha = 0.5;                  % Step size (0: uniform average)

%   Performance Variables

nMC = 1e2;                              %   Number of Monte Carlo loops for performance evaluation
E_SIB = zeros(NnRLB,nSIB);                  %   Energy used in a given SIB
S_SIB = zeros(NnRLB,nSIB);                  %   Cummulative secrecy over a given SIB

MC_WSC = zeros(NnRLB,nMC,nSIB);               %   Stored WSC per SIB for all MC loops
MC_Ang = zeros(NnRLB,nMC,nSIB);               %   Stored Angles per SIB for all MC loops
MC_H = zeros(NnRLB,nMC,nSIB);                 %   Stored Heights per SIB for all MC loops
MC_Rj = zeros(NnRLB,nMC,nSIB);                %   Stored Orbit Radius per SIB for all MC loops

dtRL = 0.1*3;                             % DUMMY TIME, to get consistent results, not depending on the processor or machine
                                        % Time per RL iteration (the smallest RL iteration)

boolFade = 1;               % Use fading

% Exhaustive Search
tic
fprintf('Exhaustive search!\n')
WSC_ES = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade );
fprintf('\n\n Exhaustive search ended!')
toc

for iRLB = 1:NnRLB
    nRLB = nRLB_V(iRLB);
    nLoops = nSIB*(nRLB+1);     % Number of loops for action choosing (positioning every nRLB+1 loops)
    for iMC = 1:nMC
        % Initialization
        WSCEst_Angle    = initWSC*ones(1,nAng);         %   Action value estimation vector for angle actions
        WSCEst_H        = initWSC*ones(1,nH);           %   Action value estimation vector for height actions
        WSCEst_Rj       = initWSC*ones(1,nRj);          %   Action value estimation vector for orbit radius actions
        WSCN_Angle      = zeros(1,nAng);                %   Vector to store angle action ocurrences
        WSCN_H          = zeros(1,nH);                  %   Vector to store height action ocurrences
        WSCN_Rj         = zeros(1,nRj);                 %   Vector to store orbit radius action ocurrences

        WSC_UAV = 0;                                    %   WSC initial value
        
        WSC_Ang = angleUAV(end);               %   Angle random initialization
        WSC_H   = hUAV(end);                   %   Height random initialization
        WSC_Rj  = rjUAV(end);                  %   Orbit Radius random initialization

        tic
        for i=1:nLoops
            if mod(i,nRLB+1)==0
                % This is a positioning block
                % No RL is performed, and Performance variables are taken at
                % this block only
                t1 = toc;                                   % Time from previous SIB end to this Positioning block
                iSIB = i/(nRLB+1);                          % Number of SIB

                % Obtain new position indeces
                [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   

                % UAV Movement
                WSC_Rj_Old = WSC_Rj;                                                    
                dH      = abs( WSC_H - hUAV(indH) );                                    % Height movement distance
                dRj     = abs( WSC_Rj - rjUAV(indRj) );                                 % Orbit radius movement distance
                dAng    = min(WSC_Rj,WSC_Rj_Old)*abs( WSC_Ang - angleUAV(indAng) )/2;	% Angle movement distance (through lesser orbit)
                dtJ     = (dAng + dH + dRj)/v_J;                                        % Flight time

                % Update UAV positioning values
                WSC_Ang = angleUAV(indAng);        
                WSC_H   = hUAV(indH);
                WSC_Rj  = rjUAV(indRj);

                % Update UAV position
                UAVs = setNewPos_N(nUAV, WSC_Ang, WSC_H, WSC_Rj);

                % Compute secrecy performance metric
                S_SIB(iRLB,iSIB) = S_SIB(iRLB,iSIB) +  dtRL*nRLB*3*WSC_UAV/nMC;

                % Compute energy metric
                E_SIB(iRLB,iSIB) = E_SIB(iRLB,iSIB) +  (E_RX + E_ACK + dtJ*P_Mov)/nMC;

                % Update UAV WSC value
                [WSC_UAV, DeltaComp] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_UAV = WSC_UAV/(nTheta*nR);

                % Store control values per MC loop
                MC_WSC(iRLB,iMC,iSIB)    = WSC_UAV;
                MC_Ang(iRLB,iMC,iSIB)    = WSC_Ang;               %   Stored Angles per SIB for all MC loops
                MC_H(iRLB,iMC,iSIB)      = WSC_H;                 %   Stored Heights per SIB for all MC loops
                MC_Rj(iRLB,iMC,iSIB)     = WSC_Rj;                %   Stored Orbit Radius per SIB for all MC loops

                fprintf('nRLB: %i \t\t MCLoop: %i of %i \t\t Positioning Block: %i of %i\t\tTime: %.3f\t\t WSC: %.3f\n',nRLB,iMC,nMC,iSIB,nSIB, t1,WSC_UAV);
                tic
                continue;
            end
            % If it is not a Positioning Block, it's an RL block
            % A single RL iteration is performed by each RL variable in
            % succession (angle, height, orbit radius) for a single dAB
            % estimate

            % dAB estimation and parameter computation
            dAB = normrnd(dAB_R,sigAB);                             %   CSI estimate

            %   RL iteration for angle (greedy on h and Rj)
            [WSCEst_Angle, WSCN_Angle]  = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Angle, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 1, alpha, nUAV, boolFade);

            %   RL iteration for H (greedy on angle and Rj)
            [WSCEst_H, WSCN_H]          = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_H, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 2, alpha, nUAV, boolFade);

            %   RL iteration for Rj (greedy on angle and H)
            [WSCEst_Rj, WSCN_Rj]        = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Rj, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 3, alpha, nUAV, boolFade);
        end
    end
    save(['SIB_EX_nRLB_GenConst-v20nNF_', num2str(iRLB), '.mat'])
end

save('SIB_EX_nRLB_nUAV2-v20-nofade.mat')


%% Multi Varying sigma
clc, clear, close all

% Environment parameters (Urban)

phi = 9.61;         % Environmental constant
omega = 0.16 ;      % Environmental constant
alpha = 0.3;        % Ground Path Loss exponent
alpha_AG = 0.3;     % Air-to-Ground Path Loss Exponent
ne_LOS = 1.0;       % Air-to-Ground LOS attenuation
ne_NLOS = 20;       % Air-to-Ground NLOS attenuation
KbVal = 5;          % K parameter for Bob's channels
KeVal = 5;          % K parameter for Eve's channels


% Channel parameters

m = 3;              % Number of parallel channels
sigma = 1/sqrt(2);  % Noise std dev of each component (real and imag) of every parallel channel

Rs = 1;


channelParam =  [   phi,...
                    omega,...
                    alpha,...
                    alpha_AG,...
                    ne_LOS,...
                    ne_NLOS,...
                    Rs,...
                    KbVal,...
                    KeVal];

% Positions ***************************************************************

    % Alice
nA = 1;             %   Number of Alices
A = [0,0,0];        %   Position of Alice (zero point)
gammaA = 200;        %   Alice Tx SNR

% -------------------------------------------------------------------------

    % Bob
nB = 1;             %   Number of Bobs
dAB_R = 20;        	%   Real distance between A and B

sigAB_V = [dAB_R/40 dAB_R/10 dAB_R/5];
nSig = length(sigAB_V);

B = [dAB_R,0,0];      %   Position of Bob

% -------------------------------------------------------------------------

    % Eve
nR = 100;           %   Number of radial points
nTheta = 360;       %   Number of angular points
nE = nR*nTheta;     %   Number of Eves

rLow = 0.1;         %   Lowest radius of Eve
rHigh = 50;         %   Highest radius of Eve
thetaLow = 0;       %   Lowest angle of Eve
thetaHigh = 2*pi;   %   Highest radius of Eve

rangeR = linspace(rLow,rHigh,nR);                       % Points in Radial dimension
rangeTheta = linspace(thetaLow,thetaHigh,nTheta);       % Points in Angular dimension

thetat  =  repmat(rangeTheta,1,nR);
rt  = (repmat(rangeR',1,nTheta)).';

E = [rt(:).*cos(thetat(:)), rt(:).*sin(thetat(:)) , zeros(nR*nTheta,1)];             % Eves' position (rectangle coordinates)

% -------------------------------------------------------------------------

    % UAV
Nu = 2;                             %   Number of simultaneous UAVs
nUAV = 2;

nAng = 90;                          %   Angle discretization level (opening angle)  -> Number of Angle Actions
nH = 30;                            %   Height discretization level for both UAVs   -> Number of Height Actions
nRj = 30;                           %   Orbit radius discretization level           -> Number of radius Actions


gammaJ = 0.1*gammaA/nUAV;                 %   UAVs Jamming SNR

angleUAV    = linspace(0,2*pi/(nUAV-1),nAng);       %   Possible angle actions (opening angles)
hUAV        = linspace(100,200,nH);                    %   Possible height actions
rjUAV       = linspace(1,rHigh,nRj);                %   Possible orbit radius actions

% -------------------------------------------------------------------------

% *************************************************************************


%   UAV technical parameters [dummy values, research due]

E_RX = 0.1;                 % Energy it takes for UAV to receive positional information [dummy value].
E_ACK = 0.1;                % Energy it takes for UAV to send ACK to Alice [dummy value].
P_Mov = 80;                 % Power it takes UAV to maneuver [dummy value].
v_J = 2;                    % Speed at which UAV moves [dummy value].

%   SIB k-Armed Bandits
nSIB = 20;                  % Number of SIBs to be considered
nRLB = 10; 
nLoops = nSIB*(nRLB+1);     % Number of loops for action choosing (positioning every nRLB+1 loops)

nRL = 1;                    % Number of RL steps per estimate
epsilon = 0;                % Epsilon for epsilon-greedy choice
initWSC = 0;                % Optimistic initial action values
c = 0.3;                    % Exploration parameter for UCB
alpha = 0.5;                  % Step size (0: uniform average)

%   Performance Variables

nMC = 1e2;                              %   Number of Monte Carlo loops for performance evaluation
E_SIB = zeros(nSig,nSIB);                  %   Energy used in a given SIB
S_SIB = zeros(nSig,nSIB);                  %   Cummulative secrecy over a given SIB

MC_WSC = zeros(nSig,nMC,nSIB);               %   Stored WSC per SIB for all MC loops
MC_Ang = zeros(nSig,nMC,nSIB);               %   Stored Angles per SIB for all MC loops
MC_H = zeros(nSig,nMC,nSIB);                 %   Stored Heights per SIB for all MC loops
MC_Rj = zeros(nSig,nMC,nSIB);                %   Stored Orbit Radius per SIB for all MC loops

dtRL = 0.1*3;                             % DUMMY TIME, to get consistent results, not depending on the processor or machine
                                        % Time per RL iteration (the smallest RL iteration)

boolFade = 1;               % Use fading

% Exhaustive Search
tic
fprintf('Exhaustive search!\n')
WSC_ES = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade );
fprintf('\n\n Exhaustive search ended!')
toc

for iRLB = 1:nSig
    sigAB = sigAB_V(iRLB); 
    for iMC = 1:nMC
        % Initialization
        WSCEst_Angle    = initWSC*ones(1,nAng);         %   Action value estimation vector for angle actions
        WSCEst_H        = initWSC*ones(1,nH);           %   Action value estimation vector for height actions
        WSCEst_Rj       = initWSC*ones(1,nRj);          %   Action value estimation vector for orbit radius actions
        WSCN_Angle      = zeros(1,nAng);                %   Vector to store angle action ocurrences
        WSCN_H          = zeros(1,nH);                  %   Vector to store height action ocurrences
        WSCN_Rj         = zeros(1,nRj);                 %   Vector to store orbit radius action ocurrences

        WSC_UAV = 0;                                    %   WSC initial value
        
        WSC_Ang = angleUAV(end);               %   Angle random initialization
        WSC_H   = hUAV(end);                   %   Height random initialization
        WSC_Rj  = rjUAV(end);                  %   Orbit Radius random initialization

        tic
        for i=1:nLoops
            if mod(i,nRLB+1)==0
                % This is a positioning block
                % No RL is performed, and Performance variables are taken at
                % this block only
                t1 = toc;                                   % Time from previous SIB end to this Positioning block
                iSIB = i/(nRLB+1);                          % Number of SIB

                % Obtain new position indeces
                [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   

                % UAV Movement
                WSC_Rj_Old = WSC_Rj;                                                    
                dH      = abs( WSC_H - hUAV(indH) );                                    % Height movement distance
                dRj     = abs( WSC_Rj - rjUAV(indRj) );                                 % Orbit radius movement distance
                dAng    = min(WSC_Rj,WSC_Rj_Old)*abs( WSC_Ang - angleUAV(indAng) )/2;	% Angle movement distance (through lesser orbit)
                dtJ     = (dAng + dH + dRj)/v_J;                                        % Flight time

                % Update UAV positioning values
                WSC_Ang = angleUAV(indAng);        
                WSC_H   = hUAV(indH);
                WSC_Rj  = rjUAV(indRj);

                % Update UAV position
                UAVs = setNewPos_N(nUAV, WSC_Ang, WSC_H, WSC_Rj);

                % Compute secrecy performance metric
                S_SIB(iRLB,iSIB) = S_SIB(iRLB,iSIB) +  dtRL*nRLB*3*WSC_UAV/nMC;

                % Compute energy metric
                E_SIB(iRLB,iSIB) = E_SIB(iRLB,iSIB) +  (E_RX + E_ACK + dtJ*P_Mov)/nMC;

                % Update UAV WSC value
                [WSC_UAV, DeltaComp] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_UAV = WSC_UAV/(nTheta*nR);

                % Store control values per MC loop
                MC_WSC(iRLB,iMC,iSIB)    = WSC_UAV;
                MC_Ang(iRLB,iMC,iSIB)    = WSC_Ang;               %   Stored Angles per SIB for all MC loops
                MC_H(iRLB,iMC,iSIB)      = WSC_H;                 %   Stored Heights per SIB for all MC loops
                MC_Rj(iRLB,iMC,iSIB)     = WSC_Rj;                %   Stored Orbit Radius per SIB for all MC loops

                fprintf('sigma: %.3f \t\t MCLoop: %i of %i \t\t Positioning Block: %i of %i\t\tTime: %.3f\t\t WSC: %.3f\n',sigAB,iMC,nMC,iSIB,nSIB, t1,WSC_UAV);
                tic
                continue;
            end
            % If it is not a Positioning Block, it's an RL block
            % A single RL iteration is performed by each RL variable in
            % succession (angle, height, orbit radius) for a single dAB
            % estimate

            % dAB estimation and parameter computation
            dAB = normrnd(dAB_R,sigAB);                             %   CSI estimate

            %   RL iteration for angle (greedy on h and Rj)
            [WSCEst_Angle, WSCN_Angle]  = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Angle, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 1, alpha, nUAV, boolFade);

            %   RL iteration for H (greedy on angle and Rj)
            [WSCEst_H, WSCN_H]          = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_H, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 2, alpha, nUAV, boolFade);

            %   RL iteration for Rj (greedy on angle and H)
            [WSCEst_Rj, WSCN_Rj]        = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Rj, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 3, alpha, nUAV, boolFade);
        end
    end
    save(['SIB_EX_nSig_GenConst-v0F_', num2str(iRLB), '.mat'])
end

save('SIB_EX_nSig_nUAV2-v10-fade.mat')

%% Multi Varying K (same for B and E)
clc, clear, close all

% Environment parameters (Urban)

phi = 9.61;         % Environmental constant
omega = 0.16 ;      % Environmental constant
alpha = 0.3;        % Ground Path Loss exponent
alpha_AG = 0.3;     % Air-to-Ground Path Loss Exponent
ne_LOS = 1.0;       % Air-to-Ground LOS attenuation
ne_NLOS = 20;       % Air-to-Ground NLOS attenuation

KbVal_v = [0, 5, 10];
KeVal_v = [0, 5, 10];

nKval = length(KbVal_v);
% Channel parameters

m = 3;              % Number of parallel channels
sigma = 1/sqrt(2);  % Noise std dev of each component (real and imag) of every parallel channel

Rs = 1;


% Positions ***************************************************************

    % Alice
nA = 1;             %   Number of Alices
A = [0,0,0];        %   Position of Alice (zero point)
gammaA = 200;        %   Alice Tx SNR

% -------------------------------------------------------------------------

    % Bob
nB = 1;             %   Number of Bobs
dAB_R = 20;        	%   Real distance between A and B
sigAB = dAB_R/10;
B = [dAB_R,0,0];      %   Position of Bob

% -------------------------------------------------------------------------

    % Eve
nR = 100;           %   Number of radial points
nTheta = 360;       %   Number of angular points
nE = nR*nTheta;     %   Number of Eves

rLow = 0.1;         %   Lowest radius of Eve
rHigh = 50;         %   Highest radius of Eve
thetaLow = 0;       %   Lowest angle of Eve
thetaHigh = 2*pi;   %   Highest radius of Eve

rangeR = linspace(rLow,rHigh,nR);                       % Points in Radial dimension
rangeTheta = linspace(thetaLow,thetaHigh,nTheta);       % Points in Angular dimension

thetat  =  repmat(rangeTheta,1,nR);
rt  = (repmat(rangeR',1,nTheta)).';

E = [rt(:).*cos(thetat(:)), rt(:).*sin(thetat(:)) , zeros(nR*nTheta,1)];             % Eves' position (rectangle coordinates)

% -------------------------------------------------------------------------

    % UAV
Nu = 2;                             %   Number of simultaneous UAVs
nUAV = 2;

nAng = 90;                          %   Angle discretization level (opening angle)  -> Number of Angle Actions
nH = 30;                            %   Height discretization level for both UAVs   -> Number of Height Actions
nRj = 30;                           %   Orbit radius discretization level           -> Number of radius Actions

gammaJ = 0.1*gammaA/nUAV;                 %   UAVs Jamming SNR

angleUAV    = linspace(0,2*pi/(nUAV-1),nAng);       %   Possible angle actions (opening angles)
hUAV        = linspace(100,200,nH);                    %   Possible height actions
rjUAV       = linspace(1,rHigh,nRj);                %   Possible orbit radius actions

% -------------------------------------------------------------------------

% *************************************************************************


%   UAV technical parameters [dummy values, research due]

E_RX = 0.1;                 % Energy it takes for UAV to receive positional information [dummy value].
E_ACK = 0.1;                % Energy it takes for UAV to send ACK to Alice [dummy value].
P_Mov = 80;                 % Power it takes UAV to maneuver [dummy value].
v_J = 2;                    % Speed at which UAV moves [dummy value].

%   SIB k-Armed Bandits
nSIB = 20;                  % Number of SIBs to be considered
nRLB = 10; % ------------------------------------------------------------------------------------- CHECK THIS
nLoops = nSIB*(nRLB+1);     % Number of loops for action choosing (positioning every nRLB+1 loops)
nRL = 1;                    % Number of RL steps per estimate
epsilon = 0;                % Epsilon for epsilon-greedy choice
initWSC = 0;                % Optimistic initial action values
c = 0.3;                    % Exploration parameter for UCB
alpha = 0.5;                  % Step size (0: uniform average)

%   Performance Variables

nMC = 1e2;                              %   Number of Monte Carlo loops for performance evaluation
E_SIB = zeros(nKval,nSIB);                  %   Energy used in a given SIB
S_SIB = zeros(nKval,nSIB);                  %   Cummulative secrecy over a given SIB

MC_WSC = zeros(nKval,nMC,nSIB);               %   Stored WSC per SIB for all MC loops
MC_Ang = zeros(nKval,nMC,nSIB);               %   Stored Angles per SIB for all MC loops
MC_H = zeros(nKval,nMC,nSIB);                 %   Stored Heights per SIB for all MC loops
MC_Rj = zeros(nKval,nMC,nSIB);                %   Stored Orbit Radius per SIB for all MC loops

dtRL = 0.1*3;                             % DUMMY TIME, to get consistent results, not depending on the processor or machine
                                        % Time per RL iteration (the smallest RL iteration)

boolFade = 1;               % Use fading

WSC_ES = zeros(nKval, length(angleUAV), length(hUAV), length(rjUAV));

for iRLB = 1:nKval
    KbVal = KbVal_v(iRLB);          % K parameter for Bob's channels
    KeVal = KeVal_v(iRLB);          % K parameter for Eve's channels
    
    channelParam =  [   phi,...
                    omega,...
                    0.3,...
                    alpha_AG,...
                    ne_LOS,...
                    ne_NLOS,...
                    Rs,...
                    KbVal,...
                    KeVal];

    % Exhaustive Search
    tic
    fprintf('Exhaustive search!\n')
    WSC_ES(iRLB,:,:,:) = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade );
    fprintf('\n\n Exhaustive search ended!')
    toc
    for iMC = 1:nMC
        % Initialization
        WSCEst_Angle    = initWSC*ones(1,nAng);         %   Action value estimation vector for angle actions
        WSCEst_H        = initWSC*ones(1,nH);           %   Action value estimation vector for height actions
        WSCEst_Rj       = initWSC*ones(1,nRj);          %   Action value estimation vector for orbit radius actions
        WSCN_Angle      = zeros(1,nAng);                %   Vector to store angle action ocurrences
        WSCN_H          = zeros(1,nH);                  %   Vector to store height action ocurrences
        WSCN_Rj         = zeros(1,nRj);                 %   Vector to store orbit radius action ocurrences

        WSC_UAV = 0;                                    %   WSC initial value
        WSC_Ang = angleUAV(end);               %   Angle random initialization
        WSC_H   = hUAV(end);                   %   Height random initialization
        WSC_Rj  = rjUAV(end);                  %   Orbit Radius random initialization

        tic
        for i=1:nLoops
            if mod(i,nRLB+1)==0
                % This is a positioning block
                % No RL is performed, and Performance variables are taken at
                % this block only
                t1 = toc;                                   % Time from previous SIB end to this Positioning block
                iSIB = i/(nRLB+1);                          % Number of SIB

                % Obtain new position indeces
                [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   

                % UAV Movement
                WSC_Rj_Old = WSC_Rj;                                                    
                dH      = abs( WSC_H - hUAV(indH) );                                    % Height movement distance
                dRj     = abs( WSC_Rj - rjUAV(indRj) );                                 % Orbit radius movement distance
                dAng    = min(WSC_Rj,WSC_Rj_Old)*abs( WSC_Ang - angleUAV(indAng) )/2;	% Angle movement distance (through lesser orbit)
                dtJ     = (dAng + dH + dRj)/v_J;                                        % Flight time

                % Update UAV positioning values
                WSC_Ang = angleUAV(indAng);        
                WSC_H   = hUAV(indH);
                WSC_Rj  = rjUAV(indRj);

                % Update UAV position
                UAVs = setNewPos_N(nUAV, WSC_Ang, WSC_H, WSC_Rj);

                % Compute secrecy performance metric
                S_SIB(iRLB,iSIB) = S_SIB(iRLB,iSIB) +  dtRL*nRLB*3*WSC_UAV/nMC;

                % Compute energy metric
                E_SIB(iRLB,iSIB) = E_SIB(iRLB,iSIB) +  (E_RX + E_ACK + dtJ*P_Mov)/nMC;

                % Update UAV WSC value
                [WSC_UAV, DeltaComp] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_UAV = WSC_UAV/(nTheta*nR);

                % Store control values per MC loop
                MC_WSC(iRLB,iMC,iSIB)    = WSC_UAV;
                MC_Ang(iRLB,iMC,iSIB)    = WSC_Ang;               %   Stored Angles per SIB for all MC loops
                MC_H(iRLB,iMC,iSIB)      = WSC_H;                 %   Stored Heights per SIB for all MC loops
                MC_Rj(iRLB,iMC,iSIB)     = WSC_Rj;                %   Stored Orbit Radius per SIB for all MC loops

                fprintf('K: %.1f \t\t MCLoop: %i of %i \t\t Positioning Block: %i of %i\t\tTime: %.3f\t\t WSC: %.3f\n',KbVal,iMC,nMC,iSIB,nSIB, t1,WSC_UAV);
                tic
                continue;
            end
            % If it is not a Positioning Block, it's an RL block
            % A single RL iteration is performed by each RL variable in
            % succession (angle, height, orbit radius) for a single dAB
            % estimate

            % dAB estimation and parameter computation
            dAB = normrnd(dAB_R,sigAB);                             %   CSI estimate

            %   RL iteration for angle (greedy on h and Rj)
            [WSCEst_Angle, WSCN_Angle]  = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Angle, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 1, alpha, nUAV, boolFade);

            %   RL iteration for H (greedy on angle and Rj)
            [WSCEst_H, WSCN_H]          = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_H, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 2, alpha, nUAV, boolFade);

            %   RL iteration for Rj (greedy on angle and H)
            [WSCEst_Rj, WSCN_Rj]        = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Rj, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 3, alpha, nUAV, boolFade);
        end
    end
    save(['SIB_EX_nK_GenConst-v0F_', num2str(iRLB), '.mat'])
end

save('SIB_EX_nK_nUAV2-v10-fade.mat')
%% Multi Varying nUAV
clc, clear, close all

% Environment parameters (Urban)

phi = 9.61;         % Environmental constant
omega = 0.16 ;      % Environmental constant
alpha = 0.3;        % Ground Path Loss exponent
alpha_AG = 0.3;     % Air-to-Ground Path Loss Exponent
ne_LOS = 1.0;       % Air-to-Ground LOS attenuation
ne_NLOS = 20;       % Air-to-Ground NLOS attenuation

KbVal = 5;
KeVal = 5;

% Channel parameters

m = 3;              % Number of parallel channels
sigma = 1/sqrt(2);  % Noise std dev of each component (real and imag) of every parallel channel

Rs = 1;

channelParam =  [   phi,...
                    omega,...
                    alpha,...
                    alpha_AG,...
                    ne_LOS,...
                    ne_NLOS,...
                    Rs,...
                    KbVal,...
                    KeVal];


% Positions ***************************************************************

    % Alice
nA = 1;             %   Number of Alices
A = [0,0,0];        %   Position of Alice (zero point)
gammaA = 200;        %   Alice Tx SNR

% -------------------------------------------------------------------------

    % Bob
nB = 1;             %   Number of Bobs
dAB_R = 20;        	%   Real distance between A and B
sigAB = dAB_R/10;
B = [dAB_R,0,0];      %   Position of Bob

% -------------------------------------------------------------------------

    % Eve
nR = 100;           %   Number of radial points
nTheta = 360;       %   Number of angular points
nE = nR*nTheta;     %   Number of Eves

rLow = 0.1;         %   Lowest radius of Eve
rHigh = 50;         %   Highest radius of Eve
thetaLow = 0;       %   Lowest angle of Eve
thetaHigh = 2*pi;   %   Highest radius of Eve

rangeR = linspace(rLow,rHigh,nR);                       % Points in Radial dimension
rangeTheta = linspace(thetaLow,thetaHigh,nTheta);       % Points in Angular dimension

thetat  =  repmat(rangeTheta,1,nR);
rt  = (repmat(rangeR',1,nTheta)).';

E = [rt(:).*cos(thetat(:)), rt(:).*sin(thetat(:)) , zeros(nR*nTheta,1)];             % Eves' position (rectangle coordinates)

% -------------------------------------------------------------------------

    % UAV
Nu = 2;                             %   Number of simultaneous UAVs
nUAV_v = [2 4 8];
NnUAV = length(nUAV_v);

nAng = 90;                          %   Angle discretization level (opening angle)  -> Number of Angle Actions
nH = 30;                            %   Height discretization level for both UAVs   -> Number of Height Actions
nRj = 30;                           %   Orbit radius discretization level           -> Number of radius Actions

hUAV        = linspace(100,200,nH);                    %   Possible height actions
rjUAV       = linspace(1,rHigh,nRj);                %   Possible orbit radius actions

% -------------------------------------------------------------------------

% *************************************************************************


%   UAV technical parameters [dummy values, research due]

E_RX = 0.1;                 % Energy it takes for UAV to receive positional information [dummy value].
E_ACK = 0.1;                % Energy it takes for UAV to send ACK to Alice [dummy value].
P_Mov = 80;                 % Power it takes UAV to maneuver [dummy value].
v_J = 2;                    % Speed at which UAV moves [dummy value].

%   SIB k-Armed Bandits
nSIB = 20;                  % Number of SIBs to be considered
nRLB = 10; 
nLoops = nSIB*(nRLB+1);     % Number of loops for action choosing (positioning every nRLB+1 loops)

nRL = 1;                    % Number of RL steps per estimate
epsilon = 0;                % Epsilon for epsilon-greedy choice
initWSC = 0;                % Optimistic initial action values
c = 0.3;                    % Exploration parameter for UCB
alpha = 0.5;                  % Step size (0: uniform average)

%   Performance Variables

nMC = 1e2;                              %   Number of Monte Carlo loops for performance evaluation
E_SIB = zeros(NnUAV,nSIB);                  %   Energy used in a given SIB
S_SIB = zeros(NnUAV,nSIB);                  %   Cummulative secrecy over a given SIB

MC_WSC = zeros(NnUAV,nMC,nSIB);               %   Stored WSC per SIB for all MC loops
MC_Ang = zeros(NnUAV,nMC,nSIB);               %   Stored Angles per SIB for all MC loops
MC_H = zeros(NnUAV,nMC,nSIB);                 %   Stored Heights per SIB for all MC loops
MC_Rj = zeros(NnUAV,nMC,nSIB);                %   Stored Orbit Radius per SIB for all MC loops

dtRL = 0.1*3;                             % DUMMY TIME, to get consistent results, not depending on the processor or machine
                                        % Time per RL iteration (the smallest RL iteration)

boolFade = 1;               % Use fading


for iRLB = 1:NnUAV
    nUAV = NnUAV(iRLB);
    gammaJ = 0.1*gammaA/nUAV;                 %   UAVs Jamming SNR
    angleUAV    = linspace(0,2*pi/(nUAV-1),nAng);       %   Possible angle actions (opening angles)

    % Exhaustive Search
    tic
    fprintf('Exhaustive search!\n')
    WSC_ES(iRLB,:,:,:) = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade );
    fprintf('\n\n Exhaustive search ended!')
    toc
    for iMC = 1:nMC
        % Initialization
        WSCEst_Angle    = initWSC*ones(1,nAng);         %   Action value estimation vector for angle actions
        WSCEst_H        = initWSC*ones(1,nH);           %   Action value estimation vector for height actions
        WSCEst_Rj       = initWSC*ones(1,nRj);          %   Action value estimation vector for orbit radius actions
        WSCN_Angle      = zeros(1,nAng);                %   Vector to store angle action ocurrences
        WSCN_H          = zeros(1,nH);                  %   Vector to store height action ocurrences
        WSCN_Rj         = zeros(1,nRj);                 %   Vector to store orbit radius action ocurrences

        WSC_UAV = 0;                                    %   WSC initial value
        
        WSC_Ang = angleUAV(end);               %   Angle random initialization
        WSC_H   = hUAV(end);                   %   Height random initialization
        WSC_Rj  = rjUAV(end);                  %   Orbit Radius random initialization

        tic
        for i=1:nLoops
            if mod(i,nRLB+1)==0
                % This is a positioning block
                % No RL is performed, and Performance variables are taken at
                % this block only
                t1 = toc;                                   % Time from previous SIB end to this Positioning block
                iSIB = i/(nRLB+1);                          % Number of SIB

                % Obtain new position indeces
                [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   

                % UAV Movement
                WSC_Rj_Old = WSC_Rj;                                                    
                dH      = abs( WSC_H - hUAV(indH) );                                    % Height movement distance
                dRj     = abs( WSC_Rj - rjUAV(indRj) );                                 % Orbit radius movement distance
                dAng    = min(WSC_Rj,WSC_Rj_Old)*abs( WSC_Ang - angleUAV(indAng) )/2;	% Angle movement distance (through lesser orbit)
                dtJ     = (dAng + dH + dRj)/v_J;                                        % Flight time

                % Update UAV positioning values
                WSC_Ang = angleUAV(indAng);        
                WSC_H   = hUAV(indH);
                WSC_Rj  = rjUAV(indRj);

                % Update UAV position
                UAVs = setNewPos_N(nUAV, WSC_Ang, WSC_H, WSC_Rj);

                % Compute secrecy performance metric
                S_SIB(iRLB,iSIB) = S_SIB(iRLB,iSIB) +  dtRL*nRLB*3*WSC_UAV/nMC;

                % Compute energy metric
                E_SIB(iRLB,iSIB) = E_SIB(iRLB,iSIB) +  (E_RX + E_ACK + dtJ*P_Mov)/nMC;

                % Update UAV WSC value
                [WSC_UAV, DeltaComp] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_UAV = WSC_UAV/(nTheta*nR);

                % Store control values per MC loop
                MC_WSC(iRLB,iMC,iSIB)    = WSC_UAV;
                MC_Ang(iRLB,iMC,iSIB)    = WSC_Ang;               %   Stored Angles per SIB for all MC loops
                MC_H(iRLB,iMC,iSIB)      = WSC_H;                 %   Stored Heights per SIB for all MC loops
                MC_Rj(iRLB,iMC,iSIB)     = WSC_Rj;                %   Stored Orbit Radius per SIB for all MC loops

                fprintf('nUAV: %.1f \t\t MCLoop: %i of %i \t\t Positioning Block: %i of %i\t\tTime: %.3f\t\t WSC: %.3f\n',nUAV,iMC,nMC,iSIB,nSIB, t1,WSC_UAV);
                tic
                continue;
            end
            % If it is not a Positioning Block, it's an RL block
            % A single RL iteration is performed by each RL variable in
            % succession (angle, height, orbit radius) for a single dAB
            % estimate

            % dAB estimation and parameter computation
            dAB = normrnd(dAB_R,sigAB);                             %   CSI estimate

            %   RL iteration for angle (greedy on h and Rj)
            [WSCEst_Angle, WSCN_Angle]  = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Angle, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 1, alpha, nUAV, boolFade);

            %   RL iteration for H (greedy on angle and Rj)
            [WSCEst_H, WSCN_H]          = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_H, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 2, alpha, nUAV, boolFade);

            %   RL iteration for Rj (greedy on angle and H)
            [WSCEst_Rj, WSCN_Rj]        = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Rj, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 3, alpha, nUAV, boolFade);
        end
    end
    save(['SIB_EX_nUAVN_GenConst-v0F_', num2str(iRLB), '.mat'])
end

save('SIB_EX_nUAVN-v10-fade.mat')

%% Multi Varying c
clc, clear, close all

% Environment parameters (Urban)

phi = 9.61;         % Environmental constant
omega = 0.16 ;      % Environmental constant
alpha = 0.3;        % Ground Path Loss exponent
alpha_AG = 0.3;     % Air-to-Ground Path Loss Exponent
ne_LOS = 1.0;       % Air-to-Ground LOS attenuation
ne_NLOS = 20;       % Air-to-Ground NLOS attenuation
KbVal = 5;          % K parameter for Bob's channels
KeVal = 5;          % K parameter for Eve's channels


% Channel parameters

m = 3;              % Number of parallel channels
sigma = 1/sqrt(2);  % Noise std dev of each component (real and imag) of every parallel channel

Rs = 1;


channelParam =  [   phi,...
                    omega,...
                    alpha,...
                    alpha_AG,...
                    ne_LOS,...
                    ne_NLOS,...
                    Rs,...
                    KbVal,...
                    KeVal];

% Positions ***************************************************************

    % Alice
nA = 1;             %   Number of Alices
A = [0,0,0];        %   Position of Alice (zero point)
gammaA = 200;        %   Alice Tx SNR

% -------------------------------------------------------------------------

    % Bob
nB = 1;             %   Number of Bobs
dAB_R = 20;        	%   Real distance between A and B
sigAB = dAB_R/10;          %   Unreliability of B's position
B = [dAB_R,0,0];      %   Position of Bob

% -------------------------------------------------------------------------

    % Eve
nR = 100;           %   Number of radial points
nTheta = 360;       %   Number of angular points
nE = nR*nTheta;     %   Number of Eves

rLow = 0.1;         %   Lowest radius of Eve
rHigh = 50;         %   Highest radius of Eve
thetaLow = 0;       %   Lowest angle of Eve
thetaHigh = 2*pi;   %   Highest radius of Eve

rangeR = linspace(rLow,rHigh,nR);                       % Points in Radial dimension
rangeTheta = linspace(thetaLow,thetaHigh,nTheta);       % Points in Angular dimension

thetat  =  repmat(rangeTheta,1,nR);
rt  = (repmat(rangeR',1,nTheta)).';

E = [rt(:).*cos(thetat(:)), rt(:).*sin(thetat(:)) , zeros(nR*nTheta,1)];             % Eves' position (rectangle coordinates)

% -------------------------------------------------------------------------

    % UAV
Nu = 2;                             %   Number of simultaneous UAVs
nUAV = 2;

nAng = 90;                          %   Angle discretization level (opening angle)  -> Number of Angle Actions
nH = 30;                            %   Height discretization level for both UAVs   -> Number of Height Actions
nRj = 30;                           %   Orbit radius discretization level           -> Number of radius Actions

gammaJ = 0.1*gammaA/nUAV;                 %   UAVs Jamming SNR

angleUAV    = linspace(0,2*pi/(nUAV-1),nAng);       %   Possible angle actions (opening angles)
hUAV        = linspace(100,200,nH);                    %   Possible height actions
rjUAV       = linspace(1,rHigh,nRj);                %   Possible orbit radius actions

% -------------------------------------------------------------------------

% *************************************************************************


%   UAV technical parameters [dummy values, research due]

E_RX = 0.1;                 % Energy it takes for UAV to receive positional information [dummy value].
E_ACK = 0.1;                % Energy it takes for UAV to send ACK to Alice [dummy value].
P_Mov = 80;                 % Power it takes UAV to maneuver [dummy value].
v_J = 2;                    % Speed at which UAV moves [dummy value].

%   SIB k-Armed Bandits
nSIB = 20;                  % Number of SIBs to be considered
nRLB = 10; 
nLoops = nSIB*(nRLB+1);     % Number of loops for action choosing (positioning every nRLB+1 loops)

nRL = 1;                    % Number of RL steps per estimate
epsilon = 0;                % Epsilon for epsilon-greedy choice
initWSC = 0;                % Optimistic initial action values

cV = [0.01 0.1 0.3];                  % Exploration parameter for UCB
nC = length(cV);

%c = 0.3;                    % Exploration parameter for UCB
alpha = 0.5;                  % Step size (0: uniform average)

%   Performance Variables

nMC = 1e2;                              %   Number of Monte Carlo loops for performance evaluation
E_SIB = zeros(nC,nSIB);                  %   Energy used in a given SIB
S_SIB = zeros(nC,nSIB);                  %   Cummulative secrecy over a given SIB

MC_WSC = zeros(nC,nMC,nSIB);               %   Stored WSC per SIB for all MC loops
MC_Ang = zeros(nC,nMC,nSIB);               %   Stored Angles per SIB for all MC loops
MC_H = zeros(nC,nMC,nSIB);                 %   Stored Heights per SIB for all MC loops
MC_Rj = zeros(nC,nMC,nSIB);                %   Stored Orbit Radius per SIB for all MC loops

dtRL = 0.1*3;                             % DUMMY TIME, to get consistent results, not depending on the processor or machine
                                        % Time per RL iteration (the smallest RL iteration)

boolFade = 1;               % Use fading

% Exhaustive Search
tic
fprintf('Exhaustive search!\n')
WSC_ES = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade );
fprintf('\n\n Exhaustive search ended!')
toc



for iRLB = 1:nC
    c = cV(iRLB);
    for iMC = 1:nMC
        % Initialization
        WSCEst_Angle    = initWSC*ones(1,nAng);         %   Action value estimation vector for angle actions
        WSCEst_H        = initWSC*ones(1,nH);           %   Action value estimation vector for height actions
        WSCEst_Rj       = initWSC*ones(1,nRj);          %   Action value estimation vector for orbit radius actions
        WSCN_Angle      = zeros(1,nAng);                %   Vector to store angle action ocurrences
        WSCN_H          = zeros(1,nH);                  %   Vector to store height action ocurrences
        WSCN_Rj         = zeros(1,nRj);                 %   Vector to store orbit radius action ocurrences

        WSC_UAV = 0;                                    %   WSC initial value

        WSC_Ang = angleUAV(end);               %   Angle random initialization
        WSC_H   = hUAV(end);                   %   Height random initialization
        WSC_Rj  = rjUAV(end);                  %   Orbit Radius random initialization

        tic
        for i=1:nLoops
            if mod(i,nRLB+1)==0
                % This is a positioning block
                % No RL is performed, and Performance variables are taken at
                % this block only
                t1 = toc;                                   % Time from previous SIB end to this Positioning block
                iSIB = i/(nRLB+1);                          % Number of SIB

                % Obtain new position indeces
                [indAng, indH, indRj] = getNewPos(WSCEst_Angle, WSCEst_H, WSCEst_Rj);   

                % UAV Movement
                WSC_Rj_Old = WSC_Rj;                                                    
                dH      = abs( WSC_H - hUAV(indH) );                                    % Height movement distance
                dRj     = abs( WSC_Rj - rjUAV(indRj) );                                 % Orbit radius movement distance
                dAng    = min(WSC_Rj,WSC_Rj_Old)*abs( WSC_Ang - angleUAV(indAng) )/2;	% Angle movement distance (through lesser orbit)
                dtJ     = (dAng + dH + dRj)/v_J;                                        % Flight time

                % Update UAV positioning values
                WSC_Ang = angleUAV(indAng);        
                WSC_H   = hUAV(indH);
                WSC_Rj  = rjUAV(indRj);

                % Update UAV position
                UAVs = setNewPos_N(nUAV, WSC_Ang, WSC_H, WSC_Rj);

                % Compute secrecy performance metric
                S_SIB(iRLB,iSIB) = S_SIB(iRLB,iSIB) +  dtRL*nRLB*3*WSC_UAV/nMC;

                % Compute energy metric
                E_SIB(iRLB,iSIB) = E_SIB(iRLB,iSIB) +  (E_RX + E_ACK + dtJ*P_Mov)/nMC;

                % Update UAV WSC value
                [WSC_UAV, DeltaComp] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_UAV = WSC_UAV/(nTheta*nR);

                % Store control values per MC loop
                MC_WSC(iRLB,iMC,iSIB)    = WSC_UAV;
                MC_Ang(iRLB,iMC,iSIB)    = WSC_Ang;               %   Stored Angles per SIB for all MC loops
                MC_H(iRLB,iMC,iSIB)      = WSC_H;                 %   Stored Heights per SIB for all MC loops
                MC_Rj(iRLB,iMC,iSIB)     = WSC_Rj;                %   Stored Orbit Radius per SIB for all MC loops

                fprintf('c: %.3f \t\t MCLoop: %i of %i \t\t Positioning Block: %i of %i\t\tTime: %.3f\t\t WSC: %.3f\n',c,iMC,nMC,iSIB,nSIB, t1,WSC_UAV);
                tic
                continue;
            end
            % If it is not a Positioning Block, it's an RL block
            % A single RL iteration is performed by each RL variable in
            % succession (angle, height, orbit radius) for a single dAB
            % estimate

            % dAB estimation and parameter computation
            dAB = normrnd(dAB_R,sigAB);                             %   CSI estimate

            %   RL iteration for angle (greedy on h and Rj)
            [WSCEst_Angle, WSCN_Angle]  = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Angle, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 1, alpha, nUAV, boolFade);

            %   RL iteration for H (greedy on angle and Rj)
            [WSCEst_H, WSCN_H]          = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_H, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 2, alpha, nUAV, boolFade);

            %   RL iteration for Rj (greedy on angle and H)
            [WSCEst_Rj, WSCN_Rj]        = computeRLBlock_UCB_N(WSCEst_Angle, WSCEst_H, WSCEst_Rj, WSCN_Rj, angleUAV, hUAV, rjUAV, ...
                                                A, E, dAB, gammaA, gammaJ, c, channelParam, i, 3, alpha, nUAV, boolFade);
        end
    end
    save(['SIB_EX_nC_GenConst-v0F_', num2str(iRLB), '.mat'])
end

save('SIB_EX_nC_nUAV2-v10-fade.mat')
