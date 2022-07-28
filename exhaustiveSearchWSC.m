function WSC_ES = exhaustiveSearchWSC(A, E, nUAV, angleUAV, hUAV, rjUAV, dAB_R, gammaA, gammaJ, channelParam, boolFade )

    B = A;
    B(1) = B(1) + dAB_R;
    % Channel parameters
    phi         = channelParam (1);
    omega       = channelParam (2);
    alpha       = channelParam (3);
    alpha_AG    = channelParam (4);
    ne_LOS      = channelParam (5);
    ne_NLOS     = channelParam (6);
    Rs          = channelParam (7);
    
    WSC_ES = zeros(length(angleUAV), length(hUAV), length(rjUAV));
    
    for iAng = 1:length(angleUAV)
        Ang = angleUAV(iAng);
        fprintf('Ang: %.3f, \t (%i / %i)\n', Ang, iAng, length(angleUAV));
        for iH = 1:length(hUAV)
            h = hUAV(iH);
            for iR = 1:length(rjUAV)
                Rj = rjUAV(iR);
                
                UAVs = setNewPos_N(nUAV, Ang, h, Rj);
                [WSC_Val, ~] = computeWSC_NUAV(A, E, UAVs, dAB_R, gammaA, gammaJ, channelParam, boolFade );
                WSC_ES(iAng, iH, iR) = WSC_Val;
                                
%                 dJE = sqrt( ( UAVs(:,1) - E(:,1)' ).^2 + ( UAVs(:,2) - E(:,2)' ).^2  + ( UAVs(:,3) - E(:,3)' ).^2);
%                 Theta_JE = (180/pi) * asin(UAVs(:,3)./dJE);
%                 PLOS_JE = 1./(1 + phi * exp( -omega*( Theta_JE - phi ) ) );
%                 LJE = PLOS_JE.*(abs(dJE).^alpha_AG)*ne_LOS + (1-PLOS_JE).*(abs(dJE).^alpha_AG)*ne_NLOS;
%                 gJE = 1./LJE;
%                 intJE = gammaJ*sum(gJE,1);
%                 
%                 dJB = sqrt( ( UAVs(:,1) - B(1) ).^2 + ( UAVs(:,2) - B(2) ).^2  + ( UAVs(:,3) - B(3) ).^2);
%                 Theta_JB = (180/pi) * asin(UAVs(:,3)./dJB);
%                 PLOS_JB = 1./(1 + phi * exp( -omega*( Theta_JB - phi ) ) );
%                 LJB = PLOS_JB.*(abs(dJB).^alpha_AG)*ne_LOS + (1-PLOS_JB).*(abs(dJB).^alpha_AG)*ne_NLOS;
%                 gJB = 1./LJB;
%                 intJB = gammaJ*sum(gJB,1);
%                 
%                 
%                 dAE = transpose(sqrt( ( A(:,1) - E(:,1) ).^2 + ( A(:,2) - E(:,2) ).^2 ));
%                 OmegaAE = dAE.^alpha;
%                 OmegaAB = dAB_R.^alpha;
%                 
%                 aNJ = gammaA;
%                 bNJ = gammaA;
%                 
%                 aJ = gammaA ./ ( 1 + intJB );
%                 bJ = gammaA ./ ( 1 + intJE );
%                 
%                 SOP_J   = 1 - (exp( -(OmegaAB./aJ ).*(2^Rs - 1) ))*( 1 ./ ( (2^Rs)*(OmegaAB./OmegaAE).*( bJ./aJ ) + 1 ) );
%                 SOP_NJ  = 1 - (exp( -(OmegaAB./aNJ).*(2^Rs - 1) ))*( 1 ./ ( (2^Rs)*(OmegaAB./OmegaAE).*(bNJ./aNJ) + 1 ) );
% 
%                 DeltaComp = (1-SOP_J)./(1-SOP_NJ);
% 
%                 coverage    =  sum( DeltaComp(:)>=1);
%                 efficiency  = mean( DeltaComp(:) );
%                 WSC_Val     = coverage*efficiency;
                    
%                 WSC_ES(iAng, iH, iR) = WSC_Val;
                
            end
        end
    end

end