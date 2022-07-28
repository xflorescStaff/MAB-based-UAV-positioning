function Fx = gammaCDF(xr, OmA, Omv, Kv)
    N = size(Kv,2);
    mterm1 = 1;
    sterm1 = 0;
    sterm2 = 0;
    for i=1:N
        alph = (1 + Kv(:,i))/Omv(:,i);
        bet  = alph ./ (alph + (xr/OmA));
        betB = 1 ./ (alph + (xr/OmA));

        mterm1 = mterm1.*bet;
        sterm1 = sterm1+ ( bet.*Kv(:,i) );
        sterm2 = sterm2 + ( betB.*( 1 + ( bet.*Kv(:,i) ) ) );
    end
    
    Fx = 1 - ( exp(-xr/OmA).*exp(-sum(Kv,2)).*exp(sterm1).*mterm1 );
end