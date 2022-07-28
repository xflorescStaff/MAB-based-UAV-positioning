function SOP = SOP_numint(x, OmAB, OmvB, KvB, OmAE, OmvE, KvE)

    %SOP_int = zeros(1,length(x));
%     SOP = [];
%     int = 0.001;
%     x = 0;
%     while 0==0 
%         fun = @(y)gammaCDF( ((2.^x)*(1+y)) - 1 , OmAB, OmvB, KvB) .*  gammaPDF(y, OmAE, OmvE, KvE);
%         num_int = integral(fun,0,Inf);
%         SOP = [SOP, num_int];
%         if (num_int >=1)
%             break;
%         end
%         x = x + int;
%     end
    
    
    SOP = zeros(size(OmAE,1),length(x));
    for i = 1:length(x)
        fun = @(y)gammaCDF( ((2.^x(i))*(1+y)) - 1 , OmAB, OmvB, KvB) .*  gammaPDF(y, OmAE, OmvE, KvE);
        %aux = integral(fun,0,Inf, 'ArrayValued', 1);
        %size(aux)
        SOP(:,i) = integral(fun,0,Inf, 'ArrayValued', 1);
    end
    
end