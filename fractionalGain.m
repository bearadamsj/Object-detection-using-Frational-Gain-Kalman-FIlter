%fractional gain Kalman filter function
%for every iteration it takes the new gain, iteration number and the sum of
%all the previous gain. as a output, it gives processed new gain , new
%iteration number, new factorial and the new summation.
function [k_new, iteration,facto,k_sum,Klist] = fractionalGain(K_kalman,a,iteration,Klist,facto,k_sum)
K_0 = ones(4,1);
% if it is the first iteration, use K0 as the previous gain.
    if iteration == 1 
        k_frac = ((-1)^iteration)*K_0;%fractional gain
        k_new = K_kalman + k_frac;
        Klist(:,:,iteration) = k_new;%update the Klist
        k_sum = k_sum + k_frac;%update K_new
%if it is not the first iteration 
    else  
        a_facto = (a-iteration+2)/(iteration-1);  %last term of (alpha 1)'   
        facto = facto*a_facto; % (alpha 1)'
        k_frac = ((-1)^(iteration-1))*facto*Klist(iteration-1); %fractional gain
        k_sum = k_sum + k_frac; %summation of gains
        k_new = K_kalman + (k_sum + k_frac)/iteration; 
        Klist(:,:,iteration) = k_new; %update the Klist
    end
    iteration = iteration + 1;
end
