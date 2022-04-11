function [error_mean,error_max,time_at_max,error] = errorCalc(estimated_sig,true_sig)
% ADDME Error calculation function
%    Output [1,2,3,4] is [error_mean, error_max, time_at_max, error]
%    Input (1,2) is (estimated signal, true signal)

true = true_sig; 
est = estimated_sig;

error = true-est;

error_mean = mse(error);

[error_max,time_at_max] = max(abs(error));


end