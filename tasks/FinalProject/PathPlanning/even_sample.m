
%% Function to perform a even sample of time and state vectors at a constant specified rate

function [Et, Ex] = even_sample(t, x, Fs)

    Et = linspace(t(1), t(end), (t(end)-t(1))*Fs)';
    
    % Using linear interpolation re-sample each signal to obtain the evenly sampled forms
    for s = 1:size(x, 2)
      Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1), 'linear'); 
    end
    
end