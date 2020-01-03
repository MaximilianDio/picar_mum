function dx = BycicleModel(t, x, u)
    % The most simple car model is used for the trajectory optimization
    
    % State: x = [x y psi]' , position and orientation of the car
    % Inputs: u = [v delta]' , input velocity and Steering
    dx = zeros(size(x));
    
    l_r = 0.1207;
    l_f = 0.1393;
    
    for i = 1:size(x,2)
        beta = atan(l_r/(l_f + l_r) * tan(u(2,i)));

        dx(1,i) = u(1,i) * cos(x(3,i) + beta);
        dx(2,i) = u(1,i) * sin(x(3,i) + beta);
        dx(3,i) = u(1,i)/l_r * sin(beta);
    end
end