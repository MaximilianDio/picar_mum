anglesDesired = 0:5:30;
anglesActual = [0;5;9;13;18;23;26];

plot(anglesDesired,anglesActual);



fun = @myfunc;
X = fsolve(fun,15);

function y = myfunc(x)

    desiredAngle = 14.89;
% Coefficients:
  p1 = -0.0014286;
  p2 = 0.92143;
  p3 = 0.071429;

    y = p1*x^2 + p2*x + p3 - desiredAngle;
end




