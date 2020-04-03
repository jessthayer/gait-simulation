function design = objfun4(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the optimization results from MPC Internal Model
COMVelocity24 = get(simulator.LoggedData,'PredictedCOMVelocity');

% define design parameters (from Internal_MPC_Model_24)
[p(1).Value p(2).Value p(3).Value p(4).Value]
    % p(1) - laguerre polynomial stance ankle/left ankle (6 coeff)
    % p(2) - laguerre polynomial stance knee/left knee (6 coeff)
    % p(3) - laguerre polynomial swing ankle/right ankle (6 coeff)
    % p(4) - laguerre polynomial swing knee/right knee (6 coeff)

% define target COM Velocity
COMVelocity = 1.3854 %[meters/second]

% define design objectives
design.F = (COMVelocity-COMVelocity24)^2*100000 %F — Value of the cost (objective) evaluated at p. The solver minimizes F.

end
