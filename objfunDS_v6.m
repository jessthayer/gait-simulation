function design = objfunDS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the optimization results from MPC Internal Model
COMVelocity24 = get(simulator.LoggedData,'PredictedCOMVelocity');
COMEnergy24 = get(simulator.LoggedData,'PredictedCOMEnergy');


% obtain the COM energy optimization results
COM_h21 = get(simulator.LoggedData,'COM_h');
COM_vy21 = get(simulator.LoggedData,'COM_vy');
COM_vz21 = get(simulator.LoggedData,'COM_vz');

% define design parameters (from Internal_MPC_Model_24)
[p(1).Value p(2).Value p(3).Value p(4).Value]
    % p(1) - laguerre polynomial stance ankle/left ankle (6 coeff)
    % p(2) - laguerre polynomial stance knee/left knee (6 coeff)
    % p(3) - laguerre polynomial swing ankle/right ankle (6 coeff)
    % p(4) - laguerre polynomial swing knee/right knee (6 coeff)

% define target COM Velocity
COMVelocity = 1.3854; %[meters/second]

% define bodymass
bodymass = 86.2; % [kg]

% Define COM Energy
E_COM = bodymass*(9.81)*COM_h21 + (0.5)*(bodymass)*COM_vy21.*COM_vy21 + (0.5)*(bodymass)*COM_vz21.*COM_vz21;

% define COM energy changes
deltaE_COM = 0;

for i = 2:length(E_COM)
    deltaE_COM = deltaE_COM + abs(E_COM(i) - E_COM(i-1))
end

% define design objectives
design.F = 2*(COMVelocity-COMVelocity24)^2*100000 + deltaE_COM %F — Value of the cost (objective) evaluated at p. The solver minimizes F.

end
