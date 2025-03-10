function design = objfun4(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the optimization results from MPC Internal Model
vy_COM = get(simulator.LoggedData,'PredictedCOMVelocity_y');
vz_COM = get(simulator.LoggedData,'PredictedCOMVelocity_z');

% obtain the COM energy optimization results
COM_h21 = get(simulator.LoggedData,'COM_h');
COM_vy21 = get(simulator.LoggedData,'COM_vy');
COM_vz21 = get(simulator.LoggedData,'COM_vz');
bodymass = 78.02; % [kg]

% Define COM Energy
E_COM = bodymass*(9.81)*COM_h21 + (0.5)*(bodymass)*COM_vy21.*COM_vy21 + (0.5)*(bodymass)*COM_vz21.*COM_vz21;

% define COM energy changes
deltaE_COM = 0;
for i = 2:length(E_COM)
    deltaE_COM = deltaE_COM + abs(E_COM(i) - E_COM(i-1));
end

% define design parameters (from Internal_MPC_Model_24)
[p(1).Value p(2).Value p(3).Value p(4).Value]
    % p(1) - laguerre polynomial stance ankle/left ankle (6 coeff)
    % p(2) - laguerre polynomial stance knee/left knee (6 coeff)
    % p(3) - laguerre polynomial swing ankle/right ankle (6 coeff)
    % p(4) - laguerre polynomial swing knee/right knee (6 coeff)

% define target COM Velocity
vy_COM_target = 1.43; %[meters/second]
vz_COM_target = 0.22; %[meters/second]

% define design objectives
design.F = (vy_COM_target-vy_COM)^2*100000 + (vz_COM_target-vz_COM)^2*100000 + deltaE_COM %F � Value of the cost (objective) evaluated at p. The solver minimizes F.

end
