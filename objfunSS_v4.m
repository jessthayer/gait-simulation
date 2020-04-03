function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
StepLength21 = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
VerticalPosEnd21 = get(simulator.LoggedData,'PredictedVerticalPosEnd');

% obtain the COM energy optimization results
COM_h21 = get(simulator.LoggedData,'COM_h');
COM_vy21 = get(simulator.LoggedData,'COM_vy');
COM_vz21 = get(simulator.LoggedData,'COM_vz');

% define design parameters (from Internal_MPC_Model_21)
[p(1).Value p(2).Value p(3).Value p(4).Value] 

% define desired step length
StepLength = 0.725; % [meters]

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
design.F = 2*(StepLength-StepLength21)^2*100000 + deltaE_COM %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21;VerticalPosEnd21-0.01]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
