function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
StepLength21 = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
VerticalPosEnd21 = get(simulator.LoggedData,'PredictedVerticalPosEnd');

dynamic_effort21 = get(simulator.LoggedData, 'Dynamic_Effort');

% define design parameters (from Internal_MPC_Model_21)
[p(1).Value p(2).Value p(3).Value p(4).Value] 

% define desired step length
StepLength = 0.725; % [meters]

% define bodymass
bodymass = 86.2; % [kg]

% define dynamic effort
dynamic_effort = sum(dynamic_effort21(:))

% define design objectives
design.F = 2*(StepLength-StepLength21)^2*100000 + 0.1*dynamic_effort; %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21;VerticalPosEnd21-0.01]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
