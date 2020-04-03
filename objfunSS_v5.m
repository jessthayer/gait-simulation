function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
StepLength_inter = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos_inter = get(simulator.LoggedData,'PredictedVerticalPos');
VerticalPosEnd_inter = get(simulator.LoggedData,'PredictedVerticalPosEnd');
ToePos_inter = get(simulator.LoggedData,'PredictedToePos');

dynamic_effort_inter = get(simulator.LoggedData, 'Dynamic_Effort');

% define design parameters (from Internal_MPC_Model_21)
[p(1).Value p(2).Value p(3).Value p(4).Value] 

% define desired step length
StepLength = 0.725; % [meters]

% define bodymass
bodymass = 86.2; % [kg]

% define dynamic effort
dynamic_effort = (sum(dynamic_effort_inter(:)))^2;

% define design objectives
design.F = 2*(StepLength-StepLength_inter)^2*100000 + 0.1*dynamic_effort; %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos_inter; -ToePos_inter; VerticalPosEnd_inter-0.01;]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
