function design = objfunSS_v0(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
StepLength21 = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
VerticalPosEnd21 = get(simulator.LoggedData,'PredictedVerticalPosEnd');

% define design parameters
[p(1).Value p(2).Value p(3).Value p(4).Value];

% define desired step length
StepLength = 0.725; % [meters]

% define design objectives
design.F = 2*(StepLength-StepLength21)^2*100000; %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21; VerticalPosEnd21-0.01]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
