function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
StepLength21 = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
VerticalPosEnd21 = get(simulator.LoggedData,'PredictedVerticalPosEnd');

% define design objectives
design.F = 2*(StepLength-StepLength21)^2*100000 %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21;VerticalPosEnd21-0.01]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
