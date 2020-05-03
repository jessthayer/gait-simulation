function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
stepLength21 = get(simulator.LoggedData,'PredictedStepLength');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
swingToeZ = get(simulator.LoggedData,'swing_toe_z');
HCflag = get(simulator.LoggedData,'Flag');

for i = length(HCflag)
    if HCflag(i) == 1
        stepLength = stepLength21(i);
    else
        stepLength = -0.7;
    end
end

% define design parameters
[p(1).Value p(2).Value p(3).Value p(4).Value];

% define desired step length
stepLength_target = 0.700; % [meters]

% define design objectives
design.F = ((stepLength_target-stepLength)-0.05*stepLength) %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21; ((stepLength_target-stepLength)-0.05*stepLength)]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.
%-swingToeZ;
end
