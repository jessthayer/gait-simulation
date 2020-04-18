function design = objfunSS(p,simulator)

% define design variables
simulator.Parameters = p;
simulator = sim(simulator);

% obtain the step length optimization results
stepLength21 = get(simulator.LoggedData,'PredictedStepLength');
dynamic_effort21 = get(simulator.LoggedData, 'Dynamic_Effort');
VerticalPos21 = get(simulator.LoggedData,'PredictedVerticalPos');
swingToeZ = get(simulator.LoggedData,'swing_toe_z');
HCflag = get(simulator.LoggedData,'Flag');

% define bodymass
bodymass = 86.2; % [kg] %THIS SHOULD BE CALLED FROM THE MODEL

for i = length(HCflag)
    if HCflag(i) == 1
        stepLength = stepLength21(i);
        return
    else
        stepLength = -0.7;
    end
end

% define design parameters
[p(1).Value p(2).Value p(3).Value p(4).Value];

% define desired step length
stepLength_target = 0.700; % [meters]

% define dynamic effort
dynamic_effort = sum(dynamic_effort21(:))

% define design objectives
design.F = dynamic_effort %F — Value of the cost (objective) evaluated at p. The solver minimizes F.
design.Cleq = [-VerticalPos21; -swingToeZ; ((stepLength_target-stepLength)-0.05*stepLength)]; %Cleq — Value of the nonlinear inequality constraint violations evaluated at p. The solver satisfies Cleq(p) <= 0.

end
