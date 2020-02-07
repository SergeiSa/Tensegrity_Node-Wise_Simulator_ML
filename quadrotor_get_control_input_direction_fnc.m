function control_input = quadrotor_get_control_input_direction_fnc(start_point, end_point, time)

g = 9.81;

ZeroOrderDerivativeNodes = {start_point(1), end_point(1);
    start_point(2), end_point(2);
    start_point(3), end_point(3)};
FirstOrderDerivativeNodes = {0, 0; 0, 0; 0, 0};
SecondOrderDerivativeNodes = {0, 0; 0, 0; 0, 0};
NodeTimes = [0; time];
Spline = TPSplineConstructorUI();
Spline.OutOfBoundariesBehaviour = 'LastValue';
Spline.GenerateSplines(NodeTimes, ZeroOrderDerivativeNodes, FirstOrderDerivativeNodes, SecondOrderDerivativeNodes);


    function desired_normal = control_input_fnc(t)
        desired_normal = Spline.EvaluateQ(t);
    end

control_input = @control_input_fnc;

end