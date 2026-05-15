function DynamicModel(block)

setup(block);

end

function setup(block)

    block.NumInputPorts  = 2;
    block.NumOutputPorts = 2;

    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    block.InputPort(1).Dimensions        = 1;
    block.InputPort(2).Dimensions        = 1;

    block.InputPort(1).DirectFeedthrough = false;
    block.InputPort(2).DirectFeedthrough = false;

    block.OutputPort(1).Dimensions = 1;
    block.OutputPort(2).Dimensions = 1;

    block.NumContStates = 3;

    block.SampleTimes = [0 0];

    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Derivatives', @Derivatives);

end

function InitializeConditions(block)

    block.ContStates.Data = [0; 0; 0];

end

function Outputs(block)

    x = block.ContStates.Data;

    v     = x(1);
    omega = x(2);
    theta = x(3);

    block.OutputPort(1).Data = v;
    block.OutputPort(2).Data = theta;

end

function Derivatives(block)

    u_sigma = block.InputPort(1).Data;
    u_delta = block.InputPort(2).Data;

    x = block.ContStates.Data;

    v     = x(1);
    omega = x(2);

    Kv     = 0.005062865;
    tau_v  = 0.1;

    Kw     = 0.032716536;
    tau_w  = 0.103454545;

    dv_dt     = (-v     + Kv * u_sigma) / tau_v;
    domega_dt = (-omega + Kw * u_delta) / tau_w;
    dtheta_dt = omega;

    block.Derivatives.Data = [dv_dt; domega_dt; dtheta_dt];

end