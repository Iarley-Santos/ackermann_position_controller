% =========================================================================
% Fuzzy control function
% =========================================================================

function [v, delta] = ackermann_control_fuzzy(rho, e_theta, fis)

    input = [rho, e_theta];
    output = evalfis(fis, input);

    v = output(1);
    delta = output(2);

end