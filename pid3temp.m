function [z, intSoFar] = pid3temp(settemp,temp,intSoFar,kp,ki,kd,timeStep)
    
    errorTerm = settemp - temp;
    %raw error
    compProp = errorTerm(end);

    %modify integral
    intSoFar = intSoFar + errorTerm(3)*timeStep;
    compInt = intSoFar; 

    %backward centred difference
    compDiff = (-1.5*temp(3) + 2*temp(2) - 0.5*temp(1))/timeStep;
    %uses temp to reduce noise


    z = kp*compProp + ki*compInt + kd*compDiff;
end

% These are not true derivatives or integrals. They are missing a factor of
% h or 1/h (step size). This can be adjusted for with the PID constants.