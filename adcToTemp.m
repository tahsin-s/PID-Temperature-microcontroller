function Tout = adcToTemp(adc)
Rpot = 10;

a = 0.00276964;
b = 0.00025192;
c = 3.2782E-7;

v = @(adc) adc/256;
Ri = @(v) (Rpot*v)/(1-v);
T = @(Ri) (a + b*log(Ri) + c*log(Ri)^3)^-1 - 273.15;

Tout = T(Ri(v(adc)));
end
