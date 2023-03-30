try
    fclose(comport)
catch
end
clear;
comport = serial('COM11','BaudRate',115200);
NPOINTS = 5;


fopen(comport);
settemp = '15';
maxPoints = 100;

axisRange = linspace(1,maxPoints,maxPoints);
tempvec = zeros(1,400);
temp = [0 0 0]; %using 3 points as derivative and integral source
setvec = zeros(1,maxPoints);
plotted = 0;

kp = 40;
ki = 10;
kd = -5;

zthresh = 10;
timeStep = 0.1;
while true
    
    %recieve adc data
    fprintf(comport,"%s",'K');
    adc = fread(comport,400,"uint8");
    
    avgAdc = mean(adc,1)
    %convert adc to temperature
    temp = [temp(2) temp(3) 0]; %shift temperature to the left
    temp(end) = adcToTemp(avgAdc) %use avg voltage for new temp
    
    % PID calculations
    % z < 0 if cooling needed, Hi, INV
    % duty cycle will be < 0
    % send msb = 0
    % z > 0 if heating needed, Low, PWM


    z = pid3temp(str2double(settemp),temp,kp,ki,kd,timeStep);
    duty = z/zthresh;
    duty = min(duty,  1); %clamp duty cycle between 1 and -1
    duty = max(duty, -1);

    dutyByte = dutyAsBytes(duty); 
    fprintf(comport,"%s",dutyByte);
    
    recievedDuty = fread(comport,1,"schar")
    %wait for a bit before changing 

    % sends out two bytes to represent the duty cycle
    % plotting 
    if plotted == maxPoints
        plotted = 1;
    else
        plotted = plotted + 1; %use plotted to keep track of temps
    end
    
    tempvec(plotted) = temp(end); %add temperature to plot
    plot(axisRange(1:plotted),tempvec(1:plotted))
    axis([0 maxPoints 0 60])  

    drawnow

    %update timeStep
    timeStep = toc;
    tic
end

% type fclose(comport) after you're done