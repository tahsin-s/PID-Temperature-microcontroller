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

settemp = 15;
maxPoints = 500;
maxTime = 20;

timeStep = 0.001;
timeRange = zeros(1,maxPoints);
tempvec = zeros(1,maxPoints);
intSoFar = 0;
temp = [0 0 0]; %using 3 points as derivative and integral source
setvec = zeros(1,maxPoints);
plotted = 0;
timer = 0;

kp = 40;
ki = 10;
kd = -5;

zthresh = 5;

  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  tic
  while RUN == 1 % ADD YOUR CODE HERE. 


    %recieve adc data
    fprintf(comport,"%s",'K');
    adc = fread(comport,400,"uint8");
    
    avgAdc = mean(adc,1);
    %convert adc to temperature
    temp = [temp(2) temp(3) 0]; %shift temperature to the left
    temp(end) = adcToTemp(avgAdc); %use avg voltage for new temp
    
    strTemp = sprintf("%.1f °C",temp(3));
    set(handles.CurrentTemp,'string',strTemp)
    % PID calculations
    % z < 0 if cooling needed, Hi, INV
    % duty cycle will be < 0
    % send msb = 0
    % z > 0 if heating needed, Low, PWM


    [z, intSoFar] = pid3temp(settemp,temp,intSoFar,kp,ki,kd,timeStep);
    duty = z/zthresh;
    duty = min(duty,  1); %clamp duty cycle between 1 and -1
    duty = max(duty, -1);

    dutyByte = dutyAsBytes(duty); 
    fprintf(comport,"%s",dutyByte);
    
    recievedDuty = fread(comport,1,"schar");
    recievedDuty = recievedDuty/127;
    %wait for a bit before changing 

    % sends out two bytes to represent the duty cycle
    % timer >= maxTime
    

    if timer >= maxTime
        plotted = 1;
        timer = 0;
        
        timeRange = linspace(0,maxPoints,maxPoints);
    else
        plotted = plotted + 1; %use plotted to keep track of temps
    end
    
    
    tempvec(plotted) = temp(end); %add temperature to plot
    setvec(plotted:end) = settemp;
    plot(timeRange(1:plotted),tempvec(1:plotted),timeRange,setvec)
    axis([0 maxTime 0 60])  
    ax = gca;
    ax.XColor = 'w';
    ax.YColor = 'w';

    drawnow

    %update timeStep
    timeStep = toc;
    tic

    timer = timer + timeStep;
    timeRange(plotted+1) = timer;
  end
% type fclose(comport) after you're done