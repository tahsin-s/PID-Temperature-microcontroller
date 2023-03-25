clear;
comport = serial('COM11','BaudRate',115200);
NPOINTS = 5;


fopen(comport);
settemp = '20';
maxPoints = 100;

axisRange = linspace(1,maxPoints,maxPoints);
tempvec = zeros(1,maxPoints);
setvec = zeros(1,maxPoints);
plotted = 0;
while true

    pause(0.2); %ensure serial packets don't overlap
    fprintf(comport,'%s',settemp(1));
    fprintf(comport,'%s',settemp(2));
    
    
    % process sent data as 3 digit decimals
    d = fread(comport,9,"uint8");

    temps = zeros(1,3);
    temps(1) = d(1)*10 + d(2) + d(3)*0.1;
    temps(2) = d(4)*10 + d(5) + d(6)*0.1;
    temps(3) = d(7)*10 + d(8) + d(9)*0.1;

    if plotted+3 >= maxPoints
        tempvec = zeros(1,maxPoints);
        plotted = 0;
    end

    tempvec((plotted+1):(plotted+3)) = temps;
    setvec((plotted+1):end) = str2double(settemp);
    plotted = plotted + 3; %3 points have been added to tempvec
    
    plot(axisRange(1:plotted),tempvec(1:plotted),axisRange,setvec);
    
    title('PID Temperature Measurement');
    xlabel('Time - s')
    ylabel('Temperature - °C')
    axis ([0 maxPoints 0 50]) %uncomment for regular scope
    legend('Current temp','Set Temp');
    % use drawnow to update the figure
    drawnow 
end

% type fclose(comport) after you're done