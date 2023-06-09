%-----------------------------------------------------------------------
% myscope.m
% Written by Kenrick Chin, Mohammadreza Shahzadeh
% Date: 2016 Jan 29
% Modified by Tahsin Sarker, Sebastian Carter, Syed Rahman
%-----------------------------------------------------------------------
function varargout = myscope(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myscope_OpeningFcn, ...
                   'gui_OutputFcn',  @myscope_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT
%-----------------------------------------------------------------------
% --- Executes just before myscope is made visible.
function myscope_OpeningFcn(hObject, eventdata, handles, varargin)
global comport
global RUN
global NPOINTS
RUN = 0;
NPOINTS = 5;
try % prevents a comport open without making it available
    fclose(comport)
catch
end
comport = serial('COM11','BaudRate',115200); %using msp as uart
fopen(comport)
% Choose default command line output for myscope
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
%-----------------------------------------------------------------------
function myscope_OutputFcn(hObject, eventdata, handles, varargin)
%-----------------------------------------------------------------------
function Quit_Button_Callback(hObject, eventdata, handles)
global comport
global RUN
RUN = 0;
fclose(comport)
clear comport
if ~isempty(instrfind)
fclose(instrfind);
delete(instrfind);
end
% use close to terminate your program
% use quit to terminate MATLAB
close
%----------------------------------------------------------------------
function Run_Button_Callback(hObject, eventdata, handles)
global comport
global NPOINTS
global RUN

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

kp = 0;
ki = 0;
kd = 0;

zthresh = 5;
if RUN == 0
    RUN = 1;

  
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  tic
  while RUN == 1 % ADD YOUR CODE HERE. 

    % Puts box values into variables
    temptext = get(handles.SetTemp,'String'); %Set Temp
    if  (~isnan(str2double(temptext))) && (str2double(temptext) ~= settemp)
        settemp = str2double(temptext);
        intSoFar = 0;
    end

    temptext = get(handles.Kp,'String'); %Kp
    if  ~isnan(str2double(temptext))
       kp = str2double(temptext);
    end

    temptext = get(handles.Ki,'String'); %Ki
    if  ~isnan(str2double(temptext))
       ki = str2double(temptext);
    end

    temptext = get(handles.Kd,'String'); %Kd
    if  ~isnan(str2double(temptext))
       kd = str2double(temptext);
    end

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
    intSoFar

    duty = z;

    duty = min(duty,  1); %clamp duty cycle between 1 and -1
    duty = max(duty, -1);

    dutyByte = dutyAsBytes(duty); 
    
    fprintf(comport,"%s",dutyByte);
    
    recievedDuty = fread(comport,1,"int8");
    recievedDuty = recievedDuty/127
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
    xlabel("Time (s)")
    ylabel("Temperature (C)")
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
else
  RUN = 0;
  % change the string on the button to RUN
  set(handles.Run_Button,'string','RUN')
end
%----------------------------------------------------------------------





function SetTemp_Callback(hObject, eventdata, handles)
% hObject    handle to SetTemp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of SetTemp as text
%        str2double(get(hObject,'String')) returns contents of SetTemp as a double


% --- Executes during object creation, after setting all properties.
function SetTemp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SetTemp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Kp_Callback(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kp as text
%        str2double(get(hObject,'String')) returns contents of Kp as a double


% --- Executes during object creation, after setting all properties.
function Kp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ki_Callback(hObject, eventdata, handles)
% hObject    handle to Ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ki as text
%        str2double(get(hObject,'String')) returns contents of Ki as a double


% --- Executes during object creation, after setting all properties.
function Ki_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ki (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Kd_Callback(hObject, eventdata, handles)
% hObject    handle to Kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kd as text
%        str2double(get(hObject,'String')) returns contents of Kd as a double


% --- Executes during object creation, after setting all properties.
function Kd_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in updatebutton.
function updatebutton_Callback(hObject, eventdata, handles)
% hObject    handle to updatebutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
