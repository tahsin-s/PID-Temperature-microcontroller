%-----------------------------------------------------------------------
% myscope.m
% Written by Kenrick Chin
% Date: 2016 Jan 29
% Modified by Mohammadreza Shahzadeh
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
comport = serial('COM11','BaudRate',115200);
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

settemp = '20';
maxPoints = 300; %edit to increase or decrease range of plot

axisRange = linspace(1,maxPoints,maxPoints);
tempvec = zeros(1,maxPoints);
setvec = zeros(1,maxPoints);
plotted = 0;

if RUN == 0
  RUN = 1;

  
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  while RUN == 1 % ADD YOUR CODE HERE. 

    temptext = get(handles.SetTemp,'String'); %type exactly 2 digits

    %check if settemp handle is a valid value
    valid = isnan(str2double(temptext));
    valid = ~valid;
    valid = valid & (length(temptext) == 2);
    if  valid
       settemp = temptext;
    end

    %ensure serial packets don't overlap
    fprintf(comport,'%s',settemp(1));
    fprintf(comport,'%s',settemp(2));
    
    
    % process recieved data as 3 digit decimals
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
