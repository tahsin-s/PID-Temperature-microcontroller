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
function msyscope_OpeningFcn(hObject, eventdata, handles, varargin)
global comport
global RUN
global NPOINTS
RUN = 0;
NPOINTS = 20;
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

%create appropriate timespan, (285.7 per 400 measurements)
t = linspace(0,285.7,NPOINTS); 
settemp = '00';
if RUN == 0
  RUN = 1;
  % change the string on the button to STOP
  set(handles.Run_Button,'string','STOP')
  while RUN == 1 % ADD YOUR CODE HERE. 
    % send a single character prompt to the MCU
    fprintf(comport,'%s','K');
    % fetch data as single 8-bit bytes
    d = fread(comport,NPOINTS,'uint8');
    d = d*548/165; %scale down reading

    % Uncomment for regular scope
    %{
    rising = (p(6) - p(1)) > 0;
    if rising
        continue
    end
    %}

    plot(t,d)
    % Here are examples on how to set graph title, labels and axes
    title('EZ Scope');
    xlabel('Time - us')
    ylabel('Voltage - mV')
    %axis ([0 285.7 0 600]) %uncomment for regular scope
    % use drawnow to update the figure
    drawnow  

    %temptext = get(handles.SetTemp,'String'); %type exactly 2 digits

    %check if settemp handle is a valid value
    %valid = isnan(str2double(temptext));
    %valid = ~valid;
    %valid = valid & (length(temptext) == 2);
    %if  valid
    %    settemp = temptext;
    %end
    %fprintf(comport,'%s',settemp(1));
    %fprintf(comport,'%s',settemp(2));

  end
else
  RUN = 0;
  % change the string on the button to RUN
  set(handles.Run_Button,'string','RUN')
end
%----------------------------------------------------------------------