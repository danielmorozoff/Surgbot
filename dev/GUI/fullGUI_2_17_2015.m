function varargout = fullGUI(varargin)
% FULLGUI MATLAB code for fullGUI.fig
%      FULLGUI, by itself, creates a new FULLGUI or raises the existing
%      singleton*.
%
%      H = FULLGUI returns the handle to a new FULLGUI or the handle to
%      the existing singleton*.
%
%      FULLGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FULLGUI.M with the given input arguments.
%
%      FULLGUI('Property','Value',...) creates a new FULLGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fullGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fullGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fullGUI

% Last Modified by GUIDE v2.5 16-Feb-2015 23:00:42

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fullGUI_OpeningFcn, ...
                   'gui_OutputFcn', @myCameraGUI_OutputFcn , ...
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


% --- Executes just before fullGUI is made visible.
function fullGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fullGUI (see VARARGIN)
try
        disp('Launching GUI');
    % Choose default command line output for fullGUI
    handles.output = hObject;

    % Create video object
    % Putting the object into manual trigger mode and then
    % starting the object will make GETSNAPSHOT return faster
    % since the connection to the camera will already have
    % been established.
        disp('Loading Dependencies...');
        %add libraries to path
        addpath('../dependencies');
        addpath('../dependencies/Sutter MP285');
        addpath('../dependencies/ArduinoIO');
        addpath('../');

        disp('Loading Properties...');
        NiDAQ_board = 'shitty';
        %ChannelS
        %fancy boards
        handles.in_out_s=[];
        %shitty boards
        handles.in_s=[];
        handles.out_s=[];

        %COM PORTS
        handles.sutter_port = 'COM5';
        handles.ardui_port = 'COM4';
        %Channels
        handles.signal_inputChannel = 'ai0';
        handles.signal_outputChannel = 'ao1';

        %Settings for session
        handles.rate = 10000; %Set to 16 KHz scans/s
        %Number of cyles to probe fxn
        handles.pulse_cycles = 0;
        %Arduino device to control BK PRECISION power unit. These vary in
        %their max current outputs. We used 1902 (15A)
        handles.ardui=[];
        handles.voltage_reg_pin = 5;%Pin to adjust voltage
        handles.current_reg_pin = 9;%Pin to adjust current max.

        %Sutter device
        handles.sutter=[];
        %DRILL STATE
        handles.skull_pierced = false;
        %Drill properties
        handles.diameterToDrill = [];
        handles.distanceBtwDepthHoles = [];

        handles.z_step_size = 50; %in micrometers
        % Sutter scaling factor
        %scales to the device r from mm to micrometers
        handles.scaling_factor = 1000;

        %Path storage
        handles.path=[]; %[x y z ang]
        handles.path_r=[];

        %Milling
        handles.spline_obj=[];
        handles.spline_pnts=[];
        handles.milling_offset = 50;

        %Fourier filter
        %Frequencies that will be displayed/potentially measured to
        %boost speed.
        handles.freq_top = 110;
        handles.freq_bot = 90;

            %Mouse FFT Params
            handles.mouse_fft_background_signal = 46.0962; % shitty comp in vivarium 1Mohm resist
            %0.7085; % This is for a gain of x100 avg over 2mins of measurements. this is measured empirically.
            handles.mouse_fft_max_signal = 86.0633; % shitty comp in vivarium- actual brain
            %266.6654; %this represents a closed loop across the NiDAQ
            handles.mouse_fft_attenuation_factor = .6;

            %Measured signals
            handles.lifetime_logged_integral_of_fft = [];
            handles.current_measurement_log_of_fft = [];

        
        
        %Injection properties
        % in microns - post callibration
        handles.injection_pin = 2;
        handles.x_injection_offset = 2330.45; %to the right camera view
        handles.y_injection_offset = 35215.8; %away from camera
        
        handles.injection_depth_1 = 250;
        handles.injection_depth_2 = 450;
        handles.data.markedPnts = [];
        
        handles.injection.pauseTime = 120; % in seconds
        
        
        %Parallel resources
        handles.parallel_cluster=[];
        handles.worker_pool=[];
        handles.current_signal_worker_job=[]; %This is reserved for the current signal generating worker

        %Listener Handles
        handles.DATA_AVAILABLE=[];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -- Properties end
        disp('Setting up Arduino...');
        %Initialize Arduino IO
        try
            handles.ardui = arduino(handles.ardui_port);
            handles.ardui.pinMode(handles.voltage_reg_pin,'output');
            handles.ardui.pinMode(handles.current_reg_pin,'output');
        catch err
            delete(instrfind({'Port'},{handles.ardui_port}));
        end
        %Initialize Sutter
        disp('Setting up Sutter...');
        try
        handles.sutter = sutterMP285(handles.sutter_port);
        catch err
          delete(instrfind({'Port'},{handles.sutter_port}));
        end
        
%         
%         %Initialize DAQa.
%         handles.daq_devices = daq.getDevices;
%         handles.device= handles.daq_devices(1).ID;
%         %only ao0 and ao1 are outputs on the USB6008/6009 boards.
% 
%         %Create NI-DAQ session object and set the in/out channels
%         %!!!Still need to queue data for out channel
%         if(strcmp(NiDAQ_board,'fancy'))
%             handles.in_out_s= daq.createSession('ni');
%             handles.in_out_s.Rate = handles.rate;
%             handles.in_out_s.addAnalogInputChannel(handles.device,handles.signal_inputChannel,'Voltage');
%             handles.in_out_s.addAnalogOutputChannel(handles.device,handles.signal_outputChannel,'Voltage');
%         elseif strcmp(NiDAQ_board,'shitty')
%                 handles.out_s= daq.createSession('ni');
%                 handles.out_s.addAnalogOutputChannel(handles.device,handles.signal_outputChannel,'Voltage');
% 
%                 handles.in_s = daq.createSession('ni');
%                 handles.in_s.addAnalogInputChannel(handles.device,handles.signal_inputChannel,'Voltage');
%                 handles.in_s.Rate = handles.rate;
%         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% -- Inital setup end


    disp('Setting GUI Props...');

    handles.drillState = 0;
    if(ispc)
    handles.video = videoinput('winvideo', 1,'MJPG_1920x1080');
    else
    handles.video = videoinput('macvideo', 1);    
    end
    
    set(handles.video,'TimerPeriod', 0.01, ...
    'TimerFcn',['if(~isempty(gco)),'...
    'handles=guidata(gcf);'... % Update handles
    'frame = getsnapshot(handles.video);'...
    'in_frame = insertShape(frame, ''circle'', [1920/2 1080/2 1], ''LineWidth'', 2);'...
    'image(in_frame);'... % Get picture using GETSNAPSHOT and put it into axes using IMAGE
    'else '...
    'delete(imaqfind);'... % Clean up - delete any image acquisition objects
    'end']);
    triggerconfig(handles.video,'manual');

    handles.video.FramesPerTrigger = Inf; % Capture frames until we manually stop it
    handles.videoState = 0;

    handles.pntPlot=[];

    handles.data.distancePoints = [0 0;0 0];
    set(handles.cameraAxes,'ytick',[],'xtick',[]);
   
    
    set(handles.x_pos,'String',0);
    set(handles.y_pos,'String',0);
    set(handles.z_pos,'String',0);
    
    % Update handles structure
    guidata(hObject, handles);
    % UIWAIT makes fullGUI wait for user response (see UIRESUME)
    uiwait(handles.figure1);
catch
    error('Execution error Occured...');    
end


% --- Outputs from this function are returned to the command line.
function varargout = myCameraGUI_OutputFcn(hObject, eventdata, handles)
% varargout cell array for returning output args (see VARARGOUT);
% hObject handle to figure
% eventdata reserved - to be defined in a future version of MATLAB
% handles structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
handles.output = hObject;
varargout{1} = handles.output;


% --- Executes on button press in captureImageButton.
function captureImageButton_Callback(hObject, eventdata, handles)
% hObject    handle to captureImageButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('Capture Frame');
fprintf('State: %d\n',handles.videoState);
% Start/Stop Camera
if ~handles.videoState
    % Camera is off. Change button string and start camera.
%     preview(handles.video);
        start(handles.video)
    handles.videoState = 1;
else
    % Camera is on. Stop camera and change button string.
%     stoppreview(handles.video) ;
        stop(handles.video)
    handles.videoState = 0;
end
guidata(hObject, handles);    
fprintf('New State: %d\n',handles.videoState);
   

% --- Executes on button press in markPoints.
function markPoints_Callback(hObject, eventdata, handles)
% hObject    handle to markPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    old_pnts = handles.data.markedPnts;
    [x,y]=getpts;
    old_pnts  =[old_pnts, x';y'];
%     %close points
    temp  = [old_pnts(1,1); old_pnts(2,1)];
    old_pnts = [old_pnts,temp]
    hold on;
    plot(old_pnts(1,:),old_pnts(2,:),'r.', 'MarkerSize', 20);
    handles.data.markedPnts =  old_pnts ;
    guidata(hObject, handles);


% --- Executes on button press in clearPoints.
function clearPoints_Callback(hObject, eventdata, handles)
% hObject    handle to clearPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
plotH = handles.pntPlot;
set(plotH, 'xdata', [], 'ydata', []);
handles.data.markedPnts = [];
guidata(hObject, handles);


% --- Executes on button press in connectPoints.
function connectPoints_Callback(hObject, eventdata, handles)
% hObject    handle to connectPoints (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

old_pnts = handles.data.markedPnts ;
curve = cscvn(old_pnts);
fx_val = fnval(curve,0:.01:max(curve.breaks));
hold on;
handles.pntPlot = plot(fx_val(1,:),fx_val(2,:),'b.','MarkerSize',10) ;
guidata(hObject, handles);


% --- Executes on button press in pushbutton13.
function setDistanceButton_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on;
twoPnts = handles.data.distancePoints;
twoPnts = [0 0 ; 0 0];
[x,y] = getpts;
while(length(x)~=2)
    [x,y] = getpts;
end
twoPnts= [x,y]
handles.distPlot = [];
handles.distPlot = plot(twoPnts(:,1), twoPnts(:,2),'g-', 'MarkerSize', 20);

handles.data.distancePoints = twoPnts;
guidata(hObject, handles);


function input_obj_Callback(hObject, eventdata, handles)
% hObject    handle to input_obj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of input_obj as text
%        str2double(get(hObject,'String')) returns contents of input_obj as a double
%Fired when enter is clicked on object
actual_dist = str2double(get(hObject,'String'));
twoPnts = handles.data.distancePoints;
pixel_dist = pdist(twoPnts,'euclidean');
scaling_factor = actual_dist/pixel_dist
hold on;
mid_x = (twoPnts(2,1)-twoPnts(1,1))/2;
mid_y = (twoPnts(2,2) - twoPnts(1,2))/2;
text(mid_x+twoPnts(1,1),mid_y+twoPnts(1,2), get(hObject,'String'),'Color', 'red');


% --- Executes during object creation, after setting all properties.
function input_obj_CreateFcn(hObject, eventdata, handles)
% hObject    handle to input_obj (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in setOriginButton.
function setOriginButton_Callback(hObject, eventdata, handles)
% hObject    handle to setOriginButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setOrigin(handles.sutter);
[xyz]=getPosition(handles.sutter);
    updateGUIPos(handles,hObject,xyz(1,1),xyz(2,1),xyz(3,1));
guidata(hObject, handles);

% --- Executes on button press in moveToButton.
function moveToButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveToButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x_pos = str2double(get(handles.moveTo_x_pos,'String'));
y_pos = str2double(get(handles.moveTo_y_pos,'String'));
z_pos = str2double(get(handles.moveTo_z_pos,'String'));
moveTo(handles.sutter,[x_pos,y_pos,z_pos]);
[xyz]=getPosition(handles.sutter);
    updateGUIPos(handles,hObject,xyz(1,1),xyz(2,1),xyz(3,1));
updatePanel(handles.sutter);
guidata(hObject, handles);

function moveTo_x_pos_Callback(hObject, eventdata, handles)
% hObject    handle to moveTo_x_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of moveTo_x_pos as text
%        str2double(get(hObject,'String')) returns contents of moveTo_x_pos as a double


% --- Executes during object creation, after setting all properties.
function moveTo_x_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to moveTo_x_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function moveTo_y_pos_Callback(hObject, eventdata, handles)
% hObject    handle to moveTo_y_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of moveTo_y_pos as text
%        str2double(get(hObject,'String')) returns contents of moveTo_y_pos as a double


% --- Executes during object creation, after setting all properties.
function moveTo_y_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to moveTo_y_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function moveTo_z_pos_Callback(hObject, eventdata, handles)
% hObject    handle to moveTo_z_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of moveTo_z_pos as text
%        str2double(get(hObject,'String')) returns contents of moveTo_z_pos as a double


% --- Executes during object creation, after setting all properties.
function moveTo_z_pos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to moveTo_z_pos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in resetDrillButton.
function resetDrillButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetDrillButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%  sendReset(handles.sutter);
%  fclose(handles.sutter);
 delete(instrfind({'Port'},{handles.ardui_port}));
 delete(instrfind({'Port'},{handles.sutter_port}));
 handles.sutter = [];
 handles.ardui = [];
 
 handles.sutter = sutterMP285(handles.sutter_port);
 handles.ardui = arduino(handles.ardui_port);
 guidata(hObject, handles);


function voltageSet_Callback(hObject, eventdata, handles)
% hObject    handle to voltageSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of voltageSet as text
%        str2double(get(hObject,'String')) returns contents of voltageSet as a double
voltage = str2double(get(hObject,'String'))
setDrillVoltage(handles,voltage);

% --- Executes during object creation, after setting all properties.
function voltageSet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to voltageSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function currentSet_Callback(hObject, eventdata, handles)
% hObject    handle to currentSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of currentSet as text
%        str2double(get(hObject,'String')) returns contents of currentSet as a double
current = str2double(get(hObject,'String'))
setDrillCurrent(handles,current);

% --- Executes during object creation, after setting all properties.
function currentSet_CreateFcn(hObject, eventdata, handles)
% hObject    handle to currentSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in drillShapeButton. This is the primary
% execution loop
function drillShapeButton_Callback(hObject, eventdata, handles)
% hObject    handle to drillShapeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

    drillState = findobj( 'Tag', 'drillStateText'); 
         drillState_bc =  get(drillState,'BackgroundColor');
     
     diam =  get(findobj( 'Tag', 'diameterToDrillInput'),'BackgroundColor');
     pnts_dist = get(findobj( 'Tag', 'distanceBetweenInput'),'BackgroundColor');
     mill_off = get(findobj( 'Tag', 'millingOffsetInput'),'BackgroundColor');
     
    if(isequal(drillState_bc, [1 0 0]) && isequal(diam, [0 1 0]) &&  isequal(pnts_dist, [0 1 0]) &&  isequal(mill_off, [0 1 0])   )
        disp('Drilling');
        set(drillState,'BackgroundColor',[0 1 0]);
        make_craniotomy(hObject,handles,handles.diameterToDrill,handles.distanceBtwDepthHoles);
    elseif isequal(drillState_bc, [0 1 0])
        set(drillState,'BackgroundColor',[1 0 0]);
        error('**Stop Drill**');
    end
% --- Executes on button press in loadShapeButton.
function loadShapeButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadShapeButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on;
fName = uigetfile;
load(fName);
%pnts should exist in file...
temp  = [pnts(1,1); pnts(2,1)];
  pnts = [pnts,temp];
 plot(pnts(1,:),pnts(2,:),'r.', 'MarkerSize', 20);
 handles.data.markedPnts =  pnts ;
 set(handles.cameraAxes,'ytick',[],'xtick',[]);
 guidata(hObject,handles);

% --- Executes on button press in drawCircleButton.
function drawCircleButton_Callback(hObject, eventdata, handles)
% hObject    handle to drawCircleButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on;
%These points should be center and perim
[x,y]=getpts;
while(length(x)~=2)
    [x,y] = getpts;
end
circlePnts = [x,y];

r = pdist(circlePnts,'euclidean')

th = 0:pi/100:2*pi;
xunit = r * cos(th) + circlePnts(1,1);
yunit = r * sin(th) + circlePnts(1,2);
h = plot(xunit, yunit,'b.');

hold off

% --- Executes on button press in sendPulseButton.
function sendPulseButton_Callback(hObject, eventdata, handles)
% hObject    handle to sendPulseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%Setup parallel compute
poolobj = gcp('nocreate');
    if(isempty(poolobj))
        handles = createParallelProcess(hObject,handles,1);
    else
        %Can't figure out how to get the current cluster obj --
        %don't really need it
        handles.worker_pool = poolobj;
    end
    background_color = get(gcbo,'BackgroundColor');
    
    if(isequal(background_color , [1 0 0]))
        sendDaqOnParallel(hObject,handles);
        set(gcbo,'String', 'Pulse On');
        set(gcbo,'BackgroundColor',[0 1 0]);
    elseif (isequal(background_color , [0 1 0]))
        fprintf('Stopping Signal \n ')
%         delete(handles.worker_pool);
%         handles.worker_pool = [];
       [~ , ~, running, completed] = findJob( handles.parallel_cluster);
       cancel(running);
       delete( completed);
         handles.current_signal_worker_job=[];
        set(gcbo,'String', 'Pulse Off');
        set(gcbo,'BackgroundColor',[1 0 0]);
    end
    guidata(hObject,handles);

% --- Executes on button press in moveToOriginButton.
function moveToOriginButton_Callback(hObject, eventdata, handles)
% hObject    handle to moveToOriginButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
moveToOrigin(hObject,handles);
 
 %%%Non interactive functions

  %Send Daq parallel job process
function [j,task]=sendDaqOnParallel(hObject,handles)
    if( isempty(handles.current_signal_worker_job) ||  strcmp(handles.current_signal_worker_job.State , 'finished') || strcmp( handles.current_signal_worker_job,'failed'))
    fprintf('Making new job for signal')
    handles.parallel_cluster
        
        j = createJob(  handles.parallel_cluster,'Name','My job');
        %Only available to MJS job
        task = createTask(j,@eval,0, {'auto=Autodrill_v0_16(''shitty'',0);auto.sendDaqProbePulse(100,1000)'});
%             J.FinishedFcn = @(job,eventdata) set(findobj(handles,'Tag','sendPulseButton'),'BackgroundColor','[1 0 0]'); 
%         J.FinishedFcn = @(job,eventdata) disp([job.Name ' now ' job.State]);
        handles.current_signal_worker_job = j;

        submit(handles.current_signal_worker_job);
        guidata(hObject,handles);
    end
%              wait(job);
%              out = fetchOutputs(job)

function finishedSignalRun(src,event)
disp('finished signal');

%Update GUI pos
function updateGUIPos(handles,hObject, x,y,z)
    set(handles.x_pos,'String',x);
    set(handles.y_pos,'String',y);
    set(handles.z_pos,'String',z);
    
    guidata(hObject, handles);


function diameterToDrillInput_Callback(hObject, eventdata, handles)
% hObject    handle to diameterToDrillInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of diameterToDrillInput as text
%        str2double(get(hObject,'String')) returns contents of diameterToDrillInput as a double
val = str2double(get(hObject,'String'));
if(val > 0)
    set(gcbo,'BackgroundColor', [0 1 0 ]);
    handles.diameterToDrill = val;
    guidata(hObject,handles);
else
    set(gcbo,'BackgroundColor', [1 1 1 ]);
end

% --- Executes during object creation, after setting all properties.
function diameterToDrillInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to diameterToDrillInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
  set(hObject,'BackgroundColor','white');




function distanceBetweenInput_Callback(hObject, eventdata, handles)
% hObject    handle to distanceBetweenInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of distanceBetweenInput as text
%        str2double(get(hObject,'String')) returns contents of distanceBetweenInput as a double
val = str2double(get(hObject,'String'));
if(val > 0 && val < handles.diameterToDrill)
    set(gcbo,'BackgroundColor', [0 1 0 ]);
    handles.distanceBtwDepthHoles = val;
    guidata(hObject,handles);
else
    set(gcbo,'BackgroundColor', [1 1 1 ]);
end


% --- Executes during object creation, after setting all properties.
function distanceBetweenInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to distanceBetweenInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns calleset(hObject,'BackgroundColor','white');

    
%Primay craniotomy function    
function make_craniotomy(hObject,handles,d,distance_between_pnts)
moveToOrigin(hObject,handles);
handles.path = [];
r = d/2;
handles.path_r = r;

%Add a step_size that is proportional to the linead distance between
%hole centers
radial_step_size = 2*asin(distance_between_pnts/(2*r));
figure;
hold on;
plot3(0,0,0,'gx','MarkerSize',50);    
grid on;
drawnow

setDrillVelocity(handles,20);

t0 = tic;
for ang=0:radial_step_size:2*pi
    z_start= -1 ; %This can be modififed in order to evaluate how to detect the surface of the bone
%                   z =randi([-2 -1],1,1);
%                 Scale to the device
   [x,y] = convertRadialToXY(handles,ang);
    z = handles.scaling_factor*z_start;


   if(ang<2*pi)
       cprintf([0 1 0],'Move To %g %g %g um...\n',x ,y ,z);
       moveTo(handles.sutter,[x y z]);

       plot3(x/handles.scaling_factor,y/handles.scaling_factor,-1*z/handles.scaling_factor,'r.','MarkerSize',30);   
       drawnow 

       %Drill
       [pierced,pierced_z]= stepDrillZ(x, y, z);
        plot3(x/handles.scaling_factor,y/handles.scaling_factor,-1*pierced_z/handles.scaling_factor,'b.','MarkerSize',20);    
        drawnow;
   end

    if(abs(x)<(10^-6)) x = 0;end
    if(abs(y)<(10^-6)) y = 0;end    
    cprintf([0 1 .5], 'STORING PATH: %d %d %d \n', x,y,pierced_z);

    if(ang==2*pi)
        handles.path = [handles.path; x y  handles.path(1,3) ang];
    else
        handles.path = [handles.path; x y  pierced_z ang];
    end

    %Retract drill prior to move
    moveTo(handles.sutter,[x,y,z]);

end
toc(t0);
cprintf([1 0 0],'Path: %d \n', handles.path);

%Interpolate and retrace
handles=splineInterpolateZDrill(handles);

%Mill the interpolated values.
mill(handles);

moveToOrigin(hObject,handles);
disp(handles.path)


%Z stepping fxn
function [pierced,curZ] = stepDrillZ(handles,x_in,y_in,z_in)
pierced = 0;
cnt = 0;
drillControl(handles,'on',28,2);

while(~pierced)
    curZ = (z_in+(handles.z_step_size)*cnt);
    fprintf(1,'Drilling to depth:  %d um \n',curZ);
    moveTo(handles.sutter,[x_in y_in curZ]); 
   %DEV
    if(curZ>=1000* 1)
        drillControl(handles,'off',0,0);
        pierced = 1;
    end
%                 !!!THIS IS WHERE THE MAIN MEASUREMENT IN THE LOOP SHOULD OCCUR!!!
%                 %mesaure Signal -.75 seconds provides approx 3
%                 measurements.

%                     acquireDAQProbePulse(handles,'foreground',5000,1000,.75); 
%                 if(handles.skull_pierced)  
%                     pierced = 1;
%                     handles.skull_pierced = false;
%                 else 
% 
%                 end
    cnt = cnt+1;
end
cprintf([1 .5 0],'***Pierced @ %d *** \n', curZ);

%Main Control fxn for drill
function drillControl(handles,state,voltage,current)
    if(strcmp(state,'on'))
            setDrillCurrent(handles,current);
            setDrillVoltage(handles,voltage);
    else
            setDrillCurrent(handles,0);
            setDrillVoltage(handles,0);
    end
       
function setDrillVoltage(handles, vIn)
    if(vIn>25)
        vIn = 25;
    end
    %convert to arduino scale
    vToSet = int8(vIn*255/60);
    fprintf('Drill voltage set to %d \n',vIn);
    handles.ardui.analogWrite(handles.voltage_reg_pin,vToSet);

%%The fuse on the foredom is at 3.1A, but the max draw is 2A
function setDrillCurrent(handles,iIn)
    if(iIn>2)
        iIn = 2;
    end
    iToSet = int8(iIn*255/15);
    fprintf('Drill current set to %d \n',iToSet);
    handles.ardui.analogWrite(handles.current_reg_pin,iToSet);
%Since it makes more sense to set a velocity based on um/sec...
function setDrillVelocity(handles,vel)
    [stepMult, ~,~] = getStatus(handles.sutter);
    velToSet = vel* stepMult; % the first unit is the multiplier.
    %The scale factor simply multiplies the speed -  so I set it to 1 
    setVelocity(handles.sutter,velToSet,1);
    getStatus(handles.sutter);
%Convert from radial to xy
function [x,y]=convertRadialToXY(handles, ang)
    xp=handles.path_r*cos(ang);
    yp=handles.path_r*sin(ang);

    x = handles.scaling_factor*xp;
    y = handles.scaling_factor*yp;
function moveToOrigin(hObject,handles)
 moveTo(handles.sutter,[0 0 0]);
 [xyz]=getPosition(handles.sutter);
    updateGUIPos(handles,hObject,xyz(1,1),xyz(2,1),xyz(3,1));
 updatePanel(handles.sutter);
 guidata(hObject, handles);
  %Interpolation functions for the drill milling mode
function handles = splineInterpolateZDrill(handles)
    path_to_user = handles.path(1:(end),1:3)';
    cprintf([1 0 0 ], 'Interpolating...');
    path_to_user

    sp = cscvn(path_to_user(:,[1:end 1]));
    handles.spline_obj = sp;
    %Points for the spline fit
    points=fnplt(sp);
    handles.spline_pnts = points'; %transpose them in order to keep the same sorting style...

%Milling mode of the drill
function mill(handles)
    %pull back drill 
    x0= handles.spline_pnts(1,1);
    y0 = handles.spline_pnts(1,2);
    moveTo(handles.sutter,[x0 y0 -5000]);
    %turn drill on -- set speed (adjusted empirically)
    obj.drillControl('on',25,2);
    %Begin milling move.
    for(i=1:1:length(obj.spline_pnts))
        x= handles.spline_pnts(i,1);
        y = handles.spline_pnts(i,2);
        z = handles.spline_pnts(i,3)-handles.milling_offset;
        moveTo(handles.sutter,[x y z]);
        hold on;
        plot3(x/1000,y/1000,-1*z/1000,'c.','MarkerSize',5);
        drawnow;
    end
    drillControl(handles,'off',0,0);
    

function millingOffsetInput_Callback(hObject, eventdata, handles)
% hObject    handle to millingOffsetInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of millingOffsetInput as text
%        str2double(get(hObject,'String')) returns contents of millingOffsetInput as a double
val = str2double(get(hObject,'String'));
if(val >= 0)
     handles.milling_offset = val;
     set(hObject,'BackgroundColor','g');      
else
    %Default value
     handles.milling_offset = 50;
     set(hObject,'BackgroundColor','w');  
end
guidata(hObject,handles);   


% --- Executes during object creation, after setting all properties.
function millingOffsetInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to millingOffsetInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% Code handling parallelism
function handles= createParallelProcess(hObject,handles,num_workers)
%Create background process           
 if isempty(handles.parallel_cluster)
    disp('Creating cluster...');
    handles.parallel_cluster =  parcluster('local');
 end
handles.worker_pool = parpool(handles.parallel_cluster,num_workers);
guidata(hObject,handles);
       
% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
  
    % The GUI is no longer waiting, just close it
    delete(handles.worker_pool);
     delete(instrfind({'Port'},{handles.ardui_port}));
     delete(instrfind({'Port'},{handles.sutter_port}));
     imaqreset;
    delete(hObject);


% --- Executes on slider movement.
function velocitySlider_Callback(hObject, eventdata, handles)
% hObject    handle to velocitySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%Slider goes from 0 - 1
%Max vel setting is 50-- might need to change this
non_scaled_val =  get(hObject,'Value');
scaledVel = 50*non_scaled_val;
setDrillVelocity(handles,scaledVel);

set(handles.displayVelocity ,'String',num2str(scaledVel));

% --- Executes during object creation, after setting all properties.
function velocitySlider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to velocitySlider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


%%Injection functions

function injectDepth1_Callback(hObject, eventdata, handles)
% hObject    handle to injectDepth1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of injectDepth1 as text
%        str2double(get(hObject,'String')) returns contents of injectDepth1 as a double
depth = str2double(get(hObject,'String'));
handles.injection_depth_1 = depth;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function injectDepth1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to injectDepth1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function injectDepth2_Callback(hObject, eventdata, handles)
% hObject    handle to injectDepth2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of injectDepth2 as text
%        str2double(get(hObject,'String')) returns contents of injectDepth2 as a double
depth = str2double(get(hObject,'String'));
handles.injection_depth_2 = depth;
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function injectDepth2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to injectDepth2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pierceDepthInput_Callback(hObject, eventdata, handles)
% hObject    handle to pierceDepthInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pierceDepthInput as text
%        str2double(get(hObject,'String')) returns contents of pierceDepthInput as a double


% --- Executes during object creation, after setting all properties.
function pierceDepthInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pierceDepthInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in injectButton.
function injectButton_Callback(hObject, eventdata, handles)
% hObject    handle to injectButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
z_backStep = -2000;
%Move back 
moveTo(handles.sutter,[0 0 z_backStep]);

%default res 1920x1080
centerPnt = [1080/2, 1920/2];
for i=1:length(handles.data.markedPnts)
    pnt_x = handles.data.markedPnts(1,i);
    pnt_y = handles.data.markedPnts(2,i);
    
    %convert marked point to
    curPos = handles.sutter.getPosition();
    cur_x = curPos(1,1);
    cur_y = curPos(2,1);
    %ensure the proper width height are represented here.
    pixel_dist_x = pdist([centerPnt(1,1),0;cur_x,0],'euclidean');
    pixel_dist_y =  pdist([0,centerPnt(1,2);0,cur_y],'euclidean');
    %divide distances by calibration factors to get distances to move
    %along x/y
    
%     pnt_x = ;
%     pnt_y = ;

% 	add offsets  to drill positions.
    
    moveTo(handles.sutter,[pnt_x pnt_y z_backStep]);
    %Step injector in Z 
    [pierced,pierced_z]= stepInjectorZ(pnt_x, pnt_y, z_backStep);
    
end
%Function to move injector into place
function stepInjectorZ(handles,x_in,y_in,z_in)

pierced = 0;
cnt = 0;

while(~pierced)
    curZ = (z_in+(handles.z_step_size)*cnt);
    fprintf(1,'Injector to depth:  %d um \n',curZ);
    moveTo(handles.sutter,[x_in y_in curZ]); 
   
        acquireDAQProbePulse(handles,'foreground',5000,1000,.75); 
        cprintf([1 .5 0],'*** Surface of brain detected... *** \n');
        handles.skull_pierced  =1;
        if(handles.skull_pierced) % this now represents- when the needle touches dura- passes the silicon elastomer
            %Pierce the dura
            cprintf([1 .5 0],'***Piercing DURA *** \n', curZ);

            fastPierceOfDura(handles,100,100,2);
            pierced = 1;
            %reset
            handles.skull_pierced = false;
            
            setOrigin(handles.sutter);
            %Consider slowing down velocity
            moveTo(handles.sutter,[0 0 handles.injection_depth_1]);
            !%Inject 1
            inject(handles);
            pause(handles.injection.pauseTime);
            
            moveTo(handles.sutter,[0 0 handles.injection_depth_2]);
            !%Inject 2
            inject(handles);
            pause(handles.injection.pauseTime);
            
        end
    cnt = cnt+1;
end

%Fast pierce of dura function. At surface step back then fast pierce and
%reset
function fastPierceOfDura(handles, velocity_fast,pierceDepth,num_of_pierceattempts)
    xyz = getPosition(handles.sutter); %surface position
    [~, currentVelocity, ~] = getStatus(handles.sutter);
    for i = 1:num_of_pierceattempts
        
        moveTo(handles.sutter,[xyz(1,1),xyz(2,1), xyz(3,1)-100]);
        setVelocity(handles.sutter, velocity_fast, 10) %10 is the default scaling factor for V
        
        moveTo(handles.sutter,[xyz(1,1),xyz(2,1), xyz(3,1)+pierceDepth]);
        setVelocity(handles.sutter, currentVelocity, 10) %10 is the default scaling factor for V
        moveTo(handles.sutter,[xyz(1,1),xyz(2,1), xyz(3,1)]);
    end
    
function inject(handles)
  handles.ardui.digitalWrite(handles.injection_pin,1);
  pause(200);
  handles.ardui.digitalWrite(handles.injection_pin,0);
    


% --- Executes during object creation, after setting all properties.
function cameraAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cameraAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate cameraAxes
% handles.video = videoinput('winvideo', 1,'MJPG_1920x1080');
% triggerconfig(handles.video,'manual');
% 
% handles.video.FramesPerTrigger = Inf; % Capture frames until we manually stop it
% 
% 
% vidRes=handles.video.VideoResolution;
% nBands = handles.video.NumberOfBands;
% 
% hImage = image( zeros(vidRes(2), vidRes(1), nBands) );
% 
% handles.dispImage = hImage;
% preview(handles.video,hImage);
% 
% handles.videoState = 1;
% guidata(hObject,handles);
