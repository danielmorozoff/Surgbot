function varargout = MyCam(varargin)
% MYCAM MATLAB code for MyCam.fig
%      MYCAM, by itself, creates a new MYCAM or raises the existing
%      singleton*.
%
%      H = MYCAM returns the handle to a new MYCAM or the handle to
%      the existing singleton*.
%
%      MYCAM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYCAM.M with the given input arguments.
%
%      MYCAM('Property','Value',...) creates a new MYCAM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MyCam_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MyCam_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MyCam

% Last Modified by GUIDE v2.5 01-Jan-2015 14:06:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MyCam_OpeningFcn, ...
                   'gui_OutputFcn',  @CloseRequestFcn, ...
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


% --- Executes just before MyCam is made visible.
function MyCam_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MyCam (see VARARGIN)

% Choose default command line output for MyCam
handles.output = hObject;

% Create video object
% Putting the object into manual trigger mode and then
% starting the object will make GETSNAPSHOT return faster
% since the connection to the camera will already have
% been established.
disp('Setting Opening Fxn...');
handles.video = videoinput('winvideo',1,'MJPG_1920x1080');


   
    
set(handles.video, ...
'StartFcn',[...
'handles = guidata(gcf);' ...
'vidRes = handles.video.VideoResolution;'... 
'nBands = handles.video.NumberOfBands; '...
'hImage = image( zeros(vidRes(2), vidRes(1), nBands) );'... 
'preview(handles.video,hImage);'... 
'hImage.Toolbar = ''figure''; ']);

triggerconfig(handles.video,'manual');


handles.video.FramesPerTrigger = Inf; % Capture frames until we manually stop it
handles.videoState = 0;

handles.pntPlot=[];
handles.data.markedPnts = [];

handles.data.distancePoints = [0 0;0 0];
set(handles.cameraAxes,'ytick',[],'xtick',[]);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MyCam wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MyCam_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
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
    start(handles.video)
    handles.videoState = 1;
    
else
    % Camera is on. Stop camera and change button string.
    stop(handles.video)
    handles.videoState = 0;
end
guidata(hObject, handles);    
fprintf('New State: %d\n',handles.videoState);

%Needs to be fixed....
% --- Function that runs on close of GUI 
% --- Executes when user attempts to close myCameraGUI.
function varargout = CloseRequestFcn(hObject, eventdata, handles)
    disp('Cleaning up objects...');
    delete(handles)
    imaqreset;

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
