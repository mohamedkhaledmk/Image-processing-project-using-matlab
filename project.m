function varargout = project(varargin)
% PROJECT MATLAB code for project.fig
%      PROJECT, by itself, creates a new PROJECT or raises the existing
%      singleton*.
%
%      H = PROJECT returns the handle to a new PROJECT or the handle to
%      the existing singleton*.
%
%      PROJECT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT.M with the given input arguments.
%
%      PROJECT('Property','Value',...) creates a new PROJECT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before project_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to project_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help project

% Last Modified by GUIDE v2.5 16-Dec-2022 23:27:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @project_OpeningFcn, ...
                   'gui_OutputFcn',  @project_OutputFcn, ...
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


% --- Executes just before project is made visible.
function project_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to project (see VARARGIN)

% Choose default command line output for project
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes project wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = project_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in uploadImg.
function uploadImg_Callback(hObject, eventdata, handles)
[name,path]=uigetfile('*.*');
fullFileName = fullfile(path, name);
x=imread(fullFileName);
x=x(:,:,1);  %convert any colored image to grayscale
axes(handles.axes1);
imshow(x);
setappdata(0,'original_image',x);
setappdata(0,'processed_image',x);
setappdata(0,"edged_image",x);

%x = imread(fullFileName);

% --- Executes on button press in sampleImg.
function sampleImg_Callback(hObject, eventdata, handles)
fileList=[dir('./image_samples/*.png');dir('./image_samples/*.jpg');dir('./image_samples/*.tif');dir('./image_samples/*.jfif');dir('./image_samples/*.tif')];
randomIndex = randi(length(fileList), 1, 1); % Get random number.
fullFileName = fullfile('./image_samples/', fileList(randomIndex).name);
img = imread(fullFileName);
img=img(:,:,1);
axes(handles.axes1);
imshow(img)
setappdata(0,'original_image',img);
setappdata(0,'processed_image',img);
setappdata(0,"edged_image",img);


% --- Executes on button press in histogramEqualization.
function histogramEqualization_Callback(~, eventdata, handles)
axes(handles.axes2);
x=getappdata(0,"processed_image");
heq=histeq(x);
imshow(heq);

% --- Executes on button press in originalImage. radio-button
function originalImage_Callback(hObject, eventdata, handles)
x=getappdata(0,"original_image");
axes(handles.axes3);
imhist(x);

% Hint: get(hObject,'Value') returns toggle state of originalImage


% --- Executes on button press in smoothFilter.
function smoothFilter_Callback(hObject, eventdata, handles)
x=getappdata(0,'processed_image');
w= [1 2 1; 2 4 2; 1 2 1]*(1/16);
smoothed=imfilter(x,w);
axes(handles.axes2);
imshow(smoothed);
setappdata(0,'processed_image',smoothed);


% --- Executes on button press in sharpeningFilter.
function sharpeningFilter_Callback(hObject, eventdata, handles)
x=getappdata(0,'processed_image');
l=[0 -1 0;-1 5 -1; 0 -1 0];
sharpenedImage=imfilter(x,l,'replicate');
axes(handles.axes2);
imshow(sharpenedImage);
setappdata(0,'processed_image',sharpenedImage);




% hObject    handle to intensityLevel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider








% --- Executes on selection change in noiseMenu.
function noiseMenu_Callback(hObject, eventdata, handles)
x=getappdata(0,'processed_image');
content = cellstr(get(hObject,'String')); %get currently selected option from menu
popupselection = content(get(hObject,'Value'))
popupselection
if (strcmp(popupselection,'Salt & Pepper Noise'))
a=getappdata(0,'processed_image');
noise=imnoise(a,'salt & pepper');
axes(handles.axes2); 
imshow(noise)
setappdata(0,"processed_image",noise);

elseif (strcmp(popupselection,'Gaussian Noise'))
    a=getappdata(0,'processed_image');
    noise=imnoise(a,'gaussian');
    axes(handles.axes2); 
    imshow(noise)
    setappdata(0,"processed_image",noise);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
elseif (strcmp(popupselection,'Pepper Noise'))
    a=getappdata(0,'processed_image');
    noise=imnoise(a,'salt & pepper');
    min_filter=ordfilt2(noise,1,ones(3,3));
    axes(handles.axes2); 
    imshow(min_filter)
    setappdata(0,"processed_image",min_filter);
elseif (strcmp(popupselection,'Salt Noise'))
    a=getappdata(0,'processed_image');
    noise=noise('cameraman.tif','salt', 0.05,0.00);
    max_filter=ordfilt2(noise,9,ones(3,3));
    axes(handles.axes2); 
    imshow(noise)
    setappdata(0,"processed_image",max_filter);
elseif (strcmp(popupselection,'Speckle Noise'))
    a=getappdata(0,'processed_image');
    noise=imnoise(a,'speckle', 0.05);
    axes(handles.axes2); 
    imshow(noise)
    setappdata(0,"processed_image",noise);
elseif (strcmp(popupselection,'Periodic Noise'))
axes(handles.axes2);
z=getappdata(0,"processed_image");
s=size(z);
[x,y]=meshgrid(1:s(1),1:s(2));
mysinusoidalnoise = (15 * sin(2*pi/14*x+2*pi/14*y));
mysinusoidal = imresize(mysinusoidalnoise, size(z));
mynoiseimg1 = double(z) + mysinusoidal;
imshow(mynoiseimg1,[]);
end

% Hints: contents = cellstr(get(hObject,'String')) returns noiseMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from noiseMenu


% --- Executes during object creation, after setting all properties.
function noiseMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to noiseMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





% --- Executes on button press in horizontalEdges.
function horizontalEdges_Callback(hObject, eventdata, handles)
x=getappdata(0,'processed_image');
setappdata(0,'edged_image',x);
e=getappdata(0,'edged_image');
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
    vmask=fspecial('sobel');
    b_h=imfilter(e,vmask,'replicate');
    setappdata(0,"edged_image",b_h);
    imshow(b_h);
elseif button_state == get(hObject,'Min')
    setappdata(0,"edged_image",x);
    imshow(x);
end


% --- Executes on button press in verticalEdges.
function verticalEdges_Callback(hObject, eventdata, handles) 
axes(handles.axes2); 
x=getappdata(0,'processed_image');
setappdata(0,'edged_image',x);
e=getappdata(0,'edged_image');
button_state = get(hObject,'Value');
if button_state == get(hObject,'Max')
    vmask=fspecial('sobel');
    b_v=imfilter(e,vmask','replicate'); 
    setappdata(0,"edged_image",b_v);
    imshow(b_v);
elseif button_state == get(hObject,'Min')
    setappdata(0,"edged_image",x);
    imshow(x);
end

% Hint: get(hObject,'Value') returns toggle state of verticalEdges


% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
    axes(handles.axes2); 
x=getappdata(0,"original_image")
setappdata(0,"processed_image",x);
imshow(x);

function image = changeclass(class, varargin)
%CHANGECLASS will change the storage class of an image.
%   I2 = CHANGECLASS(CLASS, I);
%   RGB2 = CHANGECLASS(CLASS, RGB);
%   BW2 = CHANGECLASS(CLASS, BW);
%   X2 = CHANGECLASS(CLASS, X, 'indexed');

switch class
case 'uint8'
    image = im2uint8(varargin{:});
case 'uint16'
    image = im2uint16(varargin{:});
case 'double'
    image = im2double(varargin{:});
otherwise
    error('Unsupported IPT data class.');
end

% --- Executes on selection change in SmoothFiltersMenu. MMK
function filtersMenu_Callback(hObject, eventdata, handles)
axes(handles.axes2); 
x=getappdata(0,'processed_image');
content = cellstr(get(hObject,'String')); %get currently selected option from menu
popupselection = content(get(hObject,'Value'))
popupselection
if (strcmp(popupselection,'Harmonic Filter'))
     inclass = class(x);
     g = im2double(x);
     gg=1./(g);
     g1=imfilter(gg,ones(3,3),'replicate');
     h = (3 * 3) ./ g1;
     f = changeclass(inclass,h);
     axes(handles.axes2);
     imshow(f);
     setappdata(0,"processed_image",f);
elseif (strcmp(popupselection,'+ve Contra Harmonic Filter'))
    inclass = class(x);
    g = im2double(x);
    q=3;
    f = imfilter(g.^(q+1),ones(3,3),'replicate');
    f = f ./(imfilter(g.^q,ones(3,3),'replicate') + eps);
    f = changeclass(inclass,f);
    imshow(f);
    setappdata(0,"processed_image",f);
elseif (strcmp(popupselection,'-ve Contra Harmonic Filter'))
     g=getappdata(0,"processed_image");
      inclass = class(g);
      g = im2double(g);
      q=-3;
      f = imfilter(g.^(q+1),ones(3,3),'replicate');
      f = f ./(imfilter(g.^q,ones(3,3),'replicate') + eps);
     f = changeclass(inclass,f);
     imshow(f);
    setappdata(0,"processed_image",f);
elseif (strcmp(popupselection,'Geometric Filter'))
    inclass = class(x);
    g = im2double(x);
    m=3;
    n=3;
    geomean = imfilter(log(g), ones(m,n), 'replicate');
    geomean = exp(geomean);
f = geomean .^ (1/(m*n));
f = changeclass(inclass,f);
    axes(handles.axes2); 
    imshow(x);
    setappdata(0,"processed_image",f);
elseif (strcmp(popupselection,'Median Filter'))
    b=medfilt2(x,[3 3]);
    imshow(b);
    setappdata(0,"processed_image",b);
end

% --- Executes during object creation, after setting all properties.
function filtersMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SmoothFiltersMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes during object creation, after setting all properties.
function SmoothFiltersMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SmoothFiltersMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in greyAdjust.
function greyAdjust_Callback(hObject, eventdata, handles)
x=getappdata(0,'processed_image')
adjust=imadjust(x);
axes(handles.axes2); 
imshow(adjust);
setappdata(0,'processed_image',adjust);


% --- Executes on button press in radiobutton8.
function radiobutton8_Callback(hObject, eventdata, handles)
axes(handles.axes3);
x=getappdata(0,'processed_image');
heq=histeq(x);
imhist(heq);

% Hint: get(hObject,'Value') returns toggle state of radiobutton8


% --- Executes on selection change in SmoothFiltersMenu.
function SmoothFiltersMenu_Callback(hObject, eventdata, handles)
axes(handles.axes2);
x=getappdata(0,'processed_image');
content = cellstr(get(hObject,'String')); %get currently selected option from menu
popupselection = content(get(hObject,'Value'))
popupselection
if (strcmp(popupselection,'Max'))
    max_filter=ordfilt2(x,9,ones(3,3));
    imshow(max_filter);
    setappdata(0,"processed_image",max_filter);
elseif (strcmp(popupselection,'Min'))
    min_filter=ordfilt2(x,1,ones(3,3));
    imshow(min_filter);
    setappdata(0,"processed_image",min_filter);
elseif (strcmp(popupselection,'Median'))
    median_filter=ordfilt2(x,5,ones(3,3));
    imshow(median_filter);
    setappdata(0,"processed_image",median_filter);
elseif (strcmp(popupselection,'Average'))
    m=fspecial('average',[3 3]);
    avg_filter=imfilter(x,m);
    imshow(avg_filter);
    setappdata(0,"processed_image",avg_filter);
elseif (strcmp(popupselection,'Average_replicate'))
    m=fspecial('average',[3 3]);
    avgr_filter=imfilter(x,m,'replicate');
    imshow(avgr_filter);
    setappdata(0,"processed_image",avgr_filter);
end
% hObject    handle to SmoothFiltersMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns SmoothFiltersMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SmoothFiltersMenu


% --- Executes during object creation, after setting all properties.
function popupmenu9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SmoothFiltersMenu (see GCBO)//pppo
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
x=getappdata(0,'original_image');
val = get(hObject,'Value');
adjust = x + val*(255);
setappdata(0,'processed_image',adjust);
axes(handles.axes2); 
imshow(adjust);
guidata(hObject,handles)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2
