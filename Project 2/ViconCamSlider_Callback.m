%function ViconCamSlider_Callback(hObject,event,hVicCam,rots,tV,hCam,hCamTitle,tC,camData,tI,gVec)
function ViconCamSlider_Callback(hObject,event,hVicCam,rots,tV,hCam,hCamTitle,tC,camData,varargin)
    % take slider value from ViconCam and update ViconPlot and find closest
    % CamPlot to update corresponding image
    % Update ViconPlot
    % 
    
slider_value = round(get(hObject,'Value'));
%fprintf('Slider Value: %5.0f \n',slider_value)
figure(hVicCam)
subplot(1,2,1)
rotplot(rots(:,:,slider_value),tV(slider_value));

if ~isempty(varargin)
    %plot gravity vector from linear acceleration and Vicon
    
    tI = varargin{1};
    gVec = varargin{2};
    [~,IMUTimeIdx] = min(abs(tI - tV(slider_value)));
    hold on
    plot3([0 gVec(1,IMUTimeIdx)],[0 gVec(2,IMUTimeIdx)],[0 gVec(3,IMUTimeIdx)],'-k')
    hold off
end

% compute closest camera position and plot that
[~,CamTimeIdx] = min(abs(tC - tV(slider_value)));
set(hCam,'CData',camData(:,:,:,CamTimeIdx))
set(hCamTitle,'String',sprintf('Camera Data \n Time: %5.3f',tC(CamTimeIdx)))
    