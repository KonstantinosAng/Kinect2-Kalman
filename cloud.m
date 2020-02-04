%% Point Cloud
clear
%Create devices
colorDevice = imaq.VideoDevice('kinect',1);
depthDevice = imaq.VideoDevice('kinect',2);
%Initialize devices
colorDevice();
depthDevice();
%Get one frame from each
colorImage = colorDevice();
depthImage = depthDevice();
%point cloud
ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);
 
% Initialize a player to visualize 3-D point cloud data. The axis is
% set appropriately to visualize the point cloud from Kinect.
  player = pcplayer(ptCloud.XLimits, ptCloud.YLimits, ptCloud.ZLimits,...
              'VerticalAxis', 'y', 'VerticalAxisDir', 'down');
 
  xlabel(player.Axes, 'X (m)');
  ylabel(player.Axes, 'Y (m)');
  zlabel(player.Axes, 'Z (m)');
 
  % Acquire and view Kinect point cloud data.
  while isOpen(player)
     colorImage = colorDevice();
     depthImage = depthDevice();
 
     ptCloud = pcfromkinect(depthDevice, depthImage, colorImage);
 
     view(player, ptCloud);
  end
  
%% Release video
release(colorDevice);
release(depthDevice);