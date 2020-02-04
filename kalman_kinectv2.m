%% Kalman KinectV2 track of righthand
%% Reset previous
imaqreset;
clear; 
clc; 
close all;


%% Calibrate Camera with chessboard images
% cameraCalibrator % open calibrator


%% create color and depth kinect video input objects
depthVid = videoinput('kinect', 2, 'Depth_512x424'); %start depth video
colorVid = videoinput('kinect', 1, 'BGR_1920x1080'); %start color video
triggerconfig(depthVid,'manual'); %manual triger video
triggerconfig(colorVid,'manual'); %manual trigger video
counter = 1; % Counter to store data
% depth_focal_length = 365.7; % focal length for depth map from pdf https://www.uv.es/imaging3/PDFs/2016_OEng_56_41305.pdf
% color_focal_length = 329.1; % focal length for colormap from pdf https://www.uv.es/imaging3/PDFs/2016_OEng_56_41305.pdf
% depth_FOV_horizontal = 71; %in pixels http://www.smeenk.com/webgl/kinectfovexplorer.html
% color_FOV_horizontal = 84; %in pixels http://www.smeenk.com/webgl/kinectfovexplorer.html
% depth_focal_length = (524*0.5)/tan(depth_FOV_horizontal*0.5*pi/180); %https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
% color_focal_length = (1920*0.5)/tan(color_FOV_horizontal*0.5*pi/180);%https://answers.opencv.org/question/17076/conversion-focal-distance-from-mm-to-pixels/
% depth_focal_length_x = 391.096; %https://threeconstants.wordpress.com/2014/11/09/kinect-v2-depth-camera-calibration/
% depth_focal_length_y = 463.098; %https://threeconstants.wordpress.com/2014/11/09/kinect-v2-depth-camera-calibration/
% color_focal_length_x = 1062.48808; %https://www.semanticscholar.org/paper/A-new-method-in-simultaneous-estimation-of-sensor-Safaei-Fazli/3944c7f5e39090f63ccbc4ac5664519003a3cedf/figure/4
% color_focal_length_y = 1065.01486; %https://www.semanticscholar.org/paper/A-new-method-in-simultaneous-estimation-of-sensor-Safaei-Fazli/3944c7f5e39090f63ccbc4ac5664519003a3cedf/figure/4
depth_focal_length_x = 388.198; %https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
depth_focal_length_y = 389.033; %https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
color_focal_length_x = 1144.361; %https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476
color_focal_length_y = 1147.337; %https://www.researchgate.net/figure/Parameters-from-the-calibration-of-the-Kinect-v2-by-rounding_tbl4_321048476

temp_time = 0; % store time temporary for velocity and acceleratio calc
framesPerTrig = 1; % 1 frame per second
depthVid.FramesPerTrigger=framesPerTrig; % set the frame trig frequency
depthVid.TriggerRepeat=inf; %loop forever
colorVid.FramesPerTrigger=framesPerTrig; %set frame trig frequency
colorVid.TriggerRepeat=inf; %repeat trig for infinite time
colorVid.Timeout = 100; %wait sec to obtain frame
src = getselectedsource(depthVid); %get source of depth video
src.EnableBodyTracking = 'on';  %enable skeleton tracking
start(depthVid); %start depth Video
start(colorVid); %start color Video
himg = figure; %first frame initialize

%% Fix Skeleton Mapping
SkeletonConnectionMap = [ [4 3];        %Head - Neck
                          [3 21];       %Neck - SpineShoulder
                          [21 2];       %SpineShoulder - SpineMid
                          [2 1];        %SpineMid - SpineBase
                          [21 9];       %SpineShoulder - ShoulderRight
                          [9 10];       %ShoulderRight - ElbowRight
                          [10 11];      %ElbowRight - WristRight
                          [11 12];      %WristRight - HandWright
                          [12 24];      %HandRight - HandTipRight
                          [12 25];      %HandRight - ThumbRight
                          [21 5];       %SpineShoulder - ShoulderLeft
                          [5 6];        %ShoulderLeft - ElbowLeft
                          [6 7];        %ElbowLeft - WristLeft
                          [7 8];        %WristLeft - HandLeft
                          [8 22];       %HandLeft - HandTipLeft
                          [8 23];       %HandLeft - ThumbLeft
                          [1 17];       %SpineBase - HipRight
                          [17 18];      %HipRight - KneeRight
                          [18 19];      %KneeRight - AnkleRight
                          [19 20];      %AnkleRight - FootRight
                          [1 13];       %SpineBase - HipLeft
                          [13 14];      %Hip Left - KneeLeft
                          [14 15];      %KneeLeft - AnkleLeft
                          [15 16]; ];   %AnkleLeft - FootLeft


%% Initialize parameters for kalmam filter
flagStart = 0;
time(counter) = 0; 
dt = 30; %Initialize refresh rate
Q_loc = []; %real path
vel = []; %real velocity
Q_loc_meas = []; %real path
u = .005;
noise_mag = 4; %noise for velocity how fast is speeding up

% noise_x = 0.02547; %mean of stamdard deviation of kinect in x axis position https://www.researchgate.net/publication/286624514_Performance_evaluation_of_the_1st_and_2nd_generation_Kinect_for_multimedia_applications
% noise_y = 0.02547; %mean of standard deviation of kinect in y axis position https://www.researchgate.net/publication/286624514_Performance_evaluation_of_the_1st_and_2nd_generation_Kinect_for_multimedia_applications
% noise_z = 0.02547; %mean of standard deviation of kinect in z axis position https://www.researchgate.net/publication/286624514_Performance_evaluation_of_the_1st_and_2nd_generation_Kinect_for_multimedia_applications
noise_x = 10; noise_y = 10; noise_z = 10; % rule of thumb ---> more noise is better

Ez = [noise_x 0 0; ...
      0 noise_y 0; ...
      0 0 noise_z]; % noise matrix for x y z position measurements
          
Ex = [dt^4/4 0 0 dt^3/2 0 0; ...
      0 dt^4/4 0 0 dt^3/2 0; ...
      0 0 dt^4/4 0 0 dt^3/2; ...
      dt^3/2 0 0 dt^2 0 0; ...
      0 dt^3/2 0 0 dt^2 0; ...
      0 0 dt^3/2 0 0 dt^2].*noise_mag^2; %Convert process noise (stdv) into covariance matrix
 
P = Ex;% estimate of initial position variance (covariance matrix)

% xt+1 = xt + Vxt + 0.5*ax*t^2 for one dimension
% Vxt+1 = Vxt + axt
%   [x]               [x]
%   [y]               [y]
%   [z]       = A *   [z]  + B * u
%   [Vx]              [Vx]
%   [Vy]              [Vy]
%   [Vz]              [Vz] 

% zt = C * [x;y;z;Vx;Vy;Vz]

A = [1 0 0 dt 0 0; ...
      0 1 0 0 dt 0; ...
      0 0 1 0 0 dt; ...
      0 0 0 1 0 0; ...
      0 0 0 0 1 0; ...
      0 0 0 0 0 1]; %State matrix for position in 3D space

B = [dt^2/2; ...
     dt^2/2; ...
     dt^2/2; ...
     dt; ...
     dt; ...
     dt]; %State matrix

C = [1 0 0 0 0 0; ...
     0 1 0 0 0 0; ...
     0 0 1 0 0 0]; %We multiply this to get the next predicted state only for position

Q_loc_estimate = [];
vel_estimate = []; %velocity estimation

P_estimate = P;

% Flags for displaying depth image or color image

%% Start mapping skeleton   

while ishandle(himg) %loop for every frame
%     tic
    trigger(depthVid); %start triggering depth stream 1 frame per second
%     dt = toc; %refresh rate is set to the time needed to obtain frame
    trigger(colorVid); %start triggering color stream 1 frame per second
    [depthMap, depthTime, depthMetaData] = getdata(depthVid); %get data from video
    [colorMap] = getdata(colorVid); % get colordata
    bodies = depthMetaData.IsBodyTracked; % check if body is tracked
    trackedBodies = find(bodies); % find how many bodies are tracked
    nBodies = length(trackedBodies); %find how many bodies are tracked
    depthHeight = size(depthMap, 1); % find depth Height
    depthWidth = size(depthMap ,2); % find depth Width
    colors = ['r';'g';'b';'c';'y';'m']; % colors for different bodies
    imshow(colorMap); %display color frame
%     imshow(depthMap, [0 4096]); % display depth frame
       if (sum(bodies)) > 0 % if body is tracked
            imshow(colorMap); %display color frame
%             imshow(depthMap, [0 4096]); % display depth frame
            skeletonJoints = depthMetaData.DepthJointIndices(:,:,depthMetaData.IsBodyTracked); % get the skeleton joints
            posJoints = depthMetaData.JointPositions(:,:,depthMetaData.IsBodyTracked); % get the real positions of tracked joints in meters
            hand_state = depthMetaData.HandRightState(trackedBodies); % 0 unknown | 1 not tracked | 2 open | 3 closed | 4 lasso
            hand_confidence = depthMetaData.HandRightConfidence(trackedBodies); % 0 Low Confidence | 1 High Confidence
            if hand_confidence == 1
               flagStart = 1; 
            end
            % Plot hand states
            if hand_state == 0
                text('units','pixels','position',[100 100],'color','red','fontsize',20,'string','Unknown');
            elseif hand_state == 1
                text('units','pixels','position',[100 100],'color','red','fontsize',20,'string','Not Tracked');
            elseif hand_state == 2
                text('units','pixels','position',[100 100],'color','red','fontsize',20,'string','Open');
            elseif hand_state == 3
                text('units','pixels','position',[100 100],'color','red','fontsize',20,'string','Closed');
            else
                text('units','pixels','position',[100 100],'color','red','fontsize',20,'string','Lasso')
            end
            
            if hand_confidence == 0 
               text('units','pixels','position',[100 130],'color','red','fontsize',20,'string','Low Confidence'); 
            else
               text('units','pixels','position',[100 130],'color','red','fontsize',20,'string','High Confidence');
            end
            
            hold on; %hold figure

            for i = 8:9 % only for joints at the 9 and 10 line of skeleton connection map
                for body = 1:nBodies % for every body tracked

                % Coordinates of skeleton joints in depth space
                X1 = [skeletonJoints(SkeletonConnectionMap(i,1),1,body); skeletonJoints(SkeletonConnectionMap(i,2),1,body)];
                Y1 = [skeletonJoints(SkeletonConnectionMap(i,1),2,body); skeletonJoints(SkeletonConnectionMap(i,2),2,body)];

                % Real world coordinates of joints
                hand_pos_x = posJoints(12,1,body); %meters
                hand_pos_y = posJoints(12,2,body); %meters
                hand_pos_z = posJoints(12,3,body); %meters
                thumb_x = posJoints(25,1,body);    %meters
                thumb_y = posJoints(25,2,body);    %meters
                thumb_z = posJoints(25,3,body);    %meters

                % Calculate orientation based on derivative of line
                % between arm and hand
                line_v = [1e3*(hand_pos_x - thumb_x), ...
                          1e3*(hand_pos_y - thumb_y), ...
                          1e3*(hand_pos_z - thumb_z)];
                x_axis = [1, 0, 0];
                y_axis = [0, 1, 0];
                z_axis = [0, 0, 1];

                % calculate angle between axis
                rot_x = (acos((line_v(1,1).*x_axis(1,1))/(norm(line_v)*norm(x_axis))))*(180/pi); %degrees
                rot_y = (acos((line_v(1,2).*y_axis(1,2))/(norm(line_v)*norm(y_axis))))*(180/pi);   %degrees
                rot_z = (acos((line_v(1,3).*z_axis(1,3))/(norm(line_v)*norm(z_axis))))*(180/pi);   %degrees

                % Print orientation to axis
%                     text('units','pixels','position',[100 70],'color','red','fontsize',20,'string','X:');
%                     text('units','pixels','position',[130 70],'color','red','fontsize',20,'string',num2str(rot_x));
%                     text('units','pixels','position',[100 45],'color','red','fontsize',20,'string','Y:');
%                     text('units','pixels','position',[130 45],'color','red','fontsize',20,'string',num2str(rot_y));
%                     text('units','pixels','position',[100 20],'color','red','fontsize',20,'string','Z:');
%                     text('units','pixels','position',[130 20],'color','red','fontsize',20,'string',num2str(rot_z));

                % Plot circle around hand in depth coordinates
                    if (flagStart == 1)
                        th = 0:pi/50:2*pi;
                        rd = round(90 - 1e3*min(hand_pos_z)/30);
                        coords = depthMetaData.ColorJointIndices(12,:,trackedBodies); %coordinates for color Map
                        plot(rd*sin(th) + coords(1,1) , rd*cos(th) + coords(1,2),'g'); %plot for color map
    %                         coords = depthMetaData.DepthJointIndices(25,:,trackedBodies); % coordinates for depth Map
    %                         plot(rd*sin(th) + coords(1,1),rd*cos(th) + coords(1,2),'g'); %plot for depth Map
    %                         plot(skeletonJoints(12,1,body), skeletonJoints(12,2,body), 'r*');
                    end
                end
            end

            % store or display the coordinates of the right hand
            hand_x(counter) = double(hand_pos_x(1,1)); %meters
            hand_y(counter) = double(hand_pos_y(1,1)); %meters
            hand_z(counter) = double(hand_pos_z(1,1)); %meters
            f_thumb_x(counter) = double(thumb_x(1,1)); %meters
            f_thumb_y(counter) = double(thumb_y(1,1)); %meters
            f_thumb_z(counter) = double(thumb_z(1,1)); %meters

            if (counter == 1)
                distance = [0;0;0]; %distance hand moved (m)
                velocity_x(counter) = distance(1,1)/depthTime; %velocity at x axis (m/sec)
                velocity_y(counter) = distance(2,1)/depthTime; %velocity at y axis (m/sec)
                velocity_z(counter) = distance(3,1)/depthTime; %velocity at z axis (m/sec)
                acceleration_x(counter) = velocity_x(counter)/depthTime; %acceleration at x axis (m/sec2)
                acceleration_y(counter) = velocity_y(counter)/depthTime; %acceleration at y axis (m/sec2)
                acceleration_z(counter) = velocity_z(counter)/depthTime; %acceleration at z axis (m/sec2)
            else
                sum_time = depthTime - temp_time; %seconds
                temp_time = depthTime; %seconds
                distance = [hand_x(counter) - hand_x(counter-1);hand_y(counter) - hand_y(counter-1);hand_z(counter) - hand_z(counter-1)]; % distance hand moved at each axis (meters)
                velocity_x(counter) = distance(1,1)/sum_time; %velocity at x axis (m/sec)
                velocity_y(counter) = distance(2,1)/sum_time; %velocity at y axis (m/sec)
                velocity_z(counter) = distance(3,1)/sum_time; %velocity at z axis (m/sec)
                acceleration_x(counter) = velocity_x(counter)/sum_time; %acceleration at x axis (m/sec2)
                acceleration_y(counter) = velocity_y(counter)/sum_time; %acceleration at y axis (m/sec2)
                acceleration_z(counter) = velocity_z(counter)/sum_time; %acceleration at z axis (m/sec2)
            end


%                 % Autoencoder prediction
%                 autoencoder = trainAutoencoder([f_thumb_x(counter);f_thumb_y(counter);f_thumb_z(counter)],hidden_size,'MaxEpochs',60,'ScaleData',0,'ShowProgressWindow',false,'DecoderTransferFunction','purelin');
%                 pred(:,counter) = predict(autoencoder,[f_thumb_x(counter);f_thumb_y(counter);f_thumb_z(counter)]);
%                 mseError(counter) = mse([f_thumb_x(counter);f_thumb_y(counter);f_thumb_z(counter)] - pred);
%                 

            % Kalman filter estimation (Linear)
            % Estimation
            if (counter == 1)
                Q_estimate = [hand_x(counter);hand_y(counter);hand_z(counter);velocity_x(counter);velocity_y(counter);velocity_z(counter)]; %initial state for x y z Vx Vy Vz
            end

            Q_loc_meas(:,counter) = [hand_x(counter);hand_y(counter);hand_z(counter)]; %Real location from kinect
%                 u = norm([velocity_x(counter) velocity_y(counter) velocity_z(counter)]); %norm of velocity of thumb
            u = norm([acceleration_x(counter) acceleration_y(counter) acceleration_z(counter)]); %norm of acceleration of thumb

            Q_estimate = A * Q_estimate + B * u; % Predicted next state (linear)

            P = A * P * A' + Ex; % Predict next covariance

            K = P * C' * inv( C * P * C' + Ez ); % Kalman Gain factor

            if ~isnan(Q_loc_meas(:,counter))
                Q_estimate = Q_estimate + K * ( Q_loc_meas(:,counter) - C * Q_estimate ); %Update the state estimation
            end

            P = ( eye(6) - K * C ) * P; %Update covariance

            %Store data
            Q_loc_estimate = [Q_loc_estimate; Q_estimate(1:3)];
            vel_estimate = [vel_estimate;Q_estimate(4:6)];
            F_Q_estimate(:,counter) = Q_estimate(1:3);
            F_vel_estimate(:,counter) = Q_estimate(4:6);

            %Convert from real coordinates to pixel location in depth map to print
            depth_u = (depth_focal_length_x * Q_estimate(1) / Q_estimate(3)) + 512/2;
            depth_v = -((depth_focal_length_y * Q_estimate(2) / Q_estimate(3)) - 424/2);

            %Convert from real coordinates to pixel location in depth map to print
            color_u = (color_focal_length_x * Q_estimate(1) / Q_estimate(3)) + 1920/2;
            color_v = -((color_focal_length_y * Q_estimate(2) / Q_estimate(3)) - 1080/2);
            
            if (flagStart == 1)
%                 plot(rd*sin(th)+depth_u,rd*cos(th)+depth_v,'r'); % the kalman filter tracking in depth map
                plot(rd*sin(th)+color_u,rd*cos(th)+color_v,'r'); % the kalman filter tracking in color map
            end

            hold off; % release frame


%                 disp('Hand State');
%                 disp(hand_state(trackedBodies));
%                 disp('Kinect Location');
%                 disp(double(Q_loc_meas(1,counter)));
%                 disp(double(Q_loc_meas(2,counter)));
%                 disp(double(Q_loc_meas(3,counter)));
%                 disp('Kalman prediction');
%                 disp(double(Q_estimate(1)));
%                 disp(double(Q_estimate(2)));
%                 disp(double(Q_estimate(3)));
              counter = counter + 1; %counter for storing
              time(counter) = time(counter-1) + frame_dt;
              flushdata(depthVid); %flush frames to free memory
              flushdata(colorVid); % flush frames to free ram 
    %             flushdata(depthVid,'triggers'); %flush one frame per
    %             time
        end
        if (counter == 70)
            break % break after a number of points
        end
end

%% Plot graphs for kinect measurements and kalman prediction

% Plot the position graphs

close all
figure('units','normalized','outerposition',[0 0 1 1]);

% X axis
subplot(2,2,1);
plot(time(2:counter),Q_loc_meas(1,:)*1e3,'r');
hold on
plot(time(2:counter),F_Q_estimate(1,:)*1e3,'b');
% hold on
% plot(time(2:counter),smooth(Q_loc_meas(1,:)*1e3),'m');
title('X AXIS');
xlabel('Time (sec)');
ylabel('Position at X axis (mm)');
h1 = legend('Kinect Measurement','Kalman Filter Prediction','Theoritical Path');
set(h1, 'Position',[0.6 0.2 0.2 0.2]);
hold off

% Y axis
subplot(2,2,2)
plot(time(2:counter),Q_loc_meas(2,:)*1e3,'r');
hold on
plot(time(2:counter),F_Q_estimate(2,:)*1e3,'b');
% hold on
% plot(time(2:counter),smooth(Q_loc_meas(1,:)*1e3),'m');
title('Y AXIS');
xlabel('Time (sec)');
ylabel('Position at Y axis (mm)');
hold off

% Z axis
subplot(2,2,3)
plot(time(2:counter),Q_loc_meas(3,:)*1e3,'r');
hold on
plot(time(2:counter),F_Q_estimate(3,:)*1e3,'b');
% hold on
% plot(time(2:counter),smooth(Q_loc_meas(1,:)*1e3),'m');
title('Z AXIS');
xlabel('Time (sec)');
ylabel('Position at Z axis (mm)');
hold off

% Set font size and bold
set(findobj(gcf,'type','axes'),'FontName','Arial','FontSize',12,'FontWeight','Bold', 'LineWidth', 1);

%% Release the kinect
stop(depthVid)
stop(colorVid)
