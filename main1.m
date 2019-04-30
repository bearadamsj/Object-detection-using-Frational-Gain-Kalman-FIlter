clc; 
clear all;
close all;
%% Video Initialization
video_name = 'viptraffic.avi'; %Video name
vid = VideoReader(video_name); 
nframes = vid.NumberOfFrames; %Number of frames
Height = vid.Height; % Height
Width = vid.Width; % Width
thr = 10; % Threshold for generating binary image of the noise
%% Kalman Filter Definition
a = 0.5;% value of alpha, being 0.5 gives best result see in paper
dt = 1;% sampling time for simplicity we choose 1.
A = [1 0 dt 0;0 1 0 dt;0 0 1 0 ;0 0 0 1];%state space defined from kinetic function.
B = [(dt^2)/2 (dt^2)/2 dt dt]';
Klist = zeros(4,2,119);
% Input to the system.
u = 0.1;
%We are observing the X, and Y position. So our observation is: y = [X Y]
H = [1 0 0 0;0 1 0 0];
% Covariance Matrices: 
State_Uncertainty = 10;
S = State_Uncertainty * eye(size(A)); % The state variables are independet, so the covariance matrix is a diagonal matrix.
% Defining the <Measurement Noise> Covariance Matrix R
Meas_Unertainty = 1;
R = Meas_Unertainty * eye(size(H,1));
% Defining the Dynamic Noise covariance matrix
% Dynamic noise characterize the transition noise from one state to another
Dyn_Noise_Variance = (0.01)^2;
Q = [(dt^2)/4 0 (dt^3)/2 0;0 (dt^2)/4 0 (dt^3)/2;(dt^3/2) 0 (dt^2) 0;0 (dt^3)/2 0 (dt^2)];
data=importdata('groundtruth_rect.txt');
%% Kalman Variables 
Input = [];
Kalman_Output = [];
x = [Height/2; Width/2; 0; 0;]; % Initial Values

%% Extracting Background
background_frame = BackgroundExt(video_name);
%% Extract the object(the noisy part of original background)
moving = zeros(Height,Width,nframes);
labeled_frames = zeros(Height,Width,nframes);
bb=0;
%loop over all the frames
for i=1:nframes-1
    current_frame = double(read(vid,i));
    %check if noise exist in 3 colour channels,if so give a positive value
    moving(:,:,i) = (abs(current_frame(:,:,1) - background_frame(:,:,1)) > thr)...
                   |(abs(current_frame(:,:,2) - background_frame(:,:,2)) > thr)...
                   |(abs(current_frame(:,:,3) - background_frame(:,:,3)) > thr);
    moving(:,:,i) = bwmorph(moving(:,:,i),'erode',2);%erosion of binary image
    labeled_frames(:,:,i) = bwlabel(moving(:,:,i),4); 
    %using the matlab fuction regionprops to retrive information for each
    %object patch(noise): Area(total pixel),centroid and boundbox
    stats{i} = regionprops(labeled_frames(:,:,i),'basic');
    [n_obj,features] = size(stats{i});
    area = 0;
    %loop all the object detected in the frame and pick out the biggest
    %which has the biggest area number
    if(n_obj ~= 0) 
         for k=1:n_obj
             if(stats{i}(k).Area > area)
                id(i) = k;
                area = stats{i}(k).Area;
             end
         end
    centroid(:,:,i) = stats{i}(id(i)).Centroid;
    %else give randome object centroid
    else
        centroid(:,:,i) = [x(1,1) x(2,1)];

       
    end

%%  object tracking using fractional gain kalman filter
 %-2 is because the klist will exeed the dimension and there is no need to predict the last frame
    %draw a box on the image frame indicating the position 
    tic;
    frames = read(vid,i);
    %frames = insertShape(frames,'rectangle',[data(i,1) data(i,2) data(i,3) data(i,4)],'LineWidth',2);
    frames = insertShape(frames,'rectangle',[centroid(1,1,i)-15 centroid(1,2,i)-15 30 30],'LineWidth',2);
    %%Kalman Update
    % Original Tracker.
    if(n_obj ~= 0)
        %input = [data(i,1); data(i,2)];
        input = [centroid(1,1,i); centroid(1,2,i)];
    else
        input=[];
    end
    % Estimate the next state (priori estimation)
    x = A*x;
    % Estimate the error covariance 
    S = A*S*A' + Q;
    % Kalman Gain Calculations
    %K_kalman = S*H'*inv(H*S*H'+R);
    K_kalman = (S*H')/(H*S*H' + R);
    % kalman filter initialization for frame number equals to 1
    if i == 1
        Klist = zeros(4,2,199);
        iteration = 1;
        facto = 1;
        k_sum = zeros(4,2);
    end
    %call the fractionalGain function to calculate Gain, store the new gain
    %to the k_list, and refresh the iteration number each time
    [k_new,iteration,facto,k_sum,Klist] = fractionalGain(K_kalman,a,i,Klist,facto,k_sum);
    k_new = k_new;

    % Update the estimation
    if(~isempty(input)) %Check if we have an input
        x = x + k_new*(input - H*x);
    end
    

    % Update the error covariance
    S = (eye(size(S,1)) - k_new*H)*S;
    % Save the measurements for plotting
    Kalman_Output = H*x;
    stateOutput(:,i) = [x(1),x(2),x(3),x(4)];
    newPosition1(i,:) = [Kalman_Output(1) centroid(1,1,i)];
    %RMSE(i,1) = (((x(1)-data(i,1))^2 + (x(2)-data(i,2))^2));
    RMSE(i,1) = (((x(1)-centroid(1,1,i))^2 + (x(2)-centroid(1,2,i))^2));
      frames = insertShape(frames,'rectangle',[Kalman_Output(1)-15 Kalman_Output(2)-15 30 30],'LineWidth',2,'Color','green');
    %frames = insertShape(frames,'rectangle',[Kalman_Output(1) Kalman_Output(2) data(i,3) data(i,4)],'LineWidth',2,'Color','green');
    tracking(:,:,:,i) = frames;
    toc;
    runningTime(i) = toc;
end

%% 
implay(tracking,15);