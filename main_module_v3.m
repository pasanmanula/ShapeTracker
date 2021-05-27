%Hyper Parameters
base_th = 60.0;
k_clusters = 5;
acceptable_percentage = 10.0;
saturation_val = 1;
fps = 29.97;
max_frame_reverse_seek = 10;
global_tracking_matrix = [];

%Input Args
outputFromTheObjectDetector = 'G:\RaceCar\Seq02\';
videoFilePath = 'G:\RaceCar\Seq02.mp4';
decoded_raw_file = csvread('G:\RaceCar\Seq02_decoded.csv');

video_object = VideoReader(videoFilePath);
[rows,cols] = size(decoded_raw_file);
global_first_frame = decoded_raw_file(1,1);
global_last_frame = decoded_raw_file(end,1);
%global_last_frame = 125;
skip_size = 1;
global_frame = [];

for frame_id = global_first_frame:skip_size:global_last_frame
    first_frame = frame_id;
    if first_frame ~= global_last_frame
        next_frame = first_frame+1;
        %Check next frame existance
        local_object_extractor = ismember(next_frame,decoded_raw_file(:,1));
        if local_object_extractor > 0
            between_two_frames_mapping = centroid_proximity_module(first_frame,base_th,decoded_raw_file);
            if ~isempty(between_two_frames_mapping)
                     between_frames_spatial_color = spatial_color_tracker_module(between_two_frames_mapping,saturation_val,k_clusters,acceptable_percentage,video_object,decoded_raw_file,outputFromTheObjectDetector);
                     if ~isempty(between_frames_spatial_color)      
                        global_tracking_matrix = assign_trackers(between_frames_spatial_color,global_tracking_matrix);
                        %global_frame = [global_frame;frame_id];
                        skip_size = 1;
                     else
                         global_tracking_matrix(:,next_frame) = 0;
                     end
            else
                global_tracking_matrix(:,next_frame) = 0;
            end
        else
            %do not process this current frame. try next next frame
            global_tracking_matrix(:,next_frame) = 0;
            skip_size = 2;
        end
    end
end

%Invoke occulssion handler
[tracking_data,data_only] = occulusion_handler_v3(video_object,outputFromTheObjectDetector,global_tracking_matrix,decoded_raw_file,fps,max_frame_reverse_seek);
%save('full_occ_handled_tracking_data_v3.mat','tracking_data');
save('filtered_tracking.mat','tracking_data');
%Add occ predictions from Kalman position estimator
occ_tracking_data = occ_filler(decoded_raw_file,fps,tracking_data);
save('occ_data.mat','occ_tracking_data');