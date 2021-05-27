%tracking_data_mat = load('G:\1_features\1_50_frames_tracked.mat');
%tracking_data = tracking_data_mat.global_tracking_matrix;
%raw_data = csvread('F:\University of Manitoba\Research\csv_data\Sample_Images\1_features_decoded_v2.csv');

function [average_distance] = signal_matching(master,slave,sort_mat,tracking_data,raw_data,fps)
    %{
    last_track_id = ut_id_global(1,1);
    [row,igr] = find(sort_mat(:,1) == last_track_id);
    [igr,row_track] = min(sort_mat(1:(row-1),3));
    candidate_track_id = sort_mat(row_track,1); %closest to last_track_id
    %}
    %For debugging purposes let's say
    last_track_id = master;
    candidate_track_id = slave;

    %===================================Candidate Track ID =============
    %Predict location for track_id=4
    %Make sure data is on the correct format
    [row_id,igr] = find(sort_mat(:,1) == candidate_track_id);
    [row_id_last,igr] = find(sort_mat(:,1) == last_track_id);

    start_frame = sort_mat(row_id,2);
    end_frame = sort_mat(row_id,3);
    required_gap = abs(sort_mat(row_id_last,2) - sort_mat(row_id,3));
    actual_positions = [];
    for frame_id = start_frame:end_frame
       local_obj_id = tracking_data(candidate_track_id,frame_id);
        if local_obj_id >0 
            row_extractor_1 = find(raw_data(:,1)==frame_id);
            objects_1 = raw_data(row_extractor_1,:);
            cent_x = objects_1(local_obj_id,2);
            cent_y = objects_1(local_obj_id,3);
            position = [cent_x cent_y frame_id];
            actual_positions = [actual_positions;position];
        end 
    end
    %Adaptive acceleration magnitude calculation for Kalman estimation =
    %a = (v-u)/t
    [rows,igr] = size(actual_positions);
    if rows >= 2
            initial_velocity = (sqrt((actual_positions(1,1) - actual_positions(2,1)).^2 + (actual_positions(1,2) - actual_positions(2,2)).^2))/fps;
            final_velocity = (sqrt((actual_positions((end-1),1) - actual_positions(end,1)).^2 + (actual_positions((end-1),2) - actual_positions(end,2)).^2))/fps;
            tot_frames = abs(actual_positions(end,3)-actual_positions(1,3));
            delta_t = tot_frames/fps;
            acc = (final_velocity - initial_velocity) / delta_t;
    else
        %Not enough data
        %Set acc manually (Not good)
        acc = 0.1;
    end

    for add_row=1:required_gap
        actual_positions(end+1,:) = NaN;
    end
    %Kalman estimation for track_4
    total_data_after_kalman = kalman_estimate(actual_positions,acc);

    %===================================Master Track ID =============
    %Predict location for track_id=5
    start_frame_master = sort_mat(row_id_last,2);
    end_frame = sort_mat(row_id_last,3);
    actual_positions_primary = [];
    for frame_id = start_frame:end_frame
       local_obj_id = tracking_data(last_track_id,frame_id);
        if local_obj_id >0 
            row_extractor_1 = find(raw_data(:,1)==frame_id);
            objects_1 = raw_data(row_extractor_1,:);
            cent_x = objects_1(local_obj_id,2);
            cent_y = objects_1(local_obj_id,3);
            position = [cent_x cent_y frame_id];
            actual_positions_primary = [actual_positions_primary;position];
        end 
    end

    %Change row order from first to last ---> last to first (Reversing...)
    %Descending order according to the frame number
    actual_positions_primary_reverse = sortrows(actual_positions_primary,-3);
    [rows,igr] = size(actual_positions_primary_reverse);
    if rows >= 2
            initial_velocity = (sqrt((actual_positions_primary_reverse(1,1) - actual_positions_primary_reverse(2,1)).^2 + (actual_positions_primary_reverse(1,2) - actual_positions_primary_reverse(2,2)).^2))/fps;
            final_velocity = (sqrt((actual_positions_primary_reverse((end-1),1) - actual_positions_primary_reverse(end,1)).^2 + (actual_positions_primary_reverse((end-1),2) - actual_positions_primary_reverse(end,2)).^2))/fps;
            tot_frames = abs(actual_positions_primary_reverse(end,3)-actual_positions_primary_reverse(1,3));
            delta_t = tot_frames/fps;
            acc = (final_velocity - initial_velocity) / delta_t;
    else
        %Not enough data
        %Set acc manually (Not good)
        acc = 0.1;
    end

    %Add gaps
    for add_row=1:required_gap
        actual_positions_primary_reverse(end+1,:) = NaN;
    end
    %Kalman estimation for track_5
    total_data_after_kalman_master = kalman_estimate(actual_positions_primary_reverse,acc);
    total_data_after_kalman_master_original_ori = flip(total_data_after_kalman_master,1);

    %Correlation test starts here
    [rows_s,igr] = size(total_data_after_kalman);
    init_row = rows_s - required_gap+1;
    signal_1 =  total_data_after_kalman(init_row:end,[3 4]); %Predictions for track ID 4
    master_signal = total_data_after_kalman_master_original_ori(1:required_gap,[3 4]); %Predictions for Track ID 5

    distance_vector = [];
    [no_of_frames,igr] = size(signal_1);
    for expected_frame=1:no_of_frames
        euclidean_dist = sqrt((signal_1(expected_frame,1) - master_signal(expected_frame,1))^2 + (signal_1(expected_frame,2) - master_signal(expected_frame,2))^2);
        distance_vector = [distance_vector euclidean_dist];
    end
    average_distance = sum(distance_vector(1,:))/no_of_frames;
end




