function [tracking_data,data_only] = occulusion_handler_v2(video_object,main_folder_path_for_masks,tracking_data,raw_data,fps,max_frame_reverse_seek)
    %tracking_data_mat = load('G:\1_features\1_50_frames_tracked.mat');
    %tracking_data = tracking_data_mat.global_tracking_matrix;
    %tracking_data = global_tracking_matrix;
    [tot_no_of_tracks,tot_frames] = size(tracking_data);
    %raw_data = csvread('F:\University of Manitoba\Research\csv_data\Sample_Images\1_features_decoded_v2.csv');
    %max_frame_reverse_seek = 12;
    average_distance_global = [];
    %Generating the comparison matrix
    comparison_matrix = [];
    for track_id=1:tot_no_of_tracks
        temp_var = find(tracking_data(track_id,:) >0);
        f_count = sum(tracking_data(track_id,:) >0);
        if f_count <tot_frames
            first_frame = temp_var(1,1);
            last_frame = temp_var(1,end);
            insert_row = [track_id first_frame last_frame];
            comparison_matrix = [comparison_matrix;insert_row]; 
        end
    end

    sort_mat =sortrows(comparison_matrix,2); %Sort by first frame


    %===============================================================
    %If two or more hae same starting frame, remove all. (Which is not a good
    %thing. Consider this later)
    %Assuming that the above statement has done,
    [tot_rows,igr] = size(sort_mat);
    data_only = [];
    while (tot_rows >=2)
        master_row = tot_rows;
        master = sort_mat(master_row,1);
        master_first_frame = sort_mat(master_row,2);
        master_last_frame = sort_mat(master_row,3);
        candidate_rows = find(sort_mat(:,3) < master_first_frame);
        cp_candidate_rows = candidate_rows;
        %V3 update starts here
        %Candidate pool :- Whose last frame < master's last frame and
        %within max_frame_reverse_seek
        candidate_mat = sort_mat(candidate_rows,:);
        reseverse_last_frame = master_first_frame - max_frame_reverse_seek;
        if reseverse_last_frame > 0
            candidate_rows_full = find(candidate_mat(:,3) >= reseverse_last_frame);
            candidate_rows_full = sort_mat(candidate_rows_full,:);    
            candidate_rows = candidate_rows_full;
        end
    
        if isempty(candidate_rows)
            if tot_rows >= 2
                tot_rows = tot_rows-1;
            else
                break;
            end
        else
            [candidates,igr] = size(candidate_rows);
            decision_mat = [];
            for candidate = 1:candidates
                
                slave = candidate_rows(candidate); %track id 29
                slave_first_frame = candidate_rows(candidate,2);
                slave_last_frame = candidate_rows(candidate,3);
                %From SIFT
                %SIFT check code goes here
                f1_local_id = tracking_data(slave,slave_last_frame);
                f2_local_id = tracking_data(master,master_first_frame);
                num = sift_match(video_object,main_folder_path_for_masks,slave_last_frame,master_first_frame,f1_local_id,f2_local_id);
                %From Kalman Filter
                average_distance = signal_matching(master,slave,sort_mat,tracking_data,raw_data,fps);
                insert_row = [slave num average_distance];
                decision_mat = [decision_mat;insert_row];
            end            
            %===============================================================
            %assume good slave has the min average_distance and max sift connections,
            [val,correct_row_by_distance] = min(decision_mat(:,3));
            [val,correct_row_by_sift_conn] = max(decision_mat(:,2));
            if correct_row_by_distance == correct_row_by_sift_conn
                slave = decision_mat(correct_row_by_distance,1); %Best Case
                selected_slave_row = find(sort_mat(:,1) == slave);
            else
                %wrt to distance row <> wrt to sift connections % Worst
                %case
                %Match by last appearace . [Not a good thing]
                %Picking up least frame difference between the master
                slave = decision_mat(correct_row_by_sift_conn,1); %Best Case
                selected_slave_row = find(sort_mat(:,1) == slave);
                
            end
            
            %copy master tracking data to slave
            cp_init = master_first_frame;
            cp_end = master_last_frame;
            tracking_data(slave,cp_init:cp_end) = tracking_data(master,cp_init:cp_end);
            tracking_data(master,:) = [];
            %Update sort_mat
            sort_mat(selected_slave_row,3) = cp_end;
            sort_mat(master_row,:) = [];
            %===============================================================
            tot_rows = tot_rows-1;
        end 
    end

end



