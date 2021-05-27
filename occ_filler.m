function global_occ_handler = occ_filler(raw_data,fps,tracking_data)
    %raw_data = csvread('G:\space_shuttle\space_shuttle_features_decoded.csv');
    %fps=29.97;
    global_occ_handler = [];
    full_mat = [];
    [global_tracks,tot_frames_original] = size(tracking_data);
    for track = 1:global_tracks
        candidate_track_id = track;
        occ_row = find(tracking_data(candidate_track_id,:) <= 0); %OCC SPLITTER
        if isempty(occ_row)
        else
            [igr,length] = size(occ_row);

            auto_val_from_arr = occ_row(1);
            manual_increment = occ_row(1);
            temp_mat = [];
            for check_id=2:length
                auto_val_from_arr = occ_row(check_id);
                manual_increment = manual_increment +1;
                if auto_val_from_arr == manual_increment   
                else
                    insert_row = [track auto_val_from_arr];
                    temp_mat = [temp_mat;insert_row];
                    manual_increment = auto_val_from_arr;
                end
            end
            first_p = occ_row(1);
            for run_count = 2:length
                temp_var = occ_row(run_count);
                if ismember(temp_var,temp_mat(:,2))
                    last_p = occ_row(run_count-1);
                    full_mat_row = [track first_p last_p];
                    full_mat = [full_mat;full_mat_row];
                    first_p = occ_row(run_count);
                end
            end
            last_p = occ_row(end);
            full_mat_row = [track first_p last_p];
            full_mat = [full_mat;full_mat_row];
        end

    end

    %==================================================
    [iterations,igr] = size(full_mat);
    %candidate_track_id = 1;
    %first_occ = 24;
    %last_occ = 31;
    for iteration = 1:iterations
        candidate_track_id = full_mat(iteration,1);
        first_occ = full_mat(iteration,2);
        last_occ = full_mat(iteration,3);
        start_fill = first_occ - 5; %Go before some data disappear
        data_available_first_occ = first_occ-2; %To calculate initial velocity
        if start_fill >0
            if data_available_first_occ >0
                if first_occ > 1
                    if (last_occ+1) < tot_frames_original
                        if (last_occ+2) <= tot_frames_original
                                sanity_check = tracking_data(candidate_track_id,start_fill:(first_occ-1));
                                sanity_check_2 = tracking_data(candidate_track_id,(last_occ+1):(last_occ+2));
                                if any(sanity_check(:)==0)
                                    %Not Good. Not enough data
                                else
                                    %Good
                                    if any(sanity_check_2(:) == 0)
                                        %Not enough data
                                    else
                                        initial_pos =[];
                                        local_id = tracking_data(candidate_track_id,(first_occ-2)); %Initial Instantaneous Velocity
                                        row_extractor_1 = find(raw_data(:,1)==(first_occ-2));
                                        objects_1 = raw_data(row_extractor_1,:);
                                        cent_x = objects_1(local_id,2);
                                        cent_y = objects_1(local_id,3);
                                        insert_row = [cent_x cent_y];
                                        initial_pos = [initial_pos;insert_row];
                                        local_id = tracking_data(candidate_track_id,(first_occ-1));
                                        row_extractor_1 = find(raw_data(:,1)==(first_occ-1));
                                        objects_1 = raw_data(row_extractor_1,:);
                                        cent_x = objects_1(local_id,2);
                                        cent_y = objects_1(local_id,3);
                                        insert_row = [cent_x cent_y];
                                        initial_pos = [initial_pos;insert_row];
                                        initial_velocity = (sqrt((initial_pos(1,1) - initial_pos(2,1)).^2 + (initial_pos(1,2) - initial_pos(2,2)).^2))/fps;

                                        final_pos = [];
                                        local_id = tracking_data(candidate_track_id,(last_occ+1)); %Initial Instantaneous Velocity
                                        row_extractor_1 = find(raw_data(:,1)==(last_occ+1));
                                        objects_1 = raw_data(row_extractor_1,:);
                                        cent_x = objects_1(local_id,2);
                                        cent_y = objects_1(local_id,3);
                                        insert_row = [cent_x cent_y];
                                        final_pos = [final_pos;insert_row];
                                        local_id = tracking_data(candidate_track_id,(last_occ+2)); %Final Instantaneous Velocity
                                        row_extractor_1 = find(raw_data(:,1)==(last_occ+2));
                                        objects_1 = raw_data(row_extractor_1,:);
                                        cent_x = objects_1(local_id,2);
                                        cent_y = objects_1(local_id,3);
                                        insert_row = [cent_x cent_y];
                                        final_pos = [final_pos;insert_row];
                                        final_velocity = (sqrt((final_pos(1,1) - final_pos(2,1)).^2 + (final_pos(1,2) - final_pos(2,2)).^2))/fps;


                                        tot_frames = abs(last_occ-first_occ); %Acceleration calculation
                                        delta_t = tot_frames/fps;
                                        %v=u+at
                                        acc = (final_velocity - initial_velocity) / delta_t;
                                        %s=ut+0.5at^2
                                        relative_distance = sqrt((initial_pos(2,1)-final_pos(1,1)).^2 + (initial_pos(2,2)-final_pos(1,2)).^2);
                                        %acc = 2*(relative_distance-(initial_velocity*delta_t))/(delta_t.^2);

                                        %Filling existing and non-existing (NaN) centroid positioning data for
                                        %Kalman estimator
                                        end_fill = last_occ;%Strict
                                        actual_positions = [];
                                        for frame_id = start_fill:end_fill
                                           local_obj_id = tracking_data(candidate_track_id,frame_id);
                                            if local_obj_id >0 
                                                row_extractor_1 = find(raw_data(:,1)==frame_id);
                                                objects_1 = raw_data(row_extractor_1,:);
                                                cent_x = objects_1(local_obj_id,2);
                                                cent_y = objects_1(local_obj_id,3);
                                                position = [cent_x cent_y frame_id];
                                                actual_positions = [actual_positions;position];
                                            else
                                                position = [NaN NaN frame_id];
                                                actual_positions = [actual_positions;position];
                                            end 
                                        end

                                        %Kalman estimator
                                        total_data_after_kalman = kalman_estimate(actual_positions,acc);

                                        %Save occ data in a separate .mat file
                                        occ_objects = [];
                                        search_range = find(first_occ <= total_data_after_kalman(:,5));
                                        occ_objects = total_data_after_kalman(search_range,[3 4 5]);
                                        occ_objects(:,(end+1)) = candidate_track_id; 
                                        global_occ_handler = [global_occ_handler;occ_objects];
                                    end
                                end
                        end
                    end
                end
            end
        end
        

    end


    %Remove NaN value rows from global_occ_handler
    garbage_rows = find(isnan(global_occ_handler(:,1)));
    global_occ_handler(garbage_rows,:) = [];
end


