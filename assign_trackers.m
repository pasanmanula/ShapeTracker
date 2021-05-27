function [global_tracking_matrix] = assign_trackers(between_frames_spatial_color,global_tracking_matrix)
    matching_matrix = between_frames_spatial_color;
    [objects,igr] = size(matching_matrix);
    first_frame = matching_matrix(1,1);
    second_frame = first_frame +1;

    if first_frame == 1
        for object=1:objects
            f2_obj = matching_matrix(object,3);
            if f2_obj > 0
                insert_row = [matching_matrix(object,2) f2_obj];
                global_tracking_matrix(object,[first_frame second_frame]) = insert_row;
            else
                insert_row = [matching_matrix(object,2) NaN];
                global_tracking_matrix(object,[first_frame second_frame]) = insert_row;
            end
        end
    else
        for object=1:objects
            f1_obj = matching_matrix(object,2);
            f2_obj = matching_matrix(object,3);
            [track_id,igr] = find(global_tracking_matrix(:,first_frame)==f1_obj);
            if track_id >0
                if f2_obj > 0
                    insert_row = [matching_matrix(object,2) f2_obj];
                    global_tracking_matrix(track_id,second_frame) = f2_obj;
                else
                    insert_row = [matching_matrix(object,2) NaN];
                    global_tracking_matrix(track_id,second_frame) = f2_obj;
                end
            else
                %Crete a new track
                [nooftracks,igr] = size(global_tracking_matrix(:,first_frame));
                new_track_id = nooftracks+1;
                if f2_obj > 0
                    insert_row = [matching_matrix(object,2) f2_obj];
                    global_tracking_matrix(new_track_id,[first_frame second_frame]) = insert_row;
                else
                    insert_row = [matching_matrix(object,2) NaN];
                    global_tracking_matrix(new_track_id,[first_frame second_frame]) = insert_row;
                end
                
                %error('Unexpected happened')
            end
        end
    end

end

