
function [between_two_frames_mapping] = centroid_proximity_module(first_frame,base_th,decoded_raw_file)
    
    %Read first frame from the entire video clip
    %first_frame = 31;
    local_object_extractor = find(decoded_raw_file(:,1)==first_frame);
    f1_objects = decoded_raw_file(local_object_extractor,:);
    [nofobjects_1,igr] = size(f1_objects);

    %Read second frame from the entire video clip
    %Note NOTE NOTE:
    %PLEASE CHECK NEXT FRAME EXISTS BEFORE CONTINUE....DONE!!!!
    second_frame = first_frame+1;
    local_object_extractor = find(decoded_raw_file(:,1)==second_frame);
    f2_objects = decoded_raw_file(local_object_extractor,:);
    [nofobjects_2,igr] = size(f2_objects);
    if nofobjects_2 > 0
        %Distance Matrix Calculation
        comparison_mat = zeros(nofobjects_1,nofobjects_2);
        for over_f1_object = 1:nofobjects_1
            f1_object_cent_loc = f1_objects(over_f1_object,[2 3]);
            for over_f2_object = 1:nofobjects_2
                f2_object_cent_loc = f2_objects(over_f2_object,[2 3]);
                euclidean_dist = sqrt((f1_object_cent_loc(1,1) - f2_object_cent_loc(1,1))^2 + (f1_object_cent_loc(1,2) - f2_object_cent_loc(1,2))^2);
                if euclidean_dist == 0
                    euclidean_dist = 0.001;
                end
                comparison_mat(over_f1_object,over_f2_object) = euclidean_dist;
            end
        end
           %NOTE NOTE NOTE
           %Need to compare shape areas if comparison_mat has more than 1
           %SAME min value like in between frame 32 && 33. Still didn't
           %write that code
        %Centroid mapping based on distance matrix
        between_two_frames_mapping = [];
        if nofobjects_1 <= nofobjects_2
             for first_frame_object = 1:nofobjects_1
                [minValue,minIdx]=min(comparison_mat((comparison_mat(:))>0));
                if minValue <= base_th
                    [f1_object,f2_object]=ind2sub(size(comparison_mat),minIdx);
                    comparison_mat(f1_object,f2_object) = inf;
                    map = [first_frame f1_object f2_object];
                    between_two_frames_mapping = [between_two_frames_mapping;map];
                else
                    [f1_object,f2_object]=ind2sub(size(comparison_mat),minIdx);
                    comparison_mat(f1_object,f2_object) = inf;
                    map = [first_frame f1_object 0];
                    between_two_frames_mapping = [between_two_frames_mapping;map];
                    %error('Error. Exceeds threshold max.')
                end
            end
        else
            for first_frame_object = 1:nofobjects_2
                [minValue,minIdx]=min(comparison_mat((comparison_mat(:))>0));
                if minValue <= base_th
                    [f1_object,f2_object]=ind2sub(size(comparison_mat),minIdx);
                    comparison_mat(f1_object,f2_object) = inf;
                    map = [first_frame f1_object f2_object];
                    between_two_frames_mapping = [between_two_frames_mapping;map];
                else
                    [f1_object,f2_object]=ind2sub(size(comparison_mat),minIdx);
                    comparison_mat(f1_object,f2_object) = inf;
                    map = [first_frame f1_object 0];
                    between_two_frames_mapping = [between_two_frames_mapping;map];
                    %error('Error. Exceeds threshold max.')
                end
            end
            
        end

    end
end
