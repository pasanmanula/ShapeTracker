function [between_frames_spatial_color] = spatial_color_tracker_module(between_frames_hist_matching,saturation_val,k_clusters,acceptable_percentage,video_object,decoded_raw_file,main_folder_path_for_masks)
    %Import local mapping sequence from centroid proximity module
    mapping_sequence_by_hist = between_frames_hist_matching;
    [noofshapes,igr] = size(mapping_sequence_by_hist);
    first_frame = mapping_sequence_by_hist(1,1);
    second_frame = first_frame+1;
    first_frame_rgb = read(video_object,first_frame);
    second_frame_rgb = read(video_object,second_frame);
    %Image metadata calculation
    [rows,cols,channels] = size(first_frame_rgb);
    %Load the mask
    f1_mask_filename = strcat(num2str(first_frame),'.mat');
    f2_mask_filename = strcat(num2str(second_frame),'.mat');
    full_path_1 = strcat(main_folder_path_for_masks,f1_mask_filename);
    full_path_2 = strcat(main_folder_path_for_masks,f2_mask_filename);
    f1_mask_nd = load(full_path_1);
    f2_mask_nd = load(full_path_2);
    f1_mask = f1_mask_nd.MaskData;
    f2_mask = f2_mask_nd.MaskData;
    %Saturated Image (Pre-processing)
    %First frame
    hsv_img = rgb2hsv(first_frame_rgb);
    hsv_img(:,:,2) = hsv_img(:,:,2) * saturation_val;
    hsv_img(:,:,3) = hsv_img(:,:,3) * saturation_val;
    first_frame_rgb = hsv2rgb(hsv_img);
    %Second frame
    hsv_img = rgb2hsv(second_frame_rgb);
    hsv_img(:,:,2) = hsv_img(:,:,2) * saturation_val;
    hsv_img(:,:,3) = hsv_img(:,:,3) * saturation_val;
    second_frame_rgb = hsv2rgb(hsv_img);

    local_object_extractor = find(decoded_raw_file(:,1)==first_frame);
    f1_objects = decoded_raw_file(local_object_extractor,:);
    local_object_extractor = find(decoded_raw_file(:,1)==second_frame);
    f2_objects = decoded_raw_file(local_object_extractor,:);
    between_frames_spatial_color = [];
    for f1_local_shape = 1:noofshapes
        f1_local_id = mapping_sequence_by_hist(f1_local_shape,2);
        f2_local_id = mapping_sequence_by_hist(f1_local_shape,3);
        if f2_local_id >0
            f1_shape_x_loc = f1_objects(f1_local_id,2);
            f2_shape_x_loc = f2_objects(f2_local_id,2);
            x_offset_val = f1_shape_x_loc - f2_shape_x_loc;
            if f2_local_id >0
               f1_shape_mask =mat2gray(f1_mask(:,:,(f1_local_id+1))); %Range 0-1
               f2_shape_mask =mat2gray(f2_mask(:,:,(f2_local_id+1))); %Range 0-1
                %Extract frame_1 from the video
               frame_1 = first_frame_rgb;
               frame_1(:,:,1) = im2double(frame_1(:,:,1)).*f1_shape_mask; %Range 0-1
               frame_1(:,:,2) = im2double(frame_1(:,:,2)).*f1_shape_mask;
               frame_1(:,:,3) = im2double(frame_1(:,:,3)).*f1_shape_mask;

                %Extract frame_2 from the video
               frame_2 = second_frame_rgb;
               frame_2(:,:,1) = im2double(frame_2(:,:,1)).*f2_shape_mask;
               frame_2(:,:,2) = im2double(frame_2(:,:,2)).*f2_shape_mask;
               frame_2(:,:,3) = im2double(frame_2(:,:,3)).*f2_shape_mask;

                %First frame
                %Make the 2D matrixes as column vectors for easment
                f1_red_channel_col = reshape(frame_1(:,:,1),[],1);
                f1_green_channel_col = reshape(frame_1(:,:,2),[],1);
                f1_blue_channel_col = reshape(frame_1(:,:,3),[],1);
                %Second frame
                %Make the 2D matrixes as column vectors for easment
                f2_red_channel_col = reshape(frame_2(:,:,1),[],1);
                f2_green_channel_col = reshape(frame_2(:,:,2),[],1);
                f2_blue_channel_col = reshape(frame_2(:,:,3),[],1);

                red_channel_col = [f1_red_channel_col;f2_red_channel_col];
                green_channel_col = [f1_green_channel_col;f2_green_channel_col];
                blue_channel_col = [f1_blue_channel_col;f2_blue_channel_col];
                %Concat the above vectors for easment
                X = [red_channel_col green_channel_col blue_channel_col];

                %K Mean clustering 
                [idx_f1f2,C] = kmeans(X, k_clusters); % k_clusters = number of classes

                %2D - 1 channel matrix - clustered color matrix - MASTER MATRIX FOR FURTHER
                %ANALYSIS
                range = rows*cols;
                first_frame_new_color_cluster_matrix = permute(reshape(idx_f1f2(1:range,1), rows,1,cols), [1 3 2]);
                second_frame_new_color_cluster_matrix = permute(reshape(idx_f1f2((range+1):end,1), rows,1,cols), [1 3 2]);

                %Shape Offset Compansation (Always second frame only)
                second_frame_with_x_offset = circshift(second_frame_new_color_cluster_matrix,x_offset_val,2); %2 - Horizontal offset

                %Spatial color variation between consecative frames
                resultant_img = abs(first_frame_new_color_cluster_matrix - second_frame_with_x_offset);

                %Spatial color area based error calculation
                idx2 = resultant_img > 0;
                c_out= sum(idx2(:));
                percentage = (c_out/(rows*cols))*100;
                if percentage <= acceptable_percentage
                    insert_row = [first_frame f1_local_id f2_local_id percentage];
                    between_frames_spatial_color = [between_frames_spatial_color;insert_row];
                else
                    insert_row = [first_frame f1_local_id 0 percentage];
                    between_frames_spatial_color = [between_frames_spatial_color;insert_row];
                end
            end            
        end

    end
end

%{
%Visualization Only ======================================================
%First frame
color_coded_first_frame = mat2gray(frame_1);
for pixel_loc_row = 1:rows
    for pixel_loc_col = 1:cols
        new_color_cluster_id =  first_frame_new_color_cluster_matrix(pixel_loc_row,pixel_loc_col);
        color_coded_first_frame(pixel_loc_row,pixel_loc_col,1) = C(new_color_cluster_id,1); %New Red Channel
        color_coded_first_frame(pixel_loc_row,pixel_loc_col,2) = C(new_color_cluster_id,2); %New Red Channel
        color_coded_first_frame(pixel_loc_row,pixel_loc_col,3) = C(new_color_cluster_id,3); %New Red Channel
    end
end
first_frame_vis = uint8(255 * mat2gray(color_coded_first_frame));

%Second frame
color_coded_second_frame = mat2gray(frame_2);
for pixel_loc_row = 1:rows
    for pixel_loc_col = 1:cols
        new_color_cluster_id =  second_frame_new_color_cluster_matrix(pixel_loc_row,pixel_loc_col);
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,1) = C(new_color_cluster_id,1); %New Red Channel
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,2) = C(new_color_cluster_id,2); %New Red Channel
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,3) = C(new_color_cluster_id,3); %New Red Channel
    end
end
second_frame_vis = uint8(255 * mat2gray(color_coded_second_frame));

%Second frame with x-offset
color_coded_second_frame = mat2gray(second_frame_rgb);
for pixel_loc_row = 1:rows
    for pixel_loc_col = 1:cols
        new_color_cluster_id =  second_frame_with_x_offset(pixel_loc_row,pixel_loc_col);
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,1) = C(new_color_cluster_id,1); %New Red Channel
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,2) = C(new_color_cluster_id,2); %New Red Channel
        color_coded_second_frame(pixel_loc_row,pixel_loc_col,3) = C(new_color_cluster_id,3); %New Red Channel
    end
end
second_frame_vis_x_offset = uint8(255 * mat2gray(color_coded_second_frame));

%Diff Image
resultant_img_vis = uint8(255 * mat2gray(resultant_img));

subplot(2, 2, 1);
imshow(first_frame_vis);
subplot(2, 2, 2);
imshow(second_frame_vis);
subplot(2, 2, 3);
imshow(second_frame_vis_x_offset);
subplot(2, 2, 4);
imshow(resultant_img_vis);
%}

