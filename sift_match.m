%Theory :- %https://www.cs.ubc.ca/~lowe/keypoints/
function [num] = sift_match(video_object,main_folder_path_for_masks,first_frame,second_frame,f1_local_id,f2_local_id)
    %video_object = VideoReader('F:\University of Manitoba\Research\csv_data\Input Vids\1.mp4');
    %main_folder_path_for_masks = 'G:\1_features\';
    %first_frame =122;
    %second_frame = 425;
    %f1_local_id = 1;
    %f2_local_id = 1;

    %Load the mask
    f1_mask_filename = strcat(num2str(first_frame),'.mat');
    f2_mask_filename = strcat(num2str(second_frame),'.mat');
    full_path_1 = strcat(main_folder_path_for_masks,f1_mask_filename);
    full_path_2 = strcat(main_folder_path_for_masks,f2_mask_filename);
    f1_mask_nd = load(full_path_1);
    f2_mask_nd = load(full_path_2);
    f1_mask = f1_mask_nd.MaskData;
    f2_mask = f2_mask_nd.MaskData;
    f1_shape_mask =mat2gray(f1_mask(:,:,(f1_local_id+1))); %Range 0-1
    f2_shape_mask =mat2gray(f2_mask(:,:,(f2_local_id+1))); %Range 0-1

    %Proceed if the selected masks are correct.

    frame_1 = read(video_object,first_frame);
    frame_1_gray = rgb2gray(frame_1);
    frame_1_gray_masked = uint8(255 * mat2gray(im2double(frame_1_gray(:,:)).*f1_shape_mask));


    frame_2 = read(video_object,second_frame);
    frame_2_gray = rgb2gray(frame_2);
    frame_2_gray_masked = uint8(255 * mat2gray(im2double(frame_2_gray(:,:)).*f2_shape_mask));

    num = match_v2(frame_1_gray_masked, frame_2_gray_masked);
end
