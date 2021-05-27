%Edit these params
metaDataFile = 'G:\space_shuttle\space_shuttle_features.csv';
createOutputFile = 'G:\space_shuttle\space_shuttle_features_decoded.csv';
objectOfInterest = 'car';
detectionProbability = 85;
%mask_master_folder_path = 'G:\space_shuttle\';

data_file = readtable(metaDataFile,'ReadVariableNames',0);
[row,cols] = size(data_file);
global_filter = [];
writable_file = table2cell(data_file);
writable_file_matrix = cell2mat(writable_file(:,1:3));
for row_count = 1:row
    frame_id = writable_file(row_count,1);
    %row_extractor_1 = find(writable_file_matrix(:,1)==frame_id); %Select specific frame
    %[local_obj_ids,igrn] = size(row_extractor_1);    
    process_words = writable_file(row_count,5);
    [label,probability] = strtok(process_words);
    prob = strrep(probability,'%',' ');
    prob = str2double(prob);
    if prob > detectionProbability
        inboth_1 = strcmp(objectOfInterest,label);
        %inboth_2 = strcmp('car',label);
        if inboth_1 %|| inboth_2
            left = cell2mat(writable_file(row_count,6));
            right = cell2mat(writable_file(row_count,7));
            top = cell2mat(writable_file(row_count,8));
            bottom = cell2mat(writable_file(row_count,9));
            splitter_left = strsplit(left,{',','(',')',' '});
            splitter_right = strsplit(right,{',','(',')',' '});
            splitter_top = strsplit(top,{',','(',')',' '});
            splitter_bottom = strsplit(bottom,{',','(',')',' '});
            filter_row = [writable_file(row_count,1) writable_file(row_count,2) writable_file(row_count,3) str2double(splitter_left(2)) str2double(splitter_left(3)) str2double(splitter_right(2)) str2double(splitter_right(3)) str2double(splitter_top(2)) str2double(splitter_top(3)) str2double(splitter_bottom(2)) str2double(splitter_bottom(3))];
            global_filter = [global_filter;filter_row];
        end 
    end 
end
global_filter_mat = cell2mat(global_filter);
csvwrite(createOutputFile,global_filter_mat) 



