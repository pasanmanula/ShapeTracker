import numpy as np
import cv2
from numpy import genfromtxt
from scipy.io import loadmat

#Change the following params according to the sequence.
inputVideoFile = 'G:\RaceCar\\Seq02.mp4'
outputVideoFile = 'G:\RaceCar\\Seq02\\outputSecond.avi'
trackingMetaData = 'G:\RaceCar\\Seq02\\filtered_tracking.mat'
occMetaData = 'G:\RaceCar\\Seq02\\occ_data.mat'
rawMetaData = 'G:\RaceCar\\Seq02_decoded.csv'

cap = cv2.VideoCapture(inputVideoFile)
width  = cap.get(3) # float # float
height = cap.get(4)
fps = cap.get(cv2.CAP_PROP_FPS)
fps_vis = "{:.2f}".format(fps)

overlay = cv2.imread('focus_track.png',-1)
overlay = cv2.resize(overlay, (100,100))
w,h,c = overlay.shape

#global_tracking_data = genfromtxt('F:\University of Manitoba\Research\csv_data\Overall Tracking\\long_clip_Kalman_final.csv', delimiter=',')
#import tracking data from .mat file
load_mat_file = loadmat(trackingMetaData)
global_tracking_data = load_mat_file['tracking_data']
g_row,g_cols = global_tracking_data.shape

#Import occ handled data
load_mat_file_occ = loadmat(occMetaData)
global_tracking_data_occ = load_mat_file_occ['occ_tracking_data']
gocc_row,gocc_cols = global_tracking_data_occ.shape

raw_data = genfromtxt(rawMetaData, delimiter=',')
r_row,r_cols = raw_data.shape
frame_id = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(outputVideoFile,fourcc, fps, (int(width),int(height)))
frame_id_index = 0
location_2 = (100,100)

while(cap.isOpened()):
    ret, frame = cap.read()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)
    if ret==True:
        frame_id = frame_id_index + 1
        [row_extractor_row] = np.where(raw_data[:,0] == frame_id)
        row_extractor_mat = raw_data[row_extractor_row,:]

        for global_id in range(g_row):
            local_id = global_tracking_data[global_id,frame_id_index]
            
            if local_id > 0 :
#            	print(frame_id)
                local_id = local_id -1
                location = []
                cent_x = row_extractor_mat[int(local_id),1]
                cent_y = row_extractor_mat[int(local_id),2]
                print_txt = "-> Unique ID ->" + str(global_id) #Global ID    
                location = (int(cent_x),int(cent_y))
#                print(frame_id,local_id,location)
                print_txt_2 = "Frame :" + str(frame_id) + " | fps :- "+str(fps_vis)+" | Res :- "+str(int(height))+"p"
                cv2.putText(
                    frame, #numpy array on which text is written
                    print_txt, #text
                    location, #position at which writing has to start
                    cv2.FONT_HERSHEY_SIMPLEX, #font family
                    1, #font size
                    (0, 0, 255, 0), #font color
                    2) #font stroke
                
                
                for i in range(0,w):
                    for j in range(0,h):
                        if overlay[i,j][3] != 0:
                            x_loc = (int(cent_x)-51+i)
                            y_loc = (int(cent_y)-48+j)
                            if x_loc >= width:
                                x_loc = (width-1)
                            if y_loc >= height:
                                y_loc = (height-1)
#                            print(x_loc,y_loc)
                            frame[y_loc,x_loc] = overlay[i,j]
            else:
                print_txt_2 = "Frame :" + str(frame_id) + " | fps :- "+str(fps_vis)+" | Res :- "+str(int(height))+"p" + " | OCC ON"
                selected_row = global_tracking_data_occ[(global_tracking_data_occ[:,2] == frame_id) & (global_tracking_data_occ[:,3] == (global_id+1))]
#                print("Frame ID: "+str(frame_id))
#                print("Track ID: "+str(global_id))
#                print("Result Size: "+str(selected_row.size))
                if selected_row.size != 0:
                    location = []
                    cent_x = selected_row[0,0]
                    cent_y = selected_row[0,1]
                    print_txt = "-> Expected Unique ID ->" + str(global_id) #Global ID    
                    location = (int(cent_x),int(cent_y))
                    cv2.putText(
                        frame, #numpy array on which text is written
                        print_txt, #text
                        location, #position at which writing has to start
                        cv2.FONT_HERSHEY_SIMPLEX, #font family
                        1, #font size
                        (0, 255, 0, 0), #font color
                        2) #font stroke
        cv2.putText(
                    frame, #numpy array on which text is written
                    print_txt_2, #text
                    location_2, #position at which writing has to start
                    cv2.FONT_HERSHEY_SIMPLEX, #font family
                    1, #font size
                    (255, 0, 0, 0), #font color
                    2) #font stroke
        frame = cv2.cvtColor(frame,cv2.COLOR_BGRA2BGR)                    
        out.write(frame)
        frame_id_index = frame_id_index +1
        # cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break


cap.release()
out.release()
cv2.destroyAllWindows()
