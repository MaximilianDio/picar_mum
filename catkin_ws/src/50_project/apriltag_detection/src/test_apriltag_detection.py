import apriltag
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from apriltag_detection import AprilTagDetector

# load camera matrix and distortion coefficients
output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_matrix.npy"  # Enter Directory of all images
output_path = os.path.join(output_dir)

mtx = np.load(output_path)
print mtx

output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_distortion.npy"  # Enter Directory of all images
output_path = os.path.join(output_dir)

dist = np.load(output_path)
print dist

# start webcam capture
cap = cv2.VideoCapture(0)

tag_ID = 3
tag_size = 0.075
tag_tvec = np.array([0.0, 0.0, 0.0])  # translation vector from reference coordinate system
tag_rmat = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
                     [0.0, 0.0, 1.0]])  # basic rotation matrix from reference coordination system
# external
tvec_cam = np.array([[0.0],[0.0],[0.0]])
rmat_cam = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0],
                     [0.0, 1.0, 0.0]])


tag_detector1 = AprilTagDetector(tag_ID,tag_size,tag_tvec,tag_rmat,mtx,dist,tvec_cam,rmat_cam)

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print tag_detector1.get_tag_pos(gray)
    tag_detector1.draw_tag(frame)


    # Display the resulting frame
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
