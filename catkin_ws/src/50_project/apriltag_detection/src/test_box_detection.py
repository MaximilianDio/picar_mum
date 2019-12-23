from Box_detection import BoxDetector
from Box import Box
from Tag import Tag
import numpy as np
import cv2
import os
import time

if __name__ == "__main__":

    # load camera matrix and distortion coefficients
    output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_matrix.npy"  # Enter Directory of all images
    output_path = os.path.join(output_dir)

    mtx = np.load(output_path)

    output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_distortion.npy"  # Enter Directory of all images
    output_path = os.path.join(output_dir)

    dist = np.load(output_path)

    boxes = [
        Box(0, [Tag(0, 0.0765, np.array([0.0, 0.068, 0.05]),
                    np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])),
                Tag(2, 0.0765, np.array([0.11, 0.14, 0.065]),
                    np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])),
                Tag(3, 0.0765, np.array([0.11, 0.073, 0.12]),
                    np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))],
            (.2, .14, .12))]
    box_detector = BoxDetector(boxes)

    # start webcam capture
    cap = cv2.VideoCapture(0)

    while (True):
        start = time.time()
        # Capture frame-by-frame
        ret, frame = cap.read()

        scale_percent = 100  # percent of original size
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        dim = (width, height)
        # resize image
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
        # FIXME: only works for full resulution, because camera matrix will change when resized!
        # TODO: implement dynamic resolution change (based on how well apriltags were detected

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        box_detector.detect_boxes(gray, mtx, dist)

        end = time.time()
        print(end - start)

        # visualization not part of performance!

        box_detector.draw_boxes(frame, mtx, dist)

        # Display the resulting frame
        cv2.imshow('frame', frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
