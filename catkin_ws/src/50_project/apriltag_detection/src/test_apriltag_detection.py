import apriltag
import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def draw(img, corners, imgpts):
    corner = tuple(corners.ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-.50, .50])
ax.set_ylim([-.50, .50])
ax.set_zlim([-.50, .50])
fig.show()

# load camera matrix and distortion coefficients
output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_matrix.npy"  # Enter Directory of all images
output_path = os.path.join(output_dir)

mtx = np.load(output_path)
print mtx

output_dir = "/home/maximilian/picar_mum/catkin_ws/src/50_project/apriltag_detection/pose/camera_distortion.npy"  # Enter Directory of all images
output_path = os.path.join(output_dir)

dist = np.load(output_path)
print dist

# create apriltag detector object
detector = apriltag.Detector()
# start webcam capture
cap = cv2.VideoCapture(0)

# corners in 3d of tag2
objp = np.array([[-0.03825, -0.03825, 0],
                 [0.03825, -0.03825, 0],
                 [0.03825, 0.03825, 0],
                 [-0.03825, 0.03825, 0]], np.float32)

axis = np.float32([[0.03825, 0, 0], [0, 0.03825, 0], [0, 0, -0.03825]]).reshape(-1, 3)

print objp

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    result = detector.detect(gray)

    if result:
        for apriltag in result:
            homography = np.array(apriltag.homography, dtype='float32')
            id = apriltag.tag_id

            corners = np.array(apriltag.corners, np.int32)

            center = np.array(apriltag.center, np.int32)

            # Find the rotation and translation vectors.
            imagePoints = np.array(apriltag.corners, dtype='float32')
            imagePoints = np.expand_dims(imagePoints, axis=1)

            centerPoint = np.expand_dims(center, axis=1)

            _, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, imagePoints, mtx, dist)

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            frame = draw(frame, centerPoint, imgpts)

            # draw corners of apriltags
            corners = corners.reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], True, (0, 0, 255))

            # draw centers of apriltags with ID
            cv2.circle(frame, (center[0], center[1]), 5, (0, 255, 0), -1)

            ID = "ID#" + str(id)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, ID, (center[0] + 15, center[1]), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            ax.scatter([tvecs[0]], [tvecs[1]], [tvecs[2]])

            fig.canvas.draw()
            fig.canvas.flush_events()

    # Display the resulting frame
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
