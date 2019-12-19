import apriltag
import cv2
import numpy as np

# create apriltag detector object
detector = apriltag.Detector()
# start webcam capture
cap = cv2.VideoCapture(0)

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    result = detector.detect(gray)

    if result:
        for apriltag in result:

            # draw corners of apriltags
            corners = np.array(apriltag.corners, np.int32)
            corners = corners.reshape((-1, 1, 2))
            cv2.polylines(frame, [corners], True, (0, 0, 255))

            # draw centers of apriltags with ID
            center = np.array(apriltag.center, np.int32)
            cv2.circle(frame, (center[0], center[1]), 5, (0, 255, 0), -1)

            ID = "ID#" + str(apriltag.tag_id)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(frame, ID, (center[0] + 15, center[1]) , font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
