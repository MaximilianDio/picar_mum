import cv2
import leader_detection

if __name__ == "__main__":
    img = cv2.imread("Bild5.png")

    ld = leader_detection.LeaderGetter(None)
    ((x_blue, y_blue, x_green, y_green), output), mask = ld.process_image(img)

    print 'blue: x: ' , x_blue , " - y: ", y_blue
    print 'blue: x: ' , x_green , " - y: ", y_green

    # show the output image
    cv2.imshow("output", output)

    cv2.waitKey(5000)