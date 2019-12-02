import cv2
import leader_detection

if __name__ == "__main__":
    img = cv2.imread("Bild3.png")

    ld = leader_detection.LeaderGetter(None)
    x_blue, y_blue, x_green, y_green = ld.process_image(img)

    print 'blue: x: ' , x_blue , " - y: ", y_blue
    print 'blue: x: ' , x_green , " - y: ", y_green

    cv2.waitKey(5000)