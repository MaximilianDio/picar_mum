import cv2
import leader_detection


if __name__ == "__main__":
    img = cv2.imread("qube_test_noise.jpg")
    img1 = cv2.imread("qube_test_noise2.jpg")
    img2 = cv2.imread("qube_test_noise3.jpg")

    LD = leader_detection.LeaderGetter(None)
    LD.process_image(img)
    LD.process_image(img1)
    LD.process_image(img2)