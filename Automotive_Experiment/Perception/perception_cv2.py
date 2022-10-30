# install opencv "pip install opencv-python"
import cv2
import time
# distance from camera to object(face) measured
# centimeter
Known_distance = 76.2

# width of face in the real world or Object Plane
# centimeter
Known_width = 14.3

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# face detector object
# face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml") # haarcascade_frontalface_default.xml
face_detector = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
# eye_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_eye.xml')

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length


# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame
    # return the distance
    return distance


def face_data(image):
    face_width = 0  # making face width to zero
    # converting color image to gray scale image
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # detecting face in the image
    faces = face_detector.detectMultiScale(gray_image, 1.3, 5)
    # looping through the faces detect in the image
    # getting coordinates x, y , width and height
    for (x, y, h, w) in faces:
        # draw the rectangle on the face
        cv2.rectangle(image, (x, y), (x + w, y + h), GREEN, 2)
        # getting face width in the pixels
        face_width = w

    # return the face width in pixel
    return face_width

def distance_measure(ref_image):

    tic = time.perf_counter()

    # find the face width(pixels) in the reference_image
    ref_image_face_width = face_data(ref_image)

    # get the focal by calling "Focal_Length_Finder"
    # face width in reference(pixels),
    # Known_distance(centimeters),
    # known_width(centimeters)
    Focal_length_found = Focal_Length_Finder(
        Known_distance, Known_width, ref_image_face_width)

    # print(Focal_length_found)

    face_width_in_frame = face_data(ref_image)
    Distance = Distance_finder(
        Focal_length_found, Known_width, face_width_in_frame)
    # print(Distance)

    toc = time.perf_counter()
    print("Time cost (ms): ", (toc-tic)*1000)
    # print(f"Downloaded the tutorial in {toc - tic:0.4f} seconds")
    return Distance

def camera_safety_check_human_face(ref_image):
    if distance_measure(ref_image) < 1e3:
        return True
    else:
        return False
if __name__ == '__main__':

    # reading reference_image from directory
    ref_image = cv2.imread('Ref_image.png')
    # cv2.imshow('img',ref_image)
    # cv2.waitKey(0)
    print(camera_safety_check_human_face(ref_image))