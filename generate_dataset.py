import cv2
import os 

#Enter your name
name = input('Enter your name: ')
#Open the camera, 0 is the default camera, 
#1 is the external camera, 2 is the usb camera
cam = cv2.VideoCapture(2) 


cv2.namedWindow("press space to take a photo", cv2.WINDOW_NORMAL)
cv2.resizeWindow("press space to take a photo", 500, 400)

#Initialize the counter
imgCounter = 0

#create a folder named after the named entered
os.mkdir('dataset/' + name)

while True:
    #Read the frame, ret is a boolean that checks availability of the frame
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break
    cv2.imshow("press space to take a photo", frame)

    #Check if the user pressed the spacebar
    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape key pressed, closing frame...")
        break
    elif k%256 == 32:
        #If the user press the space, the image is saved provided by the filename
        # SPACE pressed
        imgName = "dataset/"+ name +"/image_{}.jpg".format(imgCounter)
        cv2.imwrite(imgName, frame)
        print("{} written!".format(imgName))
        imgCounter += 1

cam.release()
cv2.destroyAllWindows()
