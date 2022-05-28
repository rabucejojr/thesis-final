# rasberry pi imports
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import requests
import serial
import os
import time
from tkinter import *
import threading as tr

# OLED display Imports
import time
import board
import busio
from pathlib import Path
from luma.core.interface.serial import i2c, spi, pcf8574
from luma.core.interface.parallel import bitbang_6800
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1309, ssd1325, ssd1331, sh1106, ws0010
from PIL import ImageFont

import cv2
# The paths sub-module of imutils includes a function to recursively find images based on a root directory.
from imutils import paths
import face_recognition
import pickle
import os
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import time

# Initial Setup/Declerations
# GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
reader = SimpleMFRC522()

#array of recognized names
names = []

# OLED DISPLAY, Initialization/Setup
def oled_disp(text, x, y):
    font = ImageFont.load_default()
    font_mc = ImageFont.truetype('/home/admin/Desktop/scripts/Arialn.ttf', 32)
    font_mcsmall = ImageFont.truetype(
        '/home/admin/Desktop/scripts/Arialn.ttf', 14)
    serial = i2c(port=1, address=0x3C)
    device = sh1106(serial)
    x = 0
    y = 0
    width = device.width
    height = device.height
    with canvas(device) as draw:
        draw.text((x, y),  text,  font=font_mcsmall, fill=255)

# SIM900A
def sendMessage(phone):
    port.write(b'AT\r')
    rcv = port.read(10)
    time.sleep(1)
    port.write(b"AT+CMGF=1\r")
    print("Text Mode Enabled…")
    oled_disp("Text Mode Enabled..", 0, 0)
    time.sleep(3)
    a = b'AT+CMGS="'
    b = b'"\r'
    print(a+phone+b)
    port.write(a+phone+b)
    msg = " xxx Your son/daughter scanned his/her RFID. He/she is now inside the campus. xxx"
    print("sending message….")
    oled_disp("Sending SMS...", 0, 20)
    time.sleep(3)
    port.reset_output_buffer()
    time.sleep(1)
    port.write(str.encode(msg+chr(26)))
    time.sleep(3)
    print("message sent…")
    oled_disp("SMS sent...", 0, 40)
    time.sleep(0.5)

# datasets
def generate_dataset():
    name = input('Enter your name: ')
    # Open the camera, 0 is the default camera,
    # 1 is the external camera, 2 is the usb camera
    cam = cv2.VideoCapture(0)  # -1 for raspberrypi with usb camera
    cv2.namedWindow("press space to take a photo", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("press space to take a photo", 500, 400)

    # Initialize the counter
    imgCounter = 0

    # create a folder named after the named entered
    os.mkdir('dataset/' + name)

    while True:
        # Read the frame, ret is a boolean that checks availability of the frame
        ret, frame = cam.read()
        if not ret:
            print("failed to grab frame")
            oled_disp("Failed to grab frame...", 0, 0)
            break
        cv2.imshow("press space to take a photo", frame)
        oled_disp("Press SPACE to capture..", 0, 0)

        # Check if the user pressed the spacebar
        k = cv2.waitKey(1)
        if k % 256 == 27:
            # ESC pressed
            print("Escape key pressed, closing frame...")
            oled_disp("ESC pressed; Closing frame..", 0, 20)
            break
        elif k % 256 == 32:
            # If the user press the space, the image is saved provided by the filename
            # SPACE pressed
            imgName = "dataset/" + name + "/image_{}.jpg".format(imgCounter)
            cv2.imwrite(imgName, frame)
            print("{} written!".format(imgName))
            oled_disp("{} written!".format(imgName), 0, 20)
            imgCounter += 1
    cam.release()
    cv2.destroyAllWindows()

# training model
def train_model():
    # our images are located in the dataset folder
    print("[!!!] processing faces...")
    oled_disp("[!!!]Processing faces...", 0, 0)
    imagePaths = list(paths.list_images("dataset"))

    # initialize the list of known encodings and known names
    knownEncodings = []
    knownNames = []

    # loop over image paths
    for (i, imagePath) in enumerate(imagePaths):
        # extract the person name from the image path
        print("[!!!] processing image {}/{}".format(i + 1,
                                                    len(imagePaths)))
        name = imagePath.split(os.path.sep)[-2]
        # print(imagePath)  dataset/<name>/<filename>.jpg
        # print (name) #name mentioned above

        # load the input image and convert it from RGB (OpenCV ordering)
        # to dlib ordering (RGB)
        image = cv2.imread(imagePath)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # detect the (x, y)-coordinates of the bounding boxes
        # corresponding to each face in the input image
        boxes = face_recognition.face_locations(rgb, model="hog")

        # compute the facial embedding for the face
        encodings = face_recognition.face_encodings(rgb, boxes)

        # loop over the encodings
        for encoding in encodings:
            # add each encoding + name to our set of known names and
            # encodings
            knownEncodings.append(encoding)
            knownNames.append(name)

    # dump the facial encodings + names to disk
    print("[!!!] Serializing encodings...")
    oled_disp("[!!!] Serializing encodings...", 0, 20)
    data = {"encodings": knownEncodings, "names": knownNames}
    f = open("encodings.pickle", "wb")
    f.write(pickle.dumps(data))
    f.close()

# face recognition
def facial_rec():
    # Initialize 'defaultName' to trigger only when a new person is identified.
    defaultName = "unknown"
    # Determine faces from encodings.pickle file model created from train_model.py
    encodingsP = "encodings.pickle"

    # load the known faces and embeddings along with OpenCV's Haarcascade for face detection
    print("[INFO] loading encodings + face detector...")
    data = pickle.loads(open(encodingsP, "rb").read())

    # initialize the video stream and allow the camera sensor to warm up
    # Set the ser to the followng
    # src = 0 : for the build in single web cam, could be your laptop webcam
    # src = 2 : for the USB webcam attached to the Raspberry Pi

    vs = VideoStream(src=0, framerate=10).start()
    time.sleep(2.0)

    # start the FPS counter
    fps = FPS().start()

    # loop over frames from the video file stream
    while True:
        # grab the frame from the threaded video stream and resize it
        # to 500px (to speedup processing)
        frame = vs.read()
        frame = imutils.resize(frame, width=500)
        # Detect the fce boxes
        boxes = face_recognition.face_locations(frame)
        # compute the facial embeddings for each face bounding box
        encodings = face_recognition.face_encodings(frame, boxes)
        # loop over the facial embeddings
        for encoding in encodings:
            # attempt to match each face in the input image to our known
            # encodings
            matches = face_recognition.compare_faces(data["encodings"],
                                                     encoding)
            name = "Unknown"  # if face is not recognized, then print Unknown
            oled_disp("Unknown face", 0, 0)
            # check to see if we have found a match
            if True in matches:
                # find the indexes of all matched faces then initialize a
                # dictionary to count the total number of times each face
                # was matched
                matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                counts = {}

                # loop over the matched indexes and maintain a count for
                # each recognized face face
                for i in matchedIdxs:
                    name = data["names"][i]
                    counts[name] = counts.get(name, 0) + 1

                # determine the recognized face with the largest number
                # of votes (note: in the event of an unlikely tie Python
                # will select first entry in the dictionary)
                name = max(counts, key=counts.get)

                # If someone in your dataset is identified, this then prints name on the screen
                if defaultName != name:
                    defaultName = name
                    print(defaultName)
                    oled_disp("Face Recognized: "+defaultName, 0, 0)
                    time.sleep(2)
                    #if face is recognized, then the student can now scan his/her tag
                    rfid_scan()
                    time.sleep(2)
            # update the list of names
            names.append(name)

        # loop over the recognized faces
        for ((top, right, bottom, left), name) in zip(boxes, names):
            # draw the predicted face name on the image - color is in BGR
            cv2.rectangle(frame, (left, top), (right, bottom),
                          (0, 255, 225), 2)
            y = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                        .8, (0, 255, 255), 2)

        # display the image to our screen
        cv2.imshow("Facial Recognition is Running", frame)
        key = cv2.waitKey(1) & 0xFF

        # quit when 'q' key is pressed
        if key == ord("q"):
            break

        # update the FPS counter
        fps.update()

    # stop the timer and display FPS information
    fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()

#RFID
def rfid_scan():
    print("Scan your tag...")
    oled_disp("Scan your tag...",0,0)
    try:
        id = reader.read()[0]
        print(id)
        oled_disp("Tag:"+str(id), 0, 0)
        x = requests.get(
            'http://snnhs-attendance-system.herokuapp.com/api/user/' + str(id))
        if x.status_code == 404:
            print("tag not found")
            oled_disp("tag not found",0,0)
            oled_disp("please register...",0,20)
            time.sleep(0.5)
        else:
            phone = (x.json()['phone'])
            phone = '+63'+phone
            phone = bytes(phone, 'utf-8')
            y = requests.post(
                'http://snnhs-attendance-system.herokuapp.com/api/attendance/', data={"rfid": id})
            sendMessage(phone)
    finally:
        #GPIO.setwarnings(False)
        GPIO.cleanup()


# Main code here...
facial_rec()