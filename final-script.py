# rasberry pi imports
import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import requests
import serial
import os
import os.path
import time
from tkinter import *

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


# The paths sub-module of imutils includes a function to recursively find images based on a root directory.
import cv2
from imutils import paths
import face_recognition
import pickle
import os
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import imutils
import time
from threading import Thread




# SIM900A
def sendMessage(phone):
    port.write(b'AT\r')
    rcv = port.read(10)
    time.sleep(1)
    port.write(b"AT+CMGF=1\r")
    print("Preparing to send")
    with canvas(device) as draw:
        draw.text((10, 10),  "Preparing to send",  font=font_mcsmall, fill=255)
    time.sleep(0.5)
    a = b'AT+CMGS="'
    b = b'"\r'
    print(a+phone+b)
    port.write(a+phone+b)
    msg = " Your son/daughter scanned his/her RFID. He/she is now inside the campus."
    print("Sending SMS")
    with canvas(device) as draw:
        draw.text((10, 10),  "Sending SMS",  font=font_mcsmall, fill=255)
    time.sleep(1)
    port.reset_output_buffer()
    time.sleep(1)
    port.write(str.encode(msg+chr(26)))
    time.sleep(2.5)
    print("SMS sent…")
    with canvas(device) as draw:
        draw.text((10, 10),  "SMS sent",  font=font_mcsmall, fill=255)
    time.sleep(1)

# datasets
def generate_dataset():
    name = input('Enter student name to register: ')
    with canvas(device) as draw:
        draw.text((10, 10),  "Enter student name to register",  font=font_mcsmall, fill=255)
    # Open the camera, 0 is the default camera,
    # 1 is the external camera, 2 is the usb camera
    cam = cv2.VideoCapture(0)  # -1 for raspberrypi with usb camera
    with canvas(device) as draw:
        draw.text((10, 10),  "SPACE to take photo",  font=font_mcsmall, fill=255)
    cv2.namedWindow("Press SPACE to take a photo", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Press SPACE to take a photo", 500, 400)

    # Initialize the counter
    imgCounter = 0

    # create a folder named after the named entered
    os.mkdir('dataset/' + name)

    while True:
        # Read the frame, ret is a boolean that checks availability of the frame
        ret, frame = cam.read()
        if not ret:
            print("Failed to grab frame")
            with canvas(device) as draw:
                draw.text((10, 10),  "Failed to grab frame",  font=font_mcsmall, fill=255)
            break
        cv2.imshow("Press SPACE to take a photo", frame)
        with canvas(device) as draw:
            draw.text((10, 10),  "SPACE to take photo",  font=font_mcsmall, fill=255)
        # Check if the user pressed the spacebar
        k = cv2.waitKey(1)
        if k % 256 == 27:
            # ESC pressed
            print("Escape key pressed, closing frame...")
            with canvas(device) as draw:
                draw.text((10, 10),  "ESC pressed; Closing frame..",  font=font_mcsmall, fill=255)
            break
        elif k % 256 == 32:
            # If the user press the space, the image is saved provided by the filename
            # SPACE pressed
            imgName = "dataset/" + name + "/image_{}.jpg".format(imgCounter)
            cv2.imwrite(imgName, frame)
            print("{} written!".format(imgName))
            imgCounter += 1
    cam.release()
    cv2.destroyAllWindows()

# training model
def train_model():
    # our images are located in the dataset folder
    print("[!!!] Encoding faces...")
    with canvas(device) as draw:
        draw.text((10, 10),  "[!!!] Encoding faces...",  font=font_mcsmall, fill=255)
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
    with canvas(device) as draw:
        draw.text((10, 10),  "[!!!] Serializing encodings...",  font=font_mcsmall, fill=255)
    data = {"encodings": knownEncodings, "names": knownNames}
    f = open("encodings.pickle", "wb")
    f.write(pickle.dumps(data))
    f.close()

# face recognition
def facial_rec():
    #array of recognized names
    # Initialize 'defaultName' to trigger only when a new person is identified.
    defaultName = "unknown"
    # Determine faces from encodings.pickle file model created from train_model.py
    encodingsP = "encodings.pickle"
    names = []
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
        global faceName
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
            
             # if face is not recognized, then print Unknown
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
                    faceName = defaultName
                    #oled_disp("Face Recognized: "+defaultName, 0, 0)
            # update the list of names
                names.append(name)
            else:
                defaultName = "Unknown"
        # loop over the recognized faces
        for ((top, right, bottom, left), name) in zip(boxes, names):
            # draw the predicted face name on the image - color is in BGR
            cv2.rectangle(frame, (left, top), (right, bottom),(0, 255, 225), 2)
            y = top - 15 if top - 15 > 15 else top + 15
            cv2.putText(frame, defaultName, (left, y), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 255), 2)
            faceName = defaultName

        # display the image to our screen
        cv2.imshow("Facial Recognition is Running", frame)
        key = cv2.waitKey(1) & 0xFF
        
        # quit when 'q' key is pressed
        if key == ord("q"):
            break
    # do a bit of cleanup
    cv2.destroyAllWindows()
    vs.stop()


#RFID
def rfid_scan():
    with canvas(device) as draw:
        draw.text((10, 30),  "Scan your RFID tag",  font=font_mcsmall, fill=255)
    try:
        id = reader.read()[0]
        tagName = reader.read()[1]
        print("Tag: "+tagName)
        print("RFID: "+str(id))
        print("Face: "+faceName)
        with canvas(device) as draw:
            draw.text((10, 10),  "Found RFID tag.",font=font_mcsmall, fill=255)
            draw.text((10, 30),  str(id),font=font_mcsmall, fill=255)
            draw.text((10, 50),  "Please wait.",  font=font_mcsmall, fill=255)
        time.sleep(1)
        if faceName.strip() == tagName.strip():
            x = requests.get('http://snnhs-attendance-system.herokuapp.com/api/user/' + str(id))
            if x.status_code == 404:
                print("tag not found")
                with canvas(device) as draw:
                    draw.text((10, 10),  "RFID not recognized",  font=font_mcsmall, fill=255)
                    draw.text((10, 30),  "Register it to the admin",  font=font_mcsmall, fill=255)
                time.sleep(1.5)
            else:
                phone = (x.json()['phone'])
                name = (x.json()['name'])
                with canvas(device) as draw:
                    draw.text((10, 10),  "Welcome "+str(name),  font=font_mcsmall, fill=255)
                    draw.text((10, 30),  "Please wait",  font=font_mcsmall, fill=255)
                phone = '+63'+phone
                phone = bytes(phone, 'utf-8')
                y = requests.post(
                    'http://snnhs-attendance-system.herokuapp.com/api/attendance/', data={"rfid": id})
                with canvas(device) as draw:
                    draw.text((10, 30),  "Attendance recorded!",  font=font_mcsmall, fill=255)
                time.sleep(1)
                sendMessage(phone)
        else:
            with canvas(device) as draw:
                draw.text((10, 10),  "Face and RFID does  ",  font=font_mcsmall, fill=255)
                draw.text((10, 30),  "not match",  font=font_mcsmall, fill=255)
            time.sleep(1)
        rfid_scan()
    finally:
        #GPIO.setwarnings(False)
        GPIO.cleanup()



# Main code here...
# Initial Setup/Declerations
# GPIO.setmode(GPIO.BOARD)
port = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
reader = SimpleMFRC522()
font = ImageFont.load_default()
font_mc = ImageFont.truetype('/home/admin/Desktop/Final/thesis-final/Arialn.ttf', 32)
font_mcsmall = ImageFont.truetype(
        '/home/admin/Desktop/Final/thesis-final/Arialn.ttf', 12)
serial = i2c(port=1, address=0x3C)
device = sh1106(serial)
width = device.width
height = device.height

tagName = ""


while True:
    with canvas(device) as draw:
        draw.text((10, 10),  "Do you want to register a ",  font=font_mcsmall, fill=255)
        draw.text((10, 30),  "student? Press y / n",  font=font_mcsmall, fill=255)
    option = input("Do you want to register a student? Press y / n: ")
    if option == "y" or option == "Y":
        student_name = input("Enter student name to check if they exist: ")
        with canvas(device) as draw:
            draw.text((10, 10),  "Enter student name to ",  font=font_mcsmall, fill=255)
            draw.text((10, 30),  "check if they exist.",  font=font_mcsmall, fill=255)
        file = os.path.exists('/home/admin/Desktop/Final/thesis-final/dataset/'+student_name)
        if file == False: #name does not exist
            print("Opening camera")
            with canvas(device) as draw:
                draw.text((10, 10),  "Opening camera",  font=font_mcsmall, fill=255)
            generate_dataset()
            train_model()
        else:
            print("Name already exist")
            with canvas(device) as draw:
                draw.text((10, 10),  "Name already exist",  font=font_mcsmall, fill=255)
                
        # Run RFID Write
        print("Please put RFID")
        with canvas(device) as draw:
            draw.text((10, 10),  "Please put RFID ",  font=font_mcsmall, fill=255)
        student_name2 = input("Enter student name for RFID: ")
        reader.write(student_name2.strip())
        print("Registered! Please take note of the tag below and register it to the website.")
        print("Tag: "+str(reader.read()[0]))
        print("RFID : "+str(reader.read()[1]))
        with canvas(device) as draw:
            draw.text((10, 10),  "RFID registered",  font=font_mcsmall, fill=255)
        time.sleep(1)
        
    elif option == "n" or option == "N":
        Thread(target=facial_rec).start()
        Thread(target=rfid_scan).start()
        
        
