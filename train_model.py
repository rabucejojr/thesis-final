
# import the necessary packages

#The paths sub-module of imutils includes a function to recursively find images based on a root directory.
from imutils import paths
import face_recognition
import pickle
import cv2
import os

# imutils, series of convenience functions to make basic image processing functions 
# such as translation, rotation, resizing, skeletonization, 
# and displaying Matplotlib images easier with OpenCV and both Python 2.7 and Python 3.


# our images are located in the dataset folder
print("[!!!] processing faces...")
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
	boxes = face_recognition.face_locations(rgb,model="hog")

	# compute the facial embedding for the face
	encodings = face_recognition.face_encodings(rgb, boxes)

	# loop over the encodings
	for encoding in encodings:
		# add each encoding + name to our set of known names and
		# encodings
		knownEncodings.append(encoding)
		knownNames.append(name)

# dump the facial encodings + names to disk
print("[!!!] serializing encodings...")
data = {"encodings": knownEncodings, "names": knownNames}
f = open("encodings.pickle", "wb")
f.write(pickle.dumps(data))
f.close()
