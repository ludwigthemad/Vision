#from imutils.video import VideoStream
from imutils import face_utils
import imutils
import dlib
import cv2
import numpy as np


#def StartDetection():
print("[INFO] loading facial landmark predictor...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("models/dlib/shape_predictor_68_face_landmarks.dat")
fourcc = cv2.VideoWriter_fourcc(*'XVID')
#outVideo = cv2.VideoWriter('outputMouth.mp4',fourcc, 20.0, (400,225))
vs = cv2.VideoCapture(0)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
outVideo = cv2.VideoWriter('outputMouth.avi', fourcc, 20.0, (400, 255))


distances = list()
distances.append([])
distances.append([])



while True:
	# grab the frame from the threaded video stream, resize it to
	# have a maximum width of 400 pixels, and convert it to
	# grayscale
	ok,frame = vs.read()
	if not ok:
		break;
	frame = imutils.resize(frame, width=400)
#	height,width,channel = frame.shape
	#print (height)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	# detect faces in the grayscale frame
	rects = detector(gray, 0)
		# loop over the face detections

	face_id = 0;

	for rect in rects:
		face_id += 1

		# determine the facial landmarks for the face region, then
		# convert the facial landmark (x, y)-coordinates to a NumPy
		# array
		shape = predictor(gray, rect)
		shape = face_utils.shape_to_np(shape)

		#calculate mouth width and lip distances
		mouth = []
	#	mouth.append(face_id)
		width = (shape[54][0] - shape[48][0])  # width outer
		mouth.append(width) #width outer
	#	mouth.append((shape[64][0] - shape[60][0])) #width inner

	#	mouth.append((shape[67][1] - shape[61][1])) #distance inner
	#	mouth.append((shape[66][1] - shape[62][1]))  # distance inner
	#	mouth.append((shape[65][1] - shape[63][1]))  # distance inner

	#	mouth.append((shape[59][1] - shape[49][1]))  # distance outer
	#	mouth.append((shape[58][1] - shape[50][1]))  # distance outer
	#	mouth.append((shape[57][1] - shape[51][1]))  # distance outer
	#	mouth.append((shape[56][1] - shape[52][1]))  # distance outer
	#	mouth.append((shape[55][1] - shape[53][1]))  # distance outer

	#	mouth.append((shape[61][1] - shape[50][1]))  # thickness upper
	#	mouth.append((shape[62][1] - shape[51][1]))  # thickness upper
	#	mouth.append((shape[63][1] - shape[52][1]))  # thickness upper

	#	mouth.append((shape[67][1] - shape[58][1]))  # thickness lower
	#	mouth.append((shape[66][1] - shape[57][1]))  # thickness lower
	#	mouth.append((shape[65][1] - shape[56][1]))  # thickness lower

		#print(mouth)
	#	outer = ((shape[59][1] - shape[49][1]) + (shape[55][1] - shape[53][1])) / 2
		inner = shape[66][1] - shape[62][1]
	#	mouth.append(outer)
		mouth.append(inner)

	#	print(mouth)

	#	distances.append(face_id)
		distances[face_id - 1].insert(0, mouth)
		if(len(distances[face_id-1]) > 5):
			distances[face_id-1].pop(5)

		print(distances)

		speaking = False
		if(inner >= (width /  9.5)):
			speaking = True
		else:
			for i in (0, len(distances)-1):
				distances_width = distances[face_id-1][i-1][0]
				distances_inner = distances[face_id-1][i-1][1]
				if(distances_inner >= (distances_width / 9)):
					speaking = True
					break


		#	print(distances)


		#printing lines for mouth width and lip distances
	#	(x1, y1) = shape[67][0], shape[67][1]
	#	(x2, y2) = shape[61][0], shape[61][1]
	#	cv2.line(frame, (x1, y1), (x2, y2), (0, 250, 154), 1) #distance inner

	#	(x1, y1) = shape[66][0], shape[66][1]
	#	(x2, y2) = shape[62][0], shape[62][1]
	#	cv2.line(frame, (x1, y1), (x2, y2), (0, 250, 154), 1)  # distance inner

	#	(x1, y1) = shape[65][0], shape[65][1]
	#	(x2, y2) = shape[63][0], shape[63][1]
	#	cv2.line(frame, (x1, y1), (x2, y2), (0, 250, 154), 1)  # distance inner

	#	(x1, y1) = shape[59][0], shape[59][1]
	#	(x2, y2) = shape[49][0], shape[49][1]
	#	cv2.line(frame, (x1, y1), (x2, y2), (0, 191, 255), 1)  # distance outer

	#	(x1, y1) = shape[55][0], shape[55][1]
	#	(x2, y2) = shape[53][0], shape[53][1]
	#	cv2.line(frame, (x1, y1), (x2, y2), (0, 191, 255), 1)  # distance outer

		#(x1, y1) =  #top left corner of mouth region
		#(x2, y2) =  #bottom right corner of mouth region

		if (speaking):
			cv2.rectangle(frame, (shape[48][0], shape[50][1]), (shape[54][0], shape[57][1]), (0, 255, 127), 3)  # mouth width
		else: cv2.rectangle(frame, (shape[48][0], shape[50][1]), (shape[54][0], shape[57][1]), (255, 0, 0), 2)  # mouth width


		# loop over the (x, y)-coordinates for the facial landmarks
		# and draw them on the image (except mouth)

		count = 0
		for (x, y) in shape:
			count += 1
			if(count < 48):
				cv2.circle(frame, (x, y), 1, (0, 0, 255), -1)


		#record frame
	outVideo.write(frame)

	# show the frame
	cv2.imshow("Frame", frame)

#	outVideo.write(frame)
	key = cv2.waitKey(1) & 0xFF

	outVideo.release()

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):

		break

