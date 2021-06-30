#for use Arduino
from pyfirmata import Arduino, util ,STRING_DATA
from time import sleep

#for face detection
import face_recognition
import cv2
import numpy as np

#for import image 
import glob
import os
import atexit

arduino = "COM8"
board = Arduino(arduino)

files = glob.glob('photos\\*.jpg')

#for open camera
video_capture = cv2.VideoCapture(0)

#var for servo metor 
START_ANGLE = 100
ANGLE = 0
DELAY = 3

iter8 = util.Iterator(board)
iter8.start()

pin9 = board.get_pin('d:9:s')

#import faces from photos file
known_face_encodings = [face_recognition.load_image_file(file) for file in files]
known_face_encodings = [face_recognition.face_encodings(face)[0] for face in known_face_encodings]

known_face_names = [os.path.splitext(p)[0].split('\\')[1] for p in glob.glob('photos\\*.jpg')]


# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

#for control degree motor 
def rotate(current_angle,desire_angle,pin):
  print(current_angle , desire_angle)
  if current_angle < desire_angle:
    for angle in range(current_angle,desire_angle):
      pin.write(angle)
      sleep(.01)
  else:
    count = 100
    for angle in range(desire_angle,current_angle):
      pin.write(count)
      count -= 1
      sleep(.01)
      
def msg( text ):

  if text:
        board.send_sysex( STRING_DATA, util.str_to_two_byte_iter( text ) )

def on_exit():
  print('q'*1000)
  #clear the LCD
  msg(' ')
  msg(' ')
  # Release handle to the webcam
  video_capture.release()

#but the door in start place
rotate(ANGLE,START_ANGLE,pin9)

msg('Detecting ...')
msg(' ')
#on_exit
atexit.register(on_exit)
#Loop        
while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Resize frame of video to 1/4 size for faster face recognition processing
    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

    # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
    rgb_small_frame = small_frame[:, :, ::-1]

    # Only process every other frame of video to save time
    if process_this_frame:
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding,tolerance=0.4)
            name = "Unknown"

            # # If a match was found in known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = known_face_names[first_match_index]

            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = known_face_names[best_match_index]
                #pin9.write(ANGLE)
                rotate(START_ANGLE,ANGLE,pin9)
                msg(f'Hellow {name}')
                msg(' ')
                sleep(DELAY)
                #pin9.write(START_ANGLE)
                rotate(ANGLE,START_ANGLE,pin9)
                msg('Detecting ...')
                msg(' ')
                
            face_names.append(name)

    process_this_frame = not process_this_frame

