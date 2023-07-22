import face_recognition
import cv2
import os
import pyfirmata
import time
import StepperLib
from pyfirmata import ArduinoMega, util, STRING_DATA

board = ArduinoMega("/dev/cu.usbmodem11201")
reader = pyfirmata.util.Iterator(board) # reads inputs of the circuit
reader.start()

motor = StepperLib.Stepper(2038, board, reader, 11, 12, 4, 3)
motor.set_speed(1)
# Speed up program:
#   1. Process each video frame at 1/4 resolution (though still display it at full resolution)
#   2. Only detect faces in every other frame of video.

# Get a reference to webcam 
video_capture = cv2.VideoCapture(1)

# Loop through image folder to create face encodings for several people

# Create arrays of known face encodings and their names
# Change file paths to your image locations
# Use the file names for the display names (os.path.basename)

known_face_encodings= []
known_face_names = []
for person in os.listdir(os.path.join('Authorized_Users')):
    for image in os.listdir(os.path.join('Authorized_Users', os.path.basename(person))):    
        img = face_recognition.load_image_file(os.path.join('Authorized_Users',os.path.basename(person), os.path.basename(image)))
        known_face_encoding= face_recognition.face_encodings(img)[0]
        known_face_encodings.append(known_face_encoding)
        known_face_name= os.path.basename(person)
        known_face_names.append(known_face_name)

# Initialize some variables
face_locations = []
face_encodings = []
face_names = []
process_this_frame = True
start_time=time.perf_counter()

while True:
    # Grab a single frame of video
    ret, frame = video_capture.read()

    # Only process every other frame of video to save time
    if process_this_frame:
        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        
        # Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(small_frame)
        face_encodings = face_recognition.face_encodings(small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(known_face_encodings, face_encoding, 0.55)
            name = "Unknown"
            light=0

            # If a match was found in known_face_encodings, just use the first one.
            if True in matches:
                first_match_index = matches.index(True)
                name = known_face_names[first_match_index]
             
                light=1
                
            # Or instead, use the known face with the smallest distance to the new face
            # face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
            # best_match_index = np.argmin(face_distances)
            # if matches[best_match_index]:
            #     name = known_face_names[best_match_index]

            face_names.append(name)

    
    process_this_frame = not process_this_frame


    # Display the results
    for (top, right, bottom, left), name in zip(face_locations, face_names):
        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw a box around the face
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Draw a label with a name below the face
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
      
        print(time.perf_counter()-start_time)
        print('\n')

        if (light==1 and time.perf_counter()-start_time>=10):
            board.digital[23].write(0)
            board.digital[22].write(1)
            board.send_sysex( STRING_DATA, util.str_to_two_byte_iter('Welcome, '+ str(name)))
            motor.step(2000)    #unlock
            board.pass_time(5)  #set duration before autolock
            motor.step(-2000)   #autolock 
            start_time=time.perf_counter()
        else:
            board.digital[22].write(0)
            board.digital[23].write(1)
            board.send_sysex( STRING_DATA, util.str_to_two_byte_iter("unkown" ) )
         
    # Display the resulting image
    cv2.imshow('Video', frame)

    # Hit 'q' on the keyboard to quit!
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Uncoment to save the frame along with the person's name and time each time a face is recognized
# time_c = time.asctime()
# SAVE_PATH = os.path.join('found_faces', '{} {}.jpg'.format(name, time_c))
# cv2.imwrite(SAVE_PATH, frame)

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()