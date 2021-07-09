import RPi.GPIO as GPIO
import face_recognition
import argparse
import pickle
import cv2
import sys
import imutils
import paho.mqtt.client as mqtt
import time
import threading
from threading import Thread
from imutils.video import VideoStream


mqtt_topic = "lockgab"
#mqtt_broker_ip = "broker.emqx.io"
mqtt_broker_ip = "192.168.1.89"
GPIO.setwarnings(False)
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883)
class posizione (Thread):
	def __init__(self, name):
		Thread.__init__(self)
		self.name = name
	def run(self):
		print ("Thread '" + self.name + "' avviato")
		vs = VideoStream(src=0).start()
		time.sleep(2.0)
		global rgb
		while True:
			frame = vs.read()
			rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
			rgb = imutils.resize(frame, width=200)
			if 1>0:
				cv2.imshow("Frame", rgb)
				key = cv2.waitKey(1) & 0xFF
		# if the `q` key was pressed, break from the loop
				if key == ord("q"):
					break
		print ("Thread '" + self.name + "' terminato")
		
def recognize (rgb):
	boxes = face_recognition.face_locations(rgb, model="hog")
	encodings = face_recognition.face_encodings(rgb, boxes)
	names = []
	for encoding in encodings:
		matches = face_recognition.compare_faces(data["encodings"],
			encoding)
		name = "Unknown"
		if True in matches:
			matchedIdxs = [i for (i, b) in enumerate(matches) if b]
			counts = {}
			for i in matchedIdxs:
				name = data["names"][i]
				counts[name] = counts.get(name, 0) + 1
			name = max(counts, key=counts.get)
		names.append(name)

		if name != "Unknown":
			localtime = time.asctime( time.localtime(time.time()) )
			client.publish(mqtt_topic,"approved")
			print ("approved "+ localtime)
			#pos = False
		else :
			print ("unknown")
			client.publish(mqtt_topic,"unknown")
			#pos = False

	'''
	for ((top, right, bottom, left), name) in zip(boxes, names):
		top = int(top * r)
		right = int(right * r)
		bottom = int(bottom * r)
		left = int(left * r)
		cv2.rectangle(rgb, (left, top), (right, bottom),(0, 255, 0), 2)
		y = top - 15 if top - 15 > 15 else top + 15
		cv2.putText(rgb, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
		'''


print("[INFO] loading encodings...")
data = pickle.loads(open("/home/salvo/Scrivania/definitivo/Face-recognition-door-opener-raspberry-pi/python_face_recognition_door_opener_raspberry_pi/encodings.pickle", "rb").read())

thread1 = posizione ("rilevamento")
thread1.daemon =True
thread1.start()
time.sleep(6)
print ("Waiting events...")
while True:
	print ("...")
	r = 1

	GPIO.setmode(GPIO.BCM)
	TRIG = 23
	ECHO = 24
	GPIO.setup(TRIG,GPIO.OUT)
	GPIO.setup(ECHO,GPIO.IN)
	GPIO.output(TRIG, False)
	#print ("Waiting For Sensor To Settle")
	time.sleep(0.3)
	GPIO.output(TRIG, True)
	time.sleep(0.00001)
	GPIO.output(TRIG, False)

	while GPIO.input(ECHO)==0:
	 pulse_start = time.time()

	while GPIO.input(ECHO)==1:
	 pulse_end = time.time()

	pulse_duration = pulse_end - pulse_start
	distance = pulse_duration * 17150
	distance = round(distance, 2)
	#print ("Distance:",distance,"cm")
	if distance >= 10.0 and distance <= 100.0:
	 
	 recognize(rgb)
	GPIO.cleanup()	

	#recognize()
	
			
cv2.destroyAllWindows()
vs.stop()
client.disconnect()

