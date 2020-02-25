import speech_recognition
import os
from os import path
import time
import matplotlib.pyplot as plt
import numpy as np

def get_input_from_microphone():
	p = os.system('sox -r 16000 -t alsa default sound_1.wav silence 1 0.1 1% 1 1.5 1%')

def recognize_with_google(filename):

	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)

	try:

		start = time.time()
		print"Google: \n" + recognizer.recognize_google(WAV_interpretor)
		end = time.time()
		diff = (end-start)
		print "Google used: %s"%(str(diff))
		print "\n"
		return diff
	except:
		pass

	

def recognize_with_wit_ai(filename,key):
	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)

	try:
		start = time.time()
		print "Vit.ai: \n"+recognizer.recognize_wit(WAV_interpretor,key)
		end = time.time()
		diff = (end-start)
		print "VIT: %s"%(str(diff))
		print "\n"
		return diff
	except:
		pass

	

def recognize_with_IBM(user,pwd,filename):
	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)
	try:
		start = time.time()
		print "IBM: \n"+recognizer.recognize_ibm(WAV_interpretor, user, pwd, language = "en-US", show_all = False)
		end = time.time()
		diff = (end-start)
		print "IBM: %s"%(str(diff))
		print "\n"
		return diff
	except:
		pass

	

def recognize_with_att(app_key,app_secret,filename):
	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)

	try:
		start = time.time()
		print "ATT: \n"+recognizer.recognize_att(WAV_interpretor, app_key, app_secret, language = "en-US", show_all = False)
		end = time.time()
		diff = (end-start)
		print "ATT: %s"%(str(diff))
		print "\n"
		return diff
	except:
		pass

def recognize_with_sphinx(filename):
	recognizer = speech_recognition.Recognizer()
	
	path_to_wav_source= os.getcwd()
	path_to_wav_source+="/%s"%(filename)

	with speech_recognition.WavFile(path.join(path.dirname(path.realpath(path_to_wav_source)),filename)) as speech_to_interpret:
		WAV_interpretor = recognizer.record(speech_to_interpret)
	try:
		start = time.time()
		print "CMU Sphinx: \n"+recognizer.recognize_sphinx(WAV_interpretor,language = "en-US", show_all = False)
		end = time.time()
		diff = (end-start)
		print "CMU Sphinx: %s"%(str(diff))
		print "\n"
		return diff
	except:
		pass 


def run_speech_comparizor(filename,wit_credentials_dict,ibm_credentials_dict):
	computation_time_google = []
	computation_time_vit = []
	computation_time_ibm = []
	computation_time_sphinx = []

	for i in range(0,100):
		computation_time_google.append(recognize_with_google("sound_1.wav"))
		computation_time_vit.append(recognize_with_wit_ai("sound_1.wav",wit_credentials_dict["user"]))
		computation_time_ibm.append(recognize_with_IBM(ibm_credentials_dict["user"],ibm_credentials_dict["pwd"],"sound_1.wav"))
		computation_time_sphinx.append(recognize_with_sphinx("sound_1.wav"))
		print "counter: "+str(i)

	comparizon_data = {}
	comparizon_data["google"]=computation_time_google
	comparizon_data["vit"]=computation_time_vit
	comparizon_data["ibm"]=computation_time_ibm
	comparizon_data["sphinx"]=computation_time_sphinx

	return comparizon_data

def plot_data(google_data,vit_data,ibm_data,sphinx_data):
	
	plt.plot(google_data,label="google")
	plt.ylabel("Computation time [secs]")
	plt.xlabel("Step number")

	plt.hold(True)

	plt.plot(vit_data,label="vit")
	plt.ylabel("Computation time [secs]")
	plt.xlabel("Step number")

	plt.hold(True)

	plt.plot(ibm_data,label="ibm")
	plt.ylabel("Computation time [secs]")
	plt.xlabel("Step number")
	
	plt.hold(True)

	plt.plot(sphinx_data,label="sphinx")
	plt.ylabel("Computation time [secs]")
	plt.xlabel("Step number")


	plt.title("Comparison of speech to text algorithms")
	legend = plt.legend(loc='upper right', shadow=True)

	plt.show()

def return_average_computation_time(google_data,vit_data,ibm_data,sphinx_data):
	average_google = np.mean(google_data)
	average_vit = np.mean(vit_data)
	average_ibm = np.mean(ibm_data)
	average_sphinx = np.mean(sphinx_data)
	
	average_dict={}
	average_dict["google"]=average_google
	average_dict["vit"]=average_vit
	average_dict["ibm"]=average_ibm
	average_dict["sphinx"]=average_sphinx

	return average_dict

def plot_average_computation_time(average_dict):
	x = ["Google","Vit.ai", "IBM","CMU Sphinx"]
	y = [average_dict["google"],average_dict["vit"],average_dict["ibm"],average_dict["sphinx"]]	
	x_set_bar_titles = np.arange(len(x))

	plt.bar(x_set_bar_titles,y,align="center",color="y")
	plt.xticks(x_set_bar_titles,x)
	plt.xlabel("Speech recognition platform")
	plt.ylabel("Average computation time [secs]")
	plt.title("Average speech-to-text computation time")

	axes = plt.gca()
	axes.set_ylim([0,max(average_dict["google"],average_dict["vit"],average_dict["ibm"],average_dict["sphinx"])+2])

	plt.show()





if __name__ == "__main__":
	
	try:

		#get_input_from_microphone()
		filename = "sound_1.wav"
		
		wit_credentials_dict = {}
		wit_credentials_dict["user"]="5E7RXG6EAQPJW4E7TSZTEGGEWH6WAJCP"

		ibm_credentials_dict = {}
		ibm_credentials_dict["user"]="41ce972f-592d-443c-bc81-d9c36d2cc435"
		ibm_credentials_dict["pwd"]="Kw8o21MizPjQ"

		data = run_speech_comparizor(filename,wit_credentials_dict,ibm_credentials_dict)

		google_list = data["google"]
		vit_list = data["vit"]
		ibm_list = data["ibm"]
		sphinx_list = data["sphinx"]

		plot_data(google_list,vit_list,ibm_list,sphinx_list)

		average_dict = return_average_computation_time(google_list,vit_list,ibm_list,sphinx_list)

		plot_average_computation_time(average_dict)

		#These lines could be useful if ATT is upgraded to premium account. this is not free.
		#"app_key_att = 
		#"app_secret_att = 
		#"recognize_with_att(app_key_att,app_secret_att,"sound_1.wav")

	except:
		print "some error"