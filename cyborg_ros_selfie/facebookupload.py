from facepy import GraphAPI
import settings

# Initialize the Graph API with a valid access token
def uploadPicture(picturePath):
	token = settings.ACCESS_TOKEN
	try:
		graph = GraphAPI(token)

		# Get my latest posts
		#print graph.get('me/posts')

		# Post a photo
		graph.post(
    		path = 'me/photos',
    		#source = picturePath
    		source = open(picturePath, 'rb')
    		#source = open('hslls.bmp', 'rb')
		)
	except:

		print("-------------------")
		print("")
		print("Change access token")
		print("")
		print("-------------------")

#uploadPicture("")