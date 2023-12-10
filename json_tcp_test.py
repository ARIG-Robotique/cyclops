import socket
import json



def send_json_and_receive_response(json_data, host="127.0.0.1", port=50667):
	# Create a socket object
	with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
		# Connect to the server
		s.connect((host, port))

		# Convert the JSON data to a string
		json_string = json.dumps(json_data) + "\n"

		# Send the JSON data to the server
		s.sendall(json_string.encode("utf-8"))

		# Receive and print the server"s response
		response = s.recv(1024)
		print("Received:", response.decode("utf-8"))

# Example JSON data
stopquery = {
	"query": "seppuku",
	"index" : 0
}

getdataquery = {
	"query": ""
}

# Send the JSON data to the specified TCP port and receive a response
send_json_and_receive_response(example_json_data)
