import requests

url = 'http://<ESP_IP_ADDRESS>/openDoor'

payload = {
        "action": "openDoor", "parameters": {
            "robotId": "Turtlebot3_ID"
            }
        }

headers = {'Content-Type': 'application/json'}

response = requests.post(url, json=payload, headers=headers)

print(response.text)

