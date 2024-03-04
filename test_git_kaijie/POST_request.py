import requests

url = 'http://<ESP_IP_ADDRESS>/openDoor'

payload = {
        "action": "openDoor", "parameters": {
            "robotId": "Turtlebot3_ID"
            }
        }

headers = {'Content-Type': 'application/json'}

while True:
    response = requests.post(url, json=payload, headers=headers)

    if response.status_code == 200:
        response_data = response.json()
        print(response_data['data']['message'])

        break

    elif (response.status_code == 400):
        response_data = response.json()
        print("Error:" response_data['data']['message'])




