import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from home_assistant.srv import TextToSpeech
import requests
import pyttsx3

# Update with your Home Assistant details
TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiIyZTJkNzJmNmRkMTc0N2UyYTcxOGI4ZmRkNjYxMzViNiIsImlhdCI6MTcyNzUwODYwMywiZXhwIjoyMDQyODY4NjAzfQ.3sc6hJ2q_kWEEsdhmFJbN2DWTE0midEh_PIiujQBArw"
IP_ADDRESS = "192.168.8.128"
HEADERS = {
    "Authorization": f"Bearer {TOKEN}",
    "Content-Type": "application/json",
}

class HomeAssistantInterface(Node):
    def __init__(self):
        super().__init__('home_assistant_interface')

        # Create services using SetBool
        self.srv_turn_on_all = self.create_service(SetBool, 'turn_on_all_lights', self.turn_on_all_lights_callback)
        self.srv_turn_off_all = self.create_service(SetBool, 'turn_off_all_lights', self.turn_off_all_lights_callback)
        self.srv_get_list = self.create_service(SetBool, 'get_device_list', self.get_device_list_callback)
        self.srv_tts = self.create_service(TextToSpeech, 'text_to_speech', self.text_to_speech_callback)

        # Initialize TTS engine
        self.tts_engine = pyttsx3.init()
        self.get_logger().info('TTS engine initialized')

    def turn_on_all_lights_callback(self, request, response):
        if request.data:
            lights = self.get_lights()
            success = True
            for light in lights:
                url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_on"
                data = {"entity_id": light}
                resp = requests.post(url, headers=HEADERS, json=data)
                if resp.status_code != 200:
                    success = False
                    break
            response.success = success
            response.message = "All lights turned on" if success else "Failed to turn on one or more lights"
        else:
            response.success = True
            response.message = "No action taken because request was False."
        return response

    def turn_off_all_lights_callback(self, request, response):
        if request.data:
            lights = self.get_lights()
            success = True
            for light in lights:
                url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_off"
                data = {"entity_id": light}
                resp = requests.post(url, headers=HEADERS, json=data)
                if resp.status_code != 200:
                    success = False
                    break
            response.success = success
            response.message = "All lights turned off" if success else "Failed to turn off one or more lights"
        else:
            response.success = True
            response.message = "No action taken because request was False."
        return response

    def get_device_list_callback(self, request, response):
        if request.data:
            url = f"http://{IP_ADDRESS}:8123/api/states"
            resp = requests.get(url, headers=HEADERS)
            if resp.status_code == 200:
                data = resp.json()
                devices = [entity['entity_id'] for entity in data if entity['entity_id'].startswith('light.')]
                response.success = True
                response.message = "Lights: " + ", ".join(devices)
            else:
                response.success = False
                response.message = f"Failed to retrieve devices: {resp.status_code}"
        else:
            response.success = True
            response.message = "No action taken because data was False."
        return response

    def text_to_speech_callback(self, request, response):
        try:
            self.get_logger().info(f"Speaking: {request.text}")
            self.tts_engine.say(request.text)
            self.tts_engine.runAndWait()
            response.success = True
            response.message = "Text-to-Speech executed successfully."
        except Exception as e:
            response.success = False
            response.message = f"TTS failed: {str(e)}"
        return response

    def get_lights(self):
        url = f"http://{IP_ADDRESS}:8123/api/states"
        resp = requests.get(url, headers=HEADERS)
        if resp.status_code == 200:
            data = resp.json()
            lights = [entity['entity_id'] for entity in data if entity['entity_id'].startswith('light.')]
            return lights
        else:
            return []

def main(args=None):
    rclpy.init(args=args)
    node = HomeAssistantInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
