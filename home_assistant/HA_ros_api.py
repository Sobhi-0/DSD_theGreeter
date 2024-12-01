import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  # For simple on/off services
from std_msgs.msg import String
import requests
from gtts import gTTS
import os
import speech_recognition as sr
import time

# Set up Home Assistant details
#TOKEN = #add here
#IP_ADDRESS = #ADD here
HEADERS = {
    "Authorization": f"Bearer {TOKEN}",
    "Content-Type": "application/json",
}

class HomeAssistantInterface(Node):
    def __init__(self):
        super().__init__('home_assistant_interface')

        # Define services for TTS, controlling lights, device list, and device state
        self.create_service(SetBool, 'tts_service', self.tts_callback)
        self.create_service(SetBool, 'turn_on_light', self.turn_on_light_callback)
        self.create_service(SetBool, 'turn_off_light', self.turn_off_light_callback)
        self.create_service(SetBool, 'turn_on_all_lights', self.turn_on_all_lights_callback)
        self.create_service(SetBool, 'turn_off_all_lights', self.turn_off_all_lights_callback)
        self.create_service(SetBool, 'change_light_color', self.change_light_color_callback)
        self.create_service(SetBool, 'get_light_status', self.get_light_status_callback)
        self.create_service(SetBool, 'get_device_list', self.get_device_list_callback)
        self.create_service(SetBool, 'get_device_state', self.get_device_state_callback)

        # Set up a publisher for STT output
        self.publisher_ = self.create_publisher(String, 'stt_output', 10)
        self.timer = self.create_timer(2.0, self.listen_and_publish_from_file)

    # Callback for Text-to-Speech (TTS)
    def tts_callback(self, request, response):
        try:
            text = request.data
            tts = gTTS(text=text, lang='en')
            tts.save("output.mp3")
            os.system("mpg321 output.mp3")
            response.success = True
            response.message = "Text spoken successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to play audio: {str(e)}"
        return response

    # Callback to turn on a specific light
    def turn_on_light_callback(self, request, response):
        light = request.data  # Light entity ID passed in the request
        url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_on"
        data = {"entity_id": light}
        response_data = requests.post(url, headers=HEADERS, json=data)
        if response_data.status_code == 200:
            response.success = True
            response.message = f"{light} turned on successfully"
        else:
            response.success = False
            response.message = f"Failed to turn on light: {response_data.status_code}"
        return response

    # Callback to turn off a specific light
    def turn_off_light_callback(self, request, response):
        light = request.data  # Light entity ID passed in the request
        url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_off"
        data = {"entity_id": light}
        response_data = requests.post(url, headers=HEADERS, json=data)
        if response_data.status_code == 200:
            response.success = True
            response.message = f"{light} turned off successfully"
        else:
            response.success = False
            response.message = f"Failed to turn off light: {response_data.status_code}"
        return response

    # Callback to turn on all lights
    def turn_on_all_lights_callback(self, request, response):
        lights = self.get_lights()
        for light in lights:
            url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_on"
            data = {"entity_id": light}
            response_data = requests.post(url, headers=HEADERS, json=data)
            if response_data.status_code != 200:
                response.success = False
                response.message = f"Failed to turn on {light}"
                return response
        response.success = True
        response.message = "All lights turned on successfully"
        return response

    # Callback to turn off all lights
    def turn_off_all_lights_callback(self, request, response):
        lights = self.get_lights()
        for light in lights:
            url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_off"
            data = {"entity_id": light}
            response_data = requests.post(url, headers=HEADERS, json=data)
            if response_data.status_code != 200:
                response.success = False
                response.message = f"Failed to turn off {light}"
                return response
        response.success = True
        response.message = "All lights turned off successfully"
        return response

    # Callback to change the color of a light
    def change_light_color_callback(self, request, response):
        light = request.data["entity_id"]
        rgb_color = request.data["rgb_color"]  # RGB values passed in the request
        url = f"http://{IP_ADDRESS}:8123/api/services/light/turn_on"
        data = {"entity_id": light, "rgb_color": rgb_color}
        response_data = requests.post(url, headers=HEADERS, json=data)
        if response_data.status_code == 200:
            response.success = True
            response.message = f"{light} color changed successfully"
        else:
            response.success = False
            response.message = f"Failed to change color of {light}: {response_data.status_code}"
        return response

    # Callback to get the status of a specific light
    def get_light_status_callback(self, request, response):
        light = request.data  # Light entity ID passed in the request
        url = f"http://{IP_ADDRESS}:8123/api/states/{light}"
        response_data = requests.get(url, headers=HEADERS)
        if response_data.status_code == 200:
            data = response_data.json()
            response.success = True
            response.message = f"State of {light}: {data['state']}"
        else:
            response.success = False
            response.message = f"Failed to retrieve status for {light}: {response_data.status_code}"
        return response

    # Helper function to get the list of lights
    def get_lights(self):
        url = f"http://{IP_ADDRESS}:8123/api/states"
        response_data = requests.get(url, headers=HEADERS)
        if response_data.status_code == 200:
            data = response_data.json()
            lights = [entity['entity_id'] for entity in data if entity['entity_id'].startswith('light.')]
            return lights
        else:
            return []

    # Callback to get the list of devices (light, switches, etc.)
    def get_device_list_callback(self, request, response):
        url = f"http://{IP_ADDRESS}:8123/api/states"
        response_data = requests.get(url, headers=HEADERS)
        if response_data.status_code == 200:
            data = response_data.json()
            devices = [entity['entity_id'] for entity in data]
            response.success = True
            response.message = f"Devices: {devices}"
        else:
            response.success = False
            response.message = f"Failed to retrieve devices: {response_data.status_code}"
        return response

    # Callback to get the state of a specific device
    def get_device_state_callback(self, request, response):
        entity_id = request.data
        url = f"http://{IP_ADDRESS}:8123/api/states/{entity_id}"
        response_data = requests.get(url, headers=HEADERS)
        if response_data.status_code == 200:
            data = response_data.json()
            response.success = True
            response.message = f"State of {entity_id}: {data['state']}"
        else:
            response.success = False
            response.message = f"Failed to retrieve state for {entity_id}: {response_data.status_code}"
        return response

    # Use pre-recorded audio file for speech-to-text
    def listen_and_publish_from_file(self):
        recognizer = sr.Recognizer()
        file_path = '/Users/maheenghani/Documents/22.wav'  # Path to your audio file
        try:
            with sr.AudioFile(file_path) as source:
                audio_data = recognizer.record(source)  # Read the entire audio file
                text = recognizer.recognize_google(audio_data)
                self.publisher_.publish(String(data=text))
                self.get_logger().info(f"Published STT output: {text}")
        except sr.UnknownValueError:
            self.get_logger().info("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"STT service error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HomeAssistantInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
