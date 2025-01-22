#!/usr/bin/env python3

import json
import random
import time

import requests

TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiIyZTJkNzJmNmRkMTc0N2UyYTcxOGI4ZmRkNjYxMzViNiIsImlhdCI6MTcyNzUwODYwMywiZXhwIjoyMDQyODY4NjAzfQ.3sc6hJ2q_kWEEsdhmFJbN2DWTE0midEh_PIiujQBArw"
IP_ADDRESS = "hass.mdu-smartroom.se"
HEADERS = {
    "Authorization": f"Bearer {TOKEN}",
    "content-type": "application/json",
}
LIGHTS = []


def welcome():
    print("Welcome to the Home Assistant CLI")
    print("You can use this tool to interact with your Home Assistant API")


def menu():
    print("Please select an option:")
    print("1. Turn on a light")
    print("2. Turn off a light")
    print("3. Turn on all lights")
    print("4. Turn off all lights")
    print("5. Change the color of a light")
    print("6. Get the status of a light")
    print("7. Get the status of all entities")
    print("8. List all lights")
    print("9. Party Mode")
    print("10. Exit")
    choice = input("Enter your choice: ")
    print()
    if choice == "1":
        turn_on_light()
    elif choice == "2":
        turn_off_light()
    elif choice == "3":
        turn_on_all_lights()
    elif choice == "4":
        turn_off_all_lights()
    elif choice == "5":
        change_light_color()
    elif choice == "6":
        get_light_status()
    elif choice == "7":
        get_status()
    elif choice == "8":
        list_lights()
    elif choice == "9":
        party_mode()
    elif choice == "10":
        print("Exiting...")
        exit()
    else:
        print("Invalid option, please try again")


def turn_on_light():
    light = input("Enter the name of the light you want to turn on: ")
    url = f"http://{IP_ADDRESS}/api/services/light/turn_on"
    data = {
        "entity_id": light
    }
    response = requests.post(url, headers=HEADERS, json=data, timeout=10)
    if response.status_code == 200:
        print(f"{light} turned on successfully!")
    else:
        print(f"Failed to turn on light: {response.status_code}")
    print("Press any key to continue...")
    input()


def turn_off_light():
    light = input("Enter the name of the light you want to turn off: ")
    url = f"http://{IP_ADDRESS}/api/services/light/turn_off"
    data = {
        "entity_id": light
    }
    response = requests.post(url, headers=HEADERS, json=data, timeout=10)
    if response.status_code == 200:
        print(f"{light} turned off successfully!")
    else:
        print(f"Failed to turn off light: {response.status_code}")
    print("Press any key to continue...")
    input()


def turn_on_all_lights():
    for light in LIGHTS:
        url = f"http://{IP_ADDRESS}/api/services/light/turn_on"
        data = {
            "entity_id": light
        }
        response = requests.post(url, headers=HEADERS, json=data, timeout=10)
        if response.status_code == 200:
            print(f"{light} turned on successfully!")
        else:
            print(f"Failed to turn on light: {response.status_code}")
        time.sleep(0.5)
    print("Press any key to continue...")
    input()


def change_light_color():
    light = input(
        "Enter the name of the light you want to change the color of: ")
    r = int(input("Enter the red value: "))
    g = int(input("Enter the green value: "))
    b = int(input("Enter the blue value: "))
    url = f"http://{IP_ADDRESS}/api/services/light/turn_on"
    data = {
        "entity_id": light,
        "rgb_color": [r, g, b]
    }
    response = requests.post(url, headers=HEADERS, json=data, timeout=10)
    if response.status_code == 200:
        print(f"{light} color changed successfully!")
    else:
        print(f"Failed to change color of light: {response.status_code}")
    print("Press any key to continue...")
    input()


def turn_off_all_lights():
    for light in LIGHTS:
        url = f"http://{IP_ADDRESS}/api/services/light/turn_off"
        data = {
            "entity_id": light
        }
        response = requests.post(url, headers=HEADERS, json=data, timeout=10)
        if response.status_code == 200:
            print(f"{light} turned off successfully!")
        else:
            print(f"Failed to turn off light: {response.status_code}")
        time.sleep(0.5)
    print("Press any key to continue...")
    input()


def get_light_status():
    light = input(
        "Enter the name of the light you want to get the status of: ")
    url = f"http://{IP_ADDRESS}/api/states/{light}"
    response = requests.get(url, headers=HEADERS, timeout=10)
    if response.status_code == 200:
        data = response.json()
        print(f"Status of {light}:")
        print(f"State: {data['state']}")
        print(f"Attributes: {data['attributes']}")
    else:
        print(f"Failed to get status of light: {response.status_code}")
    print("Press any key to continue...")
    input()


def get_status():
    url = f"http://{IP_ADDRESS}/api/states"
    response = requests.get(url, headers=HEADERS, timeout=10)
    if response.status_code == 200:
        data = response.json()
        with open("status.json", "w", encoding="utf-8") as file:
            json.dump(data, file, indent=4)
        print("Status of all entities saved in status.json")
    else:
        print(f"Failed to get status of all entities: {response.status_code}")
    print("Press any key to continue...")
    input()


def list_lights():
    print("List of all lights:")
    for i, light in enumerate(LIGHTS):
        print(f"{i+1}- {light}")
    print("Press any key to continue...")
    input()


def party_mode():
    print("Party Mode: Lights will change color randomly every 0.5 seconds")
    print("Press Ctrl+C to stop")
    while True:
        try:
            for light in LIGHTS:
                r = random.randint(0, 255)
                g = random.randint(0, 255)
                b = random.randint(0, 255)
                url = f"http://{IP_ADDRESS}/api/services/light/turn_on"
                data = {
                    "entity_id": light,
                    "rgb_color": [r, g, b]
                }
                requests.post(url, headers=HEADERS, json=data, timeout=10)
            time.sleep(0.5)
        except KeyboardInterrupt:
            return


def main():
    welcome()
    while True:
        menu()


if __name__ == "__main__":
    main()
