from db_connection.db_connection import get_db_connection
import re

def get_device_by_entity(entity_id):
    # Get database connection
    db = get_db_connection()
    
    # Access the Devices collection
    devices_collection = db['Devices']
    
    # Query the database to find the device with the specified entity_id
    device = devices_collection.find_one({"entity_id": entity_id})
    
    if device:
        # Return a dictionary with relevant information
        return {
            "entity_id": device.get("entity_id"),
            "friendly_name": device.get("attributes", {}).get("friendly_name"),
            "state": device.get("state")
        }
    else:
        return None

def get_devices_count_by_state():
    # Obtener la conexión a la base de datos
    db = get_db_connection()
    devices_collection = db['Devices']  # Ajusta el nombre de tu colección
    
    # Contar dispositivos por estado
    states = ['on', 'off', 'idle']
    state_counts = {state: 0 for state in states}

    devices_info = {}

    # Consultar dispositivos
    for state in states:
        devices = devices_collection.find({"state": state})

        # Filtramos los dispositivos para evitar los duplicados
        for device in devices:
            entity_id = device.get("entity_id")
            # Usamos el entity_id como clave para evitar duplicados
            if entity_id not in devices_info:
                devices_info[entity_id] = {
                    "entity_id": entity_id,
                    "state": device.get("state"),
                    "friendly_name": device.get("attributes", {}).get("friendly_name")
                }

        # Ahora contamos los dispositivos únicos para este estado
        state_counts[state] = len([device for device in devices_info.values() if device['state'] == state])

    # Convertir el diccionario a una lista de dispositivos sin duplicados
    devices_info_list = list(devices_info.values())

    return state_counts, devices_info_list

def get_all_lights():
    # Obtener la conexión a la base de datos
    db = get_db_connection()
    devices_collection = db['Devices']
    
    # Buscar todos los dispositivos de tipo 'light'
    lights = devices_collection.find({"entity_id": {"$regex": "light", "$options": "i"}})  # Usando regex para encontrar lights
    
    # Usamos un conjunto para eliminar duplicados por entity_id
    unique_lights = {}
    
    for light in lights:
        entity_id = light.get("entity_id")
        
        # Solo agregamos la luz si no existe ya en el diccionario
        if entity_id not in unique_lights:
            unique_lights[entity_id] = {
                "entity_id": light.get("entity_id"),
                "friendly_name": light.get("attributes", {}).get("friendly_name"),
                "state": light.get("state"),
                "color": light.get("attributes", {}).get("color", "yellow")  # Color por defecto
            }
    
    # Convertir el diccionario de luces únicas en una lista
    unique_lights_list = list(unique_lights.values())

    return unique_lights_list
