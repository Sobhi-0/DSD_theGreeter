from flask import Flask, jsonify, request
from devices.devices_model import get_device_by_entity, get_devices_count_by_state

app = Flask(__name__)

@app.route('/devices/<entity_id>', methods=['GET'])
def get_device(entity_id):
    # Use the model to fetch device data based on entity_id
    device = get_device_by_entity(entity_id)
    
    if device:
        return jsonify(device)
    else:
        return jsonify({"error": "Device not found"}), 404


# Ruta para obtener el conteo de dispositivos por estado
@app.route('/devices/count', methods=['GET'])
def get_devices_count():
    # Usamos la función que cuenta los dispositivos por estado
    state_counts, devices_info = get_devices_count_by_state()

    # Devolvemos el conteo de dispositivos por estado y la información de los dispositivos
    return jsonify({
        "state_counts": state_counts,
        "devices_info": devices_info
    })

if __name__ == '__main__':
    app.run(debug=True)
