# app.py
import json
import requests
from pymongo import MongoClient
from datetime import datetime
from flask import Flask, jsonify

# Configuration Variables
TOKEN = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiIyZTJkNzJmNmRkMTc0N2UyYTcxOGI4ZmRkNjYxMzViNiIsImlhdCI6MTcyNzUwODYwMywiZXhwIjoyMDQyODY4NjAzfQ.3sc6hJ2q_kWEEsdhmFJbN2DWTE0midEh_PIiujQBArw"  # Home Assistant token
IP_ADDRESS = "192.168.8.128"  # Home Assistant IP
MONGO_URI = "mongodb://localhost:27017/"  # MongoDB URI
DATABASE_NAME = "home_assistant"
COLLECTION_NAME = "smart_devices"

HEADERS = {
    "Authorization": f"Bearer {TOKEN}",
    "content-type": "application/json",
}

# MongoDB Setup
client = MongoClient(MONGO_URI)
db = client[DATABASE_NAME]
collection = db[COLLECTION_NAME]

# Flask Setup
app = Flask(__name__)

def fetch_entity_status(entity_type=None):
    """
    Retrieve data from Home Assistant API and filter by entity_type,
    excluding entities with a status of 'unknown'.
    """
    url = f"http://{IP_ADDRESS}:8123/api/states"
    try:
        response = requests.get(url, headers=HEADERS, timeout=10)
        if response.status_code == 200:
            data = response.json()

            # Filter by entity_type if specified
            if entity_type:
                data = [entity for entity in data if entity_type in entity['entity_id']]

            # Exclude entities with 'unknown' state
            data = [entity for entity in data if entity['state'] != 'unknown']
            return data
        else:
            print(f"Failed to fetch data. HTTP Status Code: {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Error fetching data from API: {e}")
        return None

def clean_data(data):
    """
    Keep only the 'name' and 'entity_id' for each device.
    """
    cleaned_data = []
    for entity in data:
        cleaned_entity = {
            'name': entity.get('attributes', {}).get('friendly_name', ''),
            'entity_id': entity.get('entity_id', '')
        }
        cleaned_data.append(cleaned_entity)
    return cleaned_data

def store_incremental_data_in_mongodb(data):
    """
    Store only updated data in MongoDB.
    """
    try:
        for entity in data:
            # Add a timestamp for TTL indexing
            entity['timestamp'] = datetime.utcnow()

            # Update the record if it exists, otherwise insert it
            query = {"entity_id": entity['entity_id']}
            update = {"$set": entity}
            collection.update_one(query, update, upsert=True)
        print("Incremental updates successfully stored in MongoDB.")
    except Exception as e:
        print(f"Error storing incremental data in MongoDB: {e}")

def get_and_update_smart_devices(entity_type="sensor"):
    """
    Fetch data, clean it, and store it in MongoDB.
    """
    data = fetch_entity_status(entity_type=entity_type)
    if data:
        data = clean_data(data)
        store_incremental_data_in_mongodb(data)
        return data
    else:
        return []

# API Endpoint to Trigger Device Update
@app.route('/update_devices', methods=['GET'])
def update_devices():
    devices = get_and_update_smart_devices(entity_type="sensor")
    if devices:
        return jsonify(devices), 200
    else:
        return jsonify({"message": "Failed to fetch devices"}), 500

# Run Flask server
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
