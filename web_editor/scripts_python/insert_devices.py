import json
from pymongo import MongoClient

# Connect to MongoDB
client = MongoClient("mongodb://localhost:27017")
db = client["Greeter"]
collection = db["Devices"]

# Read JSON file
json_file = "status.json"  # Replace with your file path
with open(json_file, "r") as file:
    data = json.load(file)  # Load JSON data

# Insert data into MongoDB
if isinstance(data, list):  # If the JSON file contains an array of objects
    collection.insert_many(data)
    print(f"Inserted {len(data)} documents into the collection Devices")
else:  # If the JSON file contains a single object
    collection.insert_one(data)
    print(f"Inserted one document into the collection Devices")
