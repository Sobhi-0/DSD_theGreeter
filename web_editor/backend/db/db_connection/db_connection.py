from pymongo import MongoClient

# MongoDB connection setup
def get_db_connection():
    # Replace the MongoDB URI with your actual connection string
    client = MongoClient("mongodb://localhost:27017/")  # Replace with your actual URI
    db = client['Greeter']  # Database name
    return db
