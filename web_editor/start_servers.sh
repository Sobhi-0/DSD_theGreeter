#!/bin/bash

# Method 2: Run Flask and React servers using separate functions

# Function to start the Flask server
start_flask() {
  # Start Flask server (app.py) in the background
  python3 backend/db/app.py &
}

# Function to start the React server
start_react() {
  # Start React development server
  npm run dev
}

# Call the functions to start both servers
start_flask
start_react
