#!/bin/bash

# Receiver Control Startup Script
# This script runs the hand-tracking motor control system using UV

# Navigate to the project directory
cd /home/frioaj1/Downloads/Reciever-Control-main

# Activate the UV environment and run the script
# Using uv run to ensure proper environment activation
uv run python main.py
