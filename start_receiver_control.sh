#!/bin/bash

# Receiver Control Startup Script
# This script runs the hand-tracking motor control system using UV

echo "=== Receiver Control Starting ==="
echo "Script: $(readlink -f "$0")"
echo "Working directory: $(pwd)"

# Navigate to the project directory
PROJECT_DIR="/home/frioaj1/Downloads/Reciever-Control-main"
echo "Changing to project directory: $PROJECT_DIR"

if [ ! -d "$PROJECT_DIR" ]; then
    echo "ERROR: Project directory does not exist: $PROJECT_DIR"
    exit 1
fi

cd "$PROJECT_DIR" || {
    echo "ERROR: Failed to change to project directory: $PROJECT_DIR"
    exit 1
}

echo "Current directory: $(pwd)"

# Check if uv is available
if ! command -v uv &> /dev/null; then
    echo "ERROR: uv command not found. Please install uv or ensure it's in PATH."
    echo "You can install uv with: curl -LsSf https://astral.sh/uv/install.sh | sh"
    exit 1
fi

# Check if main.py exists
if [ ! -f "main.py" ]; then
    echo "ERROR: main.py not found in $(pwd)"
    exit 1
fi

echo "=== Starting application ==="
# Run the script using uv
exec uv run python main.py
