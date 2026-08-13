#!/bin/bash
# ==============================================================================
# STM32 FOC Telemetry & Oscilloscope Visualizer Launcher
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================================================"
echo "⚡ Starting STM32 FOC Telemetry & Oscilloscope Visualizer"
echo "======================================================================"

# Check if python3 is installed
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: Python 3 is not installed. Please install python3."
    exit 1
fi

# Check if pyserial is installed
if ! python3 -c "import serial" 2>/dev/null; then
    echo "⚠️ Warning: 'pyserial' is not installed in the current Python environment."
    echo "👉 To install pyserial, run:"
    echo "   pip install pyserial --break-system-packages"
    echo "   (or create a venv: python3 -m venv venv && source venv/bin/activate && pip install pyserial)"
    echo "----------------------------------------------------------------------"
fi

# Auto-grant read/write permissions for USB Serial ports (/dev/ttyUSB* and /dev/ttyACM*)
echo "🔒 Checking & granting USB serial port permissions..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
            echo "🔑 Granting permissions (chmod 666) to $dev..."
            sudo chmod 666 "$dev" 2>/dev/null || pkexec chmod 666 "$dev" 2>/dev/null || chmod 666 "$dev" 2>/dev/null
        else
            echo "✅ $dev is accessible (read/write OK)."
        fi
    fi
done

# Ensure user is in dialout group for persistent access across reboots
if ! groups | grep -q "dialout"; then
    echo "🔑 Adding user $USER to 'dialout' group..."
    sudo usermod -a -G dialout "$USER" 2>/dev/null || true
fi

# Launch web server in background
PORT=5050
echo "🚀 Launching local telemetry server on http://localhost:$PORT..."
python3 server.py $PORT &
SERVER_PID=$!

# Trap exit signals to gracefully terminate server
cleanup() {
    echo -e "\n🛑 Stopping server..."
    kill $SERVER_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

sleep 1

# Try to open default browser
if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:$PORT" &> /dev/null &
elif command -v google-chrome &> /dev/null; then
    google-chrome "http://localhost:$PORT" &> /dev/null &
elif command -v firefox &> /dev/null; then
    firefox "http://localhost:$PORT" &> /dev/null &
fi

echo "======================================================================"
echo "✅ Visualizer is running at: http://localhost:$PORT"
echo "👉 Press Ctrl+C in this terminal to stop the server."
echo "======================================================================"

# Wait for server process
wait $SERVER_PID
