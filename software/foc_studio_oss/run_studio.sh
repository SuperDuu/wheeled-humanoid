#!/bin/bash
# ==============================================================================
# STM32 FOC Telemetry Studio OSS - Launcher Script
# Open-Source Software Stack: FastAPI, Uvicorn, WebSockets, NumPy, Pandas, PySerial
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "======================================================================"
echo "⚡ Starting STM32 FOC Telemetry Studio (Open Source Edition)"
echo "======================================================================"

# Check Python 3
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: Python 3 is not installed. Please install python3."
    exit 1
fi

# Ensure Python Virtual Environment or dependencies exist
VENV_DIR="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "📦 Initializing Python Virtual Environment (.venv)..."
    python3 -m venv "$VENV_DIR" || python3 -m venv "$VENV_DIR" --without-pip || true
fi

# Activate venv if valid
if [ -f "$VENV_DIR/bin/activate" ]; then
    source "$VENV_DIR/bin/activate"
fi

# Install/Verify Dependencies
echo "🔍 Checking Open-Source Dependencies (FastAPI, Uvicorn, NumPy, Pandas, PySerial)..."
if ! python3 -c "import fastapi, uvicorn, websockets, pydantic, numpy, pandas, serial" 2>/dev/null; then
    echo "⬇️ Installing required packages from requirements.txt..."
    pip install -r requirements.txt --break-system-packages 2>/dev/null || pip install -r requirements.txt
    echo "✅ Dependencies installed successfully."
else
    echo "✅ All Open-Source frameworks & libraries verified OK."
fi

# Check USB Serial Permissions on Ubuntu
echo "🔒 Checking & granting USB serial port permissions..."
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        if [ ! -r "$dev" ] || [ ! -w "$dev" ]; then
            echo "🔑 Granting permissions (chmod 666) to $dev..."
            sudo chmod 666 "$dev" 2>/dev/null || pkexec chmod 666 "$dev" 2>/dev/null || chmod 666 "$dev" 2>/dev/null || true
        else
            echo "✅ $dev is accessible (read/write OK)."
        fi
    fi
done

# Ensure dialout group
if ! groups | grep -q "dialout"; then
    sudo usermod -a -G dialout "$USER" 2>/dev/null || true
fi

PORT=1111
echo "🧹 Cleaning up previous server instances on port $PORT..."
fuser -k ${PORT}/tcp 2>/dev/null || true
sleep 0.5

echo "🚀 Launching FastAPI & Uvicorn ASGI Server on http://localhost:$PORT..."
echo "📖 Swagger API Documentation available at: http://localhost:$PORT/docs"

python3 -m uvicorn src.main:app --host 0.0.0.0 --port $PORT &
SERVER_PID=$!

cleanup() {
    echo -e "\n🛑 Stopping FOC Studio OSS Server..."
    kill $SERVER_PID 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

sleep 1.5

# Auto open browser
if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:$PORT" &> /dev/null &
elif command -v google-chrome &> /dev/null; then
    google-chrome "http://localhost:$PORT" &> /dev/null &
elif command -v firefox &> /dev/null; then
    firefox "http://localhost:$PORT" &> /dev/null &
fi

echo "======================================================================"
echo "✅ FOC Studio OSS is running at: http://localhost:$PORT"
echo "👉 Interactive API Docs:        http://localhost:$PORT/docs"
echo "👉 Press Ctrl+C to terminate."
echo "======================================================================"

wait $SERVER_PID
