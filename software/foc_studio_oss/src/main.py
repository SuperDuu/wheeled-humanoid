"""
Main FastAPI Application & WebSocket Server for FOC Studio OSS.
Utilizes FastAPI (MIT), Uvicorn (BSD), Starlette WebSockets (BSD), Pydantic (MIT),
NumPy (BSD), Pandas (BSD), PySerial (BSD), and ROS 2 Client (Apache-2.0).
"""

import os
import sys
import asyncio
import json
from typing import List, Set
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Response
from fastapi.responses import HTMLResponse, PlainTextResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware

from .models import PortInfo, ConnectRequest, MotorCommand, SystemStatus, TelemetryFrame
from .serial_manager import SerialManager
from .data_recorder import TelemetryRecorder

# Initialize FastAPI App with Open Source Metadata
app = FastAPI(
    title="⚡ STM32 FOC Telemetry Studio OSS",
    description="""
### Open Source FOC Motor Control, Real-Time Oscilloscope & Telemetry System
Built with **FastAPI**, **Uvicorn**, **WebSockets**, **NumPy**, **Pandas**, **PySerial**, **Bootstrap 5**, and **Chart.js**.

* **License**: MIT License
* **Repository**: Wheeled Humanoid Robotics Project
* **API Documentation**: Automated OpenAPI 3.0 / Swagger UI
    """,
    version="2.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    }
)

# Enable CORS for open-source integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global State & Subsystems
recorder = TelemetryRecorder()
connected_websockets: Set[WebSocket] = set()
loop: asyncio.AbstractEventLoop = None


def broadcast_telemetry(frame_data: dict) -> None:
    """Callback invoked by SerialManager thread on every new telemetry frame."""
    # Record data via Pandas engine
    recorder.add_sample(frame_data)

    # Broadcast to all connected WebSockets
    if connected_websockets and loop and loop.is_running():
        msg = json.dumps(frame_data)
        for ws in list(connected_websockets):
            try:
                asyncio.run_coroutine_threadsafe(ws.send_text(msg), loop)
            except Exception:
                pass


serial_mgr = SerialManager(on_frame_callback=broadcast_telemetry)


@app.on_event("startup")
async def startup_event():
    global loop
    loop = asyncio.get_running_loop()
    print("⚡ FOC Studio OSS Backend Initialized with FastAPI & Uvicorn.")


@app.on_event("shutdown")
async def shutdown_event():
    serial_mgr.disconnect()
    recorder.stop_recording()


# ==============================================================================
# REST API Endpoints
# ==============================================================================

@app.get("/api/status", response_model=SystemStatus, tags=["System Status"])
async def get_system_status():
    """Retrieve overall system status, connection state, and telemetry rate."""
    return SystemStatus(
        connected=serial_mgr.is_connected,
        port=serial_mgr.port,
        baudrate=serial_mgr.baudrate,
        is_simulation=serial_mgr.is_simulation,
        telemetry_hz=serial_mgr.telemetry_hz,
        samples_received=serial_mgr.sample_count,
        recorded_samples=recorder.get_sample_count(),
        is_recording=recorder.is_recording
    )


@app.get("/api/ports", response_model=List[PortInfo], tags=["Hardware & Connection"])
async def get_available_ports():
    """Scan and list all physical serial ports and virtual simulation modes."""
    return serial_mgr.list_available_ports()


@app.post("/api/connect", tags=["Hardware & Connection"])
async def connect_port(req: ConnectRequest):
    """Establish serial connection to STM32 hardware or start simulation."""
    try:
        success = serial_mgr.connect(port=req.port, baudrate=req.baudrate)
        return {"status": "connected", "port": req.port, "baudrate": req.baudrate, "is_simulation": serial_mgr.is_simulation}
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.post("/api/disconnect", tags=["Hardware & Connection"])
async def disconnect_port():
    """Disconnect active serial connection or simulation."""
    serial_mgr.disconnect()
    return {"status": "disconnected"}


@app.post("/api/control", tags=["Motor Control"])
async def send_motor_control(cmd: MotorCommand):
    """Send motor control command (IDLE, CURRENT, BRAKE, SPEED, POSITION)."""
    if not serial_mgr.is_connected:
        raise HTTPException(status_code=400, detail="Device is not connected.")

    target = cmd.target_value
    if cmd.control_mode == 2 and cmd.brake_current is not None:
        target = cmd.brake_current

    ok = serial_mgr.send_motor_control(cmd.control_mode, target)
    return {"success": ok, "mode": cmd.control_mode, "target": target}


@app.post("/api/record/start", tags=["Analysis & Recording"])
async def start_recording():
    """Start capturing telemetry samples in the Pandas recording buffer."""
    recorder.start_recording()
    return {"status": "recording_started"}


@app.post("/api/record/stop", tags=["Analysis & Recording"])
async def stop_recording():
    """Stop capturing telemetry samples and return count."""
    count = recorder.stop_recording()
    return {"status": "recording_stopped", "total_samples": count}


@app.get("/api/record/export", tags=["Analysis & Recording"])
async def export_csv():
    """Download recorded telemetry as MATLAB/Python compatible CSV file."""
    csv_content = recorder.export_csv()
    filename = f"foc_telemetry_capture_{int(asyncio.get_event_loop().time())}.csv"
    return Response(
        content=csv_content,
        media_type="text/csv",
        headers={"Content-Disposition": f"attachment; filename={filename}"}
    )


@app.get("/api/record/stats", tags=["Analysis & Recording"])
async def get_recording_statistics():
    """Retrieve statistical summary of the recorded session via Pandas."""
    return recorder.get_summary_statistics()


# ==============================================================================
# WebSocket Real-Time Telemetry Stream
# ==============================================================================

@app.websocket("/ws/telemetry")
async def websocket_telemetry_endpoint(websocket: WebSocket):
    """Real-time bi-directional WebSocket streaming at 60-100Hz."""
    await websocket.accept()
    connected_websockets.add(websocket)
    try:
        while True:
            # Keep-alive or handle incoming client messages
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                if msg.get("action") == "ping":
                    await websocket.send_text(json.dumps({"action": "pong"}))
            except Exception:
                pass
    except WebSocketDisconnect:
        connected_websockets.discard(websocket)
    except Exception:
        connected_websockets.discard(websocket)


# ==============================================================================
# Static Web Assets (Frontend)
# ==============================================================================

static_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "static")
if os.path.exists(static_dir):
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.get("/", response_class=FileResponse, tags=["Web Interface"])
    async def serve_index():
        index_file = os.path.join(static_dir, "index.html")
        if os.path.exists(index_file):
            return FileResponse(index_file)
        return PlainTextResponse("Static index.html not found.")
