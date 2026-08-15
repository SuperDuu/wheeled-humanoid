# syntax=docker/dockerfile:1
FROM python:3.12-slim-bookworm

LABEL maintainer="Wheeled Humanoid Robotics Project"
LABEL description="Open Source FOC Telemetry Studio & Oscilloscope Visualizer"
LABEL license="MIT"

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python Open-Source Frameworks
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code and static assets
COPY src/ /app/src/
COPY static/ /app/static/

# Expose API and Web UI port
EXPOSE 1111

# Run FastAPI via Uvicorn ASGI Server
CMD ["python3", "-m", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "1111"]
