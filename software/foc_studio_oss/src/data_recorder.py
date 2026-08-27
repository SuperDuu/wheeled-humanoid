"""
Telemetry Data Recorder and Pandas Analysis Engine.
Utilizes Pandas (BSD-3-Clause) for structured DataFrame management,
statistical aggregation, and MATLAB-ready CSV/Parquet export.
"""

import os
import io
import time
import threading
from typing import Dict, Any, List, Optional
import pandas as pd
import numpy as np


class TelemetryRecorder:
    """Thread-safe telemetry recording engine powered by Pandas."""

    def __init__(self, max_buffer_size: int = 200000):
        self.is_recording = False
        self.max_buffer_size = max_buffer_size
        self.records: List[Dict[str, Any]] = []
        self.lock = threading.Lock()
        self.start_time: Optional[float] = None

    def start_recording(self) -> None:
        """Start or reset recording session."""
        with self.lock:
            self.records.clear()
            self.is_recording = True
            self.start_time = time.time()

    def stop_recording(self) -> int:
        """Stop recording session and return total sample count."""
        with self.lock:
            self.is_recording = False
            return len(self.records)

    def add_sample(self, sample: Dict[str, Any]) -> None:
        """Append sample if recording is active."""
        if not self.is_recording:
            return
        with self.lock:
            if len(self.records) < self.max_buffer_size:
                self.records.append(sample.copy())

    def get_sample_count(self) -> int:
        """Return current number of recorded samples."""
        with self.lock:
            return len(self.records)

    def to_dataframe(self) -> pd.DataFrame:
        """Convert recorded buffer to a Pandas DataFrame."""
        with self.lock:
            if not self.records:
                return pd.DataFrame()
            return pd.DataFrame(self.records)

    def export_csv(self) -> str:
        """
        Export recorded data to CSV with header metadata for MATLAB and Python analysis.
        """
        df = self.to_dataframe()
        if df.empty:
            return "timestamp_ms,i_a,i_b,i_c,i_d,i_q,phase_elec,speed_rpm,v_bus\n"

        # Calculate time offset in seconds for MATLAB convenience
        first_ts = df["timestamp_ms"].iloc[0]
        df["time_sec"] = (df["timestamp_ms"] - first_ts) / 1000.0

        # Rearrange columns to put time_sec first
        cols = ["time_sec"] + [col for col in df.columns if col != "time_sec"]
        df = df[cols]

        output = io.StringIO()
        # Add metadata comment lines for MATLAB readmatrix or Python pandas
        output.write("# FOC Telemetry Studio OSS Data Capture\n")
        output.write(f"# Export Date: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        output.write(f"# Total Samples: {len(df)}\n")
        output.write("# ----------------------------------------\n")
        df.to_csv(output, index=False)
        return output.getvalue()

    def get_summary_statistics(self) -> Dict[str, Any]:
        """
        Compute summary statistical metrics using Pandas.
        """
        df = self.to_dataframe()
        if df.empty:
            return {"sample_count": 0}

        stats = {
            "sample_count": len(df),
            "duration_sec": round(float((df["timestamp_ms"].iloc[-1] - df["timestamp_ms"].iloc[0]) / 1000.0), 2) if len(df) > 1 else 0.0,
            "max_i_peak_amps": round(float(df["i_peak"].max()), 3) if "i_peak" in df else 0.0,
            "max_speed_rpm": round(float(df["speed_rpm"].max()), 1) if "speed_rpm" in df else 0.0,
            "avg_vbus_volts": round(float(df["v_bus"].mean()), 2) if "v_bus" in df else 0.0,
            "max_power_watts": round(float(df["power_watts"].max()), 2) if "power_watts" in df else 0.0,
            "mean_temp_fet": round(float(df["temp_fet"].mean()), 1) if "temp_fet" in df else 0.0
        }
        return stats
