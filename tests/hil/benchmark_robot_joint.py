#!/usr/bin/env python3
import asyncio
import websockets
import json
import requests
import time
import numpy as np
import csv

API_CMD = "http://127.0.0.1:1111/api/command"
WS_URL = "ws://127.0.0.1:1111/ws/telemetry"

async def benchmark():
    async with websockets.connect(WS_URL) as ws:
        print("=" * 80)
        print("  HỆ THỐNG ĐÁNH GIÁ ĐIỀU KHIỂN GÓC & LỰC KHỚP ROBOT (BENCHMARK SUITE)")
        print("=" * 80)

        # Ensure Home is calibrated
        requests.post(API_CMD, json={"command": "ZERO"})
        await asyncio.sleep(0.5)

        # ---------------------------------------------------------------------
        # PHẦN 1: ĐÁNH GIÁ ĐIỀU KHIỂN GÓC (POSITION, OVERSHOOT, SETTLING TIME)
        # ---------------------------------------------------------------------
        angles_test = [15, 30, 60, 90, 120, 180, 270, 360, 180, 0, -45, -90, -180, 0]
        results_pos = []

        print("\n" + "-" * 80)
        print("  PHẦN 1: ĐÁNH GIÁ ĐIỀU KHIỂN GÓC QUAY (S-CURVE + IMPEDANCE PD)")
        print("-" * 80)
        print(f"{'Target':>8} | {'Actual':>8} | {'Error':>8} | {'Overshoot':>10} | {'Settling':>9} | {'Iq_peak':>8} | {'Iq_hold':>8} | {'Status':<10}")
        print("-" * 80)

        for target in angles_test:
            # Drain buffer
            while True:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=0.01)
                except asyncio.TimeoutError:
                    break

            # Natural move time based on delta angle
            msg = await ws.recv()
            d0 = json.loads(msg)
            start_deg = d0.get('joint_angle', 0) * (180.0 / np.pi)
            delta_deg = abs(target - start_deg)
            duration_s = max(0.8, min(2.5, 0.6 + (delta_deg * np.pi / 180.0) * 0.35))

            cmd = f"MOVE {target} {duration_s:.2f}"
            requests.post(API_CMD, json={"command": cmd})
            t_cmd = time.time()

            time_history = []
            angle_history = []
            iq_history = []

            # Sample for duration + 0.8s settling
            total_wait = duration_s + 0.8
            while time.time() - t_cmd < total_wait:
                msg = await ws.recv()
                d = json.loads(msg)
                t_rel = time.time() - t_cmd
                ang = d.get('joint_angle', 0) * (180.0 / np.pi)
                iq = d.get('i_q', 0)
                time_history.append(t_rel)
                angle_history.append(ang)
                iq_history.append(iq)

            angle_arr = np.array(angle_history)
            time_arr = np.array(time_history)
            iq_arr = np.array(iq_history)

            final_angle = float(angle_arr[-1])
            err = final_angle - target
            iq_peak = float(np.max(np.abs(iq_arr)))
            iq_hold = float(np.mean(iq_arr[-30:]))

            # Calculate Overshoot
            if target > start_deg:
                max_val = np.max(angle_arr)
                overshoot_deg = max(0.0, max_val - target)
            else:
                min_val = np.min(angle_arr)
                overshoot_deg = max(0.0, target - min_val)

            overshoot_pct = (overshoot_deg / max(1.0, delta_deg)) * 100.0 if delta_deg > 5.0 else 0.0

            # Calculate Settling Time (within +/- 0.5 deg of final target)
            settled_indices = np.where(np.abs(angle_arr - target) <= 0.5)[0]
            if len(settled_indices) > 0:
                is_settled_tail = np.all(np.abs(angle_arr[settled_indices[-20:]] - target) <= 0.5)
                if is_settled_tail:
                    t_settling = float(time_arr[settled_indices[0]])
                else:
                    t_settling = float(time_arr[-1])
            else:
                t_settling = float(total_wait)

            status = "PASS" if abs(err) <= 0.5 else "TOLERANCE"
            print(f"{target:8.1f}° | {final_angle:8.2f}° | {err:+7.2f}° | {overshoot_pct:8.2f}%  | {t_settling:7.2f}s  | {iq_peak:7.2f}A | {iq_hold:7.2f}A | {status:<10}")

            results_pos.append({
                'target_deg': target,
                'actual_deg': round(final_angle, 2),
                'error_deg': round(err, 2),
                'overshoot_deg': round(overshoot_deg, 2),
                'overshoot_pct': round(overshoot_pct, 2),
                'settling_time_s': round(t_settling, 2),
                'iq_peak_a': round(iq_peak, 2),
                'iq_hold_a': round(iq_hold, 2),
            })

        # ---------------------------------------------------------------------
        # PHẦN 2: ĐÁNH GIÁ ĐÁP ỨNG LỰC / MÔ-MEN (TORQUE & CURRENT STEP RESPONSE)
        # ---------------------------------------------------------------------
        print("\n" + "-" * 80)
        print("  PHẦN 2: ĐÁNH GIÁ ĐÁP ỨNG LỰC / MÔ-MEN (TORQUE & CURRENT STEP RESPONSE)")
        print("-" * 80)
        print(f"{'Command':<12} | {'Target_Tau':>10} | {'Expected_Iq':>11} | {'Measured_Iq':>11} | {'Iq_Ripple':>10} | {'Rise_Time':>10} | {'Status':<8}")
        print("-" * 80)

        torque_steps = [0.5, 1.0, 2.0, 3.0, -1.0, -2.0]
        results_torque = []

        for tau in torque_steps:
            # Drain buffer
            while True:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=0.01)
                except asyncio.TimeoutError:
                    break

            expected_iq = tau / 5.4 # Kt_joint ~= 5.4 Nm/A
            requests.post(API_CMD, json={"command": f"TORQUE {tau}"})
            t_cmd = time.time()

            t_hist = []
            iq_hist = []

            while time.time() - t_cmd < 0.6:
                msg = await ws.recv()
                d = json.loads(msg)
                t_hist.append(time.time() - t_cmd)
                iq_hist.append(d.get('i_q', 0))

            requests.post(API_CMD, json={"command": "STOP"})
            await asyncio.sleep(0.3)

            iq_arr = np.array(iq_hist)
            t_arr = np.array(t_hist)

            steady_iq = iq_arr[len(iq_arr)//2:]
            mean_iq = float(np.mean(steady_iq))
            std_iq = float(np.std(steady_iq))

            target_90 = 0.90 * mean_iq
            if expected_iq > 0:
                reach_idx = np.where(iq_arr >= target_90)[0]
            else:
                reach_idx = np.where(iq_arr <= target_90)[0]

            rise_time_ms = float(t_arr[reach_idx[0]] * 1000.0) if len(reach_idx) > 0 else 10.0

            status = "PASS" if abs(mean_iq - expected_iq) < 0.25 else "CHECK"
            print(f"{'TORQUE '+str(tau):<12} | {tau:9.1f}Nm | {expected_iq:10.2f}A | {mean_iq:10.2f}A | {std_iq:9.3f}A | {rise_time_ms:8.1f}ms | {status:<8}")

            results_torque.append({
                'cmd_torque_nm': tau,
                'expected_iq_a': round(expected_iq, 2),
                'measured_iq_a': round(mean_iq, 2),
                'ripple_std_a': round(std_iq, 3),
                'rise_time_ms': round(rise_time_ms, 1),
            })

        requests.post(API_CMD, json={"command": "STOP"})

        report = {
            'timestamp': time.time(),
            'position_tests': results_pos,
            'torque_tests': results_torque
        }
        with open('/home/du/.gemini/antigravity-ide/brain/34a5076a-3dad-49e0-ae9b-0c8d766b848c/scratch/benchmark_results.json', 'w') as f:
            json.dump(report, f, indent=2)
        print("\n" + "=" * 80)
        print("  BENCHMARK HOÀN THÀNH VÀ ĐÃ LƯU KẾT QUẢ VÀO FILE BÁO CÁO!")
        print("=" * 80)

if __name__ == '__main__':
    asyncio.run(benchmark())
