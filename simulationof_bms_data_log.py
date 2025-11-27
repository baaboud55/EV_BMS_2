import asyncio
import json
import pandas as pd
import websockets
import http.server
import socketserver
import threading
import os
import time
import webbrowser
from functools import partial

# --- CONFIGURATION ---
HTTP_PORT = 8000
WS_PORT = 8765
UPDATE_INTERVAL = 0.05  # Playback speed

# File settings
CSV_FILENAME = "bms_data_log.csv"

# Global Data
sim_data = pd.DataFrame()

# --- 1. ROBUST DATA LOADER ---
def load_simulation_data():
    print("=" * 60)
    print(f"📂 Working Directory: {os.getcwd()}")
    
    if not os.path.exists(CSV_FILENAME):
        print(f"❌ ERROR: '{CSV_FILENAME}' not found!")
        return pd.DataFrame()

    print(f"✅ Found Log File: {CSV_FILENAME}")
    
    try:
        # 1. Read CSV (Try default read first)
        df = pd.read_csv(CSV_FILENAME)
        
        # 2. Handle Missing Headers (Auto-Detection)
        first_col = df.columns[0]
        is_header_missing = False
        try:
            float(first_col)
            is_header_missing = True
        except ValueError:
            pass

        if is_header_missing or 'Pack_Voltage_V' not in df.columns:
            print("⚠️ No text headers found. Applying manual column mapping...")
            df = pd.read_csv(CSV_FILENAME, header=None)
            
            # Map based on SDCardManager.cpp format
            # Timestamp, Cell1..8, PackV, Temp, Current
            rename_map = {
                0: 'Timestamp_ms',
                9: 'Pack_Voltage_V', 
                10: 'Temperature_C', 
                11: 'Current_A'
            }
            df = df.rename(columns=rename_map)
            for i in range(1, 9):
                if i < len(df.columns): df = df.rename(columns={i: f'Cell{i}_V'})

        # 3. Clean Data
        # Convert to numeric
        cols = ['Timestamp_ms', 'Pack_Voltage_V', 'Current_A', 'Temperature_C']
        for c in cols:
            if c in df.columns: df[c] = pd.to_numeric(df[c], errors='coerce')
            
        df = df.dropna(subset=['Pack_Voltage_V'])
        df = df[df['Pack_Voltage_V'] > 5.0] # Filter startup noise

        # --- 4. CALCULATIONS (The Fix) ---
        
        # A. Calculate Time Delta (dt) in hours
        # Shift timestamp to calculate difference between rows
        df['dt_ms'] = df['Timestamp_ms'].diff().fillna(0)
        # Fix huge jumps (e.g. restarts) - cap at 60 seconds
        df['dt_ms'] = df['dt_ms'].clip(0, 60000) 
        df['dt_hours'] = df['dt_ms'] / 3600000.0

        # B. Accurate Capacity Integration (Coulomb Counting)
        # Ah = Current(A) * Time(h)
        df['step_capacity'] = df['Current_A'] * df['dt_hours']
        # Cumulative Sum
        df['capacity'] = df['step_capacity'].cumsum().abs()

        # C. Synthesize SOC
        # Simple voltage mapping 24V(0%) to 33.2V(100%)
        df['soc'] = ((df['Pack_Voltage_V'] - 24.0) / (33.2 - 24.0)) * 100
        df['soc'] = df['soc'].clip(0, 100)

        # D. Determine State
        def get_state(current):
            if abs(current) < 0.2: return "IDLE"
            if current < 0: return "CHARGING"
            return "DISCHARGING"
            
        df['state'] = df['Current_A'].apply(get_state)

        print(f"📊 Loaded {len(df)} points.")
        print(f"   Time Duration: {df['Timestamp_ms'].max() / 3600000:.1f} hours")
        print(f"   Total Capacity Processed: {df['capacity'].max():.2f} Ah")
        
        return df

    except Exception as e:
        print(f"❌ Error parsing data: {e}")
        return pd.DataFrame()

sim_data = load_simulation_data()

# --- 2. HTTP SERVER ---
class BMSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/bms':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"packVoltage": 0}).encode())
        else:
            super().do_GET()

def run_http_server():
    web_dir = os.path.join(os.getcwd(), "data")
    handler = partial(BMSRequestHandler, directory=web_dir)
    socketserver.TCPServer.allow_reuse_address = True
    try:
        with socketserver.TCPServer(("", HTTP_PORT), handler) as httpd:
            print(f"🌍 GUI Server running at http://localhost:{HTTP_PORT}")
            httpd.serve_forever()
    except OSError:
        print(f"⚠️ Port {HTTP_PORT} in use.")

# --- 3. WEBSOCKET SIMULATION ---
async def bms_simulation(websocket):
    if sim_data.empty:
        await websocket.wait_closed()
        return

    print("▶️ Client connected! Replaying log...")
    
    for index, row in sim_data.iterrows():
        try:
            # Extract Cell Voltages
            cells = []
            for i in range(1, 9):
                col = f'Cell{i}_V'
                val = row[col] if col in row else row['Pack_Voltage_V']/8
                cells.append(float(val))

            payload = {
                "packVoltage": float(row['Pack_Voltage_V']),
                "current": float(row['Current_A']),
                "soc": float(row['soc']),
                "soh": 85.1, # VALIDATED VALUE FROM REPORT
                "batteryState": row['state'],
                "cellVoltages": cells,
                "slaveTemperatures": [float(row['Temperature_C'])] * 2,
                "prechargeActive": row['state'] == "PRECHARGING",
                "protectionPMOSActive": row['state'] == "CHARGING",
                "activeBalancingActive": bool(max(cells) - min(cells) > 0.05),
                "overvoltage": bool(max(cells) > 4.2),
                "undervoltage": bool(min(cells) < 3.0),
                "overcurrent": bool(abs(row['Current_A']) > 40),
                "cumulativeCapacity": float(row['capacity']),
                "timestamp": int(time.time())
            }
            
            await websocket.send(json.dumps(payload))
            await asyncio.sleep(UPDATE_INTERVAL)
            
        except websockets.exceptions.ConnectionClosed:
            print("⏹️ Client disconnected.")
            break
        except Exception as e:
            print(f"Error: {e}")
            break

async def main():
    async with websockets.serve(bms_simulation, "localhost", WS_PORT):
        print(f"📡 WebSocket Simulation at ws://localhost:{WS_PORT}")
        await asyncio.Future()

if __name__ == "__main__":
    t = threading.Thread(target=run_http_server, daemon=True)
    t.start()
    time.sleep(1)
    webbrowser.open(f"http://localhost:{HTTP_PORT}")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        