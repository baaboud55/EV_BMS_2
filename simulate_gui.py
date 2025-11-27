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
UPDATE_INTERVAL = 0.1  # 0.1s per data point

# File settings
CSV_FILENAME = "Validation_all_in_one.xlsx - Charing Profile.csv"
EXCEL_FILENAME = "Validation_all_in_one.xlsx"
EXCEL_SHEET = "Charing Profile" 

# Global Data
sim_data = pd.DataFrame()

# --- 1. SMART DATA LOADER ---
def load_simulation_data():
    print("=" * 60)
    print(f"📂 Working Directory: {os.getcwd()}")
    
    df_raw = None
    
    # A. Try loading Excel
    if os.path.exists(EXCEL_FILENAME):
        print(f"✅ Found Excel: {EXCEL_FILENAME}")
        try:
            # Read first 30 rows to find the header
            df_raw = pd.read_excel(EXCEL_FILENAME, sheet_name=EXCEL_SHEET, header=None, nrows=30)
        except Exception as e:
            print(f"❌ Error reading Excel: {e}")
            return pd.DataFrame()
            
    # B. Try loading CSV
    elif os.path.exists(CSV_FILENAME):
        print(f"✅ Found CSV: {CSV_FILENAME}")
        try:
            df_raw = pd.read_csv(CSV_FILENAME, header=None, nrows=30)
        except Exception as e:
            print(f"❌ Error reading CSV: {e}")
            return pd.DataFrame()
    else:
        print("❌ ERROR: No data file found!")
        return pd.DataFrame()

    # C. Find the Header Row Automatically
    header_idx = -1
    for idx, row in df_raw.iterrows():
        row_str = row.astype(str).str.lower().values
        # Look for a row that has both 'voltage' and 'current' keywords
        if any("voltage" in s for s in row_str) and any("current" in s for s in row_str):
            header_idx = idx
            break
    
    if header_idx == -1:
        print("❌ Error: Could not find header row with 'Voltage' and 'Current'.")
        return pd.DataFrame()
    
    print(f"🎯 Detected Header at Row {header_idx + 1}")

    # D. Reload Data with Correct Header
    try:
        if os.path.exists(EXCEL_FILENAME):
            df = pd.read_excel(EXCEL_FILENAME, sheet_name=EXCEL_SHEET, header=header_idx)
        else:
            df = pd.read_csv(CSV_FILENAME, header=header_idx)
            
        # E. Map Columns safely
        data = pd.DataFrame()
        
        def get_col(keywords):
            for col in df.columns:
                col_lower = str(col).lower()
                for k in keywords:
                    if k in col_lower:
                        return df[col]
            return pd.Series(dtype=float)

        data['packVoltage'] = get_col(['voltage v', 'voltage'])
        data['current'] = get_col(['current a', 'current'])
        data['soc'] = get_col(['soc'])
        data['temp'] = get_col(['tempreture', 'temperature']) 
        data['capacity'] = get_col(['cumulative mah', 'cumulative'])

        # F. Clean and Format
        for col in data.columns:
            data[col] = pd.to_numeric(data[col], errors='coerce')
        
        data = data.dropna(subset=['packVoltage'])
        
        if not data.empty and data['soc'].max() <= 1.0: 
            data['soc'] *= 100

        print(f"📊 Successfully Loaded {len(data)} simulation points!")
        return data

    except Exception as e:
        print(f"❌ Error parsing data: {e}")
        return pd.DataFrame()

# Load data immediately
sim_data = load_simulation_data()

# --- 2. HTTP SERVER ---
class BMSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/bms':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                "packVoltage": 0, "current": 0, "soc": 0, "timestamp": 0
            }).encode())
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
        print("⚠️ No data to simulate.")
        await websocket.wait_closed()
        return

    print("▶️ Client connected! Starting simulation stream...")
    
    while True:
        for index, row in sim_data.iterrows():
            try:
                # --- FIX IS HERE: Cast everything to float/bool ---
                pack_v = float(row['packVoltage'])
                current = float(row['current'])
                temp = float(row['temp']) if not pd.isna(row['temp']) else 25.0
                
                payload = {
                    "packVoltage": pack_v,
                    "current": current,
                    "soc": float(row['soc']),
                    "soh": 85.1,
                    "batteryState": "CHARGING" if current < 0 else "DISCHARGING",
                    "cellVoltages": [pack_v / 8.0] * 8,
                    "slaveTemperatures": [temp, temp],
                    "prechargeActive": False,
                    "protectionPMOSActive": True,
                    
                    # CAST TO NATIVE BOOL:
                    "activeBalancingActive": bool(pack_v > 33.0),
                    
                    "overvoltage": False, "undervoltage": False, "overcurrent": False,
                    "cumulativeCapacity": float(row['capacity']),
                    "timestamp": int(time.time())
                }
                
                await websocket.send(json.dumps(payload))
                await asyncio.sleep(UPDATE_INTERVAL)
                
            except websockets.exceptions.ConnectionClosed:
                print("⏹️ Client disconnected.")
                return 
            except Exception as e:
                print(f"Error sending: {e}")
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