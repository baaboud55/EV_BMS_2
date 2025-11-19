# ⚡ OTA Quick Reference Card

## First Upload (USB Required)
```bash
pio run --target upload
```

## Get ESP32 IP Address
```bash
pio device monitor
# Look for: "IP address: 192.168.x.x"
```

---

## Method 1: PlatformIO OTA

### Setup (One-Time)

Edit `platformio.ini`:
```ini
upload_protocol = espota
upload_port = 192.168.1.100    # Your ESP32 IP
upload_flags =
    --port=3232
    --auth=bms2024             # Change this password!
```

### Upload
```bash
pio run --target upload
```

---

## Method 2: Web Interface

### 1. Build Firmware
```bash
pio run
```

### 2. Find Binary
```
.pio/build/esp32s3dev/firmware.bin
```

### 3. Upload via Browser
```
http://192.168.x.x/update
```

---

## 🔒 Security

**IMPORTANT**: Change default password!

1. Edit `include/OTAManager.h`:
```cpp
#define OTA_PASSWORD "your_password"
```

2. Edit `platformio.ini`:
```ini
upload_flags =
    --port=3232
    --auth=your_password
```

---

## 🛠️ Common Issues

| Problem | Solution |
|---------|----------|
| "No OTA port found" | Check IP address, verify WiFi connection |
| "Authentication failed" | Password mismatch in ini and .h file |
| "Upload failed" | Restart ESP32, check firewall port 3232 |
| Web upload fails | Rebuild firmware, try different browser |

---

## ✅ Pre-Upload Checklist

- [ ] ESP32 powered on and connected to WiFi
- [ ] IP address confirmed
- [ ] Password configured correctly
- [ ] Firmware built successfully
- [ ] Same network as ESP32
- [ ] Stable power supply

---

## 💡 Quick Tips

- **Serial as Fallback**: Keep USB cable handy
- **Test First**: Try OTA on dev board before production
- **Monitor Output**: Watch Serial Monitor during update
- **Stable WiFi**: Don't update on weak/unstable connection
- **Version Tags**: Use git tags to track firmware versions

---

## 📦 File Locations

```
EV_BMS_2/
├── include/
│   └── OTAManager.h          # OTA configuration
├── src/
│   ├── OTAManager.cpp        # OTA implementation
│   └── main.cpp              # OTA integrated here
├── platformio.ini          # OTA upload config
└── .pio/build/esp32s3dev/
    └── firmware.bin        # Binary for web upload
```

---

## 🔗 Access Points

| Function | URL |
|----------|-----|
| Main Dashboard | `http://192.168.x.x/` |
| OTA Update Page | `http://192.168.x.x/update` |
| BMS API | `http://192.168.x.x/api/bms` |

---

**Need detailed help?** See [OTA_USAGE.md](OTA_USAGE.md)