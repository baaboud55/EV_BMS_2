# Over-The-Air (OTA) Firmware Update Guide

This document explains how to update the ESP32-S3 BMS firmware wirelessly using OTA (Over-The-Air) updates.

## 🎯 Overview

The BMS now supports two methods for OTA firmware updates:

1. **PlatformIO OTA Upload** - Upload directly from PlatformIO IDE/CLI
2. **Web Interface Upload** - Upload .bin files through the web dashboard

## 🔧 Initial Setup

### 1. First-Time Upload (Serial)

For the first upload, you **must** use serial (USB) connection:

```bash
pio run --target upload
```

This uploads the firmware with OTA support enabled.

### 2. Find Your ESP32's IP Address

After the first upload, open the Serial Monitor to see the IP address:

```bash
pio device monitor
```

You'll see output like:
```
WiFi connected!
IP address: 192.168.x.x
OTA Manager initialized successfully!
```

**Write down this IP address** - you'll need it for OTA uploads.

---

## 📡 Method 1: PlatformIO OTA Upload

### Configuration

1. Open `platformio.ini`
2. Uncomment the OTA configuration lines:

```ini
; OTA Upload Configuration
upload_protocol = espota
upload_port = 192.168.x.x  ; Replace with your ESP32's actual IP
upload_flags =
    --port=3232
    --auth=bms2024
```

3. Replace `192.168.x.x` with your ESP32's actual IP address
4. **Important**: Change `bms2024` to your own password in both:
   - `platformio.ini` (upload_flags --auth)
   - `include/OTAManager.h` (OTA_PASSWORD)

### Upload Firmware

Now you can upload wirelessly:

```bash
pio run --target upload
```

Or in VS Code: Click the Upload button (→) in PlatformIO toolbar.

### Expected Output

```
Uploading .pio/build/esp32s3dev/firmware.bin
192.168.x.x:3232
Sending invitation to: 192.168.x.x
Auth: bms2024
Uploading: [============] 100%
```

---

## 🌐 Method 2: Web Interface Upload

### Step 1: Build Firmware Binary

First, build the firmware .bin file:

```bash
pio run
```

The firmware binary will be located at:
```
.pio/build/esp32s3dev/firmware.bin
```

### Step 2: Access OTA Web Interface

1. Open your browser
2. Navigate to: `http://192.168.x.x/update` (replace with your ESP32's IP)

### Step 3: Upload Firmware

1. Click "Choose File" and select `firmware.bin`
2. Click "Upload Firmware"
3. Wait for upload to complete (progress bar shows status)
4. ESP32 will automatically restart with new firmware

### Web Interface Features

- **Current Version**: Displays firmware version
- **IP Address**: Shows ESP32's current IP
- **Free Heap**: Memory availability check
- **Progress Bar**: Real-time upload progress
- **Auto-restart**: Device restarts automatically after successful update

---

## 🔒 Security Considerations

### Change Default Password

**⚠️ IMPORTANT**: The default OTA password is `bms2024`. You should change it!

1. Edit `include/OTAManager.h`:
```cpp
#define OTA_PASSWORD "your_secure_password_here"
```

2. Edit `platformio.ini`:
```ini
upload_flags =
    --port=3232
    --auth=your_secure_password_here
```

3. Upload the modified firmware

### Network Security

- OTA updates only work on the same WiFi network
- Use a strong WPA2/WPA3 password for your WiFi
- Consider using a separate VLAN for IoT devices
- Monitor your network for unauthorized access

---

## 🛠️ Troubleshooting

### "No OTA port found"

**Problem**: PlatformIO can't find the ESP32

**Solutions**:
- Verify ESP32 is powered on and connected to WiFi
- Check the IP address is correct in `platformio.ini`
- Ensure ESP32 and computer are on the same network
- Try pinging the ESP32: `ping 192.168.x.x`
- Check your router's DHCP client list

### "Authentication Failed"

**Problem**: OTA password mismatch

**Solutions**:
- Verify password in `platformio.ini` matches `OTAManager.h`
- Passwords are case-sensitive
- Check for extra spaces in password string

### "Upload Failed" or "Connection Refused"

**Problem**: OTA service not responding

**Solutions**:
- Restart the ESP32
- Check Serial Monitor for OTA initialization messages
- Verify firewall isn't blocking port 3232
- Ensure only one instance of PlatformIO is uploading

### Web Upload "Update Failed"

**Problem**: Binary file incorrect or corrupted

**Solutions**:
- Ensure you're uploading `firmware.bin` (not other files)
- Rebuild the firmware: `pio run --target clean` then `pio run`
- Check file size is reasonable (typically 0.5-2 MB)
- Try a different browser

### ESP32 Won't Boot After Update

**Problem**: Firmware upload incomplete or corrupted

**Solutions**:
- Connect via serial (USB)
- Upload firmware via serial: `pio run --target upload`
- Check Serial Monitor for error messages
- If necessary, erase flash: `pio run --target erase`

---

## 📝 Best Practices

### Before Updating

1. ✅ **Test new firmware thoroughly** on a development board first
2. ✅ **Backup your configuration** (WiFi credentials, calibration data)
3. ✅ **Note current behavior** to verify after update
4. ✅ **Ensure stable power** supply during update
5. ✅ **Keep serial cable handy** as backup

### During Update

1. ⚠️ **Don't disconnect power** during upload
2. ⚠️ **Don't disconnect from WiFi** during upload
3. ⚠️ **Wait for completion** message before unplugging
4. ⚠️ **Monitor Serial output** for errors

### After Update

1. ✅ **Verify firmware version** in web dashboard or serial output
2. ✅ **Check all features** work as expected
3. ✅ **Monitor for errors** in first few minutes
4. ✅ **Test BMS functionality** (cell reading, charging control, etc.)

---

## 📊 OTA Update Process Flow

```
┌─────────────────────────────────────────────┐
│  1. Build Firmware                          │
│     pio run                                 │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│  2. Choose Upload Method                    │
│     • PlatformIO OTA                       │
│     • Web Interface                         │
└──────────────┬──────────────────────────────┘
               │
        ┌──────┴──────┐
        │             │
        ▼             ▼
   PlatformIO     Web Browser
   OTA Upload     /update
        │             │
        └──────┬──────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│  3. ESP32 Receives Firmware                 │
│     • Authenticates                         │
│     • Verifies binary                       │
│     • Writes to flash                       │
└──────────────┬──────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────┐
│  4. ESP32 Restarts                          │
│     • Boots new firmware                    │
│     • Reconnects to WiFi                    │
│     • Resumes BMS operation                 │
└─────────────────────────────────────────────┘
```

---

## 🔍 Advanced Configuration

### Custom OTA Hostname

Edit `include/OTAManager.h`:
```cpp
#define OTA_HOSTNAME "MY_CUSTOM_BMS"  // Default: "ESP32_BMS"
```

### Custom OTA Port

Edit `include/OTAManager.h`:
```cpp
#define OTA_PORT 8266  // Default: 3232
```

And update `platformio.ini`:
```ini
upload_flags =
    --port=8266
    --auth=bms2024
```

### Disable OTA at Runtime

In your code:
```cpp
otaManager.disable();  // Disable OTA
otaManager.enable();   // Re-enable OTA
```

---

## 📚 Additional Resources

- [PlatformIO OTA Documentation](https://docs.platformio.org/en/latest/platforms/espressif32.html#over-the-air-ota-update)
- [ESP32 OTA Updates](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/ota.html)
- [ArduinoOTA Library](https://github.com/esp8266/Arduino/tree/master/libraries/ArduinoOTA)

---

## 💡 Tips

1. **Faster Development**: Use OTA for quick code iterations without USB cable
2. **Remote Updates**: Update BMS firmware in vehicles without physical access
3. **Version Control**: Tag releases in Git to track firmware versions
4. **Staged Rollout**: Test updates on one device before deploying fleet-wide
5. **Fallback**: Always keep a working serial cable available

---

## ⚠️ Warnings

- ❌ Never interrupt OTA update mid-process
- ❌ Don't update while battery is actively charging
- ❌ Don't update during critical BMS operations
- ❌ Don't use on unstable WiFi connections
- ❌ Don't upload firmware built for different board/chip

---

## 📞 Support

If you encounter issues not covered here:

1. Check Serial Monitor output for error messages
2. Review the troubleshooting section
3. Verify your network configuration
4. Try serial upload as fallback
5. Check GitHub issues for similar problems

---

**Last Updated**: November 2024  
**Firmware Version**: ESP32-S3 BMS v2.0 with OTA Support