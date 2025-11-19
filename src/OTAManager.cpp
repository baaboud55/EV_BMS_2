/*
 * OTAManager.cpp - Implementation of OTA firmware update management
 */

#include "OTAManager.h"
#include <SPIFFS.h>

// Static instance pointer for callbacks
static OTAManager* _instance = nullptr;

OTAManager::OTAManager() 
    : _enabled(true), 
      _updateInProgress(false), 
      _updateProgress(0),
      _lastError("") {
    _instance = this;
}

bool OTAManager::begin(const char* hostname, const char* password) {
    if (!WiFi.isConnected()) {
        _lastError = "WiFi not connected";
        Serial.println("[OTA] ERROR: WiFi not connected!");
        return false;
    }

    // Configure ArduinoOTA
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(password);
    ArduinoOTA.setPort(OTA_PORT);

    // Set up callbacks
    ArduinoOTA.onStart([]() {
        if (_instance) {
            _instance->_updateInProgress = true;
            _instance->_updateProgress = 0;
        }
        
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        
        Serial.println("[OTA] Starting update: " + type);
        
        // If updating SPIFFS, unmount it
        if (ArduinoOTA.getCommand() == U_SPIFFS) {
            SPIFFS.end();
        }
    });

    ArduinoOTA.onEnd([]() {
        if (_instance) {
            _instance->_updateInProgress = false;
            _instance->_updateProgress = 100;
        }
        Serial.println("\n[OTA] Update complete!");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        if (_instance) {
            _instance->_updateProgress = (progress * 100) / total;
        }
        Serial.printf("[OTA] Progress: %u%%\r", (progress * 100) / total);
    });

    ArduinoOTA.onError([](ota_error_t error) {
        String errorMsg;
        Serial.printf("[OTA] Error[%u]: ", error);
        
        if (_instance) {
            _instance->_updateInProgress = false;
        }
        
        if (error == OTA_AUTH_ERROR) {
            errorMsg = "Auth Failed";
        } else if (error == OTA_BEGIN_ERROR) {
            errorMsg = "Begin Failed";
        } else if (error == OTA_CONNECT_ERROR) {
            errorMsg = "Connect Failed";
        } else if (error == OTA_RECEIVE_ERROR) {
            errorMsg = "Receive Failed";
        } else if (error == OTA_END_ERROR) {
            errorMsg = "End Failed";
        } else {
            errorMsg = "Unknown Error";
        }
        
        if (_instance) {
            _instance->_lastError = errorMsg;
        }
        
        Serial.println(errorMsg);
    });

    // Start OTA service
    ArduinoOTA.begin();
    
    Serial.println("[OTA] Service started");
    Serial.printf("[OTA] Hostname: %s\n", hostname);
    Serial.printf("[OTA] IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[OTA] Port: %d\n", OTA_PORT);
    
    return true;
}

void OTAManager::handle() {
    if (_enabled && WiFi.isConnected()) {
        ArduinoOTA.handle();
    }
}

void OTAManager::setupWebOTA(AsyncWebServer* server) {
    if (!server) return;

    // Serve OTA update page
    server->on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BMS Firmware Update</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            max-width: 600px;
            margin: 50px auto;
            padding: 20px;
            background: #f5f5f5;
        }
        .container {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #333;
            margin-bottom: 20px;
        }
        .info {
            background: #e3f2fd;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .warning {
            background: #fff3e0;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
            border-left: 4px solid #ff9800;
        }
        input[type="file"] {
            margin: 20px 0;
            padding: 10px;
            width: 100%;
            border: 2px dashed #ccc;
            border-radius: 5px;
        }
        button {
            background: #4CAF50;
            color: white;
            padding: 12px 30px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
            width: 100%;
        }
        button:hover {
            background: #45a049;
        }
        button:disabled {
            background: #cccccc;
            cursor: not-allowed;
        }
        .progress-container {
            display: none;
            margin-top: 20px;
        }
        .progress-bar {
            width: 100%;
            height: 30px;
            background: #f0f0f0;
            border-radius: 5px;
            overflow: hidden;
        }
        .progress-fill {
            height: 100%;
            background: #4CAF50;
            width: 0%;
            transition: width 0.3s;
            display: flex;
            align-items: center;
            justify-content: center;
            color: white;
            font-weight: bold;
        }
        .status {
            margin-top: 10px;
            padding: 10px;
            border-radius: 5px;
            display: none;
        }
        .status.success {
            background: #c8e6c9;
            color: #2e7d32;
        }
        .status.error {
            background: #ffcdd2;
            color: #c62828;
        }
        a {
            color: #2196F3;
            text-decoration: none;
        }
        a:hover {
            text-decoration: underline;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔄 BMS Firmware Update</h1>
        
        <div class="info">
            <strong>Current Version:</strong> <span id="version">Loading...</span><br>
            <strong>IP Address:</strong> <span id="ip">)" + WiFi.localIP().toString() + R"(</span><br>
            <strong>Free Heap:</strong> <span id="heap">)" + String(ESP.getFreeHeap()) + R"( bytes</span>
        </div>
        
        <div class="warning">
            ⚠️ <strong>Warning:</strong> Do not disconnect power during update. The device will restart automatically after the update completes.
        </div>
        
        <form id="uploadForm" enctype="multipart/form-data">
            <input type="file" name="firmware" id="firmware" accept=".bin" required>
            <button type="submit" id="uploadBtn">Upload Firmware</button>
        </form>
        
        <div class="progress-container" id="progressContainer">
            <div class="progress-bar">
                <div class="progress-fill" id="progressFill">0%</div>
            </div>
        </div>
        
        <div class="status" id="status"></div>
        
        <div style="margin-top: 20px; text-align: center;">
            <a href="/">← Back to Dashboard</a>
        </div>
    </div>
    
    <script>
        document.getElementById('version').textContent = 'ESP32-S3 BMS v2.0';
        
        document.getElementById('uploadForm').addEventListener('submit', function(e) {
            e.preventDefault();
            
            const fileInput = document.getElementById('firmware');
            const file = fileInput.files[0];
            
            if (!file) {
                alert('Please select a firmware file');
                return;
            }
            
            if (!file.name.endsWith('.bin')) {
                alert('Please select a valid .bin firmware file');
                return;
            }
            
            const formData = new FormData();
            formData.append('firmware', file);
            
            const uploadBtn = document.getElementById('uploadBtn');
            const progressContainer = document.getElementById('progressContainer');
            const progressFill = document.getElementById('progressFill');
            const statusDiv = document.getElementById('status');
            
            uploadBtn.disabled = true;
            uploadBtn.textContent = 'Uploading...';
            progressContainer.style.display = 'block';
            statusDiv.style.display = 'none';
            
            const xhr = new XMLHttpRequest();
            
            xhr.upload.addEventListener('progress', function(e) {
                if (e.lengthComputable) {
                    const percent = Math.round((e.loaded / e.total) * 100);
                    progressFill.style.width = percent + '%';
                    progressFill.textContent = percent + '%';
                }
            });
            
            xhr.addEventListener('load', function() {
                if (xhr.status === 200) {
                    statusDiv.className = 'status success';
                    statusDiv.textContent = '✓ Update successful! Device is restarting...';
                    statusDiv.style.display = 'block';
                    
                    setTimeout(function() {
                        window.location.href = '/';
                    }, 5000);
                } else {
                    statusDiv.className = 'status error';
                    statusDiv.textContent = '✗ Update failed: ' + xhr.responseText;
                    statusDiv.style.display = 'block';
                    uploadBtn.disabled = false;
                    uploadBtn.textContent = 'Upload Firmware';
                }
            });
            
            xhr.addEventListener('error', function() {
                statusDiv.className = 'status error';
                statusDiv.textContent = '✗ Upload failed. Please try again.';
                statusDiv.style.display = 'block';
                uploadBtn.disabled = false;
                uploadBtn.textContent = 'Upload Firmware';
            });
            
            xhr.open('POST', '/update', true);
            xhr.send(formData);
        });
    </script>
</body>
</html>
        )";
        
        request->send(200, "text/html", html);
    });

    // Handle firmware upload
    server->on("/update", HTTP_POST, 
        [](AsyncWebServerRequest *request) {
            bool shouldReboot = !Update.hasError();
            AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", 
                shouldReboot ? "OK" : "FAIL");
            response->addHeader("Connection", "close");
            request->send(response);
            
            if (shouldReboot) {
                Serial.println("[OTA] Update successful, restarting...");
                delay(1000);
                ESP.restart();
            }
        },
        [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
            if (!index) {
                Serial.printf("[OTA] Update Start: %s\n", filename.c_str());
                
                // Start update
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                    request->send(500, "text/plain", "Update failed to begin");
                    return;
                }
            }
            
            // Write chunk
            if (Update.write(data, len) != len) {
                Update.printError(Serial);
                request->send(500, "text/plain", "Update write failed");
                return;
            }
            
            // Complete update
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] Update Success: %u bytes\n", index + len);
                } else {
                    Update.printError(Serial);
                    request->send(500, "text/plain", "Update end failed");
                }
            }
        }
    );
}

void OTAManager::restart() {
    Serial.println("[OTA] Restarting ESP32...");
    delay(1000);
    ESP.restart();
}

// Global instance
OTAManager otaManager;