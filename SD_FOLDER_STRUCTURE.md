# 🗄️ Folder Structure

The root directory is the device's chip id.
The `SENSORSDATA` folder stores the sensor data organized in a `/<YEAR>/<MONTH>.file_extension` fashion. Only `CSV` and `TXT` files are logged.
<br>

Below is an illustration of the folder structure.
```markdown
ESP_CHIPID/
└── SENSORSDATA/
    └── 2025/
        └── MAY.csv
        └── MAY.txt
    └── failed_send_payloads.txt 
        
```

## 📂 Temporary files

1. `temp_sensor_payload.txt` file is located one level up the root directory. This temporary file stores data that failed to send while attempting to resend data from the `failed_send_payloads.txt` file. Once the resend operation is done, this temporary file is renamed to `failed_send_payloads.txt` in its appropriate `SENSORSDATA` directory.