#include <LittleFS.h>
#include <Update.h>
#include <StreamString.h>

bool otaUpdateFromLittleFS(const char *firmwarePath, const char *backupPath)
{
    if (!LittleFS.begin())
        return false;

    // Backup current firmware if backup doesn't exist
    bool backupExists = LittleFS.exists(backupPath);
    if (!backupExists)
    {
        // This is a placeholder: ESP32 cannot read its own firmware directly.
        // You must upload the backup firmware to LittleFS beforehand.
        // Alternatively, use dual OTA partitions for true rollback.
        Serial.println("No backup fimware found!");
    }

    File firmware = LittleFS.open(firmwarePath, "r");
    if (!firmware)
        return false;

    size_t firmwareSize = firmware.size();
    size_t freeSpace = ESP.getFreeSketchSpace();
    Serial.print("New firmware size: ");
    Serial.print(firmwareSize);
    Serial.print(" bytes");
    Serial.print("\tFree space: ");
    Serial.print(freeSpace);
    Serial.println(" bytes");
    if (firmwareSize > freeSpace)
    {
        Serial.println("Firmware size is large than the free space");
        // Delete the new firmware
        LittleFS.remove(firmwarePath);
        return false;
    }

    if (!Update.begin(firmwareSize))
    {
        firmware.close();
        Serial.println("Failed to begin update");
        return false; // Indicate update failed, but rollback attempted
    }

    uint8_t buf[1024];
    while (firmware.available())
    {
        size_t len = firmware.read(buf, sizeof(buf));
        if (Update.write(buf, len) != len)
        {
            firmware.close();
            StreamString error;
            Update.printError(error);
            Serial.println("Error performing update write");
            return false; // Indicate update failed, rollback will be handled after update attempt
        }
    }
    if (!Update.end(true))
    {
        firmware.close();
        StreamString error;
        Update.printError(error);

        // Try to restore backup
        if (backupExists)
        {
            Serial.println("Updating to new firmware failed! Attempting to rollback to previous firmware...");
            File backup = LittleFS.open(backupPath, "r");
            if (!backup)
                return false;
            size_t backupSize = backup.size();
            if (!Update.begin(backupSize))
            {
                backup.close();
                return false;
            }
            while (backup.available())
            {
                size_t len = backup.read(buf, sizeof(buf));
                if (Update.write(buf, len) != len)
                {
                    backup.close();
                    return false;
                }
            }
            if (!Update.end(true))
            {
                backup.close();
                Serial.println("Rollback done");
                return false;
            }
            backup.close();
            ESP.restart();
        }
        return false; // Indicate update failed, but rollback attempted
    }

    firmware.close();

    Serial.println("New firmware updated");

    // delete rollback file and rename new file as rollback
    if (backupExists)
    {
        updateFileContents(LittleFS, backupPath, firmwarePath);
    }
    else
    {
        LittleFS.rename(firmwarePath, "/current_firmware.bin");
    }

    Serial.println("Restarting ESP");
    ESP.restart();
    return true;
}
