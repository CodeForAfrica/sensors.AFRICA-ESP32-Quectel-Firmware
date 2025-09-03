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
    Serial.print("New firmware size: ");
    Serial.println(firmwareSize);
    if (!Update.begin(firmwareSize))
    {
        firmware.close();
        // StreamString error;
        // Update.printError(error);
        // Try to restore backup
        // File backup = LittleFS.open(backupPath, "r");
        // if (!backup)
        //     return false;
        // size_t backupSize = backup.size();
        // if (!Update.begin(backupSize))
        // {
        //     backup.close();
        //     return false;
        // }
        // uint8_t buf[1024];
        // while (backup.available())
        // {
        //     size_t len = backup.read(buf, sizeof(buf));
        //     if (Update.write(buf, len) != len)
        //     {
        //         backup.close();
        //         return false;
        //     }
        // }
        // if (!Update.end(true))
        // {
        //     backup.close();
        //     return false;
        // }
        // backup.close();
        // ESP.restart();
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
            // StreamString error;
            // Update.printError(error);
            Serial.println("Error performing update write");
            return false; // Indicate update failed, rollback will be handled after update attempt
        }
    }
    if (!Update.end(true))
    {
        firmware.close();
        // StreamString error;
        // Update.printError(error);

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
        LittleFS.rename(firmwarePath, "current_firmware.bin");
    }

    Serial.println("Restarting ESP");
    ESP.restart();
    return true;
}

// //! for reference
// static bool SPIFFSAutoUpdate(String newFirmware, String newMD5)
// {

//     if (!SPIFFS.exists(newFirmware))
//     {
//         Serial.print("No Firmware file found, looking for: ");
//         Serial.println(newFirmware);
//         return false;
//     }
//     File updateFile = SPIFFS.open(newFirmware, "r");
//     if (!updateFile)
//     {
//         Serial.print("Failed to open : ");
//         Serial.print(newFirmware);
//         return false;
//     }

//     unsigned int free_space = ESP.getFreeSketchSpace();
//     Serial.print("EsP free sketch space: ");
//     Serial.println(free_space);

//     if (updateFile.size() >= ESP.getFreeSketchSpace())
//     {
//         Serial.println("Cannot update, Firmware too large");
//         return false;
//     }
//     if (!Update.begin(updateFile.size(), U_FLASH))
//     {
//         StreamString error;
//         Update.printError(error);

//         Serial.print("Update.begin returned: "),
//             Serial.println(error);
//         return false;
//     }

//     // set MD5
//     Update.setMD5(newMD5.c_str());

//     if (Update.writeStream(updateFile) != updateFile.size())
//     {
//         StreamString error;
//         Update.printError(error);

//         Serial.print("Update.writeStream returned: ");
//         Serial.print(error);
//         return false;
//     }
//     updateFile.close();

//     if (!Update.end())
//     {
//         StreamString error;
//         Update.printError(error);

//         Serial.println("Update.end() returned: ");
//         Serial.print(error);
//         return false;
//     }

//     Serial.println("Erasing SDK config.");
//     ESP.eraseConfig();

//     Serial.println("Finished successfully.. Rebooting!");
//     delay(500);
//     ESP.restart();
//     return true;
// }