#include "FS.h"
#include "SD.h"
#include "SPI.h"

bool SD_Init(int CS_PIN = -1)
{
    bool SD_MOUNT = false;
    int timeout = 10;
    Serial.println("Initializing SD card...");

    while (timeout > 0)
    {
        if (CS_PIN == -1)
        {
            SD_MOUNT = SD.begin();
        }
        else
        {
            SD_MOUNT = SD.begin(CS_PIN);
        }

        if (SD_MOUNT)
        {
            Serial.println("SD card mounted successfully");
            break;
        }
        else
        {
            Serial.print(".");
        }
        delay(1000);
        timeout--;
    }
    if (!SD_MOUNT)
    {
        Serial.println("SD card mount failed");
    }
    return SD_MOUNT;
}

void createDir(fs::FS &fs, const char *path)
{
    Serial.printf("Creating Dir: %s\n", path);
    if (fs.mkdir(path))
    {
        Serial.println("Dir created");
    }
    else
    {
        Serial.println("mkdir failed");
    }
}

String readFile(fs::FS &fs, const char *path)
{
    Serial.printf("Reading file: %s\n", path);
    String file_contents = "";
    File file = fs.open(path);
    if (!file)
    {
        Serial.println("Failed to open file for reading");
        return "";
    }

    Serial.print("Read from file: ");
    while (file.available())
    {
        file_contents += file.read();
    }
    file.close();
    return file_contents;
}

void writeFile(fs::FS &fs, const char *path, const char *message)
{
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        Serial.println("File written");
    }
    else
    {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message, bool newline = true)
{
    Serial.print("Appending to file: ");
    Serial.println(path);
    File file = fs.open(path, FILE_APPEND);
    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message))
    {
        Serial.println("Message appended");
        if (newline)
        {
            file.println();
        }
    }
    else
    {
        Serial.println("Append failed");
    }
    file.close();
}

bool SDattached()
{
    if (SD.cardType() == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return false;
    }

    Serial.print("SD Card Type: ");
    switch (SD.cardType())
    {
    case CARD_MMC:
        Serial.println("MMC");
        break;
    case CARD_SD:
        Serial.println("SDSC");
        break;
    case CARD_SDHC:
        Serial.println("SDHC");
        break;
    default:
        Serial.println("UNKNOWN");
        break;
    }

    Serial.printf("SD Card Size: %lluMB\n", SD.cardSize() / (1024 * 1024));
    return true;
}

String readLine(fs::FS &fs, const char *path, int &next_char, int &from, bool closefile = true)
{
    String line = "";
    char c;

    File file = fs.open(path);

    if (!file)
    {
        Serial.println("Failed to open file to read line");
        return "";
    }

    file.seek(from);

    while (file.available())
    {
        c = file.read();
        if (c == '\n')
        {
            break;
        }
        // Skip carriage return
        if (c != '\r')
        {
            line += c;
        }
    }

    from = file.position();
    // file.seek((last_read_index), SeekMode::SeekSet);
    next_char = file.read();

    if (closefile)
    {
        file.close();
    }

    return line;
}

void updateFileContents(fs::FS &fs, const char *file_to_updated, const char *temp_file)
{

    fs.remove(file_to_updated);
    fs.rename(temp_file, file_to_updated);
}

void closeFile(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    file.close();
}