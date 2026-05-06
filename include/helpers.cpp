#include "helpers.h"

/**
    @brief Validate JSON data
    @param input : JSON data to validate
    @return : true if valid, false otherwise
    @note : //! This function is not full proof. Raw strings that don't look like incomplete or empty JSON are validated.
**/
bool validateJson(const char *input)
{
    JsonDocument doc, filter;
    return deserializeJson(doc, input, DeserializationOption::Filter(filter)) == DeserializationError::Ok;
}

String urlDecode(const String &input)
{
    String output;
    unsigned int input_length = input.length(); // Worst case: no encoding, so length is the same
    output.reserve(input_length);
    for (size_t i = 0; i < input_length; i++)
    {
        char c = input.charAt(i);
        if (c == '+')
        {
            output += ' ';
        }
        else if (c == '%' && i + 2 < input_length)
        {
            char hex[3] = {input.charAt(i + 1), input.charAt(i + 2), '\0'};
            char *end = nullptr;
            long decoded = strtol(hex, &end, 16);
            if (end != hex)
            {
                output += (char)decoded;
                i += 2;
            }
            else
            {
                output += c;
            }
        }
        else
        {
            output += c;
        }
    }
    return output;
}

String normalizePath(String p)
{
    while (p.indexOf("//") >= 0)
        p.replace("//", "/");
    if (!p.startsWith("/"))
        p = "/" + p;
    return p;
}

bool isPathTraversal(const String &p)
{
    return p.indexOf("..") >= 0;
}

bool parseURL(const char *url, char *host, size_t hostLen,
              uint16_t &port, char *path, size_t pathLen,
              bool &useSSL)
{
    useSSL = false;
    port = 80;

    const char *urlPtr = url;

    // Check for protocol
    if (strncmp(urlPtr, "http://", 7) == 0)
    {
        urlPtr += 7;
        port = 80;
    }
    else if (strncmp(urlPtr, "https://", 8) == 0)
    {
        urlPtr += 8;
        port = 443;
        useSSL = true;
    }

    // Find host end (either ':', '/', or end of string)
    const char *hostEnd = urlPtr;
    while (*hostEnd && *hostEnd != ':' && *hostEnd != '/')
    {
        hostEnd++;
    }

    // Copy host
    size_t hostLength = hostEnd - urlPtr;
    if (hostLength >= hostLen)
        hostLength = hostLen - 1;
    strncpy(host, urlPtr, hostLength);
    host[hostLength] = '\0';

    urlPtr = hostEnd;

    // Check for port
    if (*urlPtr == ':')
    {
        urlPtr++;
        port = atoi(urlPtr);
        while (*urlPtr && *urlPtr != '/')
            urlPtr++;
    }

    // Get path
    if (*urlPtr == '\0' || *urlPtr != '/')
    {
        strcpy(path, "/");
    }
    else
    {
        strncpy(path, urlPtr, pathLen - 1);
        path[pathLen - 1] = '\0';
    }

    return true;
}
