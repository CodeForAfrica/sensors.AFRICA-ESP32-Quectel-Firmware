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

static String urlDecode(const String &input)
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