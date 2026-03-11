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
