#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <SD.h>

// ============================================================================
// GLOBAL FILE OBJECT
// ============================================================================

extern File dataFile;  // SD card file handle for data logging

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

/**
 * @brief Initialize SD card and create data file with headers
 * @return true if successful, false otherwise
 */
bool initializeDataLogging();

/**
 * @brief Write sensor data to SD card in CSV format
 */
void logDataToSD();

/**
 * @brief Print sensor data to serial monitor for debugging
 */
void printDebugData();

#endif  // DATA_LOGGER_H