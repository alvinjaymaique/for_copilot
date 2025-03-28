/**
 * @file fingerprint.h
 * @brief Fingerprint sensor driver for ESP32
 *
 * ## 1. Introduction
 * This driver provides an interface for the fingerprint sensor, enabling fingerprint 
 * enrollment, matching, and template storage. It is designed for embedded systems using the 
 * ESP32 microcontroller, providing an event-driven architecture for efficient processing.
 *
 * ### 1.1 Supported Platforms
 * - ESP32-based development boards
 * - ESP-IDF (Espressif IoT Development Framework)
 *
 * ### 1.2 Compiler and Toolchain
 * - GCC (ESP32 toolchain)
 * - Compatible with ESP-IDF v4.x or later
 *
 * ### 1.3 Dependencies
 * This driver requires the following ESP-IDF components:
 * - `esp_log.h` – Logging utilities for debugging and event tracking
 * - `driver/uart.h` – UART driver for serial communication with the fingerprint module
 * - `esp_err.h` – Standard ESP-IDF error handling
 * - `freertos/FreeRTOS.h` and `freertos/task.h` – FreeRTOS support for asynchronous processing
 * - `string.h` – Standard C library for string manipulation
 *
 *
 * ## 2. Hardware Interface Description
 * ### 2.1 UART
 * - Default baud rate: 57600 bps (8 data bits, 1 stop bit, no parity)
 * - Baud rate adjustable: 9600 to 115200 bps
 * - Direct connection to MCU (3.3V) or use RS232 level converter for PC
 * 
 * ### 2.2 Power-on Sequence:
 * 1. Host receives fingerprint module (FPM) interrupt wake-up signal.
 * 2. Host powers on Vmcu (MCU power supply) **before** initializing UART.
 * 3. After communication completes, pull down serial signal lines **before** powering off Vmcu.
 * 
 * ### 2.3 Connection
 * - **TX** → ESP32 GPIO17
 * - **RX** → ESP32 GPIO16
 * - **VCC** → 3.3V
 * - **GND** → GND
 *
 * ## 3. Event-driven System
 * This system operates on an event-driven architecture where each key operation, such as image capture, feature extraction, or match status, triggers specific events. The events help the application know when an operation is complete or has failed. 
 * The following events are generated based on the corresponding system actions.
 *
 * ### 3.1 Event Handler
 * @brief Event handler for processing fingerprint events.
 *
 * This function processes various fingerprint-related events and logs messages 
 * based on the event type. It can be used as a sample for handling fingerprint events 
 * in the application. The events are processed based on the `fingerprint_event_type_t` 
 * enumeration, and corresponding messages are logged using the ESP-IDF logging system.
 *
 * @param event The fingerprint event that occurred. This can be one of the following:
 *      - `EVENT_FINGER_DETECTED`: Triggered when a finger is detected.
 *      - `EVENT_IMAGE_CAPTURED`: Triggered when the fingerprint image is captured successfully.
 *      - `EVENT_FEATURE_EXTRACTED`: Triggered when the fingerprint features are extracted successfully.
 *      - `EVENT_MATCH_SUCCESS`: Triggered when a fingerprint match is successful.
 *      - `EVENT_MATCH_FAIL`: Triggered when a fingerprint mismatch occurs.
 *      - `EVENT_ERROR`: Triggered for general errors during fingerprint processing.
 *
 * This function logs the corresponding message for each event. It can be modified or 
 * extended as needed for custom event handling in the application.
 *
 * @note To use this function in the application, assign it to the global event handler 
 *       `g_fingerprint_event_handler` in the `app_main.c`.
 *
 * @code
 * // Sample usage of the event handler
 * void handle_fingerprint_event(fingerprint_event_t event) {
 *     switch (event.type) {
 *         case EVENT_SCANNER_READY:
 *             ESP_LOGI("Fingerprint", "Fingerprint scanner is ready for operation. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_FINGER_DETECTED:
 *             ESP_LOGI("Fingerprint", "Finger detected! Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_IMAGE_CAPTURED:
 *             ESP_LOGI("Fingerprint", "Fingerprint image captured successfully! Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_FEATURE_EXTRACTED:
 *             ESP_LOGI("Fingerprint", "Fingerprint features extracted successfully! Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_MATCH_SUCCESS:
 *             ESP_LOGI("MATCH SUCCESS", "Fingerprint match successful! Status: 0x%02X", event.status);
 *             ESP_LOGI("MATCH SUCCESS", "Match found at Enrollee ID: %d", event.data.match_info.template_id);
 *             ESP_LOGI("MATCH SUCCESS", "Match score: %d", event.data.match_info.match_score);
 *             break;
 *         case EVENT_MATCH_FAIL:
 *             ESP_LOGE("Fingerprint", "Fingerprint mismatch. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_ERROR:
 *             ESP_LOGE("Fingerprint", "An error occurred during fingerprint processing. Status: 0x%02X", event.status);
 *             ESP_LOGE("Fingerprint", "Command: 0x%02X", event.command);
 *             break;
 *         case EVENT_TEMPLATE_UPLOADED:
 *             ESP_LOGI("TEMPLATE UPLOADED", "Template uploaded successfully.");
 *             if (event.multi_packet != NULL) {
 *                 ESP_LOGI("TEMPLATE UPLOADED", "Multi-packet data available: %d packets", event.multi_packet->count);
 *                 for (size_t i = 0; i < event.multi_packet->count; i++) {
 *                     if (event.multi_packet->packets[i] != NULL) {
 *                         ESP_LOGI("TEMPLATE UPLOADED", "Packet %d: ID=0x%02X, Address=0x%08X, Length=%d, Checksum=0x%04X",
 *                                  i, 
 *                                  event.multi_packet->packets[i]->packet_id,
 *                                  (unsigned int)event.multi_packet->packets[i]->address,
 *                                  event.multi_packet->packets[i]->length,
 *                                  (unsigned int)event.multi_packet->packets[i]->checksum);
 *                     }
 *                     vTaskDelay(pdMS_TO_TICKS(50));  // Prevent watchdog trigger
 *                 }
 *             } else {
 *                 ESP_LOGW("TEMPLATE UPLOADED", "No multi-packet data available.");
 *             }
 *             break;
 *         case EVENT_ENROLLMENT_COMPLETE:
 *             ESP_LOGI("ENROLLMENT COMPLETE", "Fingerprint enrollment completed. Status: 0x%02X", event.status);
 *             ESP_LOGI("ENROLLMENT INFO", "Enrollment ID: %d", event.data.enrollment_info.template_id);
 *             ESP_LOGI("ENROLLMENT INFO", "Have Duplicate: %d", event.data.enrollment_info.is_duplicate);	
 *             ESP_LOGI("ENROLLMENT INFO", "Attempts: %d", event.data.enrollment_info.attempts);	
 *             break;
 *         case EVENT_ENROLLMENT_FAIL:
 *             tag = "ENROLLMENT FAILED";
 *             ESP_LOGE(tag, "Fingerprint enrollment process failed. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_TEMPLATE_DELETED:
 *             tag = "TEMPLATE DELETED";
 *             ESP_LOGI(tag, "Fingerprint template deleted successfully. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_TEMPLATE_DELETE_FAIL:
 *             tag = "TEMPLATE DELETE FAILED";
 *             ESP_LOGE(tag, "Failed to delete fingerprint template. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_TEMPLATE_RESTORED_SUCCESSUL:
 *             tag = "TEMPLATE STORED";
 *             ESP_LOGI(tag, "Fingerprint template stored successfully. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_TEMPLATE_LOADED:
 *             ESP_LOGI(TAG, "Template loaded successfully. Status: 0x%02X", event.status);
 *             break;
 *         case EVENT_DB_CLEARED:
 *             tag = "DATABASE CLEARED";
 *             ESP_LOGI(tag, "Fingerprint database cleared successfully. Status: 0x%02X", event.status);
 *             break;
 *         default:
 *             ESP_LOGI("UNKNOWN EVENT", "Unknown event triggered. Status: 0x%02X", event.status);
 *             ESP_LOGI("UNKNOWN EVENT", "Event Type: 0x%02X", event.type);	
 *             break;
 *     }
 * }
 * @endcode
 * 
 * ## 4. Usage Guide
 * This section provides an overview of how to use the high-level functions in the fingerprint library for common operations.
 *
 * ### 4.1 Initialization
 * Initialize the fingerprint module before using any other functions:
 *
 * @code
 * #include "fingerprint.h"
 *
 * void app_main() {
 *     // Initialize with default settings
 *     esp_err_t err = fingerprint_init();
 *     if (err == ESP_OK) {
 *         ESP_LOGI(TAG, "Fingerprint module initialized successfully");
 *         
 *         // Register an event handler for processing events
 *         register_fingerprint_event_handler(my_event_handler);
 *     }
 * }
 * @endcode
 *
 * ### 4.2 Fingerprint Enrollment
 * Enroll a new fingerprint using the high-level API:
 *
 * @code
 * // Enroll a new fingerprint at location 1
 * esp_err_t err = enroll_fingerprint(1);
 * if (err == ESP_OK) {
 *     ESP_LOGI(TAG, "Fingerprint enrolled successfully");
 * } else {
 *     ESP_LOGE(TAG, "Fingerprint enrollment failed: %s", esp_err_to_name(err));
 * }
 * @endcode
 *
 * ### 4.3 Fingerprint Verification
 * Verify a fingerprint against the database:
 *
 * @code
 * esp_err_t err = verify_fingerprint();
 * if (err == ESP_OK) {
 *     // Match will be reported in the event handler
 *     ESP_LOGI(TAG, "Fingerprint verification process initiated");
 * }
 * 
 * // In the event handler:
 * case EVENT_MATCH_SUCCESS:
 *     ESP_LOGI(TAG, "Match found at template ID: %d with score: %d", 
 *              event.data.match_info.template_id,
 *              event.data.match_info.match_score);
 *     break;
 * @endcode
 *
 * ### 4.4 Template Management
 * Manage fingerprint templates with these high-level functions:
 *
 * @code
 * // Get the number of enrolled templates
 * err = get_enrolled_count();
 * // Count will be available in EVENT_TEMPLATE_COUNT
 *
 * // Delete a fingerprint template
 * err = delete_fingerprint(5);  // Delete template at location 5
 *
 * // Check if a template exists
 * err = fingerprint_check_template_exists(3);
 *
 * // Back up a template from the module to the ESP32
 * err = backup_template(2);  // Will trigger EVENT_TEMPLATE_UPLOADED
 *
 * // Clear the entire database
 * err = clear_database();
 * @endcode
 *
 * ### 4.5 System Configuration
 * Configure the fingerprint module:
 *
 * @code
 * // Change the module address
 * err = fingerprint_set_address(0x12345678, DEFAULT_FINGERPRINT_ADDRESS);
 *
 * // Read system parameters (baud rate, security level, etc.)
 * err = read_system_parameters();
 * // Parameters will be available in EVENT_SYS_PARAMS_READ
 *
 * // Read the module information page
 * err = read_info_page();
 * // Information will be available in EVENT_INFO_PAGE_READ
 * @endcode
 *
 * ### 4.6 Advanced Operation: Wait for Finger
 * Wait for a finger to be placed on the scanner:
 *
 * @code
 * // Set the operation mode
 * fingerprint_set_operation_mode(FINGER_OP_VERIFY);
 *
 * // Wait for up to 10 seconds for a finger
 * err = fingerprint_wait_for_finger(10000);
 * if (err == ESP_OK) {
 *     ESP_LOGI(TAG, "Finger detected and processed");
 * } else if (err == ESP_ERR_TIMEOUT) {
 *     ESP_LOGW(TAG, "Timeout waiting for finger");
 * }
 * 
 * 
 * ## 5. Creating Custom High-Level Functions
 * This section explains how to extend the library by creating your own high-level functions
 * using the mid-level command interface.
 *
 * ### 5.1 Mid-Level Command Functions
 * The library provides these key mid-level functions for direct module communication:
 * 
 * - `fingerprint_set_command()` - Configures a command packet with parameters
 * - `fingerprint_send_command()` - Transmits a command to the fingerprint module
 * - `fingerprint_calculate_checksum()` - Computes the checksum for a packet
 * - `initialize_event_group()` - Creates an event group for synchronization
 * - `cleanup_event_group()` - Cleans up the event group when finished
 *
 * ### 5.2 Creating a High-Level Function
 * A typical high-level function follows this pattern:
 *
 * @code
 * esp_err_t my_custom_operation(uint16_t parameter) {
 *     esp_err_t err;
 *     EventBits_t bits;
 *     
 *     // 1. Initialize event group for synchronization
 *     err = initialize_event_group();
 *     if (err != ESP_OK) {
 *         return err;
 *     }
 *     
 *     // 2. Configure the command with appropriate parameters
 *     uint8_t params[3] = {
 *         0x01,                      // Buffer ID
 *         (parameter >> 8) & 0xFF,   // High byte of parameter
 *         parameter & 0xFF           // Low byte of parameter
 *     };
 *     
 *     fingerprint_set_command(&PS_MyCommand, FINGERPRINT_CMD_CUSTOM, params, sizeof(params));
 *     
 *     // 3. Send the command
 *     err = fingerprint_send_command(&PS_MyCommand, DEFAULT_FINGERPRINT_ADDRESS);
 *     if (err != ESP_OK) {
 *         ESP_LOGE(TAG, "Failed to send command");
 *         cleanup_event_group();
 *         return err;
 *     }
 *     
 *     // 4. Wait for operation to complete (with timeout)
 *     bits = xEventGroupWaitBits(
 *         enroll_event_group,
 *         ENROLL_BIT_SUCCESS | ENROLL_BIT_FAIL,  // Bits to wait for
 *         pdTRUE,                               // Clear bits after
 *         pdFALSE,                              // Wait for ANY bit
 *         pdMS_TO_TICKS(5000)                   // Timeout (5 seconds)
 *     );
 *     
 *     // 5. Check results and return appropriate status
 *     cleanup_event_group();
 *     
 *     if (bits & ENROLL_BIT_SUCCESS) {
 *         return ESP_OK;
 *     } else if (bits & ENROLL_BIT_FAIL) {
 *         return ESP_FAIL;
 *     } else {
 *         return ESP_ERR_TIMEOUT;
 *     }
 * }
 * @endcode
 *
 * ### 5.3 Processing Events
 * Your operation will trigger events that arrive in your registered event handler:
 *
 * @code
 * void my_event_handler(fingerprint_event_t event) {
 *     switch (event.type) {
 *         case EVENT_CUSTOM_SUCCESS:
 *             // Set success bit to unblock the waiting function
 *             if (enroll_event_group) {
 *                 xEventGroupSetBits(enroll_event_group, ENROLL_BIT_SUCCESS);
 *             }
 *             break;
 *             
 *         case EVENT_CUSTOM_FAIL:
 *             // Set fail bit to unblock the waiting function
 *             if (enroll_event_group) {
 *                 xEventGroupSetBits(enroll_event_group, ENROLL_BIT_FAIL);
 *             }
 *             break;
 *     }
 * }
 * @endcode
 *
 * ### 5.4 Handling Multi-Step Operations
 * For more complex operations requiring multiple commands:
 *
 * @code
 * esp_err_t complex_operation(void) {
 *     esp_err_t err;
 *     
 *     // Initialize event group only once for the entire sequence
 *     err = initialize_event_group();
 *     if (err != ESP_OK) return err;
 *     
 *     // Step 1: Send first command
 *     fingerprint_set_command(&PS_FirstCommand, FINGERPRINT_CMD_FIRST, NULL, 0);
 *     err = fingerprint_send_command(&PS_FirstCommand, DEFAULT_FINGERPRINT_ADDRESS);
 *     if (err != ESP_OK) {
 *         cleanup_event_group();
 *         return err;
 *     }
 *     
 *     // Wait for first command to complete
 *     EventBits_t bits = xEventGroupWaitBits(
 *         enroll_event_group,
 *         ENROLL_BIT_SUCCESS | ENROLL_BIT_FAIL,
 *         pdTRUE, pdFALSE, pdMS_TO_TICKS(3000)
 *     );
 *     
 *     // Check first command result
 *     if (!(bits & ENROLL_BIT_SUCCESS)) {
 *         cleanup_event_group();
 *         return ESP_FAIL;
 *     }
 *     
 *     // Step 2: Send second command
 *     // ... similar pattern ...
 *     
 *     // Clean up when done
 *     cleanup_event_group();
 *     return ESP_OK;
 * }
 * @endcode
 * 
 * 
 *  * ## 6. Setting Up the Fingerprint Scanner
 * This section provides a step-by-step guide for setting up and testing the ZW111 fingerprint scanner with the ESP32.
 *
 * ### 6.1 Hardware Connections
 * Connect the fingerprint scanner to your ESP32 development board:
 *
 * | Fingerprint Scanner | ESP32         |
 * |---------------------|---------------|
 * | VCC                 | 3.3V          |
 * | GND                 | GND           |
 * | TX                  | GPIO5 (RX)    |
 * | RX                  | GPIO6 (TX)    |
 * | INT                 | GPIO15        | 
 *
 * @note The default pins are GPIO5 for RX and GPIO6 for TX, but these can be changed using the `fingerprint_set_pins()` function.
 * The INT pin is connected to GPIO15 and is used for interrupt-based finger detection.
 *
 * ### 6.2 Software Initialization
 * Include the following code in your application to initialize the fingerprint scanner:
 *
 * @code
 * #include "fingerprint.h"
 *
 * void app_main() {
 *     // Optional: Change default communication pins if needed
 *     // fingerprint_set_pins(GPIO_NUM_16, GPIO_NUM_17);
 *
 *     // Optional: Change the baud rate if needed
 *     // fingerprint_set_baudrate(115200);
 *
 *     // Initialize the fingerprint module
 *     esp_err_t err = fingerprint_init();
 *     if (err != ESP_OK) {
 *         ESP_LOGE(TAG, "Fingerprint initialization failed: %s", esp_err_to_name(err));
 *         return;
 *     }
 *
 *     // Register event handler
 *     register_fingerprint_event_handler(my_event_handler);
 *
 *     // Read system parameters to verify communication
 *     err = read_system_parameters();
 *     if (err != ESP_OK) {
 *         ESP_LOGE(TAG, "Failed to read system parameters: %s", esp_err_to_name(err));
 *         return;
 *     }
 *
 *     ESP_LOGI(TAG, "Fingerprint scanner ready!");
 * }
 * @endcode
 *
 * ### 6.3 Interrupt-Based Finger Detection
 * The library automatically sets up interrupt-based finger detection using GPIO15.
 * When a finger is placed on the sensor, the INT pin triggers an interrupt that:
 * 
 * 1. Detects finger placement with software debouncing
 * 2. Signals the detection to a processing task
 * 3. Automatically processes the finger based on the current operation mode
 *
 * You can set the operation mode to control how finger detection is handled:
 *
 * @code
 * // Set operation mode for verification
 * fingerprint_set_operation_mode(FINGER_OP_VERIFY);
 * 
 * // Wait for a finger with timeout
 * err = fingerprint_wait_for_finger(10000);  // 10 second timeout
 * if (err == ESP_OK) {
 *     ESP_LOGI(TAG, "Finger detected and processed for verification");
 * }
 * @endcode
 *
 * ### 6.4 Testing the Connection
 * Verify the scanner is working by reading the template count:
 *
 * @code
 * // Get current enrollment count
 * err = get_enrolled_count();
 * if (err != ESP_OK) {
 *     ESP_LOGE(TAG, "Failed to get enrollment count: %s", esp_err_to_name(err));
 * }
 * // The count will be reported in the EVENT_TEMPLATE_COUNT event
 * @endcode
 *
 * ### 6.5 Setting Device Address
 * If you need to change the device address (e.g., for multi-device setups):
 *
 * @code
 * // Change from default broadcast address to a custom address
 * uint32_t new_address = 0x12345678;
 * err = fingerprint_set_address(new_address, DEFAULT_FINGERPRINT_ADDRESS);
 * if (err != ESP_OK) {
 *     ESP_LOGE(TAG, "Failed to set address: %s", esp_err_to_name(err));
 * }
 * @endcode
 *
 * ### 6.6 Troubleshooting
 * If you encounter issues:
 *
 * 1. **No response from scanner:**
 *    - Check power connections (3.3V/GND)
 *    - Verify TX/RX connections are correct and not swapped
 *    - Try a different baud rate
 *
 * 2. **Communication errors:**
 *    - Ensure the TX/RX lines have pull-up resistors (typically 10kΩ)
 *    - Add delay between commands (vTaskDelay(pdMS_TO_TICKS(100)))
 *
 * 3. **No finger detection interrupts:**
 *    - Check the INT pin connection to GPIO15
 *    - Verify INT pin is properly pulled down at rest
 *    - Test interrupt functioning using a GPIO test program
 *
 * 4. **Enrollment failures:**
 *    - Clean both the sensor and the finger
 *    - Ensure consistent finger placement
 *    - Increase timeout values for enrollment operations
 *
 */

 #ifndef FINGERPRINT_H
 #define FINGERPRINT_H
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     // Required for QueueHandle_t
#include "freertos/timers.h"    // Required for TickType_t
#include "driver/gpio.h"
#include <string.h>
#include <stdbool.h>   // Fixes unknown type name 'bool'
#include <inttypes.h>
 
 /**
  * @brief Default UART baud rate for fingerprint module.
  */
 #define DEFAULT_BAUD_RATE 57600
 
 /**
  * @brief Default UART pins (modifiable at runtime).
  */
 #define DEFAULT_TX_PIN (GPIO_NUM_5) //17
 #define DEFAULT_RX_PIN (GPIO_NUM_6) //18
 
 /**
  * @brief Default fingerprint module header identifier.
  */
 #define FINGERPRINT_HEADER 0xEF01
 
 /**
  * @brief Default fingerprint module address (broadcast address).
  */
 #define DEFAULT_FINGERPRINT_ADDRESS 0xFFFFFFFF
 
 /**
  * @brief Timeout value for UART read operations.
  *
  * Defines the maximum wait time for receiving data over UART in milliseconds.
  * Adjust this value based on the fingerprint module's response time.
  */
 #define UART_READ_TIMEOUT 100  // Adjust based on hardware response time

 /**
 * @brief Maximum number of parameters supported in a fingerprint packet.
 *
 * Defines the fixed-size buffer for command-specific parameters in fingerprint communication.
 * This value should be adjusted based on the largest expected parameter size to ensure
 * sufficient space while maintaining efficient memory usage.
 */
#define MAX_PARAMETERS 256  /**< Adjust based on the largest required parameter size. */

/**
 * @brief Packet header identifier for fingerprint module communication.
 * 
 * The header is a fixed 2-byte value used to indicate the start of a fingerprint module packet.
 */
#define FINGERPRINT_PACKET_HEADER 0xEF01

/**
 * @brief Packet ID indicating a command packet.
 * 
 * The packet ID is a 1-byte value that defines the type of packet being sent.
 * - 0x01: Command packet
 * - 0x02: Data packet
 * - 0x03: Acknowledgment packet
 * - 0x04: End of data packet
 */
#define FINGERPRINT_PACKET_ID_CMD 0x01

/**
 * @brief Defines the maximum number of fingerprint response packets in the queue.
 * 
 * This value determines the capacity of `fingerprint_response_queue`, 
 * which stores fingerprint responses for asynchronous processing.
 * 
 * @note A larger value increases buffering but consumes more memory. 
 *       Adjust as needed based on system constraints.
 */
#define QUEUE_SIZE 64

/**
 * @brief GPIO pin for the fingerprint sensor interrupt.
 * 
 * This pin is used to detect the interrupt signal from the fingerprint sensor.
 * When the sensor detects a finger, it triggers an interrupt on this GPIO pin.
 */
#define FINGERPRINT_GPIO_PIN (GPIO_NUM_15) // 15  // GPIO pin for fingerprint sensor interrupt

/**
 * @brief GPIO pin for the fingerprint sensor reset.
 */
// Add VIN control pin definition
#define FINGERPRINT_VIN_PIN GPIO_NUM_9  // D9 for VIN control
#define TEMPLATE_QUEUE_SIZE 10
#define TEMPLATE_MAX_SIZE 2048



/**
 * @enum fingerprint_command_t
 * @brief Enumeration of fingerprint module commands.
 *
 * This enumeration defines the command codes used to communicate with
 * the fingerprint sensor module. Each command corresponds to a specific
 * operation that the module can perform.
 */
typedef enum {
    /** Capture a fingerprint image */
    FINGERPRINT_CMD_GET_IMAGE = 0x01,
    
    /** Generate character file from image buffer */
    FINGERPRINT_CMD_GEN_CHAR = 0x02,
    
    /** Match two fingerprint templates */
    FINGERPRINT_CMD_MATCH = 0x03,
    
    /** Search for a fingerprint in the database */
    FINGERPRINT_CMD_SEARCH = 0x04,
    
    /** Generate a model from two fingerprint templates */
    FINGERPRINT_CMD_REG_MODEL = 0x05,
    
    /** Store fingerprint template in the module's database */
    FINGERPRINT_CMD_STORE_CHAR = 0x06,
    
    /** Delete a fingerprint template from the database */
    FINGERPRINT_CMD_DELETE_CHAR = 0x0C,
    
    /** Empty the fingerprint database */
    FINGERPRINT_CMD_EMPTY_DATABASE = 0x0D,
    
    /** Upload fingerprint image from the module */
    FINGERPRINT_CMD_UPLOAD_IMAGE = 0x0A,
    
    /** Download fingerprint image to the module */
    FINGERPRINT_CMD_DOWNLOAD_IMAGE = 0x0B,
    
    /** Read system parameters of the fingerprint module */
    FINGERPRINT_CMD_READ_SYS_PARA = 0x0F,
    
    /** Set fingerprint module's chip address */
    FINGERPRINT_CMD_SET_CHIP_ADDR = 0x15,
    
    /** Perform a handshake with the fingerprint module */
    FINGERPRINT_CMD_HANDSHAKE = 0x35,
    
    /** Cancel current fingerprint operation */
    FINGERPRINT_CMD_CANCEL = 0x30,
    
    /** Perform automatic fingerprint enrollment */
    FINGERPRINT_CMD_AUTO_ENROLL = 0x31,
    
    /** Automatically identify a fingerprint */
    FINGERPRINT_CMD_AUTO_IDENTIFY = 0x32,
    
    /** Check the fingerprint sensor status */
    FINGERPRINT_CMD_CHECK_SENSOR = 0x36,
    
    /** Factory reset the fingerprint module */
    FINGERPRINT_CMD_FACTORY_RESET = 0x3B,
    
    /** Read information page from the module */
    FINGERPRINT_CMD_READ_INF_PAGE = 0x16,
    
    /** Burn code into the fingerprint module */
    FINGERPRINT_CMD_BURN_CODE = 0x1A,
    
    /** Set password for the fingerprint module */
    FINGERPRINT_CMD_SET_PASSWORD = 0x12,
    
    /** Verify password for the fingerprint module */
    FINGERPRINT_CMD_VERIFY_PASSWORD = 0x13,
    
    /** Retrieve a random code from the fingerprint module */
    FINGERPRINT_CMD_GET_RANDOM_CODE = 0x14,
    
    /** Write data to notepad memory of the fingerprint module */
    FINGERPRINT_CMD_WRITE_NOTEPAD = 0x18,
    
    /** Read data from notepad memory of the fingerprint module */
    FINGERPRINT_CMD_READ_NOTEPAD = 0x19,
    
    /** Control the fingerprint module's LED indicator */
    FINGERPRINT_CMD_CONTROL_LED = 0x3C,
    
    /** Retrieve information about the captured fingerprint image */
    FINGERPRINT_CMD_GET_IMAGE_INFO = 0x3D,
    
    /** Search for a fingerprint instantly */
    FINGERPRINT_CMD_SEARCH_NOW = 0x3E,
    
    /** Get the number of valid fingerprint templates stored */
    FINGERPRINT_CMD_VALID_TEMPLATE_NUM = 0x1D,
    
    /** Put the fingerprint module into sleep mode */
    FINGERPRINT_CMD_SLEEP = 0x33,

    /** Retrieve a random code from the fingerprint module */
    FINGERPRINT_CMD_GETKEYT = 0xE0,

    /** Perform a security search in the fingerprint database */
    FINGERPRINT_CMD_SECURITY_SEARCH = 0xF4,

    /** Lock the fingerprint module to prevent unauthorized access */
    FINGERPRINT_CMD_LOCKEYT = 0xE1,

    /** Retrieve encrypted data from the fingerprint module */
    FINGERPRINT_CMD_GET_CIPHER_TEXT = 0xE2,

    /** Retrieve the serial number of the fingerprint module */
    FINGERPRINT_CMD_GETCHIP_SN = 0x34,

    /** Capture an enrollment image from the fingerprint sensor */
    FINGERPRINT_CMD_GET_ENROLL_IMAGE = 0x29,

    /** Write data to a specific register in the fingerprint module */
    FINGERPRINT_CMD_WRITE_REG = 0x0E,

    /** Read the index table of stored fingerprint templates */
    FINGERPRINT_CMD_READ_INDEX_TABLE = 0x1F,

    /** Upload a fingerprint template from the module buffer */
    FINGERPRINT_CMD_UP_CHAR = 0x08,

    /** Download a fingerprint template to the module buffer */
    FINGERPRINT_CMD_DOWN_CHAR = 0x09,

    /**
     * @brief Load a fingerprint template from flash memory into the module buffer.
     */
    FINGERPRINT_CMD_LOAD_CHAR = 0x07,

} fingerprint_command_t;

/**
 * @struct fingerprint_command_info_t
 * @brief Structure to store information about a sent fingerprint command.
 *
 * This structure holds metadata related to a fingerprint command, including
 * the command type and the timestamp when it was sent. It is useful for 
 * tracking commands in a queue and debugging response mismatches.
 */
typedef struct {
    fingerprint_command_t command;  /**< Command sent to the fingerprint sensor */
    uint32_t timestamp;             /**< Timestamp when the command was sent (in ticks) */
} fingerprint_command_info_t;

/**
 * @struct FingerprintPacket
 * @brief Structure representing a command or response packet for the fingerprint module.
 *
 * This structure is used for both sending commands to and receiving responses from the fingerprint module.
 * It follows the module's communication protocol, containing fields for the header, address, packet type, 
 * length, command/confirmation code, parameters, and checksum.
 *
 * In command packets, the `command` field specifies the requested operation (e.g., 0x01 for image capture).
 * In response packets, the `command` field serves as a confirmation code indicating the success or failure of the request.
 */
typedef struct {
    uint16_t header;      /**< Fixed packet header (0xEF01) indicating the start of a fingerprint packet. */
    uint32_t address;     /**< Address of the fingerprint module (default: 0xFFFFFFFF). */
    uint8_t packet_id;    /**< Packet identifier (e.g., 0x01 for command packets, 0x07 for responses). */
    uint16_t length;      /**< Length of the packet, excluding the header and address. */
    union {
        uint8_t command;  /**< Command ID for requests or confirmation code in responses. */
        uint8_t confirmation;   /**< Confirmation code for the command. */
    } code;
    uint8_t parameters[MAX_PARAMETERS]; /**< Command-specific parameters (variable length, up to MAX_PARAMETERS bytes). */
    uint16_t checksum;    /**< Checksum for packet integrity verification. */
} FingerprintPacket;

 
 /**
  * @brief Captures a fingerprint image from the scanner's sensor.
  */
 extern FingerprintPacket PS_GetImage;
 
 /**
  * @brief Generates a character file in Buffer 1.
  */
 extern FingerprintPacket PS_GenChar1;
 
 /**
  * @brief Generates a character file in Buffer 2.
  */
 extern FingerprintPacket PS_GenChar2;
 
 /**
  * @brief Combines feature templates stored in Buffer 1 and Buffer 2 into a single fingerprint model.
  */
 extern FingerprintPacket PS_RegModel;
 
 /**
  * @brief Searches for a fingerprint match in the database.
  * 
  * This packet structure should be overwritten using fingerprint_set_command()
  * because its parameters (Buffer ID, Start Page, and Number of Pages) are 
  * dynamic and depend on the user's or developer's choice.
  * 
  * ### Packet Structure:
  * | Header  | Device Address | Packet ID | Packet Length | Command | Parameters                                           | Checksum             |
  * |---------|---------------|-----------|---------------|---------|-------------------------------------------------------|----------------------|
  * | 2 bytes | 4 bytes       | 1 byte    | 2 bytes       | 1 byte  | Buffer ID 1 bytes, Start Page 2 bytes, PageNum 2 bytes| 2 bytes              |
  * | 0xEF01  | 0xFFFFFFFF    | 0x01      | 0x0008        | 0x04    | Varies (set dynamically)                              | Computed dynamically |
  */
 extern FingerprintPacket PS_Search;
 
 
 /**
  * @brief Matches two fingerprint templates stored in RAM.
  */
 extern FingerprintPacket PS_Match;
 
 /**
  * @brief Stores a fingerprint template into the module’s database.
  * 
  * This packet structure should be overwritten using fingerprint_set_command()
  * because its parameters (Buffer ID and Page ID) are dynamic and depend on 
  * the user's or developer's choice.
  * 
  * ### Packet Structure:
  * | Header  | Device Address | Packet ID | Packet Length | Command | Parameters                                  | Checksum             |
  * |---------|---------------|-----------|---------------|---------|----------------------------------------------|----------------------|
  * | 2 bytes | 4 bytes       | 1 byte    | 2 bytes       | 1 byte  | Buffer ID 1 byte, Page ID 2 bytes          | 2 bytes              |
  * | 0xEF01  | 0xFFFFFFFF    | 0x01      | 0x0006        | 0x06    | Varies (set dynamically)                    | Computed dynamically |
  */
 extern FingerprintPacket PS_Store;
 
 
 /**
  * @brief Deletes a specific fingerprint template from the database.
  */
 extern FingerprintPacket PS_DeleteChar;
 
 /**
  * @brief Clears all stored fingerprints (factory reset).
  */
 extern FingerprintPacket PS_Empty;
 
 /**
  * @brief Reads system parameters from the fingerprint module.
  */
 extern FingerprintPacket PS_ReadSysPara;
 
 /**
  * @brief Sets the fingerprint scanner's device address.
  */
 extern FingerprintPacket PS_SetChipAddr;
 
 /**
  * @brief Cancels the current fingerprint operation.
  *
  * This command has no parameters and is used to stop any ongoing process.
  */
 extern FingerprintPacket PS_Cancel;
 
 /**
  * @brief Automatically enrolls a fingerprint template.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameters (ID number, number of entries, and additional settings)
  * are dynamic and depend on the user's choice.
  *
  * ### Parameters:
  * - **ID Number** (1 byte): The fingerprint ID to assign.
  * - **Number of Entries** (1 byte): The number of fingerprint samples.
  * - **Parameter** (3 bytes): Additional settings (sensor-specific).
  */
 extern FingerprintPacket PS_AutoEnroll;
 
 /**
  * @brief Automatically identifies a fingerprint.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameters (score level and ID number) are dynamic.
  *
  * ### Parameters:
  * - **Score Level** (1 byte): The threshold for matching accuracy.
  * - **ID Number** (2 bytes): The ID range to search.
  */
 extern FingerprintPacket PS_Autoldentify;
 
 /**
  * @brief Retrieves the key pair from the fingerprint sensor.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_GetKeyt;
 
 /**
  * @brief Securely stores a fingerprint template.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameters (Buffer ID and Page ID) are dynamic.
  *
  * ### Parameters:
  * - **Buffer ID** (1 byte): Specifies the template buffer (1 or 2).
  * - **Page ID** (2 bytes): The database location to store the template.
  */
 extern FingerprintPacket PS_SecurityStoreChar;
 
 /**
  * @brief Searches for a fingerprint template in secure mode.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameters (Buffer ID, Start Page, Number of Pages) are dynamic.
  *
  * ### Parameters:
  * - **Buffer ID** (1 byte): Specifies which buffer to use.
  * - **Start Page** (2 bytes): The starting page index in the database.
  * - **Number of Pages** (2 bytes): The range of pages to search.
  */
 extern FingerprintPacket PS_SecuritySearch;
 
 /**
  * @brief Uploads the fingerprint image from the sensor.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_Uplmage;
 
 /**
  * @brief Downloads a fingerprint image to the sensor.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_Downlmage;
 
 /**
  * @brief Checks the status of the fingerprint sensor.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_CheckSensor;
 
 /**
  * @brief Restores the fingerprint sensor to factory settings.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_RestSetting;
 
 /**
  * @brief Reads the fingerprint sensor's flash information page.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_ReadINFpage;
 
 /**
  * @brief Erases the fingerprint sensor's firmware.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameter (Upgrade Mode) is dynamic.
  *
  * ### Parameters:
  * - **Upgrade Mode** (1 byte): Mode for firmware upgrade.
  */
 extern FingerprintPacket PS_BurnCode;
 
 /**
  * @brief Sets a password for the fingerprint sensor.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameter (Password) is user-defined.
  *
  * ### Parameters:
  * - **Password** (4 bytes): The new password for sensor access.
  */
 extern FingerprintPacket PS_SetPwd;
 
 /**
  * @brief Verifies the fingerprint sensor password.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameter (Password) is user-defined.
  *
  * ### Parameters:
  * - **Password** (4 bytes): The password to verify.
  */
 extern FingerprintPacket PS_VfyPwd;
 
 /**
  * @brief Requests a random number from the fingerprint sensor.
  *
  * This command has no parameters.
  */
 extern FingerprintPacket PS_GetRandomCode;
 
 /**
  * @brief Reads data from the fingerprint sensor's notepad memory.
  *
  * This packet structure should be overwritten using `fingerprint_set_command()`
  * because its parameter (Page Number) is dynamic.
  *
  * ### Parameters:
  * - **Page Number** (1 byte): Specifies which notepad page to read.
  */
 extern FingerprintPacket PS_ReadNotepad;
 
 /**
 * @brief Downloads a fingerprint template to the module.
 */
extern FingerprintPacket PS_DownChar;

/**
 * @brief Uploads a stored fingerprint template from the module.
 */
extern FingerprintPacket PS_UpChar;

/**
 * @brief Loads a fingerprint template from flash memory into the module's buffer.
 *
 * This command instructs the fingerprint module to retrieve a stored fingerprint 
 * template from the flash database and load it into a specified template buffer.
 *
 * @note The `parameters` field should be set before use:
 *       - `parameters[0]` = Buffer ID (target buffer)
 *       - `parameters[1]` = High byte of Page ID
 *       - `parameters[2]` = Low byte of Page ID
 *
 * @warning The checksum must be recalculated before sending the packet.
 */
extern FingerprintPacket PS_LoadChar;

/**
 * @brief Erases the module firmware and enters upgrade mode.
 */
extern FingerprintPacket PS_BurnCode;

/**
 * @brief Performs a handshake with the fingerprint module.
 *
 * Used to verify communication with the fingerprint module.
 */
extern FingerprintPacket PS_HandShake;

/**
 * @brief Controls the fingerprint sensor's LED light mode.
 *
 * This packet structure should be overwritten using `fingerprint_set_command()`
 * because its parameters (Function, Start Color, End Color, Cycles) are dynamic.
 *
 * ### Parameters:
 * - **Function** (1 byte): Specifies LED function.
 * - **Start Color** (1 byte): Starting color.
 * - **End Color** (1 byte): Ending color.
 * - **Cycles** (1 byte): Number of cycles.
 */
extern FingerprintPacket PS_ControlBLN;

/**
 * @brief Retrieves information about the current fingerprint image.
 *
 * This command requests metadata about the fingerprint image stored in the buffer.
 */
extern FingerprintPacket PS_GetImageInfo;

/**
 * @brief Performs an immediate fingerprint search in the database.
 *
 * This packet structure should be overwritten using `fingerprint_set_command()`
 * because its parameters (Start Page, Number of Pages) are dynamic.
 *
 * ### Parameters:
 * - **Start Page** (2 bytes): Page to start searching from.
 * - **Number of Pages** (2 bytes): Number of pages to search.
 */
extern FingerprintPacket PS_SearchNow;

/**
 * @brief Retrieves the number of valid fingerprint templates stored in the module.
 */
extern FingerprintPacket PS_ValidTempleteNum;

/**
 * @brief Puts the fingerprint module into sleep mode.
 */
extern FingerprintPacket PS_Sleep;

/**
 * @brief Locks the fingerprint module's key pair.
 */
extern FingerprintPacket PS_LockKeyt;

/**
 * @brief Retrieves an encrypted random number from the fingerprint module.
 */
extern FingerprintPacket PS_GetCiphertext;

/**
 * @brief Retrieves the fingerprint module's unique serial number.
 */
extern FingerprintPacket PS_GetChipSN;

/**
 * @brief Captures an image for fingerprint enrollment.
 */
extern FingerprintPacket PS_GetEnrollImage;

/**
 * @brief Writes to the fingerprint module's system register.
 *
 * This packet structure should be overwritten using `fingerprint_set_command()`
 * because its parameters (Register Number, Value) are dynamic.
 *
 * ### Parameters:
 * - **Register Number** (1 byte): Specifies which register to write to.
 * - **Value** (1 byte): Data to be written.
 */
extern FingerprintPacket PS_WriteReg;

/**
 * @brief Reads the fingerprint template index table.
 *
 * This packet structure should be overwritten using `fingerprint_set_command()`
 * because its parameter (Page Number) is dynamic.
 *
 * ### Parameters:
 * - **Page Number** (1 byte): Specifies which index table page to read.
 */
extern FingerprintPacket PS_ReadIndexTable;

/**
 * @brief Upload template from buffer to main control.
 * 
 * This command uploads the fingerprint template stored in the buffer 
 * to the main control unit. The function is supported when the 
 * encryption level is set to 0.
 */
extern FingerprintPacket PS_UpChar;

/**
 * @brief Download template to buffer.
 * 
 * This command allows the master device to download a fingerprint 
 * template to the module’s template buffer. The function is supported 
 * when the encryption level is set to 0.
 */
extern FingerprintPacket PS_DownChar;

/**
 * @brief External declaration of the Read Information Page command.
 *
 * This command is used to read the information page of the fingerprint module,
 * which typically contains device-specific details such as firmware version, 
 * module serial number, and other relevant metadata.
 *
 * @note Ensure that the fingerprint module is properly initialized before 
 * calling this command.
 */
extern FingerprintPacket PS_ReadINFPage;

 
 /**
  * @brief Fingerprint sensor status codes.
  *
  * This enumeration defines various return codes for the fingerprint sensor.
  * Each value corresponds to a specific error or status condition.
  */
 typedef enum {
     /** @brief Instruction execution completed. Value: `0x00` */
     FINGERPRINT_OK = 0x00,                     
 
     /** @brief Data packet reception error. Value: `0x01` */
     FINGERPRINT_PACKET_ERROR = 0x01,            
 
     /** @brief No finger detected. Value: `0x02` */
     FINGERPRINT_NO_FINGER = 0x02,               
 
     /** @brief Failed to enter fingerprint image. Value: `0x03` */
     FINGERPRINT_IMAGE_FAIL = 0x03,              
 
     /** @brief Image too dry/light. Value: `0x04` */
     FINGERPRINT_TOO_DRY = 0x04,                 
 
     /** @brief Image too wet/muddy. Value: `0x05` */
     FINGERPRINT_TOO_WET = 0x05,                 
 
     /** @brief Image too chaotic. Value: `0x06` */
     FINGERPRINT_TOO_CHAOTIC = 0x06,             
 
     /** @brief Normal image, but not enough features. Value: `0x07` */
     FINGERPRINT_TOO_FEW_POINTS = 0x07,         
 
     /** @brief Fingerprint mismatch. Value: `0x08` */
     FINGERPRINT_MISMATCH = 0x08,                
 
     /** @brief No fingerprint found. Value: `0x09` */
     FINGERPRINT_NOT_FOUND = 0x09,               
 
     /** @brief Feature merging failed. Value: `0x0A` */
     FINGERPRINT_MERGE_FAIL = 0x0A,              
 
     /** @brief Address out of range in database. Value: `0x0B` */
     FINGERPRINT_DB_RANGE_ERROR = 0x0B,          
 
     /** @brief Error reading fingerprint template. Value: `0x0C` */
     FINGERPRINT_READ_TEMPLATE_ERROR = 0x0C,     
 
     /** @brief Failed to upload features. Value: `0x0D` */
     FINGERPRINT_UPLOAD_FEATURE_FAIL = 0x0D,     
 
     /** @brief Cannot receive subsequent data packets. Value: `0x0E` */
     FINGERPRINT_DATA_PACKET_ERROR = 0x0E,        
 
     /** @brief Failed to upload image. Value: `0x0F` */
     FINGERPRINT_UPLOAD_IMAGE_FAIL = 0x0F,       
 
     /** @brief Failed to delete template. Value: `0x10` */
     FINGERPRINT_DELETE_TEMPLATE_FAIL = 0x10,    
 
     /** @brief Failed to clear database. Value: `0x11` */
     FINGERPRINT_DB_CLEAR_FAIL = 0x11,           
 
     /** @brief Cannot enter low power mode. Value: `0x12` */
     FINGERPRINT_LOW_POWER_FAIL = 0x12,          
 
     /** @brief Incorrect password. Value: `0x13` */
     FINGERPRINT_WRONG_PASSWORD = 0x13,          
 
     /** @brief No valid original image in buffer. Value: `0x15` */
     FINGERPRINT_NO_VALID_IMAGE = 0x15,          
 
     /** @brief Online upgrade failed. Value: `0x16` */
     FINGERPRINT_UPGRADE_FAIL = 0x16,            
 
     /** @brief Residual fingerprint detected. Value: `0x17` */
     FINGERPRINT_RESIDUAL_FINGER = 0x17,         
 
     /** @brief Flash read/write error. Value: `0x18` */
     FINGERPRINT_FLASH_RW_ERROR = 0x18,          
 
     /** @brief Random number generation failed. Value: `0x19` */
     FINGERPRINT_RANDOM_GEN_FAIL = 0x19,         
 
     /** @brief Invalid register number. Value: `0x1A` */
     FINGERPRINT_INVALID_REGISTER = 0x1A,        
 
     /** @brief Register setting content error. Value: `0x1B` */
     FINGERPRINT_REGISTER_SETTING_ERROR = 0x1B,  
 
     /** @brief Incorrect notepad page number. Value: `0x1C` */
     FINGERPRINT_NOTEPAD_PAGE_ERROR = 0x1C,      
 
     /** @brief Port operation failed. Value: `0x1D` */
     FINGERPRINT_PORT_OP_FAIL = 0x1D,            
 
     /** @brief Automatic registration failed. Value: `0x1E` */
     FINGERPRINT_ENROLL_FAIL = 0x1E,             
 
     /** @brief Fingerprint database full. Value: `0x1F` */
     FINGERPRINT_DB_FULL = 0x1F,                 
 
     /** @brief Device address error. Value: `0x20` */
     FINGERPRINT_DEVICE_ADDRESS_ERROR = 0x20,    
 
     /** @brief Template is not empty. Value: `0x22` */
     FINGERPRINT_TEMPLATE_NOT_EMPTY = 0x22,      
 
     /** @brief Template is empty. Value: `0x23` */
     FINGERPRINT_TEMPLATE_EMPTY = 0x23,          
 
     /** @brief Database is empty. Value: `0x24` */
     FINGERPRINT_DB_EMPTY = 0x24,                
 
     /** @brief Incorrect entry count. Value: `0x25` */
     FINGERPRINT_ENTRY_COUNT_ERROR = 0x25,       
 
     /** @brief Timeout occurred. Value: `0x26` */
     FINGERPRINT_TIMEOUT = 0x26,                 
 
     /** @brief Fingerprint already exists. Value: `0x27` */
     FINGERPRINT_ALREADY_EXISTS = 0x27,          
 
     /** @brief Features are related. Value: `0x28` */
     FINGERPRINT_FEATURES_RELATED = 0x28,        
 
     /** @brief Sensor operation failed. Value: `0x29` */
     FINGERPRINT_SENSOR_OP_FAIL = 0x29,          
 
     /** @brief Module info not empty. Value: `0x2A` */
     FINGERPRINT_MODULE_INFO_NOT_EMPTY = 0x2A,   
 
     /** @brief Module info empty. Value: `0x2B` */
     FINGERPRINT_MODULE_INFO_EMPTY = 0x2B,       
 
     /** @brief OTP operation failed. Value: `0x2C` */
     FINGERPRINT_OTP_FAIL = 0x2C,                
 
     /** @brief Key generation failed. Value: `0x2D` */
     FINGERPRINT_KEY_GEN_FAIL = 0x2D,            
 
     /** @brief Secret key does not exist. Value: `0x2E` */
     FINGERPRINT_KEY_NOT_EXIST = 0x2E,           
 
     /** @brief Security algorithm execution failed. Value: `0x2F` */
     FINGERPRINT_SECURITY_ALGO_FAIL = 0x2F,      
 
     /** @brief Encryption and function mismatch. Value: `0x30` */
     FINGERPRINT_ENCRYPTION_MISMATCH = 0x30,     

     /** @brief Function does not match the required encryption level. Value: `0x31` */
     FINGERPRINT_FUNCTION_ENCRYPTION_MISMATCH = 0x31,

     /** @brief Secret key is locked. Value: `0x32` */
     FINGERPRINT_KEY_LOCKED = 0x32,              
 
     /** @brief Image area too small. Value: `0x33` */
     FINGERPRINT_IMAGE_AREA_SMALL = 0x33,        
 
     /** @brief Image not available. Value: `0x34` */
     FINGERPRINT_IMAGE_NOT_AVAILABLE = 0x34,     
 
     /** @brief Illegal data. Value: `0x35` */
     FINGERPRINT_ILLEGAL_DATA = 0x35             
 
 } fingerprint_status_t;

/**
 * @brief Handles fingerprint status events and triggers corresponding high-level events.
 *
 * This function processes status codes received from the fingerprint module and
 * maps them to predefined high-level fingerprint events. These events help notify
 * the application about the outcome of fingerprint operations.
 *
 * The following status codes and their corresponding events are handled:
 * 
 * - `FINGERPRINT_OK` → `EVENT_FINGER_DETECTED`
 * - `FINGERPRINT_NO_FINGER` → `EVENT_IMAGE_CAPTURED`
 * - `FINGERPRINT_IMAGE_FAIL`, `FINGERPRINT_TOO_DRY`, `FINGERPRINT_TOO_WET`,
 *   `FINGERPRINT_TOO_CHAOTIC`, `FINGERPRINT_UPLOAD_IMAGE_FAIL`, 
 *   `FINGERPRINT_IMAGE_AREA_SMALL`, `FINGERPRINT_IMAGE_NOT_AVAILABLE` → `EVENT_IMAGE_FAIL`
 * - `FINGERPRINT_TOO_FEW_POINTS` → `EVENT_FEATURE_EXTRACT_FAIL`
 * - `FINGERPRINT_MISMATCH`, `FINGERPRINT_NOT_FOUND` → `EVENT_MATCH_FAIL`
 * - `FINGERPRINT_DB_FULL` → `EVENT_DB_FULL`
 * - `FINGERPRINT_TIMEOUT` → `EVENT_ERROR`
 * - Various sensor operation failures → `EVENT_SENSOR_ERROR`
 * - Other unexpected statuses → `EVENT_ERROR`
 *
 * @param status The fingerprint status received from the sensor.
 */
void fingerprint_status_event_handler(fingerprint_status_t status, FingerprintPacket *packet);
 
/**
 * @brief Initializes the fingerprint scanner module.
 *
 * This function sets up the UART interface, configures the necessary pins, 
 * installs the UART driver, and checks for the power-on handshake signal (0x55). 
 * If the handshake is not received, it waits for 200ms before proceeding.
 * It also initializes a FreeRTOS queue for response handling and creates 
 * a background task to continuously read responses.
 *
 * @return 
 *  - ESP_OK if initialization is successful.
 *  - ESP_FAIL if queue creation or task creation fails.
 *  - UART-related errors if UART initialization fails.
 */
esp_err_t fingerprint_init(void);

 
/**
  * @brief Initializes or updates a fingerprint command packet.
  *
  * This function modifies an existing `FingerprintPacket` structure to prepare a command
  * before sending it to the fingerprint sensor. It ensures that the parameter length 
  * does not exceed the allowed size and automatically computes the checksum.
  *
  * @warning This function **modifies** the provided `FingerprintPacket` structure in-place.
  *          Developers should ensure that any previous data in the structure is not needed 
  *          before calling this function, unless they create a **new** `FingerprintPacket` instance.
  *
  * @param[out] cmd Pointer to the `FingerprintPacket` to be modified.
  * @param[in] command Command byte to be set in the packet.
  * @param[in] params Pointer to a parameter array (can be NULL if no parameters).
  * @param[in] param_length Number of parameters (max `MAX_PARAMETERS`).
  * @return 
  *      - `ESP_OK` on success.
  *      - `ESP_ERR_INVALID_ARG` if `cmd` is NULL.
  *      - `ESP_ERR_INVALID_SIZE` if `param_length` exceeds `MAX_PARAMETERS`.
  *
  * @note This function does not allocate new memory. The caller is responsible for managing `cmd`.
  *       If modification of an existing `FingerprintPacket` is not desired, a **new instance**
  *       should be created before calling this function.
  */

 esp_err_t fingerprint_set_command(FingerprintPacket *cmd, uint8_t command, uint8_t *params, uint8_t param_length);
 
 /**
  * @brief Computes the checksum for a given FingerprintPacket structure.
  *
  * The checksum is the sum of all bytes from packet_id to parameters.
  *
  * @param[in] cmd Pointer to the FingerprintPacket structure.
  * @return The computed checksum.
  */
 uint16_t fingerprint_calculate_checksum(FingerprintPacket *cmd);
 
 /**
  * @brief Sends a fingerprint command packet to the fingerprint module.
  *
  * This function constructs a command packet, calculates its checksum,
  * and sends it via UART to the fingerprint module at the specified address.
  *
  * @param cmd Pointer to a FingerprintPacket structure containing the command details.
  * @param address The address of the fingerprint module. This can be configured as needed.
  * 
  * @return 
  * - ESP_OK on success
  * - ESP_ERR_NO_MEM if memory allocation fails
  * - ESP_FAIL if the command could not be fully sent over UART
  */
 esp_err_t fingerprint_send_command(FingerprintPacket *cmd, uint32_t address);

 /**
 * @brief Task to continuously read fingerprint responses from UART.
 * 
 * This FreeRTOS task reads fingerprint sensor responses and enqueues them 
 * into the response queue for further processing. It runs indefinitely, 
 * ensuring that responses are captured asynchronously.
 * 
 * @param pvParameter Unused parameter (NULL by default).
 * 
 * @note The function reads responses using `fingerprint_read_response()`, 
 *       validates them, and sends them to `fingerprint_response_queue`. 
 *       If the queue is full, the response is dropped with a warning log.
 * 
 * @warning This task should be created only once during initialization 
 *          and should not be terminated unexpectedly, as it ensures 
 *          continuous response handling.
 */
void read_response_task(void *pvParameter);

typedef struct {
    FingerprintPacket **packets;  // Array of packet pointers
    size_t count;                 // Number of packets found
    
    // Template collection fields
    bool collecting_template;     // Flag indicating active collection
    bool template_complete;       // Flag indicating template is complete
    uint32_t start_time;          // When collection started (for timeout)
    
    // Raw template data accumulation
    uint8_t *template_data;       // Combined template data buffer
    size_t template_size;         // Current size of accumulated template
    size_t template_capacity;     // Allocated capacity of template_data
} MultiPacketResponse;
 
/**
 * @brief Reads the response packet from UART and returns a dynamically allocated FingerprintPacket.
 *
 * @note The caller is responsible for freeing the allocated memory using `free()`
 *       after processing the response to avoid memory leaks.
 *
 * @return Pointer to the received FingerprintPacket, or NULL on failure.
 */
//  FingerprintPacket* fingerprint_read_response(void);
MultiPacketResponse* fingerprint_read_response(void);

/**
 * @brief Task function to process fingerprint scanner responses.
 *
 * This FreeRTOS task continuously reads responses from the fingerprint scanner,
 * dequeues them from the response queue, and processes them accordingly.
 *
 * @param[in] pvParameter Unused parameter (can be NULL).
 */
void process_response_task(void *pvParameter);

 /**
 * @brief Structure representing a fingerprint module response.
 *
 * This structure stores the status of the fingerprint response and the received packet data.
 * It is used to facilitate communication between tasks via the response queue.
 */
typedef struct {
    fingerprint_status_t status; /**< Status code of the fingerprint response */
    FingerprintPacket packet;    /**< Data packet received from the fingerprint module */
} fingerprint_response_t;

// /** 
//  * @brief Queue handle for storing fingerprint module responses.
//  *
//  * This queue is used to store responses received from the fingerprint module.
//  * Other components can retrieve responses from this queue for processing.
//  */
// extern QueueHandle_t fingerprint_response_queue;

 /**
  * @brief Get the status of the fingerprint operation from the response packet.
  *
  * This function extracts the confirmation code from the fingerprint sensor's response
  * and maps it to a predefined fingerprint_status_t enumeration.
  *
  * @param packet Pointer to the FingerprintPacket struScture containing the response.
  *
  * @return fingerprint_status_t Status code representing the operation result.
  */
 fingerprint_status_t fingerprint_get_status(FingerprintPacket *packet);
 
 /**
  * @brief Sets the UART TX and RX pins for the fingerprint sensor.
  *
  * This function allows dynamic configuration of the TX and RX pins
  * before initializing the fingerprint module.
  *
  * @param[in] tx GPIO number for the TX pin.
  * @param[in] rx GPIO number for the RX pin.
  */
 void fingerprint_set_pins(int tx, int rx);
 
 /**
  * @brief Sets the baud rate for fingerprint module communication.
  *
  * This function adjusts the baud rate used for UART communication
  * with the fingerprint scanner. It should be called before `fingerprint_init()`
  * to ensure proper communication settings.
  *
  * @param[in] baud The desired baud rate (e.g., 9600, 57600, 115200).
  */
 void fingerprint_set_baudrate(int baud);
 
 
 
 
 
 /**
  * @brief Enum to define various fingerprint events.
  *
  * These events represent significant milestones or errors during the fingerprint processing. 
  * They allow for event-driven management of the fingerprint sensor actions.
  * 
  * Each event is triggered when a certain action or error occurs, making it easier for the application to respond to changes in the system state.
  * 
  * @note The event codes are used in an event handler system where functions can subscribe and react to specific events.
  */
 typedef enum {
    /**
     * @brief Event representing no valid fingerprint event.
     *
     * This event is used as a default value when no other event is applicable.
     */
    EVENT_NONE = -1,                  /**< No event (Default value) */

    /**
     * @brief Event triggered when a fingerprint is detected on the sensor.
     *
     * This event occurs when the fingerprint module detects a finger placed on the
     * sensor surface. It typically follows a successful FINGERPRINT_CMD_GET_IMAGE operation.
     *
     * @note This event indicates only that a finger is present on the sensor, not that
     *       a valid or good quality fingerprint image has been captured.
     *
     * @see FINGERPRINT_CMD_GET_IMAGE
     * @see FINGERPRINT_OK
     * @see EVENT_NO_FINGER_DETECTED
     */
     EVENT_FINGER_DETECTED,          /**< Finger detected */
 
    /**
     * @brief Event triggered when the fingerprint module successfully captures an image.
     *
     * This event occurs after a FINGERPRINT_CMD_GET_IMAGE command when the module
     * successfully captures a valid fingerprint image that can be used for further processing.
     *
     * @note A successful image capture does not guarantee successful feature extraction.
     *       The quality of the captured image still needs to be sufficient for further operations.
     *
     * @see FINGERPRINT_CMD_GET_IMAGE
     * @see FINGERPRINT_OK
     * @see EVENT_IMAGE_FAIL
     */
     EVENT_IMAGE_CAPTURED,           /**< Image captured */
 
    /**
     * @brief Event triggered when fingerprint features are successfully extracted.
     *
     * This event occurs after executing the FINGERPRINT_CMD_GEN_CHAR command when
     * the module successfully extracts characteristic features from the captured
     * fingerprint image.
     *
     * @note Feature extraction is a critical step in both enrollment and verification
     *       processes. A successful extraction indicates the image had sufficient
     *       quality for biometric processing.
     *
     * @see FINGERPRINT_CMD_GEN_CHAR
     * @see FINGERPRINT_OK
     * @see EVENT_FEATURE_EXTRACT_FAIL
     */
     EVENT_FEATURE_EXTRACTED,        /**< Feature extraction completed */
 
     /**
      * @brief Event triggered when a fingerprint match is successful.
      *
      * Indicates that a fingerprint match has been found in the database.
      */
     EVENT_MATCH_SUCCESS,            /**< Fingerprint matched */
 
     /**
      * @brief Event triggered when a fingerprint mismatch occurs.
      *
      * Occurs when no matching fingerprint is found in the database.
      */
     EVENT_MATCH_FAIL,               /**< Fingerprint mismatch */
 
    /**
     * @brief Event triggered when a general error occurs during fingerprint processing.
     *
     * This event is a catch-all for various errors that may occur during fingerprint
     * operations and don't have more specific error events. It typically indicates an
     * unexpected failure in command processing or hardware operation.
     *
     * @note The event.status field contains the specific error code, and event.command
     *       indicates which command was being processed when the error occurred.
     *
     * @see FINGERPRINT_ERROR
     */
     EVENT_ERROR,                    /**< General error */
 
    /**
     * @brief Event triggered when image capture fails.
     *
     * This event occurs when the fingerprint module is unable to capture a valid image
     * from the sensor. This may happen if no finger is present, if the finger placement
     * is poor, or if there are hardware issues with the image sensor.
     *
     * @note This event corresponds to various status codes including FINGERPRINT_NO_FINGER (0x02)
     *       or FINGERPRINT_IMAGE_FAIL (0x03) depending on the specific cause.
     *
     * @see FINGERPRINT_CMD_GET_IMAGE
     * @see FINGERPRINT_IMAGE_FAIL
     * @see EVENT_IMAGE_CAPTURED
     */
     EVENT_IMAGE_FAIL,               /**< Image capture failure (FINGERPRINT_IMAGE_FAIL) */
 
    /**
     * @brief Event triggered when fingerprint feature extraction fails.
     *
     * This event occurs when the fingerprint module is unable to extract valid biometric
     * features from the captured fingerprint image. This typically happens when the image
     * quality is too poor, the finger placement is incorrect, or there is dirt or damage
     * on the sensor surface.
     *
     * @note This failure often requires capturing a new image with better finger placement
     *       or cleaning the finger/sensor before retrying.
     *
     * @see FINGERPRINT_CMD_GEN_CHAR
     * @see FINGERPRINT_FEATURE_FAIL
     * @see EVENT_FEATURE_EXTRACTED
     */
     EVENT_FEATURE_EXTRACT_FAIL,     /**< Feature extraction failure (FINGERPRINT_TOO_FEW_POINTS) */
 
     /**
      * @brief Event triggered when the fingerprint database is full.
      *
      * This event corresponds to the status `FINGERPRINT_DB_FULL`.
      */
     EVENT_DB_FULL,                  /**< Database full (FINGERPRINT_DB_FULL) */
 
     /**
      * @brief Event triggered when there is a sensor operation failure.
      *
      * This event corresponds to the status `FINGERPRINT_SENSOR_OP_FAIL`.
      */
     EVENT_SENSOR_ERROR,              /**< Sensor operation failure (FINGERPRINT_SENSOR_OP_FAIL) */

    /**
     * @brief Event triggered when a fingerprint is enrolled successfully.
     *
     * This event corresponds to the status FINGERPRINT_ENROLL_SUCCESS.
     */
    EVENT_ENROLL_SUCCESS,               /**< Fingerprint enrolled successfully (FINGERPRINT_ENROLL_SUCCESS) */

    /**
     * @brief Event triggered when fingerprint enrollment fails.
     *
     * This event corresponds to the status FINGERPRINT_ENROLL_FAIL.
     */
    EVENT_ENROLL_FAIL,                  /**< Fingerprint enrollment failed (FINGERPRINT_ENROLL_FAIL) */

    /**
     * @brief Event triggered when a fingerprint template is successfully stored in the module.
     *
     * This event occurs after executing the FINGERPRINT_CMD_STORE_CHAR command when the
     * template has been successfully written to the permanent storage of the fingerprint module.
     *
     * @note After receiving this event, the template is fully stored and can be used for
     *       future fingerprint matching operations.
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see EVENT_TEMPLATE_RESTORED_FAIL
     */
    EVENT_TEMPLATE_RESTORED_SUCCESSUL,              /**< Fingerprint template stored (FINGERPRINT_TEMPLATE_STORED) */

    /**
     * @brief Event triggered when fingerprint template restoration fails.
     *
     * This event is generated when an attempt to restore (download) a template 
     * to the fingerprint module fails. This could be due to communication errors, 
     * invalid template data, memory issues in the module, or incompatible 
     * template format.
     *
     * @note Common status codes associated with this event include FINGERPRINT_FLASH_RW_ERROR,
     *       FINGERPRINT_DB_RANGE_ERROR or FINGERPRINT_PACKET_ERROR.
     *
     * @see FINGERPRINT_CMD_DOWN_CHAR
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see EVENT_TEMPLATE_RESTORED_SUCCESSUL
     */
    EVENT_TEMPLATE_RESTORED_FAIL,              /**< Fingerprint template restoration failed */

    /**
     * @brief Event triggered when a fingerprint template is successfully deleted.
     *
     * This event occurs when a fingerprint template has been successfully removed from
     * the fingerprint module's database using the FINGERPRINT_CMD_DELETE_CHAR command.
     *
     * @note After this event, the specified template ID is free and can be reused for
     *       new fingerprint enrollments.
     *
     * @see FINGERPRINT_CMD_DELETE_CHAR
     * @see EVENT_TEMPLATE_DELETE_FAIL
     */
    EVENT_TEMPLATE_DELETED,             /**< Fingerprint template deleted (FINGERPRINT_TEMPLATE_DELETED) */

    /**
     * @brief Event triggered when a fingerprint template is deleted.
     *
     * This event corresponds to the status FINGERPRINT_TEMPLATE_DELETE_FAIL.
     */
    EVENT_TEMPLATE_DELETE_FAIL,         /**< Fingerprint template deletion failed (FINGERPRINT_DELETE_TEMPLATE_FAIL) */

    /**
     * @brief Event triggered when the system enters low power mode.
     *
     * This event occurs after executing the FINGERPRINT_CMD_SLEEP command when
     * the fingerprint module successfully transitions to a low-power state to
     * conserve energy. In this state, the module has reduced functionality
     * until it is woken up.
     *
     * @note After this event, the module will not respond to most commands until
     *       it is woken up, typically by resetting the module or using a specific
     *       wake-up command if supported.
     *
     * @see FINGERPRINT_CMD_SLEEP
     * @see FINGERPRINT_OK
     */
    EVENT_LOW_POWER_MODE,               /**< Entered low power mode (FINGERPRINT_LOW_POWER_MODE) */

    /**
     * @brief Event triggered when an operation times out.
     *
     * This event occurs when a fingerprint operation fails to complete within
     * the expected time frame. This could be due to hardware issues, communication
     * problems, or other unexpected delays in processing.
     *
     * @note This event corresponds to the status code FINGERPRINT_TIMEOUT (0x26).
     *       Applications should typically retry the operation or check the module's
     *       status when this event occurs.
     *
     * @see FINGERPRINT_TIMEOUT
     */
    EVENT_TIMEOUT,                   /**< Operation timed out (FINGERPRINT_TIMEOUT) */

    /**
     * @brief Event triggered when no finger is detected on the sensor.
     *
     * This event occurs when the fingerprint module actively checks for a finger
     * on the sensor surface and confirms that no finger is present. This is useful
     * for detecting when a user has removed their finger during multi-step operations.
     *
     * @note This event corresponds to the status code FINGERPRINT_NO_FINGER (0x02).
     *
     * @see FINGERPRINT_CMD_GET_IMAGE
     * @see EVENT_FINGER_DETECTED
     */
    EVENT_NO_FINGER_DETECTED,           /**< No finger detected (FINGERPRINT_NO_FINGER_DETECTED) */

    /**
     * @brief Event triggered when the fingerprint scanner is ready for operation.
     *
     * This event occurs after the fingerprint module has successfully initialized
     * and is ready to accept commands. It typically follows power-on or reset operations
     * and indicates that the module's internal self-tests have completed successfully.
     *
     * @note Applications should wait for this event before attempting to send
     *       commands to the fingerprint module to ensure proper operation.
     *
     * @see fingerprint_init
     */
    EVENT_SCANNER_READY,                 /**< Scanner is ready for operation */

    /**
     * @brief Event triggered when fingerprint templates are successfully merged.
     *
     * This event occurs after executing the FINGERPRINT_CMD_REG_MODEL command when
     * the module successfully combines multiple fingerprint templates (typically from
     * different captures of the same finger) into a single composite template for
     * improved accuracy.
     *
     * @note Template merging is a key step in the enrollment process that improves
     *       the quality and reliability of fingerprint matching by accounting for
     *       slight variations in finger placement.
     *
     * @see FINGERPRINT_CMD_REG_MODEL
     * @see FINGERPRINT_OK
     */
    EVENT_TEMPLATE_MERGED,              /**< Fingerprint templates merged successfully */

    /**
     * @brief Event triggered when a fingerprint template is successfully stored in flash memory.
     *
     * This event occurs after executing the FINGERPRINT_CMD_STORE_CHAR command when
     * a template has been successfully written to the permanent storage of the fingerprint
     * module. The template is now available for future matching operations.
     *
     * @note This event signals the final step in template enrollment. After this event,
     *       the template ID is associated with the fingerprint and can be used for identification.
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see FINGERPRINT_OK
     * @see EVENT_TEMPLATE_STORE_PACKET_ERROR
     */
    EVENT_TEMPLATE_STORE_SUCCESS, /**< Fingerprint template successfully stored */

    /**
     * @brief Event triggered when there is an error receiving the storage packet.
     *
     * This event occurs when the fingerprint module encounters an error while receiving
     * the data packet for template storage. This typically indicates communication issues
     * or data corruption during the template storage process.
     *
     * @note This event corresponds to the status code FINGERPRINT_PACKET_ERROR (0x01).
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see FINGERPRINT_PACKET_ERROR
     * @see EVENT_TEMPLATE_STORE_SUCCESS
     */
    EVENT_TEMPLATE_STORE_PACKET_ERROR, /**< Error in receiving the fingerprint storage packet */

    /**
     * @brief Event triggered when the template storage page ID is out of valid range.
     *
     * This event occurs when attempting to store a template at a location that exceeds
     * the valid range of the fingerprint module's database. The valid range depends on
     * the specific module's capacity.
     *
     * @note This event corresponds to the status code FINGERPRINT_DB_RANGE_ERROR (0x0B).
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see FINGERPRINT_DB_RANGE_ERROR
     */
    EVENT_TEMPLATE_STORE_OUT_OF_RANGE, /**< PageID is out of range */

    /**
     * @brief Event triggered when a FLASH write error occurs during template storage.
     *
     * This event occurs when the fingerprint module encounters an internal error while
     * writing the template data to its FLASH memory. This typically indicates a hardware
     * issue or corruption of the module's internal storage.
     *
     * @note This event corresponds to the status code FINGERPRINT_FLASH_RW_ERROR (0x18).
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see FINGERPRINT_FLASH_RW_ERROR
     */
    EVENT_TEMPLATE_STORE_FLASH_ERROR, /**< Error writing to FLASH memory */

    /**
     * @brief Event triggered when the function does not match the encryption level.
     *
     * This event occurs when attempting to use a function that is incompatible with
     * the current encryption settings of the fingerprint module. Different operations
     * may require specific encryption levels to be enabled or disabled.
     *
     * @note This event corresponds to the status code FINGERPRINT_FUNCTION_ENCRYPTION_MISMATCH (0x31).
     *
     * @see FINGERPRINT_FUNCTION_ENCRYPTION_MISMATCH
     */
    EVENT_TEMPLATE_STORE_ENCRYPTION_MISMATCH, /**< Function does not match encryption level */

    /**
     * @brief Event triggered when illegal data is detected during template storage.
     *
     * This event occurs when the fingerprint module detects invalid or corrupted data
     * during the template storage process. This may indicate data corruption during
     * transmission or an invalid template format.
     *
     * @note This event corresponds to the status code FINGERPRINT_ILLEGAL_DATA (0x35).
     *
     * @see FINGERPRINT_CMD_STORE_CHAR
     * @see FINGERPRINT_ILLEGAL_DATA
     */
    EVENT_TEMPLATE_STORE_ILLEGAL_DATA, /**< Illegal data detected during storage */

    /**
     * @brief Event triggered when fingerprint search is successful.
     *
     * This event occurs after executing a search command when the fingerprint module
     * successfully finds a matching template in its database. The event data includes
     * the template ID of the match and the match confidence score.
     *
     * @note The match details are available in event.data.match_info, including
     *       the template_id of the matching fingerprint and the match_score indicating
     *       the confidence level of the match.
     *
     * @see FINGERPRINT_CMD_SEARCH
     * @see FINGERPRINT_OK
     * @see EVENT_SEARCH_FAIL
     */
    EVENT_SEARCH_SUCCESS,  /**< Event code for successful fingerprint search */

    /**
     * @brief Event triggered when the number of valid templates is retrieved.
     *
     * This event occurs after executing the FINGERPRINT_CMD_TEMPLATE_CNT command when
     * the module successfully reports the number of templates currently stored in its database.
     *
     * @note The template count is available in event.data.template_count.count and represents
     *       the total number of valid fingerprint templates currently stored in the module.
     *
     * @see FINGERPRINT_CMD_TEMPLATE_CNT
     * @see FINGERPRINT_OK
     */
    EVENT_TEMPLATE_COUNT, /**< Template count updated (FINGERPRINT_TEMPLATE_COUNT) */

    /**
     * @brief Event triggered when the index table is successfully read.
     *
     * This event occurs after executing the FINGERPRINT_CMD_GET_INDEX_TBL command when
     * the module successfully retrieves its template index table. The index table contains
     * information about which template slots are occupied and which are free.
     *
     * @note The index table data is available in the event.data or event.multi_packet structure,
     *       depending on the implementation, and provides a bitmap of occupied template slots.
     *
     * @see FINGERPRINT_CMD_GET_INDEX_TBL
     * @see FINGERPRINT_OK
     */
    EVENT_INDEX_TABLE_READ, /**< Index table read completed (FINGERPRINT_INDEX_TABLE_READ) */

    /**
     * @brief Event triggered when a fingerprint model is successfully created.
     *
     * This event occurs after executing the FINGERPRINT_CMD_REG_MODEL command when
     * the module successfully combines multiple fingerprint templates into a single
     * model. This is a critical step in the enrollment process where multiple scans
     * are combined for improved accuracy.
     *
     * @note This event indicates only that the model creation was successful.
     *       To permanently store the model, a subsequent FINGERPRINT_CMD_STORE_CHAR
     *       command must be issued.
     *
     * @see FINGERPRINT_CMD_REG_MODEL
     * @see FINGERPRINT_OK
     * @see EVENT_TEMPLATE_STORE_SUCCESS
     */
    EVENT_MODEL_CREATED, /**< Fingerprint model successfully created (FINGERPRINT_MODEL_CREATED) */

    /**
     * @brief Event triggered when a fingerprint template is successfully uploaded from the module.
     *
     * This event occurs when a template has been completely retrieved from the fingerprint module
     * and is available for processing. The template data is accessible through the event's
     * multi_packet structure or template_data fields.
     *
     * @note The template data is temporarily stored in memory and should be processed or
     *       saved promptly. The template_data property in the event structure should be
     *       checked for NULL before use.
     *
     * @see FINGERPRINT_CMD_UP_CHAR
     * @see EVENT_TEMPLATE_UPLOAD_FAIL
     */
    EVENT_TEMPLATE_UPLOADED,  /**< Template uploaded from module */

    /**
     * @brief Event triggered when a fingerprint template is successfully downloaded to the module.
     *
     * This event occurs after a complete template has been sent to the fingerprint module
     * and accepted. It indicates that the template data transfer was successful and
     * the module is ready to store the template.
     *
     * @note This event only indicates successful data transfer. To permanently store the template,
     *       a subsequent command to store the template (FINGERPRINT_CMD_STORE_CHAR) must be issued.
     *
     * @see FINGERPRINT_CMD_DOWN_CHAR
     * @see EVENT_TEMPLATE_RESTORED_SUCCESSUL
     * @see EVENT_TEMPLATE_RESTORED_FAIL
     */
    EVENT_TEMPLATE_DOWNLOADED,  /**< Template downloaded to module */

    /**
     * @brief Event triggered when the fingerprint database is successfully cleared.
     *
     * This event occurs after executing the FINGERPRINT_CMD_EMPTY command when
     * the module successfully erases all stored templates from its database,
     * returning it to an empty state.
     *
     * @note After this event, all template slots are free and available for
     *       new enrollments. This operation cannot be undone.
     *
     * @see FINGERPRINT_CMD_EMPTY
     * @see FINGERPRINT_OK
     */
    EVENT_DB_CLEARED,  /**< Database emptied successfully */

    /**
     * @brief Event triggered when system parameters are successfully read.
     *
     * This event occurs after executing the FINGERPRINT_CMD_READ_SYS_PARA command when
     * the module's system parameters have been successfully retrieved. The parameters
     * include information like status register values, security level, and database size.
     *
     * @note The parameters are available in the event.data.sys_params structure
     *       and include fields like status_register, template_size, database_size,
     *       security_level, device_address, data_packet_size, and baud_rate.
     *
     * @see FINGERPRINT_CMD_READ_SYS_PARA
     * @see FINGERPRINT_OK
     */
    EVENT_SYS_PARAMS_READ,  /**< System parameters read successfully */

    /**
     * @brief Event triggered when a template is found to already exist in the database.
     *
     * This event occurs during fingerprint operations (typically enrollment) when
     * the module detects that the current fingerprint is already enrolled in the database.
     * This helps prevent duplicate entries of the same finger.
     *
     * @note The event.data often includes information about the duplicate template's
     *       location in the database, which can be useful for informing the user.
     *
     * @see FINGERPRINT_ALREADY_EXISTS
     */
    EVENT_TEMPLATE_EXISTS,  /**< Fingerprint template successfully loaded into buffer */

    /**
     * @brief Event triggered when template upload from the fingerprint module fails.
     *
     * This event occurs when the fingerprint module cannot upload a requested template,
     * which often indicates the template doesn't exist or there are communication issues.
     * 
     * @note This event may occur with status codes FINGERPRINT_READ_TEMPLATE_ERROR,
     *       FINGERPRINT_UPLOAD_FEATURE_FAIL, or FINGERPRINT_DB_EMPTY depending on
     *       the specific cause of failure.
     *
     * @see FINGERPRINT_CMD_UP_CHAR
     * @see EVENT_TEMPLATE_UPLOADED
     */
    EVENT_TEMPLATE_UPLOAD_FAIL,  /**< Fingerprint template upload failed */

    /**
     * @brief Event triggered when the information page is successfully read.
     *
     * This event occurs after executing the FINGERPRINT_CMD_READ_INF_PAGE command and
     * receiving a complete response from the fingerprint module. The information page
     * typically contains device-specific details such as firmware version and other metadata.
     *
     * @note The information page data is available in the event's multi_packet structure or
     *       template_data field. This data may need to be parsed according to the module's
     *       specific format specification.
     *
     * @see FINGERPRINT_CMD_READ_INF_PAGE
     * @see FINGERPRINT_OK
     */
    EVENT_INFO_PAGE_READ,

    /**
     * @brief Event triggered when a fingerprint template is successfully loaded from flash memory into the buffer.
     *
     * This event occurs after executing the `PS_LoadChar` command, indicating that the fingerprint template 
     * has been retrieved from the module’s internal storage and is now available for further operations such as 
     * matching, uploading, or storing in another location.
     *
     * @note Ensure that the `PS_LoadChar` command completes successfully before relying on this event.
     *       If the template is corrupt or missing, subsequent operations like `PS_UpChar` may return zeroed data.
     *
     * @see PS_LoadChar
     * @see EVENT_TEMPLATE_UPLOADED
     * @see EVENT_TEMPLATE_MATCHED
     */
    EVENT_TEMPLATE_LOADED,

    /**
     * @brief Event triggered when the chip address is successfully set.
     *
     * This event indicates that the FINGERPRINT_CMD_SET_CHIP_ADDR command was
     * executed successfully and the module's address has been updated. After this
     * event, the module will only respond to commands sent to the new address.
     *
     * @note After changing the address, all subsequent commands must be sent to
     *       the new address or the module will not respond.
     *
     * @see FINGERPRINT_CMD_SET_CHIP_ADDR
     * @see EVENT_SET_CHIP_ADDRESS_FAIL
     */
    EVENT_SET_CHIP_ADDRESS_SUCCESS,
    
    /**
     * @brief Event triggered when setting the chip address fails.
     *
     * This event indicates that the FINGERPRINT_CMD_SET_CHIP_ADDR command
     * failed to execute, possibly due to communication issues or
     * invalid address values.
     *
     * @note When this event occurs, the module's address remains unchanged
     *       and commands should continue to be sent to the original address.
     *
     * @see FINGERPRINT_CMD_SET_CHIP_ADDR
     * @see EVENT_SET_CHIP_ADDRESS_SUCCESS
     */
    EVENT_SET_CHIP_ADDRESS_FAIL,

    /**
     * @brief Event triggered when an attempt to receive a packet fails.
     *
     * This event occurs when there is a failure in receiving an expected data packet
     * during multi-packet operations like template uploading or downloading. This
     * typically indicates communication errors or protocol issues.
     *
     * @note This event may be associated with various error codes including
     *       FINGERPRINT_PACKET_ERROR (0x01) or FINGERPRINT_TIMEOUT (0x26).
     *
     * @see FINGERPRINT_PACKET_ERROR
     * @see EVENT_TEMPLATE_UPLOADED
     */
    EVENT_PACKET_RECEPTION_FAIL, /**< Failed to receive subsequent data packets */

    /**
     * @brief Event triggered when a fingerprint is successfully enrolled.
     *
     * This event occurs when the complete enrollment process (multiple image captures, 
     * feature extraction, template generation and storage) completes successfully.
     * The event data contains enrollment details including the template ID, number of 
     * attempts required, and whether a duplicate was found.
     *
     * @note The enrollment_info structure in the event data contains the template_id, 
     *       is_duplicate flag, and attempts count fields with detailed enrollment information.
     *
     * @see fingerprint_data.enrollment_info
     * @see FINGERPRINT_OK
     * @see EVENT_ENROLLMENT_FAIL
     */
    EVENT_ENROLLMENT_COMPLETE,   /**< Fingerprint enrollment process completed */

    /**
     * @brief Event triggered when a fingerprint enrollment process fails.
     *
     * This event occurs when the enrollment process cannot be completed due to issues 
     * such as poor fingerprint quality, inconsistent placement, or internal errors
     * in the fingerprint module.
     *
     * @note The enrollment_info structure in the event data still contains information
     *       about the failed enrollment attempt, including the number of attempts made.
     *
     * @see fingerprint_data.enrollment_info
     * @see FINGERPRINT_ENROLL_FAIL
     * @see EVENT_ENROLLMENT_COMPLETE
     */
    EVENT_ENROLLMENT_FAIL,       /**< Fingerprint enrollment process failed */

 } fingerprint_event_type_t;

 /**
 * @struct fingerprint_sys_params_t
 * @brief Structure containing system parameters from the fingerprint module.
 *
 * This structure stores various system-level parameters retrieved from the 
 * fingerprint module, including device configuration, security settings, 
 * and communication parameters.
 */
typedef struct {
    /**
     * @brief Status register of the fingerprint module.
     *
     * This register contains the current operational status and any error codes
     * related to the fingerprint module.
     */
    uint16_t status_register;

    /**
     * @brief Template size for fingerprint data.
     * 
     * Specifies the size of a single fingerprint template in bytes.
     */
    uint16_t template_size;

    /**
     * @brief Number of fingerprints that can be stored in the library.
     *
     * Represents the maximum capacity of stored fingerprint templates
     * in the module's internal storage.
     */
    uint16_t database_size;

    /**
     * @brief Security level setting of the fingerprint module.
     *
     * Determines the strictness of fingerprint matching. Higher values
     * indicate stricter matching conditions.
     */
    uint16_t security_level;

    /**
     * @brief Device address used for module communication.
     *
     * The fingerprint module communicates using a unique 32-bit address.
     * The default value is `0xFFFFFFFF` (broadcast mode).
     */
    uint32_t device_address;

    /**
     * @brief Data packet size for communication.
     *
     * Specifies the length of data packets exchanged between the fingerprint
     * module and the microcontroller. Common values: 32, 64, 128, or 256 bytes.
     */
    uint16_t data_packet_size;

    /**
     * @brief Baud rate setting for UART communication.
     *
     * Determines the communication speed between the fingerprint module
     * and the microcontroller. Typically set in multiples of 9600 baud.
     */
    uint16_t baud_rate;


} fingerprint_sys_params_t;


/**
 * @struct fingerprint_match_info_t
 * @brief Stores detailed match information when a fingerprint match is successful.
 *
 * This structure holds the match result details, including the raw page ID 
 * from the fingerprint sensor, the converted template ID, and the match score.
 */
typedef struct {
    uint16_t page_id;     /**< Raw page ID returned by the fingerprint module */
    uint16_t template_id; /**< Converted template ID for application use */
    uint16_t match_score; /**< Confidence score of the match */
} fingerprint_match_info_t;

/**
 * @struct fingerprint_template_count_t
 * @brief Stores the count of enrolled fingerprints in the module.
 *
 * This structure is used when retrieving the number of stored fingerprint templates.
 */
typedef struct {
    uint16_t count; /**< Number of enrolled fingerprints */
} fingerprint_template_count_t;

// In fingerprint.h - Add this structure
typedef struct {
    uint8_t* data;    // Pointer to complete template data
    size_t size;      // Size of template data in bytes
    bool is_complete; // Flag indicating if template is complete
} fingerprint_template_buffer_t;

/**
 * @struct fingerprint_enrollment_info_t
 * @brief Stores information about a completed fingerprint enrollment.
 *
 * This structure contains details about a fingerprint enrollment operation,
 * including the template ID and storage location of the enrolled fingerprint.
 */
typedef struct {
    uint16_t template_id;    /**< The ID assigned to the newly enrolled fingerprint template */
    bool is_duplicate;       /**< Flag indicating if this fingerprint matches an existing one */
    uint8_t attempts;        /**< Number of attempts made during the enrollment process */
} fingerprint_enrollment_info_t;

/**
 * @struct fingerprint_event_t
 * @brief Defines a generic fingerprint event structure with flexible response types.
 *
 * This structure represents an event triggered by the fingerprint module. It 
 * includes the event type, response status, and a flexible data union that can 
 * store different types of structured response data.
 */
typedef struct {
    fingerprint_event_type_t type;  /**< The type of fingerprint event */
    fingerprint_status_t status;    /**< Status code returned from the fingerprint module */
    FingerprintPacket packet;       /**< Raw response packet (for backward compatibility) */
    MultiPacketResponse *multi_packet; /**< Multi-packet response data */
    uint8_t command;                /**< Command byte associated with the event */
    /**
     * @union data
     * @brief Stores structured response data depending on the event type.
     *
     * This union allows different event types to carry relevant data, improving 
     * readability and reducing the need for manual parsing in the main application.
     */
    union {
        fingerprint_match_info_t match_info;   /**< Data for fingerprint match events */
        fingerprint_template_count_t template_count; /**< Data for template count events */
        fingerprint_sys_params_t sys_params;     // Added system parameters
        fingerprint_template_buffer_t template_data; /**< Fingerprint template data */
        fingerprint_enrollment_info_t enrollment_info; /**< Enrollment information */
        // Extend with additional structured types as needed
    } data;
} fingerprint_event_t;

 
 /**
  * @brief Typedef for the fingerprint event handler callback.
  *
  * This type is used to define a function pointer for handling fingerprint-related events.
  * The function pointed to by the callback should take a single parameter of type `fingerprint_event_type_t`,
  * representing the specific event that occurred.
  *
  * @param event The fingerprint event that occurred. This is an enumeration value of type `fingerprint_event_type_t`.
  */
 typedef void (*fingerprint_event_handler_t)(fingerprint_event_t event);
 
 /**
  * @brief Global pointer to the fingerprint event handler function.
  *
  * This pointer is used to assign a function that will handle fingerprint-related events.
  * The function should be defined by the user and should match the `fingerprint_event_handler_t` callback type.
  * The event handler is called whenever a fingerprint-related event occurs, such as detection, image capture, or match results.
  */
 extern fingerprint_event_handler_t g_fingerprint_event_handler;
 
 /**
  * @brief Function to register an event handler for fingerprint events.
  * 
  * This function allows the application to register a callback handler that will
  * be called when a fingerprint event is triggered. The handler will process the
  * event according to its type (e.g., finger detected, match success, etc.).
  * 
  * @param handler The function pointer to the event handler function.
  */
 void register_fingerprint_event_handler(void (*handler)(fingerprint_event_t));
 
/**
 * @brief Triggers a fingerprint event and processes it.
 * 
 * This function initializes a fingerprint event structure with the given 
 * event type and status, then passes it to the event handler.
 * 
 * @param event_type The high-level event type to trigger.
 * @param status The original fingerprint status that caused the event.
 */
void trigger_fingerprint_event(fingerprint_event_t event);

/**
 * @brief Performs manual fingerprint enrollment at a specified storage location.
 *
 * This function executes a step-by-step fingerprint enrollment process, handling
 * user interaction and ensuring proper fingerprint capture and storage at the given location.
 * 
 * ## Enrollment Process:
 * 1. Wait for a finger to be placed on the scanner (`PS_GetImage`).
 * 2. If a fingerprint is detected, generate the first fingerprint template (`PS_GenChar1`).
 * 3. Prompt the user to remove their finger and wait for confirmation of removal.
 * 4. Once the finger is removed, wait for the same finger to be placed again.
 * 5. Capture the fingerprint again (`PS_GetImage`) and generate the second template (`PS_GenChar2`).
 * 6. Merge both templates into a single fingerprint model (`PS_RegModel`).
 * 7. Store the fingerprint template in the fingerprint module’s database at the specified `location` (`PS_StoreChar`).
 *
 * ## Error Handling:
 * - If a finger is not detected within the allowed retries, the function will return an error.
 * - If the finger is not removed within a timeout period, the process will restart.
 * - If enrollment fails at any step, it retries up to three times before failing.
 *
 * @note This function does not run as a FreeRTOS task but can be called from one.
 *       It dynamically creates an event group for synchronization and deletes it upon completion.
 *
 * @param[in] location  The storage location (ID) where the fingerprint template will be saved.
 *
 * @return
 * - `ESP_OK` if fingerprint enrollment is successful.
 * - `ESP_FAIL` if enrollment fails after the maximum number of attempts.
 * - `ESP_ERR_NO_MEM` if memory allocation for the event group fails.
 */
esp_err_t enroll_fingerprint(uint16_t location);


/**
 * @brief Event bit indicating a successful fingerprint enrollment.
 * 
 * This bit is set in the `enroll_event_group` when a fingerprint enrollment 
 * operation completes successfully.
 */
#define ENROLL_BIT_SUCCESS BIT0

/**
 * @brief Event bit indicating a failed fingerprint enrollment.
 * 
 * This bit is set in the `enroll_event_group` when a fingerprint enrollment 
 * operation fails due to issues such as poor fingerprint quality or sensor errors.
 */
#define ENROLL_BIT_FAIL BIT1

/**
 * @brief Event bit flag for checking if a fingerprint template location is available.
 * 
 * This bit is set in the event group to indicate that the system is currently verifying 
 * whether a specified fingerprint template location is occupied.
 * 
 * @note Used in conjunction with `enroll_event_group` to synchronize location checking.
 */
#define CHECKING_LOCATION_BIT BIT2

/**
 * @brief Event bit indicating information page reading is complete
 * 
 * This bit is set in the enroll_event_group when the fingerprint module's
 * information page has been successfully read and processed.
 */
#define INFO_PAGE_COMPLETE_BIT BIT3  // You may need to adjust the bit position

/**
 * @brief Event bit indicating a fingerprint has been found as a duplicate
 * 
 * This bit is set in the enroll_event_group when a fingerprint being enrolled
 * is detected as a duplicate of an existing template in the database.
 */
#define DUPLICATE_FOUND_BIT BIT4 

/**
 * @brief Event group handle for fingerprint enrollment status.
 * 
 * This FreeRTOS event group is used to synchronize fingerprint enrollment 
 * operations by setting event bits based on success or failure.
 */
static EventGroupHandle_t enroll_event_group = NULL;

/**
 * @brief Verifies a fingerprint by capturing an image, extracting features, and searching in the database.
 *
 * This function attempts to verify a fingerprint up to a maximum number of attempts.
 * It captures a fingerprint image, processes it, and searches for a match in the database.
 * If a match is found, the function returns ESP_OK; otherwise, it returns ESP_FAIL after all attempts.
 *
 * @return 
 *      - ESP_OK if the fingerprint matches an entry in the database.
 *      - ESP_FAIL if verification fails after the maximum attempts.
 *      - ESP_ERR_NO_MEM if the event group could not be created.
 */
esp_err_t verify_fingerprint(void);

/**
 * @brief Deletes a fingerprint template from the specified storage location.
 *
 * This function sends a command to delete a fingerprint stored at the given
 * location in the fingerprint module. It waits for a response event to confirm success.
 *
 * @param[in] location The storage location of the fingerprint template (valid range depends on module).
 * @return
 *      - ESP_OK on successful deletion.
 *      - ESP_ERR_NO_MEM if event group creation fails.
 *      - ESP_FAIL if deletion is unsuccessful.
 */
esp_err_t delete_fingerprint(uint16_t location);

/**
 * @brief Clears the entire fingerprint database.
 *
 * This function sends a command to erase all stored fingerprints in the fingerprint module.
 * It waits for a response event to confirm success.
 *
 * @return
 *      - ESP_OK on successful database clearance.
 *      - ESP_ERR_NO_MEM if event group creation fails.
 *      - ESP_FAIL if clearance fails.
 */
esp_err_t clear_database(void);

/**
 * @brief Checks if the scanned fingerprint already exists in the database.
 *
 * This function sends a search command to the fingerprint module to compare 
 * the scanned fingerprint against stored templates within a predefined range.
 *
 * @return 
 *      - ESP_OK if the search command is sent successfully.
 *      - ESP_FAIL if there is an error sending the search command.
 */
esp_err_t check_duplicate_fingerprint(void);

/**
 * @brief Validates whether a given template storage location is within range.
 *
 * This function sends a command to the fingerprint module to check if the specified 
 * template location exists in the storage index table.
 *
 * @param[in] location The template storage location to validate.
 *
 * @return 
 *      - ESP_OK if the location is valid.
 *      - ESP_FAIL if the index table read operation fails.
 */
esp_err_t validate_template_location(uint16_t location);


/**
 * @brief Get the number of enrolled fingerprints.
 *
 * This function retrieves the count of fingerprints that have been enrolled in the fingerprint scanner.
 *
 * @param[out] count Pointer to a variable where the count of enrolled fingerprints will be stored.
 *
 * @return
 *     - ESP_OK: Successfully retrieved the count.
 *     - ESP_ERR_INVALID_ARG: The provided argument is invalid.
 *     - ESP_FAIL: Failed to retrieve the count due to other reasons.
 */
esp_err_t get_enrolled_count(void);

/**
 * @brief Indicates whether an enrollment process is currently in progress.
 * 
 * This external boolean variable is used to track the state of the fingerprint
 * enrollment process. When `true`, it signifies that the enrollment process
 * is ongoing. When `false`, it indicates that no enrollment is currently
 * taking place.
 */
extern bool enrollment_in_progress;

/**
 * @brief Converts fingerprint module page ID to a sequential page index
 * 
 * Maps page IDs to their actual indices:
 * - Page ID 0   -> Index 0
 * - Page ID 256 -> Index 1
 * - Page ID 512 -> Index 2
 * - Page ID 768 -> Index 3
 * 
 * @param page_id The raw page ID from the fingerprint module
 * @return uint16_t The sequential page index
 */
uint16_t convert_page_id_to_index(uint16_t page_id);

/**
 * @brief Converts a sequential page index back to the fingerprint module page ID
 * 
 * Maps indices back to their page IDs:
 * - Index 0 -> Page ID 0
 * - Index 1 -> Page ID 256
 * - Index 2 -> Page ID 512
 * - Index 3 -> Page ID 768
 * 
 * @param index The sequential page index
 * @return uint16_t The corresponding page ID
 */
uint16_t convert_index_to_page_id(uint16_t index);

/**
 * @brief Reads system parameters from the fingerprint module.
 */
esp_err_t read_system_parameters(void);

/**
 * @brief Loads a fingerprint template from flash storage into the module's template buffer.
 *
 * This function reads a stored fingerprint template from the flash database and loads 
 * it into the specified template buffer for further processing (e.g., matching, modification).
 *
 * @param template_id The unique ID of the fingerprint template in flash.
 * @param buffer_id The buffer ID where the template should be loaded.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t load_template_to_buffer(uint16_t template_id, uint8_t buffer_id);

/**
 * @brief Uploads a fingerprint template from the module's buffer to the host system.
 *
 * This function transfers a fingerprint template stored in the module's buffer to the host
 * (e.g., microcontroller or computer) for backup, analysis, or transmission.
 *
 * @param buffer_id The buffer ID containing the fingerprint template.
 * @param template_data Pointer to the buffer where the uploaded template will be stored.
 * @param template_size Pointer to a variable that will store the size of the template.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t upload_template(uint8_t buffer_id, uint8_t* template_data, size_t* template_size);

/**
 * @brief Stores a fingerprint template from the buffer into the module's flash memory.
 *
 * This function saves a fingerprint template currently in the module's buffer 
 * into the permanent flash database for later use.
 *
 * @param buffer_id The buffer ID containing the fingerprint template.
 * @param template_id The unique ID under which the template should be stored.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t store_template(uint8_t buffer_id, uint16_t template_id);

/**
 * @brief Backs up a fingerprint template by reading it from flash and storing it in a host system.
 *
 * This function retrieves a fingerprint template from the module's flash storage,
 * uploads it to the host, and allows it to be stored elsewhere (e.g., external memory).
 *
 * @param template_id The ID of the fingerprint template to be backed up.
 * @return ESP_OK on success, ESP_FAIL on failure.
 */
esp_err_t backup_template(uint16_t template_id);

/**
 * @brief Initializes the enrollment event group if it doesn't exist and clears any pending event bits.
 *
 * This function ensures that the `enroll_event_group` is properly initialized before use.
 * If the event group does not exist, it will be created. Additionally, it clears
 * `ENROLL_BIT_SUCCESS` and `ENROLL_BIT_FAIL` to remove any previous events.
 *
 * @return ESP_OK if successful, ESP_ERR_NO_MEM if event group creation fails.
 */
esp_err_t initialize_event_group(void);

/**
 * @brief Cleans up the enrollment event group.
 *
 * Deletes the event group if it exists and sets it to NULL.
 * Ensures that no invalid memory is accessed after deletion.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if the event group was already NULL.
 */
esp_err_t cleanup_event_group(void);

/**
 * @brief Reads the information page from the fingerprint module.
 *
 * This function sends the Read Information Page command to the fingerprint module 
 * and retrieves details such as firmware version, serial number, and other metadata.
 *
 * @return 
 *      - `ESP_OK` if the information page is successfully read.
 *      - `ESP_ERR_INVALID_RESPONSE` if an unexpected response is received.
 *      - `ESP_FAIL` if communication with the module fails.
 */
esp_err_t read_info_page(void);

typedef enum {
    WAIT_HEADER,       // Waiting for 0xEF01 header
    READ_ADDRESS,      // Reading 4-byte address
    READ_PACKET_ID,    // Reading packet type
    READ_LENGTH,       // Reading 2-byte length
    READ_CONTENT,      // Reading packet content
    READ_CHECKSUM      // Reading 2-byte checksum
} ParserState;

/**
 * @brief Event bit indicating template upload has completed
 * 
 * This bit is set in the enroll_event_group when a template upload operation
 * has successfully completed and all template data has been received.
 */
#define TEMPLATE_UPLOAD_COMPLETE_BIT (1 << 3) // Bit 3 for template upload complete

FingerprintPacket* extract_packet_from_raw_data(uint8_t* data, size_t data_len, uint8_t target_packet_id);

/**
 * @brief Restores a fingerprint template from a MultiPacketResponse structure
 *
 * This function takes a template stored in a MultiPacketResponse structure (as received during
 * template upload), and downloads it to the fingerprint module, storing it at the specified location.
 *
 * @param template_id The ID where to store the template in the fingerprint database
 * @param response Pointer to the MultiPacketResponse structure containing template data
 * @return ESP_OK on success, or appropriate error code on failure
 */
esp_err_t restore_template_from_multipacket(uint16_t template_id, MultiPacketResponse *response);

/**
 * @brief Task that waits for finger detection interrupts and processes them.
 * 
 * This task waits for signals from the finger detection interrupt and
 * initiates the fingerprint capture and processing when a finger is detected.
 * 
 * @param pvParameter Unused parameter (NULL by default).
 */
void finger_detection_task(void *pvParameter);

/**
 * @enum finger_operation_mode_t
 * @brief Defines the operational modes for fingerprint detection
 *
 * This enumeration controls how the fingerprint detection system behaves
 * when a finger is detected. Each mode corresponds to a specific operation
 * sequence that will be executed automatically.
 */
typedef enum {
    FINGER_OP_NONE = 0,       /*!< No specific operation, default search behavior */
    FINGER_OP_VERIFY,         /*!< Verification mode - search database for match */
    FINGER_OP_ENROLL_FIRST,   /*!< First step of enrollment - capture first image */
    FINGER_OP_ENROLL_SECOND,  /*!< Second step of enrollment - capture second image */
    FINGER_OP_CUSTOM          /*!< Custom operation mode - caller handles next steps */
} finger_operation_mode_t;

/**
 * @brief Set the operation mode for the finger detection task
 * 
 * This function configures how the finger detection task will process
 * the next detected fingerprint.
 * 
 * @param mode The operation mode to set
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t fingerprint_set_operation_mode(finger_operation_mode_t mode);

/**
 * @brief Get the current operation mode of the finger detection task
 * 
 * @return The current operation mode
 */
finger_operation_mode_t fingerprint_get_operation_mode(void);

/**
 * @brief Wait for a finger to be detected and processed according to the current operation mode
 * 
 * This function blocks until a finger is detected and processed, or until the timeout expires.
 * 
 * @param timeout_ms Timeout in milliseconds, or 0 for no timeout
 * @return ESP_OK if finger was detected and processed successfully,
 *         ESP_ERR_TIMEOUT if the timeout expired,
 *         or another error code on failure
 */
esp_err_t fingerprint_wait_for_finger(uint32_t timeout_ms);

/**
 * @brief Free all resources associated with a fingerprint event
 * 
 * This function properly cleans up any memory allocated during event processing.
 * It should be called after an event has been fully processed.
 * 
 * @param event Pointer to the event to clean up
 */
void fingerprint_event_cleanup(fingerprint_event_t* event);

/**
 * @brief Set the fingerprint module's device address
 *
 * This function changes the address used to communicate with the fingerprint
 * module. The function sends the command to the current address and configures
 * the module to respond to the new address afterwards.
 *
 * @param[in] new_address The 32-bit device address to set
 * @param[in] current_address The current 32-bit device address
 * @return esp_err_t ESP_OK on success, or an error code on failure
 */
esp_err_t fingerprint_set_address(uint32_t new_address, uint32_t current_address);


 #ifdef __cplusplus
 }
 #endif
 
 #endif // FINGERPRINT_H