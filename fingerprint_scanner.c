#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "fingerprint.h"

#define TAG "FINGERPRINT"
static void internal_handle_fingerprint_event(fingerprint_event_t event);

void app_main(void)
{
    register_fingerprint_event_handler(internal_handle_fingerprint_event);

    esp_err_t err = fingerprint_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fingerprint initialization failed");
        return;
    }
    // vTaskDelay(pdMS_TO_TICKS(3000));  // Wait for a second to ensure the module is ready

    // // Allow module to stabilize before sending commands
    // vTaskDelay(pdMS_TO_TICKS(100));

    err = fingerprint_set_address(DEFAULT_FINGERPRINT_ADDRESS, DEFAULT_FINGERPRINT_ADDRESS);
    // vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Fingerprint scanner initialized and waiting for a finger to be detected.");
    uint16_t location = 0;  // Storage location for fingerprint template
    
    // err = enroll_fingerprint(location);
    // esp_err_t out = delete_fingerprint(location);

    // err = clear_database();
    
    ESP_LOGI(TAG, "Starting fingerprint verification...");
    vTaskDelay(pdMS_TO_TICKS(250));  // Delay before sending the next command
    esp_err_t result = verify_fingerprint();

    // Get number of enrolled fingerprints
    err = get_enrolled_count();

    // Read system parameters
    err = read_system_parameters();

    // // Backup Template
    // uint16_t template_id = 0;
    // ESP_LOGI(TAG, "1. Backing up template id 0x%04X", template_id);
    // err = backup_template(template_id);

    // // Wait for the backup to complete and template to be saved
    // vTaskDelay(pdMS_TO_TICKS(10000));



    // // Read information page
    // err = read_info_page();
    
}

void send_command_task(void *pvParameter)
{
    while (1)
    {
        ESP_LOGI(TAG, "Attempting to send Get Image command...");

        esp_err_t err = fingerprint_send_command(&PS_GetImage, DEFAULT_FINGERPRINT_ADDRESS);
        
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send Get Image command! Error: %d", err);
        } else {
            ESP_LOGI(TAG, "Get Image command sent successfully.");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay before sending the next command
    }
}

// Event handler function
static void internal_handle_fingerprint_event(fingerprint_event_t event) {
    char *tag;
    vTaskDelay(pdMS_TO_TICKS(50));  // Delay before processing the event and prevent watchdog trigger
    switch (event.type) {
        case EVENT_SCANNER_READY:
            // ESP_LOGI(TAG, "Fingerprint scanner is ready for operation. Status: 0x%02X", event.status);
            break;
        case EVENT_FINGER_DETECTED:
            ESP_LOGI(TAG, "Finger detected! Status: 0x%02X\n", event.status);
            // check_duplicate_fingerprint();
            // ESP_LOGI(TAG, "Event address: 0x%08lX", (unsigned long)event.packet.address);
            // ESP_LOG_BUFFER_HEX("Event packet address: ", &event.packet, sizeof(FingerprintPacket));
            // ESP_LOG_BUFFER_HEX("Finger Detected Parameters ", event.packet.parameters, sizeof(event.packet.parameters));
            break;
        case EVENT_IMAGE_CAPTURED:
            ESP_LOGI(TAG, "Fingerprint image captured successfully! Status: 0x%02X\n", event.status);
            break;
        case EVENT_FEATURE_EXTRACTED:
            ESP_LOGI(TAG, "Fingerprint features extracted successfully! Status: 0x%02X\n", event.status);
            // ESP_LOG_BUFFER_HEX("Feautre Extracted Parameters ", event.packet.parameters, sizeof(event.packet.parameters));
            break;
        case EVENT_MATCH_SUCCESS:
            tag = "MATCH SUCCESS";
            ESP_LOGI(tag, "EVENT MATCH SUCCESS");
            ESP_LOGI(tag, "Fingerprint match successful! Status: 0x%02X", event.status);
            ESP_LOGI(tag, "Match found at Enrollee ID: %d", event.data.match_info.template_id);
            ESP_LOGI(tag, "Match score: %d\n", event.data.match_info.match_score);
            break;
        case EVENT_MATCH_FAIL:
            ESP_LOGE(TAG, "Fingerprint mismatch. Status: 0x%02X\n", event.status);
            break;
        case EVENT_ERROR:
            ESP_LOGE(TAG, "An error occurred during fingerprint processing. Status: 0x%02X", event.status);
            ESP_LOGE(TAG, "Command: 0x%02X\n", event.command);
            break;
        case EVENT_NO_FINGER_DETECTED:
            // ESP_LOGI(TAG, "No finger detected. Status: 0x%02X\n", event.status);
            break;
        case EVENT_ENROLL_SUCCESS:
            ESP_LOGI(TAG, "Fingerprint enrollment successful! Status: 0x%02X", event.status);
            ESP_LOGI("Event Type", "Event: 0x%02X\n", event.type);
            break;
        case EVENT_ENROLL_FAIL:
            ESP_LOGI(TAG, "Fingerprint enrollment failed. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_MERGED:
            ESP_LOGI(TAG, "Fingerprint templates merged successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_STORE_SUCCESS:
            ESP_LOGI(TAG, "Fingerprint template stored successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_SEARCH_SUCCESS:
            tag = "SEARCH SUCCESS";
            ESP_LOGI(tag, "EVENT SEARCH SUCCESS");
            ESP_LOGI(tag, "Fingerprint search successful. Status: 0x%02X", event.status);
            ESP_LOGI(tag, "Match found at Enrollee ID: %d", event.data.match_info.template_id);
            ESP_LOGI(tag, "Match score: %d\n", event.data.match_info.match_score);
            break;
        case EVENT_INDEX_TABLE_READ:
            tag = "INDEX TABLE READ";
            ESP_LOGI(tag, "Index table read successful. Status: 0x%02X\n", event.status);
            // ESP_LOG_BUFFER_HEX("Index Table Parameters", event.packet.parameters, sizeof(event.packet.parameters));
            break;
        case EVENT_TEMPLATE_COUNT:
            ESP_LOGI("EVENT TEMPLATE COUNT", "Number of valid templates: %d\n", event.data.template_count.count);
            break;
        case EVENT_SYS_PARAMS_READ:
            tag = "SYSTEM PARAMETERS READ";
            ESP_LOGI(tag, "System parameters read successfully. Status: 0x%02X", event.status);
            ESP_LOGI(tag, "Status Register: 0x%04X", event.data.sys_params.status_register);
            ESP_LOGI(tag, "Fingerprint Template Size:: 0x%04X", event.data.sys_params.template_size);
            ESP_LOGI(tag, "Fingerprint Database Size: 0x%04X", event.data.sys_params.database_size);
            ESP_LOGI(tag, "Security Level: 0x%04X", event.data.sys_params.security_level);
            ESP_LOGI(tag, "Device Address: 0x%08" PRIX32, event.data.sys_params.device_address);  // Fix for uint32_t
            ESP_LOGI(tag, "Data Packet Size: %u bytes", event.data.sys_params.data_packet_size); // No need for hex
            ESP_LOGI(tag, "Baud Rate: %u bps\n", event.data.sys_params.baud_rate); // Convert baud multiplier to actual baud rate
            break;
    // Add this to the handle_fingerprint_event function
    case EVENT_TEMPLATE_UPLOADED:
        tag = "TEMPLATE UPLOADED";
        ESP_LOGI(tag, "Template uploaded\n");

        // Check if multi-packet data is available
        if (event.multi_packet != NULL) {
            ESP_LOGI(tag, "Multi-packet data available: %d packets\n", event.multi_packet->count);

            // Now log the fixed packets
            for (size_t i = 0; i < event.multi_packet->count; i++) {
                if (event.multi_packet->packets[i] != NULL) {
                    ESP_LOGI(tag, "Packet %d: ID=0x%02X, Address=0x%08X, Length=%d, Checksum=0x%04X", 
                        i, 
                        event.multi_packet->packets[i]->packet_id,
                        (unsigned int)event.multi_packet->packets[i]->address,
                        event.multi_packet->packets[i]->length,
                        (unsigned int)event.multi_packet->packets[i]->checksum);
            
                    
                    // // Print full packet data
                    // if (event.multi_packet->packets[i]->length > 2) {
                    //     ESP_LOG_BUFFER_HEX_LEVEL("Packet Data", 
                    //                             event.multi_packet->packets[i]->parameters,
                    //                             event.multi_packet->packets[i]->length - 2,
                    //                             ESP_LOG_INFO);
                    // }
                }
                vTaskDelay(pdMS_TO_TICKS(50));  // Prevent watchdog trigger
            }
            
            // If template data is available in the multi_packet
            if (event.multi_packet->template_data && event.multi_packet->template_size > 0) {
                ESP_LOGI(tag, "Complete template data received (%d bits)", event.multi_packet->template_size);
                ESP_LOG_BUFFER_HEXDUMP("Template Data", event.multi_packet->template_data, 
                                    (event.multi_packet->template_size > 64) ? 64 : event.multi_packet->template_size, 
                                    ESP_LOG_INFO);
            }

            // Make a deep copy of the template data to ensure we own it
            if (event.multi_packet != NULL) {
                // Log that we received a template
                
                if (event.multi_packet->template_data != NULL && 
                    event.multi_packet->template_size > 0) {
                    
                    uint16_t new_location = 9; // Target storage location
                    ESP_LOGI(tag, "Restoring template to location %d", new_location);
                    // Option 1: Directly use the event.multi_packet (simplest)
                    esp_err_t err = restore_template_from_multipacket(new_location, event.multi_packet);
                    
                    if (err == ESP_OK) {
                        ESP_LOGI(tag, "Template restored successfully to location %d", new_location);
                    } else {
                        ESP_LOGE(tag, "Failed to restore template: %s", esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGW(tag, "Template data is empty or missing");
                }
            }              
        } else {
            ESP_LOGW(tag, "No multi-packet data available in template uploaded event");
        }
        ESP_LOGI("","\n");
        break;

        case EVENT_TEMPLATE_EXISTS:
            tag = "TEMPLATE EXISTS";
            ESP_LOGI(tag, "Fingerprint template successfully loaded into buffer. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_UPLOAD_FAIL:
            tag = "TEMPLATE UPLOAD FAILED";
            ESP_LOGE(tag, "Fingerprint template upload failed. Status: 0x%02X\n", event.status);
            break;
        // Add to handle_fingerprint_event function
        case EVENT_INFO_PAGE_READ:
            tag = "INFO PAGE READ";
            ESP_LOGI(tag, "Information page read successfully. Status: 0x%02X", event.status);
            ESP_LOGI(tag, "Packet ID read successfully. Status: 0x%02X", event.packet.packet_id);
            ESP_LOGI(tag, "Packet length read successfully. Status: 0x%02X", event.packet.length);

            if (event.multi_packet != NULL)
            {
                // Now log the fixed packets
                for (size_t i = 0; i < event.multi_packet->count; i++) {
                    if (event.multi_packet->packets[i] != NULL) {
                        ESP_LOGI(TAG, "Packet %d: ID=0x%02X, Address=0x%08X, Length=%d, Checksum=0x%04X", 
                            i, 
                            event.multi_packet->packets[i]->packet_id,
                            (unsigned int)event.multi_packet->packets[i]->address,
                            event.multi_packet->packets[i]->length,
                            (unsigned int)event.multi_packet->packets[i]->checksum);
                
                        
                        // Print full packet data
                        if (event.multi_packet->packets[i]->length > 2) {
                            ESP_LOG_BUFFER_HEX_LEVEL("Packet Data", 
                                                    event.multi_packet->packets[i]->parameters,
                                                    event.multi_packet->packets[i]->length - 2,
                                                    ESP_LOG_INFO);
                        }
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));  // Prevent watchdog trigger
                }
            }
            else{
                ESP_LOGI(TAG, "No multi-packet data available in information page read event");
            }

            if (event.data.template_data.data != NULL) {
                ESP_LOGI(tag, "Information page data received (%d bytes)", event.data.template_data.size);
                ESP_LOG_BUFFER_HEXDUMP("Information Page Data", event.data.template_data.data, 
                                    (event.data.template_data.size > 64) ? 64 : event.data.template_data.size, 
                                    ESP_LOG_INFO);
                
                // Free the memory when done
                heap_caps_free(event.data.template_data.data);
            }
            ESP_LOGI("", "\n");	
            break;

        case EVENT_ENROLLMENT_COMPLETE:
            tag = "ENROLLMENT COMPLETE";
            ESP_LOGI(tag, "Fingerprint enrollment process completed. Status: 0x%02X", event.status);
            ESP_LOGI("ENROLLMENT INFO", "Enrollment ID: %d", event.data.enrollment_info.template_id);
            ESP_LOGI("ENROLLMENT INFO", "Have Duplicate: %d", event.data.enrollment_info.is_duplicate);	
            ESP_LOGI("ENROLLMENT INFO", "Attempts: %d\n", event.data.enrollment_info.attempts);	
            break;
        case EVENT_ENROLLMENT_FAIL:
            tag = "ENROLLMENT FAILED";
            ESP_LOGE(tag, "Fingerprint enrollment process failed. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_DELETED:
            tag = "TEMPLATE DELETED";
            ESP_LOGI(tag, "Fingerprint template deleted successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_DELETE_FAIL:
            tag = "TEMPLATE DELETE FAILED";
            ESP_LOGE(tag, "Failed to delete fingerprint template. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_RESTORED_SUCCESSUL:
            tag = "TEMPLATE STORED";
            ESP_LOGI(tag, "Fingerprint template stored successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_TEMPLATE_LOADED:
            ESP_LOGI(TAG, "Template loaded successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_DB_CLEARED:
            tag = "DATABASE CLEARED";
            ESP_LOGI(tag, "Fingerprint database cleared successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_SET_CHIP_ADDRESS_SUCCESS:
            tag = "CHIP ADDRESS SET";
            ESP_LOGI(tag, "Fingerprint chip address set successfully. Status: 0x%02X\n", event.status);
            break;
        case EVENT_SET_CHIP_ADDRESS_FAIL:
            tag = "CHIP ADDRESS SET FAILED";
            ESP_LOGE(tag, "Failed to set fingerprint chip address. Status: 0x%02X\n", event.status);
            break;
        default:
            tag = "UNKNOWN EVENT";
            ESP_LOGI(tag, "Unknown event triggered. Status: 0x%02X", event.status);
            ESP_LOGI(tag, "Event Type: 0x%02X\n", event.type);	
            break;
    }
}
