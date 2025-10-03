#include "control_pixhawk.h"

static const char *TAG = "control_pixhawk";
void uart_init (void) {

    uart_config_t uart_config = {
        .baud_rate = 57600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh = 122
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_PIXHAWK, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_PIXHAWK, TX_PIN_PIXHAWK, RX_PIN_PIXHAWK, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_PIXHAWK, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));

}

void test_control_pixhawk(void)
{
    printf("This is a test function from control_pixhawk component.\n");
}

void send_mavlink_message(mavlink_message_t* msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    ESP_ERROR_CHECK(uart_write_bytes(UART_NUM_PIXHAWK, (const char *)buf, len));
}

void send_disarm () {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Tạo message COMMAND_LONG
    mavlink_msg_command_long_pack(
        255,               // system_id của GCS (Ground station)
        200,             // component_id của GCS
        &msg,
        1,               // target_system (thường là 1 = autopilot)
        1,               // target_component (thường là 1 = autopilot)
        MAV_CMD_COMPONENT_ARM_DISARM, // command ID
        0,               // confirmation
        0.0,             // param1 = 0 → DISARM (1.0 = ARM)
        21196,           // param2 = magic force number (có thể để 0 nếu không cần force)
        0,0,0,0,0        // các param khác
    );

    // Đóng gói ra buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int written = uart_write_bytes(UART_NUM_PIXHAWK, (const char *)buf, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
    } else {
        ESP_LOGI(TAG, "Wrote %d bytes", written);
        for (int i = 0; i < len; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    }

    // uint16_t len_read = uart_read_bytes(UART_NUM_PIXHAWK, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
    // if (len_read > 0) {
    //     ESP_LOGI(TAG, "Read %d bytes", len_read);
    //     for (int i = 0; i < len_read; i++) {
    //         printf("%02X ", buf[i]);
    //         if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, NULL)) {
    //             if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
    //                 uint16_t command = mavlink_msg_command_ack_get_command(&msg);
    //                 uint8_t result   = mavlink_msg_command_ack_get_result(&msg);
    //                 if (command == MAV_CMD_COMPONENT_ARM_DISARM) {
    //                     if (result == MAV_RESULT_ACCEPTED) {
    //                         printf("✅ DISARM/ARM command accepted!\n");
    //                     } else {
    //                         printf("❌ DISARM/ARM command failed, result=%d\n", result);
    //                     }
    //                 }
    //             } else {
    //                 printf("\n");
    //                 ESP_LOGW(TAG, "Received message with ID %d", msg.msgid);
    //             }
    //         } 
    //     }
    // } else {
    //     printf("\n");
    //     ESP_LOGW(TAG, "No response received");
    // }


    printf("\n");
}


// Hàm đọc dữ liệu MAVLink từ UART và parse ACK
void mavlink_receive_task(void *pvParameters) {
    uint8_t data[UART_BUF_SIZE];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (1) {
        int len = uart_read_bytes(UART_NUM_PIXHAWK, data, UART_BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
                    printf("[MAVLINK] msgid=%d sysid=%d compid=%d\n", msg.msgid, msg.sysid, msg.compid);
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_COMMAND_ACK: {
                            uint16_t command = mavlink_msg_command_ack_get_command(&msg);
                            uint8_t result   = mavlink_msg_command_ack_get_result(&msg);
                            printf("COMMAND_ACK received: command=%d, result=%d\n", command, result);
                            if (command == MAV_CMD_COMPONENT_ARM_DISARM) {
                                if (result == MAV_RESULT_ACCEPTED) {
                                    printf("✅ DISARM/ARM command accepted!\n");
                                } else {
                                    printf("❌ DISARM/ARM command failed, result=%d\n", result);
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_HEARTBEAT: {
                            mavlink_heartbeat_t hb;
                            mavlink_msg_heartbeat_decode(&msg, &hb);
                            printf("HEARTBEAT: type=%d, autopilot=%d, base_mode=%d, system_status=%d\n",
                                   hb.type, hb.autopilot, hb.base_mode, hb.system_status);
                            break;
                        }
                        case MAVLINK_MSG_ID_COMMAND_LONG: {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            printf("COMMAND_LONG: command=%d, param1=%.2f, param2=%.2f\n",
                                   cmd.command, cmd.param1, cmd.param2);
                            break;
                        }
                        default:
                            printf("[MAVLINK] Unknown/Other msgid=%d\n", msg.msgid);
                            break;
                    }
                }
            }
        }
    }
}

void create_mavlink_receive_task() {
    xTaskCreate(mavlink_receive_task, "mavlink_receive_task", 4096, NULL, 10, NULL);
}



void send_heartbeat(void)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(255, MAV_COMP_ID_MISSIONPLANNER, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int written = uart_write_bytes(UART_NUM_PIXHAWK, (const char *)buf, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
    } else {
        ESP_LOGI(TAG, "Wrote %d bytes", written);
        for (int i = 0; i < len; i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    }
    printf("Sent HEARTBEAT message.\n");
}
