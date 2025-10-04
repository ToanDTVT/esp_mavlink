#include "gpio_config.h"

void gpio_init()
{
    /*Config GPIO for keypad*/
    gpio_config_t button_config;
    button_config.intr_type = GPIO_INTR_DISABLE;                           // Không có ngắt
    button_config.mode = GPIO_MODE_INPUT;                                  // Chế độ đầu ra
    button_config.pin_bit_mask = (1ULL << BUTTON_GPIO);                    // Chọn chân GPIO
    button_config.pull_down_en = 0;                                        // kéo xuống
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;                         //kéo lên
    gpio_config(&button_config);
}

void press_button()
{
    int button_state = gpio_get_level(BUTTON_GPIO);
    if (button_state == 0) // Nút được nhấn
    {
        printf("Button Pressed!\n");
        int evt = 1;
        xQueueSend(button_evt_queue, &evt, 0);
    }
    else
    {
        printf("Button Released!\n");
    }
}

void test_gpio_config(void)
{
    printf("This is a test function from gpio_config component.\n");
}

void button_task(void *pvParameters)
{
    while (1)
    {
        press_button();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Kiểm tra trạng thái nút mỗi giây
    }
}

void create_button_task()
{
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
}