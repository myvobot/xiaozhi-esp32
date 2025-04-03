#include "button.h"
#include "config.h"
#include <esp_log.h>
#include <esp_sleep.h>
#include "wifi_board.h"
#include "application.h"
#include <wifi_station.h>
#include "power_manager.h"
#include "esp_lcd_st7735.h"
#include "power_save_timer.h"
#include "iot/thing_manager.h"
#include <driver/i2c_master.h>
#include "driver/spi_master.h"
#include "assets/lang_config.h"
#include "display/lcd_display.h"
#include "audio_codecs/es8311_audio_codec.h"

#define TAG "VobotGlobalESP32S3"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_16_4);

st7735_lcd_init_cmd_t st7735_init_cmds1[] = {
    // {ST7735_SWRESET, (uint8_t[]){0x00}, 1, 120}, // 软件复位，延迟120ms
    {0x11, (uint8_t[]){0x00}, 0, 120},           // 退出睡眠模式，延迟120ms
    {0xF0, (uint8_t[]){0x11}, 1, 0},              // 设置显示控制
    {0xD6, (uint8_t[]){0xCB}, 1, 0},              // 设置显示控制
    {0xB1, (uint8_t[]){0x05, 0x3C, 0x3C}, 3, 0}, // 设置帧率
    {0xB4, (uint8_t[]){0x03}, 1, 0},              // 点反转
    {0xC0, (uint8_t[]){0xE0, 0x00, 0x87}, 3, 0}, // 设置电压
    {0xC1, (uint8_t[]){0xC5}, 1, 0},              // 设置电压
    {0xC2, (uint8_t[]){0x0D, 0x00}, 2, 0},        // 电源控制 - 正常模式
    {0xC5, (uint8_t[]){0x26}, 1, 0},              // 设置电源控制
    {0xE0, (uint8_t[]){0x18, 0x3C, 0x03, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x25, 0x10, 0x16, 0x0A, 0x00}, 16, 0}, // 正向伽马
    {0xE1, (uint8_t[]){0x10, 0x37, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0x10, 0x14, 0x0B, 0x00}, 16, 0}, // 负向伽马
    {0x2A, (uint8_t[]){0x00, 0x00, 0x00, 0x5F}, 4, 0}, // 设置列地址
    {0x2B, (uint8_t[]){0x00, 0x00, 0x00, 0x5F}, 4, 0}, // 设置行地址
    {0x3A, (uint8_t[]){0x05}, 1, 0},              // 设置颜色格式
    {0x36, (uint8_t[]){0x40}, 1, 0},              // 设置内存访问控制
    {0x21, (uint8_t[]){0x00}, 0, 0},              // 反转显示
    {0x29, (uint8_t[]){0x00}, 0, 0},              // 打开显示
    {0x2C, (uint8_t[]){0x00}, 0, 0},              // 开始写入到显示内存
};

class VobotGlobalESP32S3 : public WifiBoard {
private:
    Button boot_button_;
    Button power_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    i2c_master_bus_handle_t codec_i2c_bus_;
    bool power_button_pressed_=false;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    SpiLcdDisplay* display_;

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    void InitializeSpi() {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializePowerManager() {
        gpio_set_direction(CTRL_SYSTEM_POWER_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(CTRL_SYSTEM_POWER_PIN, 1);

        power_manager_ = new PowerManager(BATTARY_MEASURE_PIN);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Enabling sleep mode");
            display_->SetChatMessage("system", "");
            display_->SetEmotion("sleepy");
            GetBacklight()->SetBrightness(1);
        });
        power_save_timer_->OnExitSleepMode([this]() {
            display_->SetChatMessage("system", "");
            display_->SetEmotion("neutral");
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            esp_lcd_panel_disp_on_off(panel_, false); //关闭显示
            esp_deep_sleep_start();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeSt7735Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = DISPLAY_SPI_SCLK_HZ;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        st7735_vendor_config_t st7735_vendor_config = {
            .init_cmds = st7735_init_cmds1,
            .init_cmds_size = sizeof(st7735_init_cmds1) / sizeof(st7735_lcd_init_cmd_t),
        };
        panel_config.vendor_config = &st7735_vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7735(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));

        display_ = new SpiLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
        {
            .text_font = &font_puhui_16_4,
            .icon_font = &font_awesome_16_4,
            .emoji_font = font_emoji_64_init(),
        });
    }

    void InitializeButtons() {
        power_button_.OnPressDown([this]() {
            power_button_pressed_ = true;
        });

        power_button_.OnPressUp([this]() {
            if (power_button_pressed_) {
                gpio_set_level(CTRL_SYSTEM_POWER_PIN, 0);
            }
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });

        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeIot() {
        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("Screen"));
        thing_manager.AddThing(iot::CreateThing("Battery"));
    }

public:
    VobotGlobalESP32S3():
    boot_button_(BOOT_BUTTON_GPIO),
    power_button_(POWER_BUTTON_GPIO),
    volume_up_button_(VOLUME_UP_BUTTON_GPIO),
    volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        // 初始化电源相关模块
        InitializePowerManager();
        InitializePowerSaveTimer();

        // 初始化SPI & I2C
        InitializeSpi();
        InitializeCodecI2c();

        // 按键初始化
        InitializeButtons();

        // LCD屏幕初始化
        InitializeSt7735Display();

        // Iot初始化
        InitializeIot();

        // 背光点亮
        GetBacklight()->RestoreBrightness();
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveMode(bool enabled) override {
        if (!enabled) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveMode(enabled);
    }

};

DECLARE_BOARD(VobotGlobalESP32S3);
