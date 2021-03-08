#ifndef HTCW_ESP32_DS1307_HPP
#define HTCW_ESP32_DS1307_HPP
#include <time.h>
#include "i2c_master.hpp"
namespace rtc
{
    // indicates the square wave cycle
    enum ds1307_sqw : uint8_t {
        off = 0x00,            // Low
        on = 0x80,             // High
        cycle_1hz = 0x10,
        cycle_4khz = 0x11,
        cycle_8khz = 0x12,
        cycle_32khz = 0x13
    };
    // drives a DS1307 real-time clock module
    class ds1307 final
    {
        bool m_initialized;
        esp_err_t m_last_error;
        esp32::i2c_master *m_i2c;
        uint8_t m_address;
        gpio_num_t m_sync;
        inline void last_error(esp_err_t last_error)
        {
            m_last_error = last_error;
        }
        static uint8_t bin_to_bcd(uint8_t bin)
        {
            return ((bin / 10) << 4) | (bin % 10);
        }

        static uint8_t bcd_to_bin(uint8_t bcd)
        {
            return ((bcd >> 4) * 10) + (bcd & 0x0f);
        }
        
        ds1307(const ds1307& rhs)=delete;
        ds1307& operator=(const ds1307& rhs)=delete;
    public:
        // the i2c address for the clock
        const uint8_t default_address = 0x68;
        // initialize the DS1307
        ds1307(esp32::i2c_master *i2c, uint8_t address = 0x68,gpio_num_t sync= GPIO_NUM_NC) : m_initialized(false),m_last_error(ESP_OK), m_i2c(nullptr), m_address(0x68),m_sync(sync)
        {
            if (nullptr == i2c)
            {
                last_error(ESP_ERR_INVALID_ARG);
                return;
            }
            m_address = address;
            m_i2c = i2c;
        }
        ds1307(ds1307&& rhs) : m_initialized(rhs.m_initialized),m_last_error(rhs.m_last_error),m_i2c(rhs.m_i2c),m_address(rhs.m_address),m_sync(rhs.m_sync) {
            rhs.m_initialized=false;
            rhs.m_i2c=nullptr;
        }
        ds1307& operator=(ds1307&& rhs) {
            m_initialized=rhs.m_initialized;
            m_last_error=rhs.m_last_error;
            m_i2c = rhs.m_i2c;
            m_address = rhs.m_address;
            m_sync=rhs.m_sync;
            rhs.m_initialized=false;
            rhs.m_i2c=nullptr;
            return *this;
        }
        ~ds1307()=default;
        // indicates the most recent clock error
        inline esp_err_t last_error() const
        {
            return m_last_error;
        }
        // indiciates whether or not the clock driver has been initialized
        inline bool initialized() const
        {
            return m_initialized;
        }
        // forces an initialization of the clock driver
        bool initialize() {
            if(m_initialized)
                return true;
            if(nullptr==m_i2c) {
                last_error(ESP_ERR_INVALID_STATE);
                return false;
            }
            if(m_sync!=GPIO_NUM_NC) {
                esp_err_t res=gpio_set_pull_mode(m_sync,GPIO_PULLUP_ONLY);
                if(ESP_OK!=res) {
                    last_error(res);
                    return false;
                }
                res=gpio_set_intr_type(m_sync,GPIO_INTR_DISABLE);
                if(ESP_OK!=res) {
                    last_error(res);
                    return false;
                }
                res=gpio_set_direction(m_sync,GPIO_MODE_INPUT);
                if(ESP_OK!=res) {
                    last_error(res);
                    return false;
                }
            }
            esp32::i2c_master_command cmd;
            
            if(!cmd.start()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_write(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.stop())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!m_i2c->execute(cmd, pdMS_TO_TICKS(1000)))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            m_initialized=true;
            return true;
        }
        // indicates whether or not the clock is running
        bool running(bool *result)
        {
            if (nullptr == result)
            {
                last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if (!initialize())
            {
                return false;
            }
            esp32::i2c_master_command cmd;
            if (!cmd.start())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_write(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.write(0, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.start())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_read(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            uint8_t r = 0xFF;
            if (!cmd.read(&r, I2C_MASTER_NACK))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.stop())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!m_i2c->execute(cmd,pdMS_TO_TICKS(5000)))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            *result = !(r >> 7);
            return true;
        }
        // sets the clock. this will make it start running if it's not.
        bool set(tm* tm) {
            if(nullptr==tm) {
                last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if (!initialize())
            {
                return false;
            }
            esp32::i2c_master_command cmd;
            if (!cmd.start())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_write(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.write(0, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_sec),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_min),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_hour),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_wday+1),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_mday),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_mon+1),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(bin_to_bcd(tm->tm_year-100),true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.stop())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!m_i2c->execute(cmd,pdMS_TO_TICKS(5000)))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            return true;
        }
        // sets the clock. this will make it running if it's not
        bool set(time_t time) {
            return set(localtime(&time));
        }
        // retrieves the current time
        bool now(tm* result)
        {
            if(nullptr==result) {
                last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if (!initialize())
            {
                return false;
            }
            esp32::i2c_master_command cmd;
            if (!cmd.start())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_write(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.write(0, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.start())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.begin_read(m_address, true))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            uint8_t data[7];
            if (!cmd.read(data, sizeof(data), I2C_MASTER_ACK))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!cmd.read((uint8_t*)(data+6), I2C_MASTER_NACK))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            
            if (!cmd.stop())
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if (!m_i2c->execute(cmd,5000/portTICK_PERIOD_MS))
            {
                last_error(esp32::i2c::last_error());
                return false;
            }
            result->tm_sec = bcd_to_bin(data[0]);
            result->tm_min = bcd_to_bin(data[1]);
            result->tm_hour = bcd_to_bin(data[2]);
            result->tm_mday = bcd_to_bin(data[4]);
            result->tm_mon = bcd_to_bin(data[5]) - 1;    // 0-11 - Note: The month on the DS1307 is 1-12.
            result->tm_year = bcd_to_bin(data[6]) + 100; // Years since 1900
            return true;
        }
        // retrieves the current time
        bool now(time_t* result) {
             if(nullptr==result) {
                last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            tm tm;
            if(now(&tm)) {
                *result = mktime(&tm);
            }
            return false;
        }
        // gets the square wave info
        bool sqw(ds1307_sqw* result) {
            if(nullptr==result) {
                last_error(ESP_ERR_INVALID_ARG);
                return false;
            }
            if (!initialize())
            {
                return false;
            }
            int mode;
            esp32::i2c_master_command cmd;
            if(!cmd.start()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.begin_write(m_address,true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(0x07,true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.start()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.begin_read(m_address,true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            uint8_t r;
            if(!cmd.read(&r,I2C_MASTER_NACK)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.stop()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!m_i2c->execute(cmd,5000/portTICK_PERIOD_MS)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            mode=r & 0x93;
            *result=static_cast<ds1307_sqw>(mode);
            return true;
        }
        // sets the square wave info
        bool sqw(ds1307_sqw value) {
            if (!initialize())
            {
                return false;
            }
            esp32::i2c_master_command cmd;
            if(!cmd.start()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.begin_write(m_address,true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write(0x07,true)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.write((uint8_t)value)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!cmd.stop()) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            if(!m_i2c->execute(cmd,5000/portTICK_PERIOD_MS)) {
                last_error(esp32::i2c::last_error());
                return false;
            }
            return true;
        }
        
    };
}
#endif