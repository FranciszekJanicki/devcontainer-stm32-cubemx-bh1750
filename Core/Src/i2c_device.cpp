#include "i2c_device.hpp"
#include "common.hpp"

namespace Utility {

    I2CDevice::I2CDevice(I2CBusHandle const i2c_bus, std::uint16_t const device_address) noexcept :
        i2c_bus_{i2c_bus}, device_address_{device_address}
    {
        this->initialize();
    }

    void I2CDevice::transmit_dword(DWord const transmit_data) const noexcept
    {
        this->transmit_dwords(DWords<1UL>{transmit_data});
    }

    void I2CDevice::transmit_word(Word const transmit_data) const noexcept
    {
        this->transmit_words(Words<1UL>{transmit_data});
    }

    void I2CDevice::transmit_byte(Byte const transmit_data) const noexcept
    {
        this->transmit_bytes(Bytes<1UL>{transmit_data});
    }

    DWord I2CDevice::receive_dword() const noexcept
    {
        return this->receive_dwords<1UL>()[0];
    }

    Word I2CDevice::receive_word() const noexcept
    {
        return this->receive_words<1UL>()[0];
    }

    Byte I2CDevice::receive_byte() const noexcept
    {
        return this->receive_bytes<1UL>()[0];
    }

    DWord I2CDevice::read_dword(std::uint8_t const reg_address) const noexcept
    {
        return this->read_dwords<1UL>(reg_address)[0];
    }

    Word I2CDevice::read_word(std::uint8_t const reg_address) const noexcept
    {
        return this->read_words<1UL>(reg_address)[0];
    }

    Byte I2CDevice::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->read_bytes<1UL>(reg_address)[0];
    }

    Bit I2CDevice::read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
    {
        return get_bit(this->read_byte(reg_address), read_position);
    }

    void I2CDevice::write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept
    {
        this->write_dwords(reg_address, DWords<1UL>{write_data});
    }

    void I2CDevice::write_word(std::uint8_t const reg_address, Word const write_data) const noexcept
    {
        this->write_words(reg_address, Words<1UL>{write_data});
    }

    void I2CDevice::write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept
    {
        this->write_bytes(reg_address, Bytes<1UL>{write_data});
    }

    void I2CDevice::write_bit(std::uint8_t const reg_address,
                              Bit const write_data,
                              std::uint8_t const write_position) const noexcept
    {
        Byte write{this->read_byte(reg_address)};
        set_bit(write, write_data, write_position);
        this->write_byte(reg_address, write);
    }

    std::uint16_t I2CDevice::device_address() const noexcept
    {
        return this->device_address_;
    }

    void I2CDevice::initialize() noexcept
    {
        if (this->i2c_bus_ != nullptr) {
            if (HAL_I2C_IsDeviceReady(this->i2c_bus_, this->device_address_ << 1, I2C_SCAN_RETRIES, I2C_TIMEOUT) ==
                HAL_OK) {
                this->initialized_ = true;
            }
        }
    }

}; // namespace Utility