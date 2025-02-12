#include "bh1750.hpp"
#include "stm32l4xx_hal.h"
#include <algorithm>
#include <cstdint>
#include <optional>
#include <utility>

namespace BH1750 {

    BH1750::BH1750(I2CDevice&& i2c_device, std::uint8_t const mtreg, Mode const mode) noexcept :
        mtreg_{mtreg}, mode_{mode}, i2c_device_{std::forward<I2CDevice>(i2c_device)}
    {
        this->initialize(mtreg);
    }

    BH1750::~BH1750() noexcept
    {
        this->deinitialize();
    }

    std::optional<std::uint16_t> BH1750::get_light_raw() const noexcept
    {
        if (!this->initialized_) {
            std::optional<std::uint16_t>{std::nullopt};
        }
        this->device_trigger_conversion();
        return std::optional<std::uint16_t>{this->i2c_device_.receive_word()};
    }

    std::optional<float> BH1750::get_light_scaled() const noexcept
    {
        return this->get_light_raw().transform(
            [this](std::uint16_t const raw) { return raw_to_scaled(raw, this->mode_, this->mtreg_); });
    }

    float BH1750::mode_to_resolution(Mode const mode) noexcept
    {
        if (mode == Mode::CONTINUOUS_HIGH_RES_MODE || mode == Mode::ONETIME_HIGH_RES_MODE) {
            return 1.0F;
        } else if (mode == Mode::CONTINUOUS_HIGH_RES_MODE_2 || mode == Mode::ONETIME_HIGH_RES_MODE_2) {
            return 0.5F;
        } else if (mode == Mode::CONTINUOUS_LOW_RES_MODE || mode == Mode::ONETIME_LOW_RES_MODE) {
            return 4.0F;
        }
    }

    float BH1750::raw_to_scaled(std::uint16_t const raw, Mode const mode, std::uint8_t const mtreg) noexcept
    {
        return raw * (1.0F / MEASUREMENT_ACCURACY) * (MTREG_DEFAULT / mtreg) * mode_to_resolution(mode);
    }

    void BH1750::initialize(std::uint8_t const mtreg) noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_reset();
            this->device_on();
            this->set_mtreg(mtreg);
            this->initialized_ = true;
        }
    }

    void BH1750::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->device_off();
            this->initialized_ = false;
        }
    }

    bool BH1750::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == this->i2c_device_.dev_address();
    }

    std::uint8_t BH1750::get_device_id() const noexcept
    {
        return this->i2c_device_.receive_byte();
    }

    void BH1750::device_on() const noexcept
    {
        this->set_power_state(Instruction::POWER_ON);
    }

    void BH1750::device_off() const noexcept
    {
        this->set_power_state(Instruction::POWER_DOWN);
    }

    void BH1750::device_reset() const noexcept
    {
        this->set_power_state(Instruction::RESET);
    }

    void BH1750::set_power_state(Instruction const instruction) const noexcept
    {
        this->i2c_device_.transmit_byte(0x01 & std::to_underlying(instruction));
    }

    void BH1750::set_mode(Mode const mode) const noexcept
    {
        this->i2c_device_.transmit_byte(std::to_underlying(mode));
    }

    void BH1750::set_mtreg(std::uint8_t const mtreg) const noexcept
    {
        std::uint8_t mtreg_value{std::clamp(mtreg, MTREG_MIN, MTREG_MAX)};
        this->i2c_device_.transmit_byte(0x40 | (mtreg_value >> 5));
        this->i2c_device_.transmit_byte(0x60 | (mtreg_value & 0x1F));
    }

    void BH1750::device_trigger_conversion() const noexcept
    {
        this->set_mode(this->mode_);
    }

}; // namespace BH1750