#ifndef BH1750_HPP
#define BH1750_HPP

#include "i2c_device.hpp"
#include "stm32l4xx_hal.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

namespace BH1750 {

    struct BH1750 {
    public:
        enum struct DevAddress : std::uint8_t {
            AD0_LOW = 0x55,
            AD0_HIG = 0x23,
        };

        enum struct Instruction : std::uint8_t {
            POWER_DOWN = 0x00,
            POWER_ON = 0x01,
            RESET = 0x07,
        };

        enum struct Mode : std::uint8_t {
            CONTINUOUS_HIGH_RES_MODE = 0x10,
            CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
            CONTINUOUS_LOW_RES_MODE = 0x13,
            ONETIME_HIGH_RES_MODE = 0x20,
            ONETIME_HIGH_RES_MODE_2 = 0x21,
            ONETIME_LOW_RES_MODE = 0x23,
        };

        using Raw = std::uint8_t;
        using Scaled = std::float_t;
        using OptionalRaw = std::optional<Raw>;
        using OptionalScaled = std::optional<Scaled>;
        using I2CDevice = Utility::I2CDevice;

        static constexpr std::uint8_t MTREG_MIN = 0x1F;
        static constexpr std::uint8_t MTREG_DEFAULT = 0x45;
        static constexpr std::uint8_t MTREG_MAX = 0xFE;

        BH1750() noexcept = default;
        BH1750(I2CDevice&& i2c_device, std::uint8_t const mtreg, Mode const mode) noexcept;

        BH1750(BH1750 const& other) noexcept = delete;
        BH1750(BH1750&& other) noexcept = default;

        BH1750& operator=(BH1750 const& other) noexcept = delete;
        BH1750& operator=(BH1750&& other) noexcept = default;

        ~BH1750() noexcept;

        [[nodiscard]] OptionalRaw get_light_raw() const noexcept;
        [[nodiscard]] OptionalScaled get_light_scaled() const noexcept;

    private:
        static Scaled mode_to_resolution(Mode const mode) noexcept;
        static Scaled raw_to_scaled(Raw const count, Mode const mode, std::uint8_t const mtreg) noexcept;

        static constexpr Scaled MEASUREMENT_ACCURACY{1.2F};

        bool is_valid_device_id() const noexcept;
        std::uint8_t get_device_id() const noexcept;

        void device_on() const noexcept;
        void device_off() const noexcept;
        void device_reset() const noexcept;
        void device_trigger_conversion() const noexcept;

        void set_power_state(Instruction const instruction) const noexcept;
        void set_mtreg(std::uint8_t const mtreg) const noexcept;
        void set_mode(Mode const Mode) const noexcept;

        void initialize(std::uint8_t const mtreg) noexcept;
        void deinitialize() noexcept;

        bool initialized_{false};

        std::uint8_t mtreg_{};
        Mode mode_{};

        I2CDevice i2c_device_{};
    };

}; // namespace BH1750

#endif // BH1750_HPP