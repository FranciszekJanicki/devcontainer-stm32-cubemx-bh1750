#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include "common.hpp"
#include "stm32l4xx_hal.h"
#include "utility.hpp"
#include <array>
#include <bitset>
#include <cstdint>

using namespace Utility;

namespace BH1750 {

    struct I2CDevice {
        template <std::size_t READ_SIZE>
        [[nodiscard]] DWords<READ_SIZE> read_dwords(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_dwords(this->read_bytes<4 * READ_SIZE>(reg_address));
        }

        [[nodiscard]] DWord read_dword(std::uint8_t const reg_address) const noexcept
        {
            return this->read_dwords<1>(reg_address)[0];
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Words<READ_SIZE> read_words(std::uint8_t const reg_address) const noexcept
        {
            return bytes_to_words(this->read_bytes<2 * READ_SIZE>(reg_address));
        }

        [[nodiscard]] Word read_word(std::uint8_t const reg_address) const noexcept
        {
            return this->read_words<1>(reg_address)[0];
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Bytes<READ_SIZE> read_bytes(std::uint8_t const reg_address) const noexcept
        {
            Bytes<READ_SIZE> read{};
            HAL_I2C_Mem_Read(this->i2c_bus,
                             this->device_address << 1,
                             reg_address,
                             sizeof(reg_address),
                             read.data(),
                             read.size(),
                             I2C_TIMEOUT);
            return read;
        }

        [[nodiscard]] Byte read_byte(std::uint8_t const reg_address) const noexcept
        {
            return this->read_bytes<1>(reg_address)[0];
        }

        [[nodiscard]] Bit read_bit(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            return this->read_byte(reg_address) & (1 << read_position);
        }

        template <std::size_t READ_SIZE>
        [[nodiscard]] Byte read_bits(std::uint8_t const reg_address, std::uint8_t const read_position) const noexcept
        {
            Byte read = this->read_byte(reg_address);
            Byte mask = ((1 << READ_SIZE) - 1) << (read_position - READ_SIZE + 1);
            read &= mask;
            read >>= (read_position - READ_SIZE + 1);
            return read;
        }

        template <std::size_t WRITE_SIZE>
        void write_dwords(std::uint8_t const reg_address, DWords<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, dwords_to_bytes(write_data));
        }

        void write_dword(std::uint8_t const reg_address, DWord const write_data) const noexcept
        {
            this->write_dwords(reg_address, DWords<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_words(std::uint8_t const reg_address, Words<WRITE_SIZE> const& write_data) const noexcept
        {
            this->write_bytes(reg_address, words_to_bytes(write_data));
        }

        void write_word(std::uint8_t const reg_address, Word const write_data) const noexcept
        {
            this->write_words(reg_address, Words<1>{write_data});
        }

        template <std::size_t WRITE_SIZE>
        void write_bytes(std::uint8_t const reg_address, Bytes<WRITE_SIZE> const& write_data) const noexcept
        {
            Bytes<WRITE_SIZE> write{write_data};
            HAL_I2C_Mem_Write(this->i2c_bus,
                              this->device_address << 1,
                              reg_address,
                              sizeof(reg_address),
                              write.data(),
                              write.size(),
                              I2C_TIMEOUT);
        }

        void write_byte(std::uint8_t const reg_address, Byte const write_data) const noexcept
        {
            this->write_bytes(reg_address, Bytes<1>{write_data});
        }

        void write_bit(std::uint8_t const reg_address,
                       Bit const write_data,
                       std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            if (write_data) {
                write |= (1 << write_position);
            } else {
                write &= ~(1 << write_position);
            }
            this->write_byte(reg_address, write);
        }

        template <std::size_t WRITE_SIZE>
        void write_bits(std::uint8_t const reg_address,
                        Byte const write_data,
                        std::uint8_t const write_position) const noexcept
        {
            Byte write = this->read_byte(reg_address);
            Byte mask = ((1 << WRITE_SIZE) - 1) << (write_position - WRITE_SIZE + 1);
            Byte temp = write_data << (write_position - WRITE_SIZE + 1);
            temp &= mask;
            write &= ~(mask);
            write |= temp;
            this->write_byte(reg_address, write);
        }

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] DWords<RECEIVE_SIZE> receive_dwords() const noexcept
        {
            return bytes_to_dwords(this->receive_bytes<4 * RECEIVE_SIZE>());
        }

        [[nodiscard]] DWord receive_dword() const noexcept
        {
            return this->receive_dwords<1>()[0];
        }

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Words<RECEIVE_SIZE> receive_words() const noexcept
        {
            return bytes_to_words(this->receive_bytes<2 * RECEIVE_SIZE>());
        }

        [[nodiscard]] Word receive_word() const noexcept
        {
            return this->receive_words<1>()[0];
        }

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Bytes<RECEIVE_SIZE> receive_bytes() const noexcept
        {
            Bytes<RECEIVE_SIZE> receive{};
            HAL_I2C_Master_Receive(this->i2c_bus,
                                   this->device_address << 1,
                                   receive.data(),
                                   receive.size(),
                                   I2C_TIMEOUT);
            return receive;
        }

        [[nodiscard]] Byte receive_byte() const noexcept
        {
            return this->receive_bytes<1>()[0];
        }

        [[nodiscard]] Bit receive_bit(std::uint8_t const receive_position) const noexcept
        {
            return (this->receive_byte() & (1 << receive_position)) ? 1 : 0;
        }

        template <std::size_t RECEIVE_SIZE>
        [[nodiscard]] Bits<RECEIVE_SIZE> receive_bits(std::uint8_t const receive_position) const noexcept
        {
            Byte receive = this->receive_byte();
            Byte mask = ((1 << RECEIVE_SIZE) - 1) << (receive_position - RECEIVE_SIZE + 1);
            receive &= mask;
            receive >>= (receive_position - RECEIVE_SIZE + 1);
            return receive;
        }

        template <std::size_t TRANSMIT_SIZE>
        void transmit_dwords(DWords<TRANSMIT_SIZE> const& transmit_data) const noexcept
        {
            this->transmit_bytes(dwords_to_bytes(transmit_data));
        }

        void transmit_dword(DWord const transmit_data) const noexcept
        {
            this->transmit_dwords(DWords<1>{transmit_data});
        }

        template <std::size_t TRANSMIT_SIZE>
        void transmit_words(Words<TRANSMIT_SIZE> const& transmit_data) const noexcept
        {
            this->transmit_bytes(words_to_bytes(transmit_data));
        }

        void transmit_word(Word const transmit_data) const noexcept
        {
            this->transmit_words(Words<1>{transmit_data});
        }

        template <std::size_t TRANSMIT_SIZE>
        void transmit_bytes(Bytes<TRANSMIT_SIZE> const& transmit_data) const noexcept
        {
            Bytes<TRANSMIT_SIZE> transmit{transmit_data};
            HAL_I2C_Master_Transmit(this->i2c_bus,
                                    this->device_address << 1,
                                    transmit.data(),
                                    transmit.size(),
                                    I2C_TIMEOUT);
        }

        void transmit_byte(Byte const transmit_data) const noexcept
        {
            this->transmit_bytes(Bytes<1>{transmit_data});
        }

        void transmit_bit(Bit const transmit_data, std::uint8_t const transmit_position) const noexcept
        {
            Byte transmit = this->receive_byte();
            if (transmit_data) {
                transmit |= (1 << transmit_position);
            } else {
                transmit &= ~(1 << transmit_position);
            }
            this->transmit_byte(transmit);
        }

        template <std::size_t TRANSMIT_SIZE>
        void transmit_bits(Byte const transmit_data, std::uint8_t const transmit_position) const noexcept
        {
            Byte transmit = this->receive_byte();
            Byte mask = ((1 << TRANSMIT_SIZE) - 1) << (transmit_position - TRANSMIT_SIZE + 1);
            Byte temp = transmit_data << (transmit_position - TRANSMIT_SIZE + 1);
            temp &= mask;
            transmit &= ~(mask);
            transmit |= temp;
            this->transmit_byte(transmit);
        }

        I2CBusHandle i2c_bus{nullptr};
        std::uint16_t device_address{};

        static constexpr std::uint32_t I2C_TIMEOUT{100};
    };

}; // namespace BH1750

#endif // I2C_DEVICE_HPP