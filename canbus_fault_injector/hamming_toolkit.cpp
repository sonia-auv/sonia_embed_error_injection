#include "hamming_toolkit.h"

    int HammingToolkit::pack_7_bits_values(uint8_t* unpack_values, size_t size_unpack_values, uint8_t* pack_values, size_t size_pack_values)
    {
        int8_t current_writing_pos = 0;
        size_t current_writing_index = 0;
        uint8_t current_value;
        uint8_t nb_bits_written;
        const uint8_t mask = 0b1111111;

        const uint8_t nb_bit_unpack = 7;
        const uint8_t nb_bit_pack = 8;

        pack_values[0] = 0;
        for (size_t i = 0; i < size_unpack_values; i++)
        {
            current_value = unpack_values[i] & mask;

            if (nb_bit_pack - (current_writing_pos + nb_bit_unpack) > 0)
            {

                pack_values[current_writing_index] = current_value << current_writing_pos;
                current_writing_pos += nb_bit_unpack;
            }
            else
            {
                //we start by writing the LSB can fit in the current element of the array
                pack_values[current_writing_index] |= current_value << current_writing_pos;

                //we write the rest in the next element array
                nb_bits_written = nb_bit_pack - current_writing_pos;
                current_writing_index++;

                
                if (nb_bits_written < 7)
                {
                    pack_values[current_writing_index] = current_value >> nb_bits_written;
                }
                current_writing_pos = nb_bit_unpack - nb_bits_written;
            }

        }

        return 0;
    }

    int HammingToolkit::unpack_7_bits_values(const uint8_t* pack_values, size_t size_pack_values, uint8_t* unpack_values, size_t size_unpack_values )
    {
        int8_t current_writing_pos = 0;
        size_t current_writing_index = 0;
        uint8_t current_value;
        uint8_t nb_bits_written;
        const uint8_t mask = 0b1111111;

        const uint8_t nb_bit_unpack = 7;
        const uint8_t nb_bit_pack = 8;

        unpack_values[0] = 0;
        for (size_t i = 0; i < size_pack_values; i++)
        {
            current_value = pack_values[i];

            if (current_writing_pos == 0)
            {
                unpack_values[current_writing_index] = 0;
            }

            //we finish writing the current element
            unpack_values[current_writing_index] |= (current_value << current_writing_pos) & mask;

            //we start writing the next element
            current_writing_index++;

            if (current_writing_index < size_unpack_values)
            {
                nb_bits_written = nb_bit_unpack - current_writing_pos;
                unpack_values[current_writing_index] = (current_value >> nb_bits_written) & mask;
                current_writing_pos = nb_bit_pack - nb_bits_written;
                if (current_writing_pos == 7)
                {
                    current_writing_pos = 0;
                    current_writing_index++;
                }
            }
        }

        return 0;
    }


    int HammingToolkit::decode_hamming_74_message(const uint8_t* encoded_data, size_t encoded_data_size, uint8_t* decoded_data, size_t decoded_data_size)
    {
        size_t nb_byte_decode = encoded_data_size / 2;
        uint8_t current_decoded_value;
        //additional partial byte
        if (encoded_data_size % 2)
        {
            nb_byte_decode += 1;
        }

        if (decoded_data_size < nb_byte_decode)
        {
            return -1;
        }
        for (size_t i = 0; i < encoded_data_size; i++)
        {
            current_decoded_value = HammingTableDecode(encoded_data[i] & 0x7f);

            if (i % 2 == 0)
            {
                decoded_data[i / 2] = current_decoded_value & 0xF;
            }
            else
            {
                decoded_data[i / 2] |= (current_decoded_value & 0xF) << 4;
            }
        }
        return 0;
    }



    int HammingToolkit::encode_hamming_74_message(const uint8_t* data, size_t data_size, uint8_t* encoded_data, size_t encoded_data_size)
    {
        if (encoded_data_size < 2 * data_size)
        {
            return -1;
        }
        for (size_t i = 0; i < data_size; i++)
        {
            encoded_data[2 * i] = HammingTableEncode(data[i] & 0xf);
            encoded_data[(2 * i) + 1] = HammingTableEncode((data[i] >> 4) & 0xf);
        }
        return 0;
    }

    uint8_t HammingToolkit::get_bit(uint8_t data, uint8_t pos)
    {
        return (data >> pos) & 1;
    }

    void HammingToolkit::interleaving_pre_pack(uint8_t* data, size_t data_size, uint8_t* interleaved_data)
    {

        uint8_t current_byte = 0;
        uint8_t current_bit_write = 0;
        uint8_t current_bit_read = 0;
        uint8_t data_index = 0;
        uint8_t interleaved_data_index = 0;
        while (interleaved_data_index < data_size)
        {
            current_byte |= get_bit(data[data_index], current_bit_read) << current_bit_write;

            current_bit_write++;
            if (current_bit_write == 7)
            {
                interleaved_data[interleaved_data_index] = current_byte;
                current_bit_write = 0;
                current_byte = 0;
                interleaved_data_index++;
            }

            data_index++;
            if (data_index >= data_size)
            {
                current_bit_read++;
                data_index = 0;
            }
        }
    }

    void HammingToolkit::deinterleaving_post_depack(uint8_t* data, size_t data_size, uint8_t* interleaved_data)
    {

        uint8_t current_byte = 0;
        uint8_t current_bit_write = 0;
        uint8_t current_bit_read = 0;
        uint8_t data_index = 0;
        uint8_t interleaved_data_index = 0;

        for (size_t i = 0; i < data_size; i++)
        {
            data[i] = 0;
        }

        while (interleaved_data_index < data_size)
        {
            if (current_bit_read == 0)
            {
                current_byte = interleaved_data[interleaved_data_index];
            }

            data[data_index] |= get_bit(current_byte, current_bit_read) << current_bit_write;

            current_bit_read++;
            if (current_bit_read == 7)
            {
                current_bit_read = 0;
                interleaved_data_index++;
            }

            data_index++;

            if (data_index >= data_size)
            {
                data_index = 0;
                current_bit_write++;
            }

        }
    }