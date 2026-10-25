#include "endpoint.h"
#include "diag/uart_console.h"
#include "clock.h"

const static char HEX_DEF[17] = "0123456789ABCDEF";
const static size_t UART_MSG_SIZE = 6 + (2 * DIAG_CAN_MAX_SIZE);
const uart_port_t UART_PORT = uart_port_t::UART_NUM_0;
// At 921600 baud, a maximum 4099-byte 8N1 request needs about 45 ms
// of wire time. Allow 250 ms without a successful receive for a partial
// frame: above the 2/20 ms polling periods, below the paired 1500 ms timeout.
static constexpr uint32_t UART_RX_INTERBYTE_TIMEOUT_MS = 250;

UsbEndpoint::UsbEndpoint() : AbstractEndpoint()
{
    this->read_buffer = nullptr;
    this->write_buffer = nullptr;
    this->write_mutex = nullptr;
    data_size = 0;
    line_idx = 0;
    max_bytes_left = 0;
    to_read = 0;
    length = 0;
    this->status = uart_driver_install(UART_PORT, 512, 512, 0, nullptr, 0);
    if (this->status == ESP_OK)
    {
        // Driver-backed stdio shares the UART driver's TX lock. The log
        // callback itself never writes here; queued records drain separately.
        UartConsole::configure_vfs_driver();
        this->write_mutex = xSemaphoreCreateMutex();
        if (this->write_mutex != nullptr)
        {
            this->read_buffer = static_cast<char *>(malloc(UART_MSG_SIZE));
            this->write_buffer = static_cast<char *>(malloc(UART_MSG_SIZE));
            if (nullptr != this->read_buffer && nullptr != this->write_buffer)
            {
                this->status = uart_flush(UART_PORT);
                if (this->status == ESP_OK)
                {
                    this->read_pos = 0;
                    UartConsole::mark_transport_ready();
                }
            } else {
                free(this->read_buffer);
                free(this->write_buffer);
                this->read_buffer = nullptr;
                this->write_buffer = nullptr;
                this->status = ESP_ERR_NO_MEM;
            }
        }
        else
        {
            this->status = ESP_ERR_NO_MEM;
        }
    }
}

esp_err_t UsbEndpoint::init_state() {
    return this->status;
}

void UsbEndpoint::send_data(uint32_t id, uint8_t *buf, uint16_t len)
{
    if (this->status != ESP_OK || this->write_buffer == nullptr || this->write_mutex == nullptr ||
        len > DIAG_CAN_MAX_SIZE || (len != 0 && buf == nullptr))
    {
        return;
    }
    if (xSemaphoreTake(this->write_mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }

    this->write_buffer[0] = '#';
    this->write_buffer[1] = HEX_DEF[(id >> 12) & 0x0F];
    this->write_buffer[2] = HEX_DEF[(id >> 8) & 0x0F];
    this->write_buffer[3] = HEX_DEF[(id >> 4) & 0x0F];
    this->write_buffer[4] = HEX_DEF[id & 0x0F];
    for (uint16_t i = 0; i < len; i++)
    {
        this->write_buffer[5 + (i * 2)] = HEX_DEF[(buf[i] >> 4) & 0x0F];
        this->write_buffer[6 + (i * 2)] = HEX_DEF[buf[i] & 0x0F];
    }
    this->write_buffer[(len * 2) + 5] = '\n';
    uart_write_bytes(UART_PORT, &this->write_buffer[0], (len * 2) + 6);
    xSemaphoreGive(this->write_mutex);
}

bool UsbEndpoint::read_data(DiagMessage *dest)
{
    const uint32_t now_ms = GET_CLOCK_TIME();
    if (this->read_pos != 0)
    {
        bool partial_frame = this->read_pos < 2;
        if (!partial_frame)
        {
            const uint16_t declared_size =
                (uint16_t(uint8_t(this->read_buffer[0])) << 8) |
                uint8_t(this->read_buffer[1]);
            partial_frame = this->read_pos < size_t(declared_size) + 2;
        }
        if (partial_frame && uint32_t(now_ms - this->last_rx_byte_ms) >= UART_RX_INTERBYTE_TIMEOUT_MS)
        {
            // Discard only our stale prefix before appending queued bytes.
            // A fresh request already waiting in the driver remains readable.
            this->read_pos = 0;
        }
    }
    this->length = 0;
    uart_get_buffered_data_len(UART_PORT, &length);
    if (length != 0)
    {
        max_bytes_left = UART_MSG_SIZE - this->read_pos;
        to_read = MIN(length, max_bytes_left);
        if (to_read == 0) {
            ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Rx buffer overflow, discarding message");
            uart_flush_input(UART_PORT);
            this->read_pos = 0;
            return false;
        }
        int received = uart_read_bytes(UART_PORT, &this->read_buffer[this->read_pos], to_read, 0);
        if (received > 0) {
            this->read_pos += received;
            this->last_rx_byte_ms = now_ms;
        }
        return false;
    }
    else if (this->read_pos != 0)
    {
        if (this->read_pos < 2)
        {
            // An empty driver poll does not end a partially received frame.
            return false;
        }
        else
        {
            const uint16_t read_size =
                (uint16_t(uint8_t(this->read_buffer[0])) << 8) |
                uint8_t(this->read_buffer[1]);
            // ID consumes two bytes and a diagnostic request needs a SID.
            if (read_size < 3 || read_size - 2 > DIAG_CAN_MAX_SIZE)
            {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Invalid incoming diagnostic length");
                this->read_pos = 0;
                return false;
            }
            const size_t frame_size = size_t(read_size) + 2;
            if (this->read_pos < frame_size)
            {
                return false;
            }
            if (this->read_pos != frame_size)
            {
                ESP_LOG_LEVEL(ESP_LOG_ERROR, "USBEndpoint", "Corrupt incoming msg. Msg size is %d bytes, buffer has %d bytes", read_size, this->read_pos - 2);
                this->read_pos = 0;
                return false;
            }
            else
            {
                // Valid msg!
                dest->id = (uint16_t(uint8_t(this->read_buffer[2])) << 8) |
                           uint8_t(this->read_buffer[3]);
                dest->data_size = read_size - 2;
                memcpy(dest->data, &this->read_buffer[4], dest->data_size);
                this->read_pos = 0;
                return true;
            }
        }
    }
    return false;
}
