#include "uart_console.h"

#include <atomic>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

struct QueuedRecord {
    char bytes[UartConsole::RecordBytes];
    uint16_t length;
};

static StaticQueue_t s_queue_control;
alignas(4) static uint8_t s_queue_storage[UartConsole::QueueDepth * sizeof(QueuedRecord)];
static std::atomic<QueueHandle_t> s_queue{nullptr};
static std::atomic<bool> s_initialized{false};
static std::atomic<bool> s_vfs_configured{false};
static std::atomic<bool> s_transport_ready{false};
static bool s_callback_installed = false;

static std::atomic<uint32_t> s_queued_records{0};
static std::atomic<uint32_t> s_dropped_records{0};
static std::atomic<uint32_t> s_truncated_records{0};
static std::atomic<uint32_t> s_write_errors{0};

static TickType_t s_last_summary_tick = 0;
static bool s_has_summary_tick = false;
static uint32_t s_reported_dropped = 0;
static uint32_t s_reported_truncated = 0;
static uint32_t s_reported_write_errors = 0;

void count_dropped()
{
    s_dropped_records.fetch_add(1, std::memory_order_relaxed);
}

bool queue_record(const QueuedRecord &record)
{
    QueueHandle_t queue = s_queue.load(std::memory_order_acquire);
    if (queue == nullptr || record.length == 0) {
        count_dropped();
        return false;
    }
    if (xQueueSend(queue, &record, 0) != pdTRUE) {
        count_dropped();
        return false;
    }
    s_queued_records.fetch_add(1, std::memory_order_relaxed);
    return true;
}

int log_vprintf(const char *format, va_list args)
{
    QueueHandle_t queue = s_queue.load(std::memory_order_acquire);
    if (queue == nullptr || format == nullptr) {
        count_dropped();
        return 0;
    }

    QueuedRecord record{};
    va_list copy;
    va_copy(copy, args);
    const int formatted = vsnprintf(record.bytes, sizeof(record.bytes), format, copy);
    va_end(copy);
    if (formatted < 0) {
        count_dropped();
        return 0;
    }

    bool truncated = static_cast<size_t>(formatted) >= sizeof(record.bytes);
    size_t length = truncated ? sizeof(record.bytes) - 1 : static_cast<size_t>(formatted);
    while (length > 0 && (record.bytes[length - 1] == '\n' || record.bytes[length - 1] == '\r')) {
        --length;
    }

    if (length >= sizeof(record.bytes) - 1) {
        // Reserve the final payload byte for LF, including the exact-fit case.
        length = sizeof(record.bytes) - 2;
        truncated = true;
    }
    record.bytes[length++] = '\n';
    record.bytes[length] = '\0';
    record.length = static_cast<uint16_t>(length);

    if (truncated) {
        s_truncated_records.fetch_add(1, std::memory_order_relaxed);
    }
    queue_record(record);
    return formatted;
}

void report_counters_if_needed()
{
    const uint32_t dropped = s_dropped_records.load(std::memory_order_relaxed);
    const uint32_t truncated = s_truncated_records.load(std::memory_order_relaxed);
    const uint32_t write_errors = s_write_errors.load(std::memory_order_relaxed);
    if (dropped == s_reported_dropped && truncated == s_reported_truncated &&
        write_errors == s_reported_write_errors) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t report_period = pdMS_TO_TICKS(1000);
    if (s_has_summary_tick && static_cast<TickType_t>(now - s_last_summary_tick) < report_period) {
        return;
    }

    char summary[96];
    const int length = snprintf(summary, sizeof(summary),
                                "UART_LOG dropped=%lu truncated=%lu write_errors=%lu\n",
                                static_cast<unsigned long>(dropped),
                                static_cast<unsigned long>(truncated),
                                static_cast<unsigned long>(write_errors));
    if (length <= 0 || static_cast<size_t>(length) >= sizeof(summary)) {
        return;
    }

    const int written = uart_write_bytes(UART_NUM_0, summary, static_cast<size_t>(length));
    s_last_summary_tick = now;
    s_has_summary_tick = true;
    if (written != length) {
        s_write_errors.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    s_reported_dropped = dropped;
    s_reported_truncated = truncated;
    s_reported_write_errors = write_errors;
}

} // namespace

namespace UartConsole {

bool init()
{
    if (!s_callback_installed) {
        // The callback never calls the previous writer: an unavailable queue
        // drops the message instead of reaching the UART polling VFS writer.
        esp_log_set_vprintf(log_vprintf);
        s_callback_installed = true;
    }

    if (s_initialized.load(std::memory_order_acquire)) {
        return true;
    }

    QueueHandle_t queue = xQueueCreateStatic(
        QueueDepth, sizeof(QueuedRecord), s_queue_storage, &s_queue_control);
    if (queue == nullptr) {
        return false;
    }
    s_queue.store(queue, std::memory_order_release);
    s_initialized.store(true, std::memory_order_release);
    return true;
}

void configure_vfs_driver()
{
    uart_vfs_dev_use_driver(UART_NUM_0);
    s_vfs_configured.store(true, std::memory_order_release);
}

bool mark_transport_ready()
{
    if (!s_initialized.load(std::memory_order_acquire)) {
        return false;
    }
    if (!s_vfs_configured.load(std::memory_order_acquire)) {
        return false;
    }
    s_transport_ready.store(true, std::memory_order_release);
    return true;
}

size_t drain(size_t max_records)
{
    if (max_records == 0 || !s_transport_ready.load(std::memory_order_acquire)) {
        return 0;
    }
    QueueHandle_t queue = s_queue.load(std::memory_order_acquire);
    if (queue == nullptr) {
        return 0;
    }

    const size_t budget = max_records < DrainBudget ? max_records : DrainBudget;
    size_t sent = 0;
    QueuedRecord record{};
    while (sent < budget && xQueueReceive(queue, &record, 0) == pdTRUE) {
        const int written = uart_write_bytes(UART_NUM_0, record.bytes, record.length);
        if (written != record.length) {
            s_write_errors.fetch_add(1, std::memory_order_relaxed);
        }
        ++sent;
    }
    report_counters_if_needed();
    return sent;
}

bool enqueue_text(const char *text, size_t length)
{
    if (text == nullptr && length != 0) {
        count_dropped();
        return false;
    }

    QueuedRecord record{};
    const size_t max_payload = sizeof(record.bytes) - 1;
    bool truncated = false;
    size_t copied = length;
    bool needs_lf = length == 0 || text[length - 1] != '\n';
    if (copied > max_payload || (needs_lf && copied == max_payload)) {
        copied = max_payload - 1;
        needs_lf = true;
        truncated = true;
    }
    if (copied != 0) {
        memcpy(record.bytes, text, copied);
    }
    size_t record_length = copied;
    if (needs_lf) {
        record.bytes[record_length++] = '\n';
    }
    record.length = static_cast<uint16_t>(record_length);
    if (truncated) {
        s_truncated_records.fetch_add(1, std::memory_order_relaxed);
    }
    return queue_record(record);
}

Stats get_stats()
{
    Stats stats{};
    stats.queued_records = s_queued_records.load(std::memory_order_relaxed);
    stats.dropped_records = s_dropped_records.load(std::memory_order_relaxed);
    stats.truncated_records = s_truncated_records.load(std::memory_order_relaxed);
    stats.write_errors = s_write_errors.load(std::memory_order_relaxed);
    stats.initialized = s_initialized.load(std::memory_order_acquire);
    stats.transport_ready = s_transport_ready.load(std::memory_order_acquire);
    return stats;
}

} // namespace UartConsole
