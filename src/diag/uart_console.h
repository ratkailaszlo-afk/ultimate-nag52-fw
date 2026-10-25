#ifndef DIAG_UART_CONSOLE_H
#define DIAG_UART_CONSOLE_H

#include <stddef.h>
#include <stdint.h>

namespace UartConsole {

constexpr size_t QueueDepth = 32;
constexpr size_t RecordBytes = 192;
constexpr size_t DrainBudget = 2;

struct Stats {
    uint32_t queued_records;
    uint32_t dropped_records;
    uint32_t truncated_records;
    uint32_t write_errors;
    bool initialized;
    bool transport_ready;
};

// Install a bounded, nonblocking ESP_LOG producer. Call before starting tasks.
// If the static queue cannot be created, the installed callback drops output
// and never falls back to the default UART writer.
bool init();

// Call after UART0's driver has been installed successfully. This switches
// stdout/stderr to the driver-backed VFS path, but does not enable queue drains.
void configure_vfs_driver();

// Call after endpoint buffers/mutex are valid and UART flush succeeded. Queue
// drains are enabled only when init() created its queue and VFS was configured.
bool mark_transport_ready();

// Drain at most max_records (and never more than DrainBudget) complete records.
// If counters changed, at most one rate-limited summary record is also emitted.
size_t drain(size_t max_records);

// Queue an already formatted record without allocating. Embedded newlines are
// preserved; a final LF is added when absent.
bool enqueue_text(const char *text, size_t length);

Stats get_stats();

} // namespace UartConsole

#endif // DIAG_UART_CONSOLE_H
