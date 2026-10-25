#include "perf_mon.h"
#include <stdint.h>

// The previous idle-loop calibration modified WAITI behavior and was not a
// valid CPU load measurement on this SDK. Preserve the ABI, mark it unknown.
void PerfMon::init_perfmon(void) {}
void PerfMon::update_sample(void) {}
CpuStats PerfMon::get_cpu_stats(void) { return {UINT16_MAX, UINT16_MAX}; }
