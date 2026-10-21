#pragma once
// Call only from loop(). Network/JSON work runs on a worker that NEVER touches SD/LVGL.
namespace roads_download {
void step(double lat, double lon, float rangeKm, bool enabled, bool transferBusy);
const char *status();
const char *lastError();
int httpStatus();      // 0 before a request, negative for an HTTPClient transport error
int responseLength(); // -1 when unknown/chunked
}
