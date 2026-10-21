#include "roads_download.h"
#ifdef ARDUINO
#include "road_tile_builder.h"
#include "roads_sd.h"
#include "sdcard.h"
#include "theme_sd.h"
#include "config.h"
#include <Arduino.h>
#include <SD.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <StreamUtils.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <new>

namespace roads_download {
namespace {
// A failed public query is retried slowly, never in a boot-blocking loop.
constexpr uint32_t RETRY_MS = 10 * 60 * 1000;
constexpr uint32_t MAX_RESPONSE = 8 * 1024 * 1024;
constexpr const char *LENGTHS = "/roads/_download.len";
constexpr const char *POINTS = "/roads/_download.pts";
constexpr const char *PART = "/roads/_download.part";
struct Packet { uint16_t kind, size; uint8_t data[512]; }; // 0=length, 1=points, 2=success, 3=error
QueueHandle_t queue = nullptr;
std::atomic<bool> cancelled {false}, running {false};
std::atomic<int> lastHttp {0}, lastLength {-1};
int tileLat = 0, tileLon = 0;
const char *state = "idle", *failure = "";
File lengths, points, part, source;
uint16_t numPolys = 0;
uint32_t numPoints = 0, nextScan = 15000;
unsigned combinePhase = 0;
struct Attempt { int lat = 999, lon = 999; uint32_t retry = 0; unsigned count = 0; } attempts[9];
int centreLat = 999, centreLon = 999;

struct CardGuard {
    bool held = theme_sd::try_lock();
    ~CardGuard() { if (held) theme_sd::unlock(); }
};
void *json_alloc(size_t n) { return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }
void *json_realloc(void *p, size_t n) { return heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); }

bool send(Packet &p) {
    const uint32_t start = millis();
    while (!cancelled && millis() - start < 30000) {
        if (xQueueSend(queue, &p, pdMS_TO_TICKS(100)) == pdTRUE) return true;
    }
    return false;
}
struct Sink {
    bool line(const int16_t *coords, unsigned n) {
        Packet p {}; p.kind = 0; p.size = 2;
        p.data[0] = n & 255; p.data[1] = n >> 8;
        if (!send(p)) return false;
        // Encode explicitly instead of depending on host endianness.
        for (unsigned i = 0; i < n * 2;) {
            p.kind = 1; p.size = 0;
            while (i < n * 2 && p.size < sizeof(p.data)) {
                const uint16_t v = (uint16_t)coords[i++];
                p.data[p.size++] = v & 255; p.data[p.size++] = v >> 8;
            }
            if (!send(p)) return false;
        }
        return true;
    }
};
// ArduinoJson reads one byte at a time. Buffering below amortizes socket reads;
// every blocking read remains on the worker, bounded by idle and total deadlines.
struct Input {
    Stream &body;
    WiFiClient &client;
    uint32_t start = millis(), bytes = 0;
    uint8_t buffer[1024];
    size_t position = 0, size = 0;
    int read() {
        if (cancelled || bytes >= MAX_RESPONSE || millis() - start >= 150000) return -1;
        if (position == size) {
            if (!client.connected() && !body.available()) return -1;
            const size_t want = MAX_RESPONSE - bytes < sizeof(buffer) ? MAX_RESPONSE - bytes : sizeof(buffer);
            size = body.readBytes(buffer, want);
            position = 0;
            if (!size) return -1;
        }
        ++bytes;
        return buffer[position++];
    }
};

void worker(void *) {
    const char *error = "map download failed";
    char detail[160] = {};
    bool ok = false;
    {
        auto scratch = (road_tile_builder::Scratch *)heap_caps_malloc(sizeof(road_tile_builder::Scratch), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!scratch) error = "not enough PSRAM for road generation";
        else {
            WiFiClientSecure client;
            client.setInsecure(); // matches existing firmware HTTPS clients
            client.setTimeout(10000);
            client.setHandshakeTimeout(10);
            HTTPClient http;
            http.setReuse(false);
            http.setConnectTimeout(5000);
            // HTTPClient uses this timeout while waiting for RESPONSE HEADERS too.
            // Overpass may queue for ~15s and then execute the query. Its uint16_t
            // timeout cannot exceed 65535ms: allow 45s execution plus that queue time.
            http.setTimeout(65000);
            const char *headers[] = { "Transfer-Encoding" };
            http.collectHeaders(headers, 1);
            // One bounded one-degree cell. out geom omits tags/nodes with 'skel'.
            char query[256];
            snprintf(query, sizeof(query),
                     "[out:json][timeout:45][maxsize:33554432];way[highway~\"^(motorway|trunk)$\"](%d,%d,%d,%d);out skel geom;",
                     tileLat, tileLon, tileLat + 1, tileLon + 1);
            if (http.begin(client, ROAD_OVERPASS_URL)) {
                http.setUserAgent(ORB_USER_AGENT);
                http.addHeader("Accept", "application/json");
                http.setAcceptEncoding("identity");
                http.addHeader("Content-Type", "text/plain");
                const int code = http.POST((uint8_t *)query, strlen(query));
                lastHttp = code; lastLength = http.getSize();
                Serial.printf("[roads_fetch] HTTP %d for r%d_%d, length=%d internal_largest=%u psram_free=%u\n",
                              code, tileLat, tileLon, http.getSize(),
                              (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                              (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
                if (code == 200 && http.getSize() <= (int)MAX_RESPONSE) {
                    http.setTimeout(10000); // body reads have a separate idle timeout
                    WiFiClient *raw = http.getStreamPtr();
                    raw->setTimeout(10000);
                    ChunkDecodingStream chunked(*raw);
                    const bool chunks = http.header("Transfer-Encoding").equalsIgnoreCase("chunked");
                    Stream &decoded = chunks ? static_cast<Stream &>(chunked) : static_cast<Stream &>(*raw);
                    decoded.setTimeout(10000);
                    Input input { decoded, client };
                    road_tile_builder::BoundedAllocator allocator(96 * 1024, json_alloc, json_realloc, heap_caps_free);
                    Sink sink;
                    ok = road_tile_builder::generate(input, sink, *scratch, &allocator, error);
                    if (ok && ((http.getSize() >= 0 && input.bytes != (uint32_t)http.getSize()) ||
                               (chunks && (!chunked.ended() || chunked.error())) || cancelled)) {
                        ok = false; error = "incomplete map response";
                    }
                } else {
                    if (code < 0) {
                        char tls[80] = {};
                        const int tlsCode = client.lastError(tls, sizeof(tls));
                        if (tlsCode)
                            snprintf(detail, sizeof(detail), "HTTP transport %d (%s); TLS %d: %.70s",
                                     code, HTTPClient::errorToString(code).c_str(), tlsCode, tls);
                        else
                            snprintf(detail, sizeof(detail), "HTTP transport %d: %s",
                                     code, HTTPClient::errorToString(code).c_str());
                    } else if (code != 200) {
                        snprintf(detail, sizeof(detail), "Overpass HTTP %d%s", code,
                                 code == 429 ? ": rate limited" :
                                 code == 504 ? ": query timed out or server busy" :
                                 code == 400 ? ": query rejected" : "");
                    } else {
                        snprintf(detail, sizeof(detail), "Overpass response %d bytes exceeds %lu-byte limit",
                                 http.getSize(), (unsigned long)MAX_RESPONSE);
                    }
                    error = detail;
                }
                http.end();
            } else error = "cannot initialize Overpass HTTPS request";
            heap_caps_free(scratch);
        }
    }
    Packet done {}; done.kind = ok ? 2 : 3;
    if (!ok) {
        strlcpy((char *)done.data, error ? error : "map download failed", sizeof(done.data));
        done.size = strlen((char *)done.data) + 1;
    }
    send(done);
    running = false;
    vTaskDelete(nullptr);
}

char errorText[160] = "";
void cleanup() {
    lengths.close(); points.close(); source.close(); part.close();
    SD.remove(LENGTHS); SD.remove(POINTS); SD.remove(PART);
    if (queue) { vQueueDelete(queue); queue = nullptr; }
    combinePhase = 0;
}
void fail(const char *why) {
    strlcpy(errorText, why, sizeof(errorText)); failure = errorText;
    Serial.printf("[roads_fetch] r%d_%d: %s; retry in 10 minutes (max 3 attempts per cell/boot)\n", tileLat, tileLon, failure);
    cancelled = true;
    state = "failed";
    // The worker may still own the queue. Only step() cleans it after running=false.
    nextScan = millis() + 60000;
}
bool cached(const char *path) {
    File f = SD.open(path, FILE_READ);
    uint8_t h[6];
    if (!f || f.read(h, 6) != 6) return false;
    const unsigned polys = h[0] | (h[1] << 8);
    const uint32_t pts = h[2] | ((uint32_t)h[3] << 8) | ((uint32_t)h[4] << 16) | ((uint32_t)h[5] << 24);
    return polys <= road_tile_builder::MAX_POLYS && pts <= 2 * 1024 * 1024 &&
           pts >= polys * 2 && f.size() == 6 + polys * 2 + pts * 4;
}
void pathFor(char *path, size_t size, int lat, int lon) {
    snprintf(path, size, "/roads/r%d_%d.bin", lat, lon);
}
void start(int lat, int lon) {
    tileLat = lat; tileLon = lon;
    cleanup();
    if (!SD.exists("/roads") && !SD.mkdir("/roads")) { fail("cannot create /roads"); return; }
    lengths = SD.open(LENGTHS, FILE_WRITE); points = SD.open(POINTS, FILE_WRITE);
    if (!lengths || !points) { fail("cannot create temporary map files"); cleanup(); return; }
    queue = xQueueCreate(4, sizeof(Packet));
    if (!queue) { fail("not enough memory for map queue"); cleanup(); return; }
    numPolys = 0; numPoints = 0; cancelled = false; running = true; failure = "";
    lastHttp = 0; lastLength = -1;
    state = "fetching";
    if (xTaskCreatePinnedToCore(worker, "roads_fetch", 12288, nullptr, 1, nullptr, 0) != pdPASS) {
        running = false; fail("cannot start map worker"); cleanup(); return;
    }
    Serial.printf("[roads_fetch] generating r%d_%d on device\n", lat, lon);
}
} // namespace

const char *status() { return state; }
const char *lastError() { return failure; }
int httpStatus() { return lastHttp.load(); }
int responseLength() { return lastLength.load(); }

void step(double lat, double lon, float rangeKm, bool enabled, bool transferBusy) {
    // Pause SD work during installs. Backpressure stops the worker; after 30s it gives
    // up and the partial tile is discarded. No other task ever writes map files.
    if (transferBusy) return;
    CardGuard guard;
    if (!guard.held) return;
    if (!sdcard::mounted() && queue && !cancelled) fail("SD card unavailable");
    if (queue && (!enabled || WiFi.status() != WL_CONNECTED)) cancelled = true;
    if (queue && cancelled) {
        if (!running) { if (strcmp(state, "failed")) fail("map request cancelled"); cleanup(); }
        return;
    }
    if (!sdcard::mounted()) return;
    if (queue && combinePhase == 0) {
        Packet p;
        // At most 2 KB of output per loop; SD writes are protected against audio's reader.
        for (unsigned i = 0; i < 4 && xQueueReceive(queue, &p, 0) == pdTRUE; ++i) {
            if (p.kind == 0) {
                if (lengths.write(p.data, 2) != 2) { fail("SD length write failed"); return; }
                ++numPolys; numPoints += p.data[0] | (p.data[1] << 8);
            } else if (p.kind == 1) {
                if (points.write(p.data, p.size) != p.size) { fail("SD point write failed"); return; }
            } else if (p.kind == 3) { fail((char *)p.data); return; }
            else {
                lengths.close(); points.close();
                part = SD.open(PART, FILE_WRITE);
                uint8_t h[6] = { (uint8_t)numPolys, (uint8_t)(numPolys >> 8),
                    (uint8_t)numPoints, (uint8_t)(numPoints >> 8), (uint8_t)(numPoints >> 16), (uint8_t)(numPoints >> 24) };
                if (!part || part.write(h, 6) != 6) { fail("SD header write failed"); return; }
                source = SD.open(LENGTHS, FILE_READ);
                if (!source) { fail("SD length read failed"); return; }
                combinePhase = 1; state = "saving";
                break;
            }
        }
        if (combinePhase == 0 && !running && uxQueueMessagesWaiting(queue) == 0) fail("map worker stopped without a complete tile");
        return;
    }
    if (queue && combinePhase) {
        uint8_t buf[2048];
        const size_t want = source.size() - source.position();
        if (want) {
            const size_t n = source.read(buf, want < sizeof(buf) ? want : sizeof(buf));
            if (!n || part.write(buf, n) != n) fail("SD tile assembly failed");
            return;
        }
        source.close();
        if (combinePhase == 1) {
            source = SD.open(POINTS, FILE_READ);
            if (!source || source.size() != numPoints * 4) { fail("SD geometry size mismatch"); return; }
            combinePhase = 2; return;
        }
        if (running) return; // never free a queue the worker could still be using
        part.close();
        char path[48]; pathFor(path, sizeof(path), tileLat, tileLon);
        if (!cached(PART)) { fail("generated tile failed validation"); return; }
        // Only missing/invalid tiles are generated. Don't replace a valid tile installed
        // while the query was in flight. A reset during generation leaves no final file.
        if (!cached(path)) {
            if (SD.exists(path)) SD.remove(path);
            if (!SD.rename(PART, path)) { fail("cannot publish generated tile"); return; }
            roads_sd::tiles_changed();
        }
        Serial.printf("[roads_fetch] cached %s: %u roads, %lu points\n", path, numPolys, (unsigned long)numPoints);
        cleanup(); state = "ready"; nextScan = millis() + 10000;
        return;
    }
    if (!enabled || WiFi.status() != WL_CONNECTED || !std::isfinite(lat) || !std::isfinite(lon) ||
        lat < -90 || lat >= 90 || lon < -180 || lon >= 180 || !std::isfinite(rangeKm) || rangeKm <= 0 ||
        (int32_t)(millis() - nextScan) < 0) return;
    nextScan = millis() + 30000;
    const int cy = (int)floor(lat), cx = (int)floor(lon);
    if (cy != centreLat || cx != centreLon) {
        centreLat = cy; centreLon = cx;
        for (auto &a : attempts) a = Attempt {};
    }
    const double dy = rangeKm / 111.0, dx = dy / fmax(0.15, cos(lat * 3.141592653589793 / 180.0));
    // Match the reader's 3x3 neighbourhood; home first, then the cells the scope reaches.
    const int offsets[9][2] = {{0,0},{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{1,-1},{-1,1},{1,1}};
    for (int i = 0; i < 9; ++i) {
        const int y = cy + offsets[i][1], x = cx + offsets[i][0];
        if (y < -90 || y >= 90 || x < -180 || x >= 180 || y > lat + dy || y+1 < lat-dy || x > lon+dx || x+1 < lon-dx) continue;
        char path[48]; pathFor(path, sizeof(path), y, x);
        if (cached(path)) continue;
        Attempt &a = attempts[i];
        if (a.count >= 3 || (a.count && (int32_t)(millis() - a.retry) < 0)) continue;
        a.lat = y; a.lon = x; ++a.count; a.retry = millis() + RETRY_MS;
        start(y, x); return;
    }
}
} // namespace roads_download
#endif
