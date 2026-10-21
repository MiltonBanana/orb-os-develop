#pragma once
// Portable streaming Overpass -> existing little-endian road-tile geometry.
// One way is held at a time; the response and tile are never accumulated in RAM.
#include <ArduinoJson.h>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace road_tile_builder {
class BoundedAllocator : public ArduinoJson::Allocator {
    struct alignas(std::max_align_t) Header { size_t size; };
    size_t limit_, held_ = 0, peak_ = 0;
    void *(*alloc_)(size_t);
    void *(*realloc_)(void *, size_t);
    void (*free_)(void *);
public:
    BoundedAllocator(size_t limit, void *(*a)(size_t), void *(*r)(void *, size_t), void (*f)(void *))
        : limit_(limit), alloc_(a), realloc_(r), free_(f) {}
    size_t held() const { return held_; }
    size_t peak() const { return peak_; }
    void *allocate(size_t n) override {
        if (n > limit_ - held_) return nullptr;
        auto p = (Header *)alloc_(n + sizeof(Header));
        if (!p) return nullptr;
        p->size = n; held_ += n; if (held_ > peak_) peak_ = held_; return p + 1;
    }
    void deallocate(void *ptr) override {
        if (!ptr) return;
        auto p = (Header *)ptr - 1; held_ -= p->size; free_(p);
    }
    void *reallocate(void *ptr, size_t n) override {
        if (!ptr) return allocate(n);
        const size_t old = ((Header *)ptr - 1)->size;
        if (n > limit_ - (held_ - old)) return nullptr;
        auto p = (Header *)realloc_((Header *)ptr - 1, n + sizeof(Header));
        if (!p) return nullptr;
        held_ = held_ - old + n; if (held_ > peak_) peak_ = held_;
        p->size = n; return p + 1;
    }
};
constexpr unsigned MAX_POINTS = 2048;
constexpr unsigned MAX_POLYS = 16000; // roads_sd's table capacity
struct Point { double lat, lon; };
struct Scratch {
    Point points[MAX_POINTS];
    uint16_t stack[MAX_POINTS * 2];
    bool keep[MAX_POINTS];
    int16_t encoded[MAX_POINTS * 2];
};

inline unsigned simplify(Scratch &s, unsigned n) {
    if (n < 2) return 0;
    memset(s.keep, 0, n);
    s.keep[0] = s.keep[n - 1] = true;
    unsigned top = 0;
    s.stack[top++] = 0; s.stack[top++] = (uint16_t)(n - 1);
    while (top) {
        const unsigned hi = s.stack[--top], lo = s.stack[--top];
        const Point a = s.points[lo], b = s.points[hi];
        const double dx = b.lon - a.lon, dy = b.lat - a.lat;
        const double length2 = dx * dx + dy * dy;
        double best = 0.0025 * 0.0025;
        unsigned index = 0;
        for (unsigned i = lo + 1; i < hi; ++i) {
            const Point p = s.points[i];
            double t = length2 ? ((p.lon-a.lon)*dx + (p.lat-a.lat)*dy)/length2 : 0;
            if (t < 0) t = 0;
            if (t > 1) t = 1;
            const double ex = p.lon - a.lon - t * dx, ey = p.lat - a.lat - t * dy;
            if (ex * ex + ey * ey > best) { best = ex * ex + ey * ey; index = i; }
        }
        if (index) {
            s.keep[index] = true;
            s.stack[top++] = (uint16_t)lo; s.stack[top++] = (uint16_t)index;
            s.stack[top++] = (uint16_t)index; s.stack[top++] = (uint16_t)hi;
        }
    }
    unsigned count = 0;
    for (unsigned i = 0; i < n; ++i) if (s.keep[i]) {
        const int16_t lat = (int16_t)std::lround(s.points[i].lat * 180);
        const int16_t lon = (int16_t)std::lround(s.points[i].lon * 180);
        if (count && s.encoded[count*2-2] == lat && s.encoded[count*2-1] == lon) continue;
        s.encoded[count*2] = lat; s.encoded[count*2+1] = lon; ++count;
    }
    return count >= 2 ? count : 0;
}

template<class Input> struct Reader {
    Input &in;
    int cached = -2;
    int peek() { if (cached == -2) cached = in.read(); return cached; }
    int read() { const int c = peek(); cached = -2; return c; }
    size_t readBytes(char *dst, size_t n) {
        size_t i = 0;
        for (; i < n; ++i) { int c = read(); if (c < 0) break; dst[i] = (char)c; }
        return i;
    }
    int space() { int c; do { c = peek(); if (c == ' ' || c == '\r' || c == '\n' || c == '\t') read(); else return c; } while (true); }
    bool take(int c) { if (space() != c) return false; read(); return true; }
    // Skip metadata without retaining it. Strings and containers are bounded in depth;
    // the network reader independently bounds bytes and time.
    bool skip(unsigned depth = 0) {
        if (depth > 24) return false;
        int c = space();
        if (c == '"') {
            read();
            while ((c = read()) >= 0) {
                if (c == '"') return true;
                if (c < 32) return false;
                if (c == '\\' && read() < 0) return false;
            }
            return false;
        }
        if (c == '{' || c == '[') {
            read(); const int end = c == '{' ? '}' : ']';
            if (take(end)) return true;
            do {
                if (c == '{' && (!skip(depth+1) || !take(':'))) return false;
                if (!skip(depth+1)) return false;
                if (take(end)) return true;
            } while (take(','));
            return false;
        }
        unsigned n = 0;
        while ((c = peek()) >= 0 && c != ',' && c != '}' && c != ']' && c > 32) { read(); if (++n > 64) return false; }
        return n != 0;
    }
};

// Sink.line(int16 lat/lon pairs, point count) must consume/copy the data before returning.
// Any error, including an Overpass 'remark' on an otherwise HTTP-200 response, rejects
// the WHOLE tile. The caller publishes only after this function succeeds.
template<class Input, class Sink>
bool generate(Input &input, Sink &sink, Scratch &s, ArduinoJson::Allocator *allocator,
              const char *&error) {
    Reader<Input> r { input };
    JsonDocument doc = allocator ? JsonDocument(allocator) : JsonDocument();
    JsonDocument filter;
    filter["type"] = true;
    filter["geometry"][0]["lat"] = true;
    filter["geometry"][0]["lon"] = true;
    error = "invalid Overpass response";
    if (!r.take('{')) return false;
    bool found = false;
    unsigned polys = 0;
    do {
        if (!r.take('"')) return false;
        char key[64]; unsigned k = 0; int c;
        while ((c = r.read()) != '"') {
            if (c < 32 || c == '\\' || k >= sizeof(key)-1) return false;
            key[k++] = (char)c;
        }
        key[k] = 0;
        if (!r.take(':')) return false;
        if (!strcmp(key, "remark")) { error = "Overpass reported an error"; return false; }
        if (strcmp(key, "elements")) { if (!r.skip()) return false; }
        else {
            if (found || !r.take('[')) return false;
            found = true;
            if (!r.take(']')) {
                do {
                    if (r.space() != '{') return false;
                    const auto e = deserializeJson(doc, r, DeserializationOption::Filter(filter),
                                                   DeserializationOption::NestingLimit(16));
                    if (e) { error = "road JSON exceeds memory limit or is incomplete"; return false; }
                    if (strcmp(doc["type"] | "", "way")) return false;
                    JsonArrayConst geometry = doc["geometry"].template as<JsonArrayConst>();
                    if (geometry.isNull() || geometry.size() > MAX_POINTS) {
                        error = "road geometry exceeds point limit"; return false;
                    }
                    unsigned n = 0;
                    for (JsonObjectConst p : geometry) {
                        if (!p["lat"].is<double>() || !p["lon"].is<double>()) return false;
                        const double lat = p["lat"], lon = p["lon"];
                        if (!std::isfinite(lat) || !std::isfinite(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) return false;
                        s.points[n++] = {lat, lon};
                    }
                    n = simplify(s, n);
                    if (n) {
                        if (++polys > MAX_POLYS) { error = "tile exceeds polyline limit"; return false; }
                        if (!sink.line(s.encoded, n)) { error = "tile output cancelled"; return false; }
                    }
                } while (r.take(','));
                if (!r.take(']')) return false;
            }
        }
        if (r.take('}')) {
            if (!found || r.space() >= 0) return false;
            error = nullptr;
            return true; // empty tiles are valid and cached too
        }
    } while (r.take(','));
    return false;
}
} // namespace road_tile_builder
