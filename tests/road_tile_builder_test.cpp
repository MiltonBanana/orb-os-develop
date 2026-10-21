#include "road_tile_builder.h"
#include <cassert>
#include <string>
#include <vector>
#include <fstream>
#include <iterator>
#include <iostream>
#ifdef _WIN32
#include <windows.h>
#endif

struct Input {
    const std::string &s; size_t pos = 0;
    int read() { return pos < s.size() ? (unsigned char)s[pos++] : -1; }
};
struct Sink {
    std::vector<std::vector<int16_t>> lines;
    bool line(const int16_t *p, unsigned n) { lines.emplace_back(p, p + 2*n); return true; }
};
struct RejectAllocator : ArduinoJson::Allocator {
    void *allocate(size_t) override { return nullptr; }
    void deallocate(void *) override {}
    void *reallocate(void *, size_t) override { return nullptr; }
};
static road_tile_builder::Scratch scratch;
static bool build(const std::string &s, Sink &sink, ArduinoJson::Allocator *a = nullptr) {
    Input in {s}; const char *err = nullptr;
    return road_tile_builder::generate(in, sink, scratch, a, err);
}
int main(int argc, char **argv) {
#ifdef _WIN32
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
#endif
    const std::string way = R"({"type":"way","id":1,"nodes":[1,2,3],"geometry":[{"lat":37.1,"lon":-113.2},{"lat":37.15,"lon":-113.2},{"lat":37.2,"lon":-113.2}]})";
    Sink sink;
    assert(build("{\"version\":0.6,\"osm3s\":{\"copyright\":\"a \\\"quote\\\"\"},\"elements\":[" + way + "]}", sink));
    assert(sink.lines.size() == 1 && sink.lines[0].size() == 4);
    assert(sink.lines[0][0] == 6678 && sink.lines[0][1] == -20376);
    assert(sink.lines[0][2] == 6696);
    Sink empty;
    assert(build("{ \"elements\" : [ ] }\n", empty));
    assert(empty.lines.empty());
    assert(build("{\"elements\":[" + way + "," + way + "]}", empty));
    assert(empty.lines.size() == 2);
    for (const std::string bad : {
        "{}", "{\"elements\":[]", "{\"elements\":[]}junk", "{\"elements\":[],\"elements\":[]}",
        "{\"remark\":\"timeout\",\"elements\":[]}", "{\"elements\":[],\"remark\":\"runtime error\"}",
        "{\"elements\":[{\"type\":\"way\",\"geometry\":[{\"lat\":91,\"lon\":1}]}]}",
        "{\"elements\":[{\"type\":\"way\",\"geometry\":[{\"lat\":null,\"lon\":1}]}]}",
        "{\"elements\":[{\"type\":\"way\",\"geometry\":[{\"lat\":1}]}]}"
    }) { Sink out; assert(!build(bad, out)); }
    // Every truncated prefix must fail, even after a complete way has been emitted.
    const std::string full = "{\"elements\":[" + way + "]}";
    for (size_t i = 0; i < full.size(); ++i) { Sink out; assert(!build(full.substr(0, i), out)); }
    RejectAllocator reject;
    Sink out; assert(!build(full, out, &reject));
    road_tile_builder::BoundedAllocator budget(96 * 1024, std::malloc, std::realloc, std::free);
    Sink bounded; assert(build(full, bounded, &budget));
    assert(budget.held() == 0 && budget.peak() > 0 && budget.peak() <= 96 * 1024);
    assert(budget.allocate(96 * 1024 + 1) == nullptr);
    void *block = budget.allocate(100);
    assert(block && budget.held() == 100);
    assert(budget.reallocate(block, 96 * 1024 + 1) == nullptr);
    assert(budget.held() == 100);
    block = budget.reallocate(block, 200);
    assert(block && budget.held() == 200);
    budget.deallocate(block); assert(budget.held() == 0);
    // Quantized duplicates must not create a zero-length road.
    Sink duplicates;
    assert(build(R"({"elements":[{"type":"way","geometry":[{"lat":1,"lon":2},{"lat":1.00001,"lon":2}]}]})", duplicates));
    assert(duplicates.lines.empty());
    // Preserve a bend while removing intermediate collinear points.
    scratch.points[0] = {0,0}; scratch.points[1] = {0,0.5};
    scratch.points[2] = {0,1}; scratch.points[3] = {1,1};
    assert(road_tile_builder::simplify(scratch, 4) == 3);
    std::string many = "{\"elements\":[{\"type\":\"way\",\"geometry\":[";
    for (unsigned i = 0; i <= road_tile_builder::MAX_POINTS; ++i) {
        if (i) many += ',';
        many += "{\"lat\":1,\"lon\":2}";
    }
    many += "]}]}";
    Sink oversized; assert(!build(many, oversized));
    if (argc == 2) {
        std::ifstream f(argv[1], std::ios::binary);
        assert(f.good());
        const std::string response((std::istreambuf_iterator<char>(f)), {});
        Sink live; assert(build(response, live, &budget));
        assert(budget.held() == 0);
        std::cout << "Live fixture: " << live.lines.size() << " roads; peak JSON memory " << budget.peak() << " bytes\n";
    }
    std::cout << "Road tile builder tests passed\n";
}
