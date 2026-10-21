# Road maps on the SD card

Flight Tracker's Map switch enables drawing and automatic on-device generation
of missing road tiles once Wi-Fi, a saved home location and a mounted SD card
are available. Startup does not wait for roads: the first check runs after
15 seconds, then the home cell is fetched before neighbouring cells intersected
by the flight tracker's range. Cached tiles work offline and survive restarts.

Orb Studio's preview uses stand-in streets and does not prove that the device
has road data for its location. The device automatically stores its generated
OpenStreetMap road tiles in `/roads/` on the microSD card, separate from the theme folder.

The Orb queries Overpass directly, parses one road at a time, simplifies its
geometry using a 0.0025-degree tolerance, and
encodes coordinates at 180 units per degree. The network worker never accesses
the card or LVGL. A four-packet queue transfers at most 512 bytes per packet to
the main loop; the main loop writes at most 2 KB per pass and defers map writes
while audio owns the card or a theme transfer is active. Failed/interrupted
queries leave temporary files, which are removed before the next attempt.
Only a finished tile is renamed into its final filename. Roads refresh when
the tile is published, without a restart; aircraft trails are preserved.

This is an experimental direct-Overpass implementation. Limits per query are
8 MB of response data, 150 seconds of reads (plus connection/request time),
96 KB of JSON allocations in PSRAM, 2,048 raw points per road and 16,000
output roads per tile. The geometry scratch is 50 KB of PSRAM, with
additional memory needed for TLS and the worker stack. Exceeding a limit
rejects the whole tile rather than caching incomplete coverage. Successful
empty tiles are cached too. Failed cells wait ten minutes before retrying,
with at most three attempts per cell per boot while the home cell is unchanged.
There is at least a one-minute gap after failures and ten seconds between
successful queries. Existing tiles are reused indefinitely; automatic refresh,
cache eviction and coverage beyond the reader's 3x3 neighbourhood are not
implemented. The endpoint is configurable with `ROAD_OVERPASS_URL` in
`src/config.h`.

Overpass queries allow 45 seconds of server execution. The Orb waits up to
65 seconds for response headers, allowing for server queue time, then uses
a separate ten-second idle timeout for body reads. TLS handshakes have a
ten-second timeout.

The default endpoint is `https://maps.mail.ru/osm/tools/overpass/api/interpreter`.
A live query for `r37_-113` was verified
over TLS 1.2 using the firmware's existing `setInsecure()` certificate policy:
213,638 bytes of geometry produced 122 roads, with 20,696 bytes of peak JSON
allocation in the desktop parser. TLS 1.2 is the protocol enabled in this
firmware's prebuilt SDK. The previous default, `overpass-api.de`, rejected TLS 1.2
handshakes in desktop probes while accepting TLS 1.3, matching the Orb's
`TLS -30592` peer-fatal-alert failure. Changing certificate verification or
query syntax does not address that handshake failure.

Check `http://theorb.local/roads.json` for `status`, `error`, `http` (the actual
HTTP status, or a negative HTTPClient transport error), `responseLength` (-1
for unknown/chunked responses), `endpoint`, and the number of
tiles published this boot (`revision`). Serial logs prefixed `[roads_fetch]`
report generation, HTTP status and completion/failure. A busy/unavailable
Overpass instance or insufficient free memory leaves the flight tracker usable
with whatever tiles are already cached.

Ensure the device's home location is correct and the theme's Map switch is on,
with nonzero road strength and a visible road colour.

The on-device generator includes motorways and trunk roads; local residential
streets are not included. A rural location or a tight range may therefore have
no visible roads even with valid tiles. Airports use separate built-in data and
require both the theme's Airports switch and the device's airports setting.

USB serial logs at 115200 baud report `[roads_sd] flight:` when the projection
changes. Zero tiles opened means the card or expected files could not be read.
Tiles opened with zero visible polylines can mean roads are outside the current
range or the tile data is invalid. These counters do not validate tile contents.

Run the portable generator tests with `python tools/test_road_tiles.py` after
installing the PlatformIO dependencies. Add `--live` to fetch one real Utah
Overpass cell and run it through the same parser/simplifier used by the Orb.
The tests require a host C++17 compiler and do not flash the device.
Use `--live --tls12 --insecure` to test the configured endpoint with TLS 1.2
and the firmware's certificate policy explicitly;
`--tile 37 -113` selects the one-degree cell, and `--url` overrides the endpoint
for the test without changing the firmware configuration.
