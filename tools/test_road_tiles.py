#!/usr/bin/env python3
"""Build/run portable road-generator tests; --live also tests one real Overpass cell.

Requires PlatformIO's ArduinoJson dependency plus a host C++17 compiler (CXX or
g++, or Visual Studio's C++ tools on Windows). Does not flash a device.
"""
import argparse
import os
from pathlib import Path
import shutil
import ssl
import re
import subprocess
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--url", help="Override the firmware's ROAD_OVERPASS_URL for the live test")
    parser.add_argument("--tls12", action="store_true", help="Restrict live HTTPS to the firmware SDK's TLS 1.2")
    parser.add_argument("--insecure", action="store_true", help="Match the firmware's existing setInsecure certificate policy")
    parser.add_argument("--tile", type=int, nargs=2, default=(37, -113), metavar=("LAT", "LON"))
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    os.chdir(root)
    build = root / ".pio" / "road-tests"
    build.mkdir(parents=True, exist_ok=True)
    include = root / ".pio/libdeps/esp32-s3-amoled-175/ArduinoJson/src"
    source = root / "tests/road_tile_builder_test.cpp"
    exe = build / ("test.exe" if os.name == "nt" else "test")
    compiler = os.environ.get("CXX") or shutil.which("g++") or shutil.which("clang++")
    if compiler:
        subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-I", str(root / "src"),
                        "-I", str(include), str(source), "-o", str(exe)], check=True)
    elif os.name == "nt":
        vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        installation = subprocess.check_output([str(vswhere), "-latest", "-products", "*",
                                                "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                                                "-property", "installationPath"], text=True).strip()
        if not installation:
            raise SystemExit("No host C++ compiler found; set CXX or install Visual Studio C++ tools")
        vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
        script = build / "build.cmd"
        script.write_text(f'@echo off\ncall "{vcvars}" >nul\n'
                          f'cl /nologo /EHsc /std:c++17 /W4 /I "{root / "src"}" /I "{include}" '
                          f'"{source}" /Fe:"{exe}" /Fo:"{build / "test.obj"}"\n', encoding="utf-8")
        subprocess.run(["cmd", "/c", str(script)], check=True)
    else:
        raise SystemExit("No host C++ compiler found; set CXX")
    subprocess.run([str(exe)], check=True)
    if args.live:
        lat, lon = args.tile
        if not (-90 <= lat < 90 and -180 <= lon < 180):
            raise SystemExit("Tile coordinates out of bounds")
        query = ('[out:json][timeout:45][maxsize:33554432];way[highway~"^(motorway|trunk)$"]'
                 f'({lat},{lon},{lat+1},{lon+1});out skel geom;')
        url = args.url or re.search(r'#define ROAD_OVERPASS_URL "([^"]+)"', (root / "src/config.h").read_text())[1]
        context = ssl.create_default_context()
        if args.insecure:
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        if args.tls12:
            context.minimum_version = ssl.TLSVersion.TLSv1_2
            context.maximum_version = ssl.TLSVersion.TLSv1_2
        request = urllib.request.Request(url, data=query.encode(),
                                         headers={"User-Agent": "TheOrbOS-road-generator-test/1.0", "Content-Type": "text/plain"})
        print(f"Live query: {url}, r{lat}_{lon}, TLS 1.2 only={args.tls12}", flush=True)
        with urllib.request.urlopen(request, timeout=65, context=context) as response:
            data = response.read(8 * 1024 * 1024 + 1)
        if len(data) > 8 * 1024 * 1024:
            raise SystemExit("Live response exceeded firmware's 8 MB limit")
        fixture = build / "overpass.json"
        fixture.write_bytes(data)
        print(f"Downloaded {len(data)} bytes of live road geometry", flush=True)
        subprocess.run([str(exe), str(fixture)], check=True)


if __name__ == "__main__":
    main()
