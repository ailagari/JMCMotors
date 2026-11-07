#!/usr/bin/env python3
"""
Build the JMC DMX Bridge app for the OS this runs on:

    Windows  dist/JMC-DMX-Bridge.exe          (one file)
    macOS    dist/JMC DMX Bridge.app          (app bundle)
    Linux    dist/JMC-DMX-Bridge              (one file)

and a zip for the release page: dist/JMC-DMX-Bridge-<version>-<os>.zip
(the app, the guide and the license).

    python -m pip install pyinstaller pyserial
    python build_app.py

PyInstaller cannot cross-compile: build each OS on that OS. GitHub does all
three for every release (.github/workflows/dmx-bridge.yml).
"""

import os
import platform
import shutil
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from jmc_dmx_bridge import VERSION  # noqa: E402


def main():
    dist = os.path.join(HERE, "dist")
    work = os.path.join(HERE, "build")
    shutil.rmtree(dist, ignore_errors=True)
    shutil.rmtree(work, ignore_errors=True)

    mac = sys.platform == "darwin"
    win = sys.platform.startswith("win")
    name = "JMC DMX Bridge" if mac else "JMC-DMX-Bridge"
    args = [sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
            "--name", name, "--windowed",
            "--distpath", dist, "--workpath", work, "--specpath", work,
            "--hidden-import", "serial", "--hidden-import", "serial.tools.list_ports",
            os.path.join(HERE, "jmc_dmx_bridge.py")]
    if mac:
        args[5:5] = ["--osx-bundle-identifier", "com.ailagari.jmcdmxbridge"]
    else:
        args[5:5] = ["--onefile"]
    icon = os.path.join(HERE, "icon.ico")
    if win and os.path.exists(icon):
        args[5:5] = ["--icon", icon, "--add-data", icon + os.pathsep + "."]
    print(" ".join(args))
    subprocess.check_call(args)

    osname = {"win32": "windows", "darwin": "macos"}.get(sys.platform, "linux")
    arch = platform.machine().lower().replace("amd64", "x64").replace("x86_64", "x64")
    zpath = os.path.join(dist, "JMC-DMX-Bridge-%s-%s-%s.zip" % (VERSION, osname, arch))
    with zipfile.ZipFile(zpath, "w", zipfile.ZIP_DEFLATED) as z:
        if mac:
            app = os.path.join(dist, name + ".app")
            for root, _, files in os.walk(app):
                for f in files:
                    full = os.path.join(root, f)
                    rel = os.path.relpath(full, dist)
                    info = zipfile.ZipInfo.from_file(full, rel)
                    with open(full, "rb") as fh:
                        z.writestr(info, fh.read())   # keeps the executable bits
        else:
            exe = os.path.join(dist, name + (".exe" if win else ""))
            info = zipfile.ZipInfo.from_file(exe, os.path.basename(exe))
            with open(exe, "rb") as fh:
                z.writestr(info, fh.read(), zipfile.ZIP_DEFLATED)
        z.write(os.path.join(HERE, "README.md"), "Guide.md")
        z.write(os.path.join(REPO, "LICENSE"), "LICENSE.txt")
    print("Built", zpath)


if __name__ == "__main__":
    main()
