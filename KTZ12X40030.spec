# -*- mode: python ; coding: utf-8 -*-
import os
import sysconfig
import platform
import customtkinter
_ctk_path = os.path.dirname(customtkinter.__file__)
_arch = "x86_64" if platform.machine() == "AMD64" else platform.machine().lower()
_libusb_dll = os.path.join(
    sysconfig.get_path("purelib"),
    "libusb", "_platform", "windows", _arch, "libusb-1.0.dll"
)

a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=[(_libusb_dll, ".")] if os.path.exists(_libusb_dll) else [],
    datas=[
        (_ctk_path, 'customtkinter'),
    ],
    hiddenimports=[
        'can',
        'can.interfaces',
        'can.interfaces.canalystii',
        'can.interfaces.slcan',
        'can.interfaces.robotell',
        'can.interfaces.socketcan',
        'canalystii',
        'serial',
        'serial.tools',
        'serial.tools.list_ports',
        'customtkinter',
        'tkinter',
        'tkinter.ttk',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
)

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name='KTZ12X40030',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
