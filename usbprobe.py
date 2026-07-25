"""
usbprobe.py — dump the USB descriptor tree of the Kinect devices via libusb.

Reads config/interface/endpoint descriptors, which libusb exposes WITHOUT
opening the device (so no driver rebind needed). Used to locate the audio
isochronous IN endpoint and understand the "security" interface on the
Kinect-for-Windows audio composite (VID 045E, PID 02BB).
"""

from __future__ import annotations
import ctypes as C
import os
from pathlib import Path

_DIST = Path(__file__).resolve().parent / "dist"
os.add_dll_directory(str(_DIST))
lib = C.CDLL(str(_DIST / "libusb-1.0.dll"))

VID_MS = 0x045E


class DeviceDescriptor(C.Structure):
    _fields_ = [(n, C.c_uint8 if s == 1 else C.c_uint16) for n, s in [
        ("bLength", 1), ("bDescriptorType", 1), ("bcdUSB", 2),
        ("bDeviceClass", 1), ("bDeviceSubClass", 1), ("bDeviceProtocol", 1),
        ("bMaxPacketSize0", 1), ("idVendor", 2), ("idProduct", 2),
        ("bcdDevice", 2), ("iManufacturer", 1), ("iProduct", 1),
        ("iSerialNumber", 1), ("bNumConfigurations", 1)]]


class EndpointDescriptor(C.Structure):
    _fields_ = [
        ("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
        ("bEndpointAddress", C.c_uint8), ("bmAttributes", C.c_uint8),
        ("wMaxPacketSize", C.c_uint16), ("bInterval", C.c_uint8),
        ("bRefresh", C.c_uint8), ("bSynchAddress", C.c_uint8),
        ("extra", C.POINTER(C.c_ubyte)), ("extra_length", C.c_int)]


class InterfaceDescriptor(C.Structure):
    _fields_ = [
        ("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
        ("bInterfaceNumber", C.c_uint8), ("bAlternateSetting", C.c_uint8),
        ("bNumEndpoints", C.c_uint8), ("bInterfaceClass", C.c_uint8),
        ("bInterfaceSubClass", C.c_uint8), ("bInterfaceProtocol", C.c_uint8),
        ("iInterface", C.c_uint8), ("endpoint", C.POINTER(EndpointDescriptor)),
        ("extra", C.POINTER(C.c_ubyte)), ("extra_length", C.c_int)]


class Interface(C.Structure):
    _fields_ = [("altsetting", C.POINTER(InterfaceDescriptor)),
                ("num_altsetting", C.c_int)]


class ConfigDescriptor(C.Structure):
    _fields_ = [
        ("bLength", C.c_uint8), ("bDescriptorType", C.c_uint8),
        ("wTotalLength", C.c_uint16), ("bNumInterfaces", C.c_uint8),
        ("bConfigurationValue", C.c_uint8), ("iConfiguration", C.c_uint8),
        ("bmAttributes", C.c_uint8), ("MaxPower", C.c_uint8),
        ("interface", C.POINTER(Interface)),
        ("extra", C.POINTER(C.c_ubyte)), ("extra_length", C.c_int)]


lib.libusb_init.argtypes = [C.POINTER(C.c_void_p)]
lib.libusb_get_device_list.argtypes = [C.c_void_p, C.POINTER(C.POINTER(C.c_void_p))]
lib.libusb_get_device_list.restype = C.c_ssize_t
lib.libusb_get_device_descriptor.argtypes = [C.c_void_p, C.POINTER(DeviceDescriptor)]
lib.libusb_get_config_descriptor.argtypes = [C.c_void_p, C.c_uint8, C.POINTER(C.POINTER(ConfigDescriptor))]
lib.libusb_free_config_descriptor.argtypes = [C.POINTER(ConfigDescriptor)]
lib.libusb_free_device_list.argtypes = [C.POINTER(C.c_void_p), C.c_int]

_XFER = {0: "control", 1: "iso", 2: "bulk", 3: "interrupt"}
_SYNC = {0: "no-sync", 1: "async", 2: "adaptive", 3: "sync"}
_ICLASS = {0x01: "audio", 0x03: "HID", 0xFF: "vendor", 0xFE: "app-spec"}


def _parse_uac(raw: bytes):
    """Walk class-specific descriptors; decode a UAC Type-I FORMAT_TYPE if present."""
    i = 0
    while i + 1 < len(raw):
        blen = raw[i]
        if blen < 2 or i + blen > len(raw):
            break
        d = raw[i:i + blen]
        # CS_INTERFACE=0x24, subtype FORMAT_TYPE=0x02
        if len(d) >= 8 and d[1] == 0x24 and d[2] == 0x02 and d[3] == 0x01:
            nch, subframe, bits, nfreq = d[4], d[5], d[6], d[7]
            freqs = []
            for f in range(nfreq):
                off = 8 + f * 3
                if off + 3 <= len(d):
                    freqs.append(d[off] | (d[off + 1] << 8) | (d[off + 2] << 16))
            print(f"      -> FORMAT: {nch}ch  {bits}-bit  ({subframe}B/sample)  "
                  f"rates={freqs} Hz")
        i += blen


def _ep(e):
    addr = e.bEndpointAddress
    d = "IN" if addr & 0x80 else "OUT"
    xt = _XFER[e.bmAttributes & 0x3]
    extra = f" sync={_SYNC[(e.bmAttributes>>2)&0x3]}" if xt == "iso" else ""
    return (f"      EP 0x{addr:02X} {d:3s} {xt:9s} wMaxPkt={e.wMaxPacketSize:4d} "
            f"bInterval={e.bInterval}{extra}")


def dump():
    ctx = C.c_void_p()
    if lib.libusb_init(C.byref(ctx)) != 0:
        raise RuntimeError("libusb_init failed")
    lst = C.POINTER(C.c_void_p)()
    n = lib.libusb_get_device_list(ctx, C.byref(lst))
    for i in range(n):
        dev = lst[i]
        dd = DeviceDescriptor()
        if lib.libusb_get_device_descriptor(dev, C.byref(dd)) != 0:
            continue
        if dd.idVendor != VID_MS:
            continue
        print(f"\n=== VID_{dd.idVendor:04X} PID_{dd.idProduct:04X}  "
              f"bcdDevice={dd.bcdDevice:04X}  class={dd.bDeviceClass} "
              f"numCfg={dd.bNumConfigurations} ===")
        cfgp = C.POINTER(ConfigDescriptor)()
        if lib.libusb_get_config_descriptor(dev, 0, C.byref(cfgp)) != 0:
            print("  (could not read config descriptor)")
            continue
        cfg = cfgp.contents
        for ii in range(cfg.bNumInterfaces):
            iface = cfg.interface[ii]
            for a in range(iface.num_altsetting):
                idsc = iface.altsetting[a]
                cls = _ICLASS.get(idsc.bInterfaceClass, f"0x{idsc.bInterfaceClass:02X}")
                print(f"  IF {idsc.bInterfaceNumber} alt {idsc.bAlternateSetting}: "
                      f"class={cls} sub=0x{idsc.bInterfaceSubClass:02X} "
                      f"proto=0x{idsc.bInterfaceProtocol:02X} nEP={idsc.bNumEndpoints}"
                      + (f" extra={idsc.extra_length}B" if idsc.extra_length else ""))
                if idsc.extra_length:
                    raw = bytes(idsc.extra[k] for k in range(idsc.extra_length))
                    print("      cs-desc:", raw.hex())
                    _parse_uac(raw)
                for e in range(idsc.bNumEndpoints):
                    print(_ep(idsc.endpoint[e]))
        lib.libusb_free_config_descriptor(cfgp)
    lib.libusb_free_device_list(lst, 1)
    lib.libusb_exit(ctx)


if __name__ == "__main__":
    dump()
