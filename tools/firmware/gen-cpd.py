#!/usr/bin/env python3
"""Generate a minimal Intel CPD firmware blob that passes the driver's
file-level validation in `ipu6_cpd_validate_cpd_file()`.

Enough to move probe past `probe:fw_load` to `probe:fw_valid`. The
blob does NOT contain real iunit / isys / psys package data, so probe
still fails later in `ipu6_cpd_create_pkg_dir()` — that work lives in
subsequent M4 commits.

Layout (all little-endian):
  +---------------------------------+
  | ipu6_cpd_hdr (20 bytes)         |
  +---------------------------------+
  | ent[0] MANIFEST (24 bytes)      |  offset=0, len=0
  | ent[1] METADATA (24 bytes)      |  offset=METADATA_OFF, len=sizeof(extn)
  | ent[2] MODULEDATA (24 bytes)    |  offset=MODULEDATA_OFF, len=sizeof(module_data_hdr)+20
  +---------------------------------+
  | metadata: ipu6_cpd_metadata_extn|
  +---------------------------------+
  | moduledata: module_data_hdr     |
  |            + nested empty CPD   |
  +---------------------------------+
"""
from __future__ import annotations

import struct
import sys

CPD_HDR_MARK = 0x44504324
CPD_HDR_LEN = 0x14  # IPU6 expects 20 bytes including 9 bytes trailing pad.
ENT_SIZE = 24       # sizeof(struct ipu6_cpd_ent)

EXTN_TYPE_IUNIT = 0x10
IMG_TYPE_MAIN_FIRMWARE = 2

METADATA_EXTN_SIZE = 4 + 4 + 4 + 16    # u32 extn_type; u32 len; u32 img_type; u8 rsvd[16]

MODDATA_HDR_SIZE = 4 + 4 + 4 + 4 + 4 + 4 + 11 + 7 + 2  # struct ipu6_cpd_module_data_hdr, __packed


def cpd_header(ent_cnt: int) -> bytes:
    pad = CPD_HDR_LEN - (4 + 4 + 1 + 1 + 1)
    return (
        struct.pack("<II", CPD_HDR_MARK, ent_cnt)
        + struct.pack("<BBB", 1, 1, CPD_HDR_LEN)
        + b"\x00" * pad
    )


def cpd_ent(name: bytes, offset: int, length: int) -> bytes:
    name = name.ljust(12, b"\x00")[:12]
    return name + struct.pack("<II", offset, length) + b"\x00" * 4


def metadata_extn() -> bytes:
    return (
        struct.pack("<III", EXTN_TYPE_IUNIT, METADATA_EXTN_SIZE, IMG_TYPE_MAIN_FIRMWARE)
        + b"\x00" * 16
    )


def moduledata_body() -> bytes:
    # ipu6_cpd_module_data_hdr followed by an empty nested CPD.
    mod_hdr = (
        struct.pack("<IIIII", MODDATA_HDR_SIZE, 0, 0, 0, 0)  # hdr_len, endian, dates...
        + struct.pack("<I", 0)                                # target_platform_type
        + b"\x00" * 11                                        # sys_ver
        + b"\x00" * 7                                         # fw_arch_ver
        + b"\x00" * 2                                         # rsvd
    )
    assert len(mod_hdr) == MODDATA_HDR_SIZE, (len(mod_hdr), MODDATA_HDR_SIZE)
    nested_cpd = cpd_header(ent_cnt=0)
    return mod_hdr + nested_cpd


def build() -> bytes:
    hdr = cpd_header(ent_cnt=3)
    entries_size = 3 * ENT_SIZE

    metadata = metadata_extn()
    moddata = moduledata_body()

    metadata_off = CPD_HDR_LEN + entries_size
    moduledata_off = metadata_off + len(metadata)

    ents = (
        cpd_ent(b"IUNIT_MFTKEY", 0, 0)  # MANIFEST: empty
        + cpd_ent(b"IUNIT_MDTA", metadata_off, len(metadata))
        + cpd_ent(b"IUNIT_MDL", moduledata_off, len(moddata))
    )

    return hdr + ents + metadata + moddata


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(f"usage: {argv[0]} <out.bin>", file=sys.stderr)
        return 2
    blob = build()
    with open(argv[1], "wb") as f:
        f.write(blob)
    print(f"wrote {argv[1]}: {len(blob)} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
