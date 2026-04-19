# QEMU patches

Files here are mirrored into the forked QEMU tree by `tools/bootstrap.sh`.

- `hw/misc/ipu4.c` — device model source, copied verbatim.

The Kconfig / meson.build wiring for `CONFIG_IPU4` is appended to the
parent files by `bootstrap.sh` directly (idempotent); there is no
separate patch file for those lines.
