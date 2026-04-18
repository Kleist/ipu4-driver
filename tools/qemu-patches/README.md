# QEMU patches

Files here are mirrored into the forked QEMU tree by `tools/bootstrap.sh`.

- `hw/misc/ipu4.c` — device model source, copied verbatim.
- `hw/misc/Kconfig.ipu4` — stanza to append to `hw/misc/Kconfig`.
- `hw/misc/meson.ipu4.build` — stanza to append to `hw/misc/meson.build`.

The two append-files are not copied automatically; `bootstrap.sh` prints
a reminder to add them during the initial fork set-up. Once applied they
live permanently in the forked QEMU branch.
