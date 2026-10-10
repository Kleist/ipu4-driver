# Probe-test snippets of `init.ipu6-probe`

The guest side of each ipu6-ladder probe test (`tools/tests/ipu6/tests.d/`)
is its own snippet here, so two tracks adding probe checks add two files
instead of editing the same init. `tools/rootfs/build.sh` packs this
directory as `/init.d/` next to `/init`.

`init.ipu6-probe` (the frame) loads the modules, finds the IPU and dumps its
log, then sources:

- `NNN-<test>.bound.sh`: checks while intel-ipu6 is bound, in name order;
- `NNN-<test>.unloaded.sh`: checks after unbind and rmmod, in name order;

and finally scans for oopses and prints `PROBE: DONE`. Use the test's ladder
number and name (`060-probe-identify.bound.sh` belongs to
`tests.d/060-probe-identify.sh`).

A snippet prints its own markers with `say` (`PROBE:<test> ok|fail ...`,
or `PROBE:info ...`); the matching ladder test greps for them. Do not print
`PROBE:fail`: that marker fails every probe test. Variables: `$ipu` (the
IPU's PCI address, e.g. `0000:00:04.0`) and `$want` (its PCI device ID,
`0x5a88` for IPU4, or the QEMU model variant's from `ipu_pci_id=` on the
kernel command line). Snippets are POSIX sh (busybox).
