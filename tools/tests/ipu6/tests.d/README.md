# ipu6 ladder tests

One file per test of `tools/tests/ipu6/ladder.sh`. A test lives in its own
file, so two tracks adding tests at the same time add two files instead of
editing the same list.

- **Name:** `NNN-<test>.sh`. The ladder sources the files in name order,
  which is the ladder order. The numbers leave gaps of 10; to put a test
  between two others, pick a number in the gap (`085-isys-irq.sh`).
  Renumbering existing files is never needed.
- **Content:** `ladder_test <test> [<dependency>]` registers the test (when
  both run, it is SKIPped unless the dependency PASSed). `test_<test>()`
  runs it: stdout/stderr go to `tools/tests/out/ipu6-ladder/<test>.log`;
  return 0 for PASS. A registered test without the function is
  NOT-IMPLEMENTED (placeholders for later plan steps).
- **Shared helpers:** `../probe-boot.sh` (the probe-* tests' shared VM
  boot, `boot_probe_vm`, `probe_log_ok`, `$PROBE_LOG`, `$PROBE_WRITES`).
  The guest side of each probe test is its own snippet under
  `tools/rootfs/init.ipu6-probe.d/` (see the README there).
  `../stream-boot.sh` is the stream tests' shared VM boot
  (`boot_stream_vm`, `stream_log_ok`, `$STREAM_LOG`, `$STREAM_WRITES`):
  `tools/rootfs/init.ipu6-stream` captures one frame from CSI-2 port 0
  with `streamon` and the tests read its markers and driver output.
- **Legs:** the ladder runs one driver (`$DRIVER`: ipu4-out-of-tree |
  ipu4p | ipu6 | ipu4-patches) under one QEMU model variant (`$VARIANT`,
  exported as `$IPU4_VARIANT`: ipu4 | ipu4p | ipu6); table in
  `../ladder.sh`. A test that differs per driver flavor reads `$FLAVOR`
  (ipu6 | out-of-tree), one that differs per model `$IPU4_VARIANT`. Which
  tests gate a leg is its `enabled-tests.<driver>.<variant>` file, owned
  by the driver's track.
- **Variables available:** `$ROOT`, `$OUT`, `$LINUX_DIR`, `$PATCH_DIR`,
  `$DRIVER`, `$VARIANT`, `$LEG` (`<driver>/<variant>`), `$FLAVOR`,
  `${WANT[<test>]}` (selected in this run).
