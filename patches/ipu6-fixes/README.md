# Pending fixes for the mainline ipu6 driver

Fixes for bugs in the mainline ipu6 driver that the ladder hits, not yet
in mainline. `tools/tests/ipu6/ladder.sh` applies them under every
ipu6-flavor tree: on the mainline pin for the `ipu6` baseline, after
`patches/ipu6-ipu4p/` for the `ipu4p` baseline and `ipu4-patches`
(before `patches/ipu6-ipu4/`). They apply on mainline as is.

Upstream-quality `git format-patch` output, like `patches/ipu6-ipu4/`.
Send them to linux-media and drop each once mainline has it (TODO in
[`tools/notes/ipu6-ipu4-plan.md`](../../tools/notes/ipu6-ipu4-plan.md));
how to send them with b4 is in
[`patches/ipu6-ipu4/README.md`](../ipu6-ipu4/README.md#submitting-with-b4).
