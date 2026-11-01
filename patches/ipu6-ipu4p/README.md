# IPU4P support for the upstream ipu6 driver (base series)

Ruslan Bay's IPU4P series, RFC v2 (v1 on [lore][ruslan]), not yet in
mainline.
Applied on the mainline pin (`tools/ipu6/pin`) before
`patches/ipu6-ipu4/`. It builds on Antti Laakso's IPU7 support, which is
in mainline since the 7.4 merge window.

Copied from the `upstream` branch of [ruslanbay/ipu4-drivers][repo]
(`f0b3dfa4873`, `patches/v2-00*.patch`, based on media-committers
`next` `27285b099fbc`), without the `v2-` prefix. Not edited here:
refresh by copying, and drop patches once they land in mainline.

[ruslan]: https://lore.kernel.org/all/aedlbUANNY6QvZ_mrVqU9aH15j76MKgYxxwkpmayED7tFEDztNQN8A-vDuLUIRpb9At6QpBp4gg9RAOA6gvD-LpCOUpFm8aVDJ5IK87tpPg=@proton.me/
[repo]: https://github.com/ruslanbay/ipu4-drivers/tree/upstream
