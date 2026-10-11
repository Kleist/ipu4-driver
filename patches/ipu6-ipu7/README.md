# IPU7 support in the ipu6 driver (base series)

Antti Laakso's IPU7-in-ipu6 series ([lore][antti]), not yet in mainline.
Applied on the mainline pin (`tools/ipu6/pin`) first, before
`patches/ipu6-ipu4p/` and `patches/ipu6-ipu4/`; Ruslan's IPU4P series
builds on its runtime-variation helpers.

Exported with `git format-patch` from Ruslan Bay's `ipu4p-upstream` branch
(`340985596f38`, on v7.3-rc4). Not edited here: refresh by re-exporting,
and drop patches once they land in mainline.

[antti]: https://lore.kernel.org/all/20260907113004.2489993-1-sakari.ailus@linux.intel.com/
