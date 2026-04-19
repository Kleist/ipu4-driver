# IPU4 register log

This file is the source of truth for every register behavior implemented
in `tools/qemu-patches/hw/misc/ipu4.c`. Every new handler lands with a
row here — what the driver expects, what the guess is, and whether the
guess has been validated against real-silicon mmiotrace.

Columns:

- **Offset** — byte offset from BAR0.
- **Block** — buttress / ISYS / PSYS / MMU / CSE-IPC.
- **Driver caller** — file:line in the in-tree driver
  (`drivers/media/pci/intel/ipu4/`) that reads or writes this register.
- **Model behavior** — what QEMU does on read/write.
- **Confidence** — `doc` (documented in driver constants),
  `inferred` (deduced from driver code), `guess` (best effort, flagged
  for hardware validation).

| Offset | Block    | Driver caller                           | Model behavior                                              | Confidence |
|--------|----------|-----------------------------------------|-------------------------------------------------------------|------------|
| 0x008  | Buttress | `ipu6-buttress.c` WDT kick              | Write: ignored. Read: 0.                                    | inferred   |
| 0x00c  | Buttress | `ipu6-buttress.c` BTRS_CTRL             | R/W latched; value replayed on read.                        | inferred   |
| 0x030  | Buttress | `ipu6-buttress.c` FW_RESET_CTL          | Write START → read DONE on next read (no timer).            | inferred   |
| 0x05c  | Buttress | `ipu6-buttress.c` PWR_STATE poll        | Read always returns PWR_RDY | IS_PWR_RDY | PS_PWR_UP.      | guess      |
| 0x078  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_LO     | R/W latched.                                                | inferred   |
| 0x07c  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_HI     | R/W latched.                                                | inferred   |
| 0x080  | Buttress | `ipu6-buttress.c` FW_SOURCE_SIZE        | R/W latched.                                                | inferred   |
| 0x090  | Buttress | `ipu6-buttress.c` ISR_STATUS            | Read: `status & enable`.                                    | inferred   |
| 0x094  | Buttress | `ipu6-buttress.c` ISR_ENABLED_STATUS    | Read: `status & enable`.                                    | inferred   |
| 0x098  | Buttress | `ipu6-buttress.c` ISR_ENABLE            | R/W latched.                                                | inferred   |
| 0x09c  | Buttress | `ipu6-buttress.c` ISR_CLEAR             | Write-1-to-clear bits of `status`.                          | inferred   |
| 0x100  | CSE IPC  | `ipu6-buttress.c` IU2CSEDB0             | Write: ignored for now. Starts IPC reset handshake.         | guess      |
| 0x104  | CSE IPC  | `ipu6-buttress.c` IU2CSEDATA0           | Write: ignored.                                             | guess      |
| 0x108  | CSE IPC  | `ipu6-buttress.c` IU2CSECSR             | Write: echo to `cse2iu_csr` so receive-side poll passes.    | guess      |
| 0x300  | Buttress | `ipu6-buttress.c` SECURITY_CTL          | R/W latched.                                                | inferred   |
| 0x304  | CSE IPC  | `ipu6-buttress.c` CSE2IUDB0             | Read: 0.                                                    | guess      |
| 0x308  | CSE IPC  | `ipu6-buttress.c` CSE2IUDATA0           | Read: 0.                                                    | guess      |
| 0x30c  | CSE IPC  | `ipu6-buttress.c` CSE2IUCSR             | Echoed IU2CSECSR; driver write clears it.                   | guess      |
| 0x314  | Buttress | `ipu6-buttress.c` SKU                   | Read: 0 (unfused).                                          | guess      |

## M3 progress

The M3 PR added MSI (`msi_init()` in `ipu4_realize()`) so probe got
past `pci_alloc_irq_vectors(..., PCI_IRQ_MSI)`. The non-secure branch
of buttress init is entered because `SECURITY_CTL` reads 0, and
`Skip IPC reset for non-secure mode` elides the CSE IPC handshake.
Probe stopped at the firmware request with `-ENOENT`.

## M4 progress

`tools/firmware/gen-cpd.py` now synthesizes a minimal CPD blob that
satisfies `ipu6_cpd_validate_cpd_file()`:

- `hdr_mark = 0x44504324`, `hdr_len = 0x14`, `ent_cnt = 3`.
- Three entries (MANIFEST, METADATA, MODULEDATA). MANIFEST is empty.
- METADATA points to a single `ipu6_cpd_metadata_extn` with
  `extn_type = IPU6_CPD_METADATA_EXTN_TYPE_IUNIT (0x10)` and
  `img_type = IPU6_CPD_METADATA_IMAGE_TYPE_MAIN_FIRMWARE (2)`.
- MODULEDATA contains a `module_data_hdr` followed by an empty nested
  CPD (zero entries) — `ipu6_cpd_validate_moduledata()` recurses into
  this with no further checks to fail.

`rootfs/build.sh` generates the blob on demand and places it at
`/lib/firmware/ipu4_cpd_b0.bin` in the initramfs.

With this blob, probe progresses through:

1. `FW version: 0` — firmware loaded, top-level CPD validated.
2. CPD validation completes (metadata + moduledata recurse OK).
3. `Found supported sensor 0-000e` — ambu-ipu-bridge config scan.
4. `Invalid I2C adapter 3 for port 1` — ambu-ipu-bridge needs I2C
   adapters 0 and 3 (see `kernel/ipu4/ambu-ipu-bridge.c:28`); QEMU
   presents none, probe aborts with `IPU6 bridge init failed` and
   `-EINVAL`.

`probe-smoke` reaches `probe:bridge` and passes against the new
default required marker `probe:fw_valid`.

## M4.5 progress

`kernel/ipu4/virt-sensor.c` (landed under
`CONFIG_VIDEO_IPU4_VIRT_SENSOR`) installs a software-node graph on
`pdev->dev.fwnode->secondary` and a `v4l2_subdev` advertising
`MEDIA_BUS_FMT_RGB888_1X24` at 800×800, replacing the ambu bridge
call when the option is set. Probe progresses past the fwnode check
into `ipu6_psys_init()`, then **panics** in `ipu6_dma_unmap_sg()` at
`sg_dma_address(sglist)` on a null scatterlist. The crash is
expected until the MMU handler below lands; the kernel command line
carries `oops=panic` so the VM halts in ~6 s instead of hanging.

## Next targets (unimplemented)

- **MMU (0x2e0000)** — the M4.5 panic point.
  `ipu6_buttress_map_fw_image()` does DMA alloc that walks the IPU's
  MMU page tables. The QEMU model must translate IOVA→host via
  `pci_dma_rw()` on page-directory-base writes. This is the first
  real M5 deliverable.
- **Firmware magic (BAR+0x8000)** — CPD verifier expects `0xb00710ad`
  at a specific offset after `FW_SOURCE_BASE_*` / `SIZE` are populated.
  Not yet exercised because probe aborts earlier; ready to wire when
  needed.
- **ISYS DMEM (0x200000)** — syscom ring head/tail and doorbell
  registers. Layout comes from `ipu6-fw-com.h` (`FW_COM_WR_REG`,
  `FW_COM_RD_REG`).
- **ISYS IRQ (TBD)** — frame-done interrupt raised by the frame
  generator.

## KUnit exposure

- `ipu6_mmu_pgsize()` was un-staticed in `ipu6-mmu.c` and declared in
  `ipu6-mmu.h` so `ipu4_mmu_kunit.c` can call it directly. No other
  driver-internal symbols are exposed for testing; if more are needed,
  add their declarations next to this one.
