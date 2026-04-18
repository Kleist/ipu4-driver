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

## Next targets (unimplemented)

These are the register ranges the M3 fuzzing loop is expected to hit next
once the current handlers get the probe past IPC reset:

- **Firmware magic (BAR+0x8000)** — CPD verifier expects `0xb00710ad` at
  a specific offset after FW_SOURCE_BASE_*/SIZE are populated. See
  `ipu6-cpd.c`.
- **MMU (0x2e0000)** — page-directory-base write triggers page-table walk.
  Will be stubbed to `pci_dma_rw()` translation for host-side paging.
- **ISYS DMEM (0x200000)** — syscom ring head/tail and doorbell registers.
  Layout comes from `ipu6-fw-com.h` (`FW_COM_WR_REG`, `FW_COM_RD_REG`).
- **ISYS IRQ (TBD)** — frame-done interrupt raised by the frame generator.

## KUnit exposure

- `ipu6_mmu_pgsize()` was un-staticed in `ipu6-mmu.c` and declared in
  `ipu6-mmu.h` so `ipu4_mmu_kunit.c` can call it directly. No other
  driver-internal symbols are exposed for testing; if more are needed,
  add their declarations next to this one.
