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
| 0x008  | Buttress | `ipu6-buttress.c` WDT kick              | Write: ignored (watchdog kick). Read: `0xfff0fff` (silicon's constant from data/trace.txt). | inferred   |
| 0x00c  | Buttress | `ipu6-buttress.c` BTRS_CTRL             | R/W latched; reset value `0x10` to match silicon's pre-write read. | inferred   |
| 0x030  | Buttress | `ipu6-buttress.c` FW_RESET_CTL          | Write START → read DONE on next read (no timer).            | inferred   |
| 0x034  | Buttress | `ipu6-buttress.c` IS_FREQ_CTL           | Write latched. Driver writes divisor + ICCMAX-level bit; no readback in trace. | inferred   |
| 0x038  | Buttress | `ipu6-buttress.c` PS_FREQ_CTL           | Write latched. Same shape as IS_FREQ_CTL, distinct register. | inferred   |
| 0x05c  | Buttress | `ipu6-buttress.c` PWR_STATE poll        | Step 5 FSM: read composes the word from per-island `is_powered` / `ps_powered` flags (mirrored from the FREQ_CTL writes' BIT(31) START bit) plus tied-high PWR_RDY[1:0]=3 and HH_DONE[13:12]=2. Powered up reads as `0x0fa02003`; powered down reads as `0x00002003`; either side independently. Lets the bus-level runtime-PM suspend complete cleanly, which is what made Step 2 need a driver-side `isys->power = 1` workaround. | inferred |
| 0x078  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_LO     | R/W latched.                                                | inferred   |
| 0x07c  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_HI     | R/W latched.                                                | inferred   |
| 0x080  | Buttress | `ipu6-buttress.c` FW_SOURCE_SIZE        | R/W latched.                                                | inferred   |
| 0x088  | Buttress | `ipu6-buttress.c` FABRIC_CMD            | Write latched. data/trace.txt shows a single 0x1 write during init; no readback. | inferred   |
| 0x090  | Buttress | `ipu6-buttress.c` ISR_STATUS            | Read: `status & enable`.                                    | inferred   |
| 0x094  | Buttress | `ipu6-buttress.c` ISR_ENABLED_STATUS    | Read: `status & enable`.                                    | inferred   |
| 0x098  | Buttress | `ipu6-buttress.c` ISR_ENABLE            | R/W latched.                                                | inferred   |
| 0x09c  | Buttress | `ipu6-buttress.c` ISR_CLEAR             | Write-1-to-clear bits of `status`.                          | inferred   |
| 0x100  | CSE IPC  | `ipu6-buttress.c` IU2CSEDB0             | Write latched only. Full secure-mode model would fire `BUTTRESS_ISR_IPC_EXEC_DONE_BY_CSE` here on the BUSY-bit kick, but `SECURITY_CTL`'s pre-set `AUTH_DONE` short-circuits `ipu6_buttress_authenticate()` before any BOOT_LOAD / AUTH_RUN command ships. | inferred |
| 0x104  | CSE IPC  | `ipu6-buttress.c` IU2CSEDATA0           | Driver-to-CSE command word. Latched only; no command is parsed because the auth path is short-circuited via SECURITY_CTL. | inferred |
| 0x108  | CSE IPC  | `ipu6-buttress.c` IU2CSECSR             | CSR_OUT — driver writes ENTRY/EXIT/QUERY/VALID-REQ tokens to step the IPC reset state machine (`ipu6-buttress.c:67-181`). Model mirrors the minimum CSE-side ack needed for the loop's success path: `ENTRY (BIT(0))` write → `EXIT (BIT(1))` set in CSR_IN; `ASSERTED_REG_VALID_REQ (BIT(3))` write → `ACKED_REG_VALID (BIT(4))` set in CSR_IN. | inferred |
| 0x164  | Buttress | `ipu6-buttress.c` TSC_LO                | Read returns low 32 bits of `qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)`. Re-sampled per read; the driver's HI/LO/HI rollover-retry stays cold because HI only flips every ~4.3s. | inferred   |
| 0x168  | Buttress | `ipu6-buttress.c` TSC_HI                | Read returns high 32 bits of the same clock. Silicon shows 0x4 throughout the trace (host uptime when captured); ours starts at 0 since QEMU_CLOCK_VIRTUAL counts guest time. | inferred   |
| 0x300  | Buttress | `ipu6-buttress.c` SECURITY_CTL          | R/W latched. Reset value is `0x37002` (silicon-matching: `AUTH_DONE` BIT(1), the silicon-specific bits `[14:12]` and `[17]`, plus `FW_SECURE_MODE` BIT(16)). With AUTH_DONE pre-asserted, `ipu6_buttress_authenticate()` short-circuits via `ipu6_buttress_auth_done()` and skips the BOOT_LOAD / AUTH_RUN dance. `FW_SECURE_MODE` still routes the driver through `ipu6_buttress_ipc_reset()` at probe time; the IU2CSECSR / CSE2IUCSR ping-pong below handles that. | inferred |
| 0x304  | CSE IPC  | `ipu6-buttress.c` CSE2IUDB0             | CSE→IU doorbell. Driver writes 0 to acknowledge a delivered response. Read returns the latched value. | inferred |
| 0x308  | CSE IPC  | `ipu6-buttress.c` CSE2IUDATA0           | CSE→IU response data slot. Latched; only set when the model synthesises a fake CSE response (currently never, since auth is short-circuited). | inferred |
| 0x30c  | CSE IPC  | `ipu6-buttress.c` CSE2IUCSR             | CSR_IN. Reads return the model-driven CSE response tokens (set in the CSR_OUT write handler — see 0x108). Driver writes are W1C: each bit the driver writes a 1 to is cleared. | inferred |
| 0x314  | Buttress | `ipu6-buttress.c` SKU                   | Read: 0 (unfused).                                          | guess      |
| 0x17c000 | ISYS   | `ipu6-isys.c` unispart IRQ_EDGE         | R/W latched.                                                | inferred   |
| 0x17c004 | ISYS   | `ipu6-isys.c` unispart IRQ_MASK         | R/W latched.                                                | inferred   |
| 0x17c008 | ISYS   | `ipu6-isys.c` unispart IRQ_STATUS       | Read: `status & enable`. `status` stays 0 without a backing ISYS frame generator; silicon cycles 0/0x40000000 as real IRQs fire, which compare.py will flag as a legitimate value_mismatch until the ISYS simulator lands. | inferred   |
| 0x17c00c | ISYS   | `ipu6-isys.c` unispart IRQ_CLEAR        | Write-1-to-clear bits of `status`.                          | inferred   |
| 0x17c010 | ISYS   | `ipu6-isys.c` unispart IRQ_ENABLE       | R/W latched.                                                | inferred   |
| 0x17c014 | ISYS   | `ipu6-isys.c` unispart IRQ_LEVEL_NOT_PULSE | R/W latched.                                             | inferred   |
| 0x17c414 | ISYS   | `ipu6-isys.c` unispart SW_IRQ           | Silicon writes 0 on every access; absorb the write as a no-op. Once a driver path writes non-zero the mux wiring needs modelling. | inferred   |
| 0x17c418 | ISYS   | `ipu6-isys.c` unispart SW_IRQ_MUX       | R/W latched.                                                | inferred   |
| 0x64000 | CSI2-0  | `ipu6-isys-csi2.c` RX_ENABLE            | R/W latched; bit 0 = enable.                                | inferred   |
| 0x64004 | CSI2-0  | `ipu6-isys-csi2.c` RX_NOF_ENABLED_LANES | R/W latched.                                                | inferred   |
| 0x64008 | CSI2-0  | `ipu6-isys-csi2.c` RX_CONFIG            | R/W latched. Silicon writes 0x3 (RELEASE_LP11 + DISABLE_BYTE_CLK_GATING) then 0, and reads back; the latch is authoritative. | inferred   |
| 0x6402c | CSI2-0  | `ipu6-isys-csi2.c` DLY_CNT_TERMEN_CLANE | R/W latched.                                                | inferred   |
| 0x64030 | CSI2-0  | `ipu6-isys-csi2.c` DLY_CNT_SETTLE_CLANE | R/W latched.                                                | inferred   |
| 0x64034 | CSI2-0  | `ipu6-isys-csi2.c` DLY_CNT_TERMEN_DLANE(0) | R/W latched.                                             | inferred   |
| 0x64038 | CSI2-0  | `ipu6-isys-csi2.c` DLY_CNT_SETTLE_DLANE(0) | R/W latched.                                             | inferred   |
| 0x64400-0x64414 | CSI2-0 | `ipu6-isys-csi2.c` CSI2PART IRQ trio | EDGE/MASK/ENABLE/LEVEL_NOT_PULSE latched; STATUS = `status & enable`; CLEAR is W1C. `status` stays 0 (no backing event generator). | inferred |
| 0x64500-0x64514 | CSI2-0 | `ipu6-isys-csi2.c` CSI RX IRQ trio   | Same shape as the PART trio.                                 | inferred   |
| 0x64600-0x64614 | CSI2-0 | `ipu6-isys-csi2.c` S2M IRQ trio      | Same shape as the PART trio.                                 | inferred   |
| 0x100000 | ISYS    | `ipu6-fw-isys.c` ISYS SPC_STATUS_CTRL    | R/W latched. Reads force `READY` (BIT(5)) on and `START` (BIT(1)) off so `query_sp()` succeeds on the first poll — the SPC has no real backing in the model. Writes still latch so `start_sp()`'s START/RUN/ICACHE writes stay visible if anything reads them back. | inferred |
| 0x108000-0x1080ff | ISYS | `ipu6-fw-com.c` DMEM syscom window   | Backed by a flat `uint32_t[0x40]` array. The syscom command parser (Steps 2-4) treats writes to `SYSCOM_CONFIG_REG` (DMEM[1], offset 0x004) as latching the IPU IOVA of the syscom config struct and invalidating the cached queue descriptors, writes to `SYSCOM_STATE_REG` (DMEM[2], offset 0x008) with the `UNINIT` magic as resetting the per-stream send-cursor cache, and writes to any msg-send `WR_REG` slot (DMEM[10..24] step 2, offsets 0x028..0x060 step 8) as token bumps. The model DMA-reads each new 16-byte send token out of the input ring and dispatches by `send_type`: STREAM_OPEN→STREAM_OPEN_DONE, STREAM_START→STREAM_START_ACK, STREAM_START_AND_CAPTURE→STREAM_START_AND_CAPTURE_ACK + frame delivery, STREAM_CAPTURE→frame delivery, STREAM_FLUSH→STREAM_FLUSH_ACK, STREAM_CLOSE→STREAM_CLOSE_ACK. Step 4 frame delivery: the model DMA-reads the token's `payload` IOVA as a `frame_buff_set` (kernel/ipu4/ipu6-fw-isys.h:688), writes the deterministic `byte[k] = (k+seq) & 0xff` pattern into each non-zero `output_pins[i].addr` (64 KB capped, walked page-by-page through the IPU MMU), then posts FRAME_SOF + PIN_DATA_READY responses so the driver's `ipu6_isys_queue_buf_ready` (kernel/ipu4/ipu6-isys-queue.c:935) matches the buffer by IOVA and completes the vb2 buffer. `SEND_RD_POS` (offset 0x02c) echoes `SEND_WR_POS` on read; `SYSCOM_STATE` always reads `SYSCOM_STATE_READY` (`0x57A7E001`); `RECV_WR_POS` (offset 0x070) is bumped by the model when posting responses. | inferred |
| 0x400000 | PSYS    | `ipu6-fw-isys.c` PSYS SPC_STATUS_CTRL    | Same shape as the ISYS SPC reg above. Touched by the PSYS firmware-init path mirror. | inferred |
| 0x1e0000-0x1e04ff | MMU | `ipu6-mmu.c` ISYS MMU page-table window | Flat `uint32_t[0x140]` array, plain R/W latches, plus Step 2 latches sub-block 0's `REG_L1_PHYS` (BAR+0x1e0004) into `is_mmu_l1_pfn` so the syscom command parser can walk IOVA → host-phys for `pci_dma_read` / `pci_dma_write`. Silicon does 208 writes here during FW bringup (page directory + L1/L2 entries + invalidate bits) and no reads. Streaming (Step 4 onwards) may need `pci_dma_rw()` off the PDE writes for frame buffers. | inferred |
| 0x4b0000-0x4b09ff | MMU | `ipu6-mmu.c` PSYS MMU page-table window | Same shape as the ISYS MMU — flat `uint32_t[0x280]` latch. 200 silicon writes, no reads. | inferred |

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

## M4.5 / M5a progress

`kernel/ipu4/virt-sensor.c` installs a software-node graph +
v4l2_subdev under `CONFIG_VIDEO_IPU4_VIRT_SENSOR`.

The M4.5 crash in `ipu6_dma_unmap_sg` was a false lead. Root cause
was `BTRS_PWR_STATE_IS_PWR_RDY` encoded with the wrong shift (19
instead of IPU4's 20). The driver's `readl_poll_timeout()` for ISYS
power-up timed out, probe unwound with partially-initialised state,
and the unwinder hit the `ipu6_dma_unmap_sg`/`sg_dma_address(NULL)`
path.

Fixing the PWR_STATE constant to return the right bits for IPU4 —
`bits 13:12=HH_DONE (2), 23:20=IS_PWR_FSM_IS_RDY (0xa), 28:24=PS_PWR_FSM_PS_PWR_UP (0xf), 1:0=PWR_RDY (3)`
→ aggregate `0x0fa02003` — unblocks all four polls
(`ipu6_buttress_power()` for ISYS and PSYS; `ipu6_buttress_start_tsc_sync()`;
`ipu6_buttress_powerup_*()`). Probe completes and 23 `/dev/video*`
nodes appear.

A single cosmetic line remains: `ipu6_buttress_power(on=false)`
polls for `PWR_STATE & mask == 0` (power-down done), which our
always-ready constant never satisfies, so it logs "Change power
status timeout". probe ignores the return value in that path, so it
is non-fatal — documented here rather than "fixed" with a proper
on/off state machine until we need to exercise runtime-PM.

## Next targets (unimplemented)

- **MMU (0x2e0000)** — real DMA-alloc backing. Probe completes
  today because the polls return ready, but any actual DMA
  read/write through the emulated IPU MMU just hits unhandled MMIO.
  Streaming (M5b) needs `pci_dma_rw()` on page-directory-base
  writes.
- **Firmware magic (BAR+0x8000)** — CPD verifier expects
  `0xb00710ad`. Not yet exercised because probe doesn't load the
  real firmware post-CPD-validate; may matter once real M5b tests
  run.
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
