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
| 0x030  | Buttress | `ipu6-buttress.c` FW_RESET_CTL          | Write START → read DONE on next read (no timer). START also drops the CSE's authentication: SECURITY_CTL's FW setup field (0x300) clears, so the next probe runs BOOT_LOAD / AUTH_RUN. | inferred   |
| 0x034  | Buttress | `ipu6-buttress.c` IS_FREQ_CTL           | Write latched. Driver writes divisor + ICCMAX-level bit; no readback in trace. | inferred   |
| 0x038  | Buttress | `ipu6-buttress.c` PS_FREQ_CTL           | Write latched. Same shape as IS_FREQ_CTL, distinct register. | inferred   |
| 0x05c  | Buttress | `ipu6-buttress.c` PWR_STATE poll        | Step 5 FSM: read composes the word from per-island `is_powered` / `ps_powered` flags (mirrored from the FREQ_CTL writes' BIT(31) START bit) plus tied-high PWR_RDY[1:0]=3 and HH_DONE[13:12]=2. Powered up reads as `0x0fa02003`; powered down reads as `0x00002003`; either side independently. Lets the bus-level runtime-PM suspend complete cleanly, which is what made Step 2 need a driver-side `isys->power = 1` workaround. ipu6 variant (`ipu6v_pwr_state()`): upstream IPU6 polls a 2-bit status per island instead, IS at [4:3] and PS at [7:6], UP_DONE (3) while powered, DN_DONE (0) otherwise, and the TSC-sync handshake status at [12:11] (IPU4: [13:12]) reads DONE (2) (powered up: `0x10db`); no silicon trace, taken from the driver's `IPU6_BUTTRESS_PWR_STATE_*`. ipu4p variant (`ipu4pv_pwr_state()`): IS FSM at [24:20] reads IS_RDY (0xc), PS FSM at [29:25] PS_PWR_UP (0x10), HH DONE at IPU6's [12:11] (powered up: `0x20c01003`); from Ruslan Bay's `IPU4P_BUTTRESS_PWR_STATE_*`, no trace. | inferred (ipu6, ipu4p: driver-derived) |
| 0x078  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_LO     | R/W latched.                                                | inferred   |
| 0x07c  | Buttress | `ipu6-buttress.c` FW_SOURCE_BASE_HI     | R/W latched.                                                | inferred   |
| 0x080  | Buttress | `ipu6-buttress.c` FW_SOURCE_SIZE        | R/W latched.                                                | inferred   |
| 0x088  | Buttress | `ipu6-buttress.c` FABRIC_CMD            | Write latched. data/trace.txt shows a single 0x1 write during init; no readback. | inferred   |
| 0x090  | Buttress | `ipu6-buttress.c` ISR_STATUS            | Read: `status & enable`.                                    | inferred   |
| 0x094  | Buttress | `ipu6-buttress.c` ISR_ENABLED_STATUS    | Read: `status & enable`.                                    | inferred   |
| 0x098  | Buttress | `ipu6-buttress.c` ISR_ENABLE            | R/W latched.                                                | inferred   |
| 0x09c  | Buttress | `ipu6-buttress.c` ISR_CLEAR             | Write-1-to-clear bits of `status`.                          | inferred   |
| 0x100  | CSE IPC  | `ipu6-buttress.c` IU2CSEDB0             | A write with BUSY (BIT(31)) runs the command latched in IU2CSEDATA0 (`ipu4_cse_command()`): raises `IPC_EXEC_DONE_BY_CSE` (ISR BIT(2)) and, for BOOT_LOAD / AUTH_RUN, posts the answer in CSE2IUDATA0 with `IPC_FROM_CSE_IS_WAITING` (ISR BIT(4)). Reached only after a FW reset (see 0x030); a fresh probe sees AUTH_DONE and sends nothing. | inferred |
| 0x104  | CSE IPC  | `ipu6-buttress.c` IU2CSEDATA0           | Driver-to-CSE command word, latched for the IU2CSEDB0 kick: BOOT_LOAD (1) sets SECURITY_CTL FW setup to FW_SETUP_DONE and makes the bootloader status word read `0xb00710ad` (BAR + 0x408000 on IPU4/IPU4P, 0x15c on IPU6); AUTH_RUN (2) sets it to AUTH_DONE. | inferred |
| 0x108  | CSE IPC  | `ipu6-buttress.c` IU2CSECSR             | CSR_OUT — driver writes ENTRY/QUERY/EXIT tokens to step the IPC reset handshake (`ipu6_buttress_ipc_reset()`). The model answers in CSR_IN as silicon does (`data/trace.txt` lines 31-45): first `ENTRY (BIT(0))` → ENTRY, second ENTRY → ENTRY\|EXIT, `QUERY (BIT(2))` → `EXIT (BIT(1))`; the driver's EXIT ends it. `ASSERTED_REG_VALID_REQ (BIT(3))` write → `ACKED_REG_VALID (BIT(4))` set in CSR_IN. | inferred |
| 0x164  | Buttress | `ipu6-buttress.c` TSC_LO                | Read returns low 32 bits of `qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL)`. Re-sampled per read; the driver's HI/LO/HI rollover-retry stays cold because HI only flips every ~4.3s. | inferred   |
| 0x168  | Buttress | `ipu6-buttress.c` TSC_HI                | Read returns high 32 bits of the same clock. Silicon shows 0x4 throughout the trace (host uptime when captured); ours starts at 0 since QEMU_CLOCK_VIRTUAL counts guest time. | inferred   |
| 0x300  | Buttress | `ipu6-buttress.c` SECURITY_CTL          | R/W latched. Reset value is `0x37002` (silicon-matching: `AUTH_DONE` BIT(1), the silicon-specific bits `[14:12]` and `[17]`, plus `FW_SECURE_MODE` BIT(16)). With AUTH_DONE pre-asserted, `ipu6_buttress_authenticate()` short-circuits via `ipu6_buttress_auth_done()`. A FW reset (0x030) clears the FW setup field [4:0]; BOOT_LOAD / AUTH_RUN (0x100) set it again. With the `fail-cse-auth` property AUTH_DONE is clear at reset and AUTH_RUN sets `AUTH_FAILED` (BIT(3)) instead. `FW_SECURE_MODE` routes the driver through `ipu6_buttress_ipc_reset()` at probe time; the IU2CSECSR / CSE2IUCSR ping-pong below handles that. | inferred |
| 0x304  | CSE IPC  | `ipu6-buttress.c` CSE2IUDB0             | CSE→IU doorbell. Driver writes 0 to acknowledge a delivered response. Read returns the latched value. | inferred |
| 0x308  | CSE IPC  | `ipu6-buttress.c` CSE2IUDATA0           | CSE→IU response data: BOOT_LOAD_DONE (BIT(0)) / AUTH_RUN_DONE (BIT(1)) after the matching command (0x100). | inferred |
| 0x30c  | CSE IPC  | `ipu6-buttress.c` CSE2IUCSR             | CSR_IN. Reads return the model-driven CSE response tokens (set in the CSR_OUT write handler — see 0x108). Driver writes are W1C: each bit the driver writes a 1 to is cleared. | inferred |
| 0x314  | Buttress | `ipu6-buttress.c` SKU                   | Read: 0 (unfused).                                          | guess      |
| 0x4000 | Buttress | `ipu6-fw-com.c` FW boot param SYSCOM_CONFIG_ID (ipu6 variant) | `ipu6v_mmio_write()`: latches the SysCom config IOVA (IPU4 has it in DMEM[1]) and drops the cached queue descriptors. | inferred (ipu6: driver-derived) |
| 0x4004 | Buttress | `ipu6-fw-com.c` FW boot param SYSCOM_STATE_ID (ipu6 variant) | Read: always SYSCOM_STATE_READY. Write UNINIT resets the send-cursor tracking (IPU4: DMEM[2]). | inferred (ipu6: driver-derived) |
| 0x17c000 | ISYS   | `ipu6-isys.c` unispart IRQ_EDGE         | R/W latched.                                                | inferred   |
| 0x17c004 | ISYS   | `ipu6-isys.c` unispart IRQ_MASK         | R/W latched.                                                | inferred   |
| 0x17c008 | ISYS   | `ipu6-isys.c` unispart IRQ_STATUS       | Read: `status & enable`. `status` stays 0 without a backing ISYS frame generator; silicon cycles 0/0x40000000 as real IRQs fire, which compare.py will flag as a legitimate value_mismatch until the ISYS simulator lands. A posted FW response sets the SW bit: BIT(30) on IPU4, BIT(22) on the ipu4p variant. | inferred   |
| 0x166308 | ISYS (ipu4p variant) | `ipu4p-isys-csi2-regs.h` CSI port 0 IRQ CTRL STATUS | Read: ACTIVE (bit 0) while a port 0 receiver error is pending (`ipu4pv_mmio_read()`); other CTRL writes absorbed. | inferred (ipu4p: driver-derived) |
| 0x166408-0x16640c | ISYS (ipu4p variant) | `ipu4p-isys-csi2-regs.h` CSI port 0 IRQ CTRL0 STATUS/CLEAR | Status: the pending receiver errors; each delivered frame sets bit 0 (corrected packet header error), CLEAR is write-1-to-clear. | inferred (ipu4p: driver-derived) |
| 0x166d08 | ISYS (ipu4p variant) | `ipu4p-isys-csi2-regs.h` SIP0 IRQ STATUS | Read: SIP0_CSI2_IRQ (bit 3) while a port 0 receiver error is pending, so `ipu4p_isys_isr()` dispatches port 0. | inferred (ipu4p: driver-derived) |
| 0x17c00c | ISYS   | `ipu6-isys.c` unispart IRQ_CLEAR        | Write-1-to-clear bits of `status`.                          | inferred   |
| 0x17c010 | ISYS   | `ipu6-isys.c` unispart IRQ_ENABLE       | R/W latched.                                                | inferred   |
| 0x17c014 | ISYS   | `ipu6-isys.c` unispart IRQ_LEVEL_NOT_PULSE | R/W latched.                                             | inferred   |
| 0x17c414 | ISYS   | `ipu6-isys.c` unispart SW_IRQ           | Silicon writes 0 on every access; absorb the write as a no-op. Once a driver path writes non-zero the mux wiring needs modelling. | inferred   |
| 0x17c418 | ISYS   | `ipu6-isys.c` unispart SW_IRQ_MUX       | R/W latched.                                                | inferred   |
| 0x164000-0x164fff | CSI2 port 0 | `ipu6-isys-csi2.c` per-port block | R/W latched per port. RX_ENABLE/CONFIG/DLY_CNT_* / per-trio EDGE/MASK/ENABLE/LEVEL_NOT_PULSE all latched; STATUS reads `status & enable`; CLEAR is W1C. Step 5b sets PART_IRQ_STATUS bit 0 (FS_VC0) on every PIN_DATA_READY delivery so the driver's CSI2 ISR runs and exercises the per-port read/W1C path. The ports are at ISYS base (0x100000) + port offset, as on silicon (`data/trace.txt` lines 651–668); until the core-csi2-irq fix the model put them at BAR + 0x64000, where nothing accesses them. | inferred |
| 0x165000-0x165fff | CSI2 port 1 | `ipu6-isys-csi2.c` per-port block | Same handler logic as port 0 — one entry in the model's `csi2[]` array. Silicon's `data/trace.txt` doesn't exercise this port, so it stays at zero in the coverage report; landing it now means a future driver path that does enable port 1 is handled without a `LOG_UNIMP` fallback. | guess |
| 0x166000-0x166fff | CSI2 port 2 | `ipu6-isys-csi2.c` per-port block | Same as port 1. | guess |
| 0x167000-0x167fff | CSI2 port 3 | `ipu6-isys-csi2.c` per-port block | Same as port 1. | guess |
| 0x16c000-0x16c7ff | CSI2 port 4 | `ipu6-isys-csi2.c` per-port block | Same as port 1. Note: ports 4 and 5 share a 0x1000 BAR window split into 0x800 halves (`kernel/ipu4/ipu4-platform-isys-csi2-reg.h:13-15`); the model resolves that via `ipu4_csi2_resolve()`. | guess |
| 0x16c800-0x16cfff | CSI2 port 5 | `ipu6-isys-csi2.c` per-port block | Same as port 4. | guess |
| 0x100000 | ISYS    | `ipu6-fw-isys.c` ISYS SPC_STATUS_CTRL    | R/W latched. Reads force `READY` (BIT(5)) on and `START` (BIT(1)) off so `query_sp()` succeeds on the first poll — the SPC has no real backing in the model. Writes still latch so `start_sp()`'s START/RUN/ICACHE writes stay visible if anything reads them back. | inferred |
| 0x108000-0x1080ff | ISYS | `ipu6-fw-com.c` DMEM syscom window   | Backed by a flat `uint32_t[0x40]` array. The syscom command parser (Steps 2-4) treats writes to `SYSCOM_CONFIG_REG` (DMEM[1], offset 0x004) as latching the IPU IOVA of the syscom config struct and invalidating the cached queue descriptors, writes to `SYSCOM_STATE_REG` (DMEM[2], offset 0x008) with the `UNINIT` magic as resetting the per-stream send-cursor cache, and writes to a msg-send queue's wr cursor as token bumps (`ipu4_syscom_write()`; every queue's cursor registers come from its descriptor, so no register number is fixed). The model DMA-reads each new 16-byte send token out of the input ring and dispatches by `send_type`: STREAM_OPEN→STREAM_OPEN_DONE, STREAM_START→STREAM_START_ACK, STREAM_START_AND_CAPTURE→STREAM_START_AND_CAPTURE_ACK + frame delivery, STREAM_CAPTURE→frame delivery, STREAM_FLUSH→STREAM_FLUSH_ACK, STREAM_CLOSE→STREAM_CLOSE_ACK. Step 4 frame delivery: the model DMA-reads the token's `payload` IOVA as a `frame_buff_set` (kernel/ipu4/ipu6-fw-isys.h:688), writes the deterministic `byte[k] = (k+seq) & 0xff` pattern into each non-zero `output_pins[i].addr` (64 KB capped, walked page-by-page through the IPU MMU), then posts FRAME_SOF + PIN_DATA_READY responses (PIN_DATA_READY's `buf_handle`/`buf_id` echoes the capture token's `buf_handle`, as firmware does) so the driver's `ipu6_isys_queue_buf_ready` (kernel/ipu4/ipu6-isys-queue.c:935) matches the buffer by IOVA and completes the vb2 buffer. Each msg-send queue's rd cursor reads as its wr cursor (for every stream, not only stream 0); `SYSCOM_STATE` always reads `SYSCOM_STATE_READY` (`0x57A7E001`); the msg-recv queue's wr cursor is bumped by the model when posting responses. ipu4p variant: the queue cursor pairs start at register 8 instead of 6, which the descriptors already say; proxy tokens (iwake) on the proxy send queue are answered on the proxy receive queue as for the ipu6 variant (`ipu4_proxy_send()`); from the IPU4P series, no trace. | inferred |
| 0x400000 | PSYS    | `ipu6-fw-isys.c` PSYS SPC_STATUS_CTRL    | Same shape as the ISYS SPC reg above. Touched by the PSYS firmware-init path mirror. | inferred |
| 0x1e0000-0x1e04ff | MMU | `ipu6-mmu.c` ISYS MMU page-table window | Flat `uint32_t[0x140]` array, plain R/W latches, plus Step 2 latches sub-block 0's `REG_L1_PHYS` (BAR+0x1e0004) into `is_mmu_l1_pfn` so the syscom command parser can walk IOVA → host-phys for `pci_dma_read` / `pci_dma_write`. Silicon does 208 writes here during FW bringup (page directory + L1/L2 entries + invalidate bits) and no reads. Streaming (Step 4 onwards) may need `pci_dma_rw()` off the PDE writes for frame buffers. | inferred |
| 0x4b0000-0x4b09ff | MMU | `ipu6-mmu.c` PSYS MMU page-table window | Same shape as the ISYS MMU — flat `uint32_t[0x280]` latch. 200 silicon writes, no reads. | inferred |
| 0x200000-0x2000ff | ISYS DMEM (ipu6 variant) | `ipu6-fw-com.c` SysCom registers | `ipu6v_mmio_*()` → the shared `ipu4_syscom_*()`: PKG_DIR, TUNIT_CFG, COMMAND, IRQ, then queue cursor pairs from register 4; each queue is found by its descriptor's wr_reg/rd_reg. A stream send queue's rd cursor reads as its wr cursor; a write to the proxy send queue's wr cursor (input queue 0) is consumed and answered on proxy output queue 0 (request_id, no error); a stream send queue bump goes to the IPU4 token handler. Responses are written in IPU6's 48-byte `ipu6_fw_isys_resp_info_abi` layout. | inferred (ipu6: driver-derived) |
| 0x210000 | ISYS SPC (ipu6 variant) | `ipu6-fw-isys.c` query_sp / start_sp | As IPU4's 0x100000: latched; reads READY set, START clear. | inferred (ipu6: driver-derived) |
| 0x220088 / 0x22008c, 0x238208 | CSI-2 port 0 / CSI top (ipu6 variant) | `ipu6-isys.c` isys_isr, `ipu6-isys-csi2.c` ipu6_isys_register_errors | Port 0 IRQ_CSI STATUS latches bit 0 (corrected packet header error) with each delivered frame, CLEAR is W1C; CTRL0 STATUS reads port 0's bits (0x3) while it is set. Covers upstream's CSI-2 error IRQ path. | inferred (ipu6: driver-derived) |
| 0x238008 / 0x23800c (+8 per PHY) | CSI-2 hub (ipu6 variant) | `ipu6-isys-mcd-phy.c` GPREG_PHY_CTL / PHY_STATUS | CTL latched per MCD PHY; STATUS reads POWER_ACK (bit 0) while CTL PWR_EN (bit 0) is set and READY (bit 4) while CTL RESET (bit 4) is set. | inferred (ipu6: driver-derived) |
| 0x27c008 / 0x27c00c | ISYS UNISPART (ipu6 variant) | `ipu6-isys.c` isys_isr | STATUS reads the model's firmware-IRQ flag as IPU6's BIT(22) (IPU4: BIT(30)); CLEAR BIT(22) clears it. | inferred (ipu6: driver-derived) |
| 0x2e0004 | ISYS MMU0 (ipu6 variant) | `ipu6-mmu.c` REG_L1_PHYS | Latches the L1 page-table pfn for the IOVA walker (IPU4: 0x1e0004). | inferred (ipu6: driver-derived) |

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
