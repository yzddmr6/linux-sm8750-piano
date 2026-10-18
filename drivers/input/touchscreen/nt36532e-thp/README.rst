NT36532E host touch transport
============================

This experimental driver transports raw frames. It does not generate input
events. A separate native THP processor must supply its own panel calibration
and report coordinates through uinput. No firmware or calibration is included.

Sources and changes
-------------------

The paging, chip identification, firmware partition interpretation and download
sequence are adapted from Novatek's GPL-2.0-or-later driver:

* https://github.com/MiCode/vendor_xiaomi_proprietary_touch-driver
  at 6957f6b646d1c919e175e6f9000eb50c8635273c:
  ``p81/nt36532/nt36xxx.c``, ``nt36xxx_fw_update.c``,
  ``nt36xxx.h`` and ``nt36xxx_mem_map.h``.
* ianchb's GPL-2.0-or-later mainline adaptation:
  https://github.com/ianchb/sm8550-mainline
  at 42f3b40c702abbc37c3a04c95af008cd283aeaf3,
  ``drivers/input/touchscreen/nt36532e/``. Its NTP1 record format is retained.

Only the observed NT36532E cascade configuration is supported. The vendor's
zero-offset header marker selects the second header; lengths include the last
byte, and zero-length entries are reserved. The piano CSOT layout has its
marker exactly halfway through the header, so this input alone does not prove
the reference driver's half-header calculation incorrect. Golden ILM/DLM CRCs
come from the firmware header, as in the piano vendor flow.

The transport uses per-device state, regmap and an exclusive misc device instead
of global state and proc controls. The frame length is read from firmware, not
copied from another panel. SPI errors stop capture; there is no blind retry,
automatic error retry or reference-board RAM write.

After each download the transport retains the INIT check and then waits for
``EVENT_MAP_RESET_COMPLETE`` (``0x11c460``) to enter REK through MAX
(``0xa1..0xaf``), before enabling the IRQ. The named reset states follow
``nt36xxx.h:489-495``: INIT, REK, REK_FINISH, NORMAL_RUN and MAX. The wait
follows ``nvt_check_fw_reset_state()`` (``nt36xxx.c:1693-1727``): read immediately,
then poll at 10 ms intervals, with at most 50 sleeps and a 500 ms deadline.
Scheduling and bus latency can extend the final sample beyond that deadline.
Timeout returns ``-ETIMEDOUT`` with the last state; bus errors propagate without
retrying. No host command is sent and the IRQ remains disabled on failure.

In the vendor ``nvt_ts_resume()`` path (``nt36xxx.c:5671-5699``),
``nvt_update_firmware()`` is followed by a REK wait before IRQ enable and
``NVT_IC_RESUME_OUT``. The THP DATA_MODE_53/124 handlers gate BF01/BF19 on that
state (``nt36xxx.c:4114-4152``). An early valid frame proves frame delivery,
not firmware readiness for host commands; it cannot replace the REK contract.

After REK the transport enables the IRQ and still waits up to 500 ms for
the first valid frame. It then disables stylus scanning (``BF 04``, value 0)
and sends the two host commands the piano stock stack sends while frames
flow: idle-baseline update (``BF 19``, value 1) and op
mode 2 (``BF 01``, value 2), each confirmed by the vendor's command-byte clear
within 110 ms. Sent before any frame, the first command did not clear on piano.
A missing first frame or a command that does not clear stops the IRQ and fails
the open; the log names the command and the byte last read. Frames read while
the commands run stay queued. The stock stack repeats the pair on idle events;
this transport sends it once per firmware start (open or panel enable).

The stream-ready log reports reset-to-REK, reset-to-first-frame and BF-sequence
durations, plus the final reset-state byte. Both reset-relative durations start
immediately before the final bootloader reset used for the download (not the
earlier chip-identification reset). The existing first-frame log measures the
wait after IRQ enable. BF timing covers only the three-command sequence.
Diagnostic reads after that sequence or a first-frame failure preserve the last
sample on a bus error, report the read error separately, and do not change the
startup result. Firmware failures report the last state from the boot attempt;
zero before any state sample is available is not evidence of a completed reset.

Stream contract
---------------

``/dev/nvt-thp-<spi-device-name>`` is root-only and read-only. Opening it downloads
the explicitly named firmware once and enables capture if the panel is enabled;
otherwise open succeeds and capture waits for panel enable. A second open fails
with EBUSY. Probe itself does not issue SPI commands. Reads form a byte stream;
callers must handle partial records. Poll reports readiness, error or removal.

Each record has a 32-byte little-endian header followed by ``frame_length``
bytes. Header fields, in order:

* u32 magic: 0x3150544e (NTP1)
* u16 header_length: 32
* u16 frame_length: 257 plus the firmware-reported payload length
* u64 sequence: starts at 1 on probe and stays monotonic across opens and panel
  cycles; gaps indicate dropped records
* u64 timestamp_ns: CLOCK_BOOTTIME at completion of the SPI read
* u16 checksum: firmware payload checksum field
* u16 flags: bit 0 valid, bit 1 first valid record of the firmware epoch
* u32 firmware_length_words: checksum region size in 32-bit words

Frame byte 0 is a zero command placeholder, bytes 1..256 are the controller's
point prefix, and the remaining bytes are the THP payload. The valid flag checks
the 0xa5 protocol, length/checksum complements, bounds and the firmware-defined
16-bit checksum region. It does not validate finger geometry or unchecksummed
trailing fields. Invalid records remain observable with valid clear.

The bounded FIFO drops whole new records on overflow. It does not splice or
overwrite a partially read record. A userspace consumer must release contacts
on loss, invalid data, epoch changes, timeout and errors. Packet types other
than finger data must not be interpreted as finger matrices.

Power and test boundary
-----------------------

With the generic ``panel`` phandle, a DRM panel follower stops the IRQ before
panel disable removes the display timing needed by TDDI scanning. Unprepare
also stops capture as an idempotent fallback before the shared reset. Open
succeeds while the panel is disabled and the reader waits without a hangup.
After panel enable, an open stream uses the same firmware/first-frame/host-command
sequence as an open on an already enabled panel. Prepare alone is too early:
panel power does not guarantee display timing or touch frames. The first valid
queued frame is marked EPOCH; sequence numbers are not reset. Queued bytes are
preserved across a panel cycle so a partial read cannot splice two records.
The consumer must reject stale records and reset its baseline on EPOCH.
Restart failures are logged and reported to readers without retrying.

Without a panel phandle, platform-supplied power is still required. System
suspend of an open stream remains rejected; panel following is not system
suspend support. Closing stops capture without powering down the panel.
Panel restart and physical input after DPMS still require hardware validation.

The optional KUnit suite uses a user-supplied ``nt36532e/piano-test.bin`` fixture
through the firmware loader. It checks the piano layout, malformed inputs,
byte-exact simulated SRAM writes, golden CRC setup, identity rejection, the three
host commands, their timeout and the reported command byte, and immediate
propagation of each injected register-bus error. REK tests cover delayed and
immediate readiness, all named accepted states including MAX, timeout below
REK and above MAX, last-state preservation on read error, and boot refusing to
complete while stuck at INIT. The standalone REK tests need no firmware fixture. This does not test physical
SPI, IRQs, the first-frame wait in open, panel power, live framing, or real
gestures.
