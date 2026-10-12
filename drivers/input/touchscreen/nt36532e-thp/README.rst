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

After each download the transport enables the IRQ and waits up to 500 ms for
the first valid frame. It then disables stylus scanning (``BF 04``, value 0)
and sends the two host commands the piano stock stack sends while frames
flow: idle-baseline update (``BF 19``, value 1) and op
mode 2 (``BF 01``, value 2), each confirmed by the vendor's command-byte clear
within 110 ms. Sent before any frame, the first command did not clear on piano.
A missing first frame or a command that does not clear stops the IRQ and fails
the open; the log names the command and the byte last read. Frames read while
the commands run stay queued. The stock stack repeats the pair on idle events;
this transport sends it once per firmware start (open or panel enable).

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
byte-exact simulated SRAM writes, golden CRC setup, identity rejection, the two
host commands, their timeout and the reported command byte, and immediate
propagation of each injected register-bus error. This does not test physical
SPI, IRQs, the first-frame wait in open, panel power, live framing, or real
gestures.
