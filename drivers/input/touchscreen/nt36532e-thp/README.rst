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
  ``nt36xxx.h``, ``nt36xxx_mem_map.h``, ``nt36xxx_ext_proc.c``
  and ``nt36xxx_mp_ctrlram.c``.
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

The startup contract is::

  firmware download -> REK -> IRQ enable -> first valid frame
                    -> NORMAL_RUN -> BF 04=0 -> BF 19=1 -> BF 01=2

After each download the transport retains the INIT check and then waits for
``EVENT_MAP_RESET_COMPLETE`` (``0x11c460``) to enter REK through MAX
(``0xa1..0xaf``), before enabling the IRQ. The named reset states follow
``nt36xxx.h:489-495``: INIT, REK, REK_FINISH, NORMAL_RUN and MAX. The wait
follows ``nvt_check_fw_reset_state()`` (``nt36xxx.c:1693-1727``): read immediately,
then poll at 10 ms intervals, with at most 50 sleeps and a 500 ms deadline.
Scheduling and bus latency can extend the final sample beyond that deadline.
Timeout returns ``-ETIMEDOUT`` with the last state; bus errors propagate without
retrying. No host command is sent and the IRQ remains disabled if REK fails.

In the vendor ``nvt_ts_resume()`` path (``nt36xxx.c:5671-5699``),
``nvt_update_firmware()`` is followed by a REK wait before IRQ enable and
``NVT_IC_RESUME_OUT``. The THP DATA_MODE_53/124 handlers gate BF01/BF19 on that
state (``nt36xxx.c:4114-4152``). After REK the transport enables the IRQ and
waits up to 500 ms for the first valid frame. It then samples the reset state
and waits for NORMAL_RUN through MAX (``0xa3..0xaf``), using the same bounded
polling contract. The vendor MP paths wait for NORMAL_RUN before commands
requiring the normal firmware loop (``nt36xxx_mp_ctrlram.c:2085,2095``,
``2342,2352,2431,2441``). If this wait fails, the transport stops the IRQ and
reports the first-frame state, last state and elapsed wait without sending
host commands.

A first valid frame proves delivery, not readiness for host commands. On a
hot restart after killing and reopening the userspace service, firmware is
soft-reset and downloaded again. The first frame arrives about 5 ms after
REK, while the sampled state is still ``0xa1``. On cold boot and after panel
power-up it arrives about 120 ms after REK, already in ``0xa3``. An early or
residual frame can satisfy the frame admission before the normal firmware
loop is ready; the exact origin of that early frame has not been established.
In this interval BF04 can complete while BF19/BF01 fail to clear their command
byte within 110 ms. Waiting for NORMAL_RUN after the hot-restart frame took
117-120 ms and allowed the original write sequence to complete in five of
five SIGKILL tests; both DPMS tests also passed.

The three writes disable stylus scanning (``BF 04=0``), update the idle
baseline (``BF 19=1``), and select operation mode 2 (``BF 01=2``). Each write
must be confirmed by the vendor command-byte clear within 110 ms. A missing
frame, failed readiness wait or uncleared command stops the IRQ and fails
the startup. Frames read while commands run stay queued. This sequence runs
once per firmware start (open or panel enable), without a fixed delay or
command retry.

Only after a failed BF write, the transport reads operation mode (``BF 01``),
idle-baseline state (``BF 19``) and stylus state (``BF 04``) for diagnostics.
The protocol follows ``nvt_get_extend_custom_cmd()`` in
``nt36xxx_ext_proc.c:1065-1135``: write ``bf, sub, 01`` once, sleep 10-11 ms
before each of at most eleven samples, and accept the little-endian value
only when the command byte clears. The vendor six-byte SPI buffer includes
the address; regmap reads five response bytes, with the value at offsets 3/4.
Read errors are logged as negative errno values. These reads never run before
the startup writes and never replace the original failure or change its
stop path.

The stream-ready log reports reset-to-REK, reset-to-first-frame and BF-sequence
durations, the final reset-state byte, the state sampled after first-frame
admission, and the first-frame-to-NORMAL_RUN duration. Both reset-relative
durations start immediately before the final bootloader reset used for the
download, not the earlier chip-identification reset. The first-frame log
measures the wait after IRQ enable. The NORMAL_RUN duration includes the
initial state sample and polling; BF timing covers only the three writes.
The BF failure log also preserves the first-frame state and NORMAL_RUN wait.
Diagnostic reset-state reads preserve the last sample on a bus error and
report that error separately. Firmware failures report the last state from
the boot attempt; zero before any sample is not evidence of a completed reset.

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
After panel enable, an open stream uses the same download/REK/first-frame/
NORMAL_RUN/host-command sequence as an open on an already enabled panel. Prepare alone is too early:
panel power does not guarantee display timing or touch frames. The first valid
queued frame is marked EPOCH; sequence numbers are not reset. Queued bytes are
preserved across a panel cycle so a partial read cannot splice two records.
The consumer must reject stale records and reset its baseline on EPOCH.
Restart failures are logged and reported to readers without retrying.

Without a panel phandle, platform-supplied power is still required. System
suspend of an open stream remains rejected; panel following is not system
suspend support. Closing stops capture without powering down the panel.
The NORMAL_RUN path passed two DPMS stream-restart and hardware-scanout checks.
These checks do not replace physical touch or gesture validation.

The optional KUnit suite uses a user-supplied ``nt36532e/piano-test.bin`` fixture
through the firmware loader. It checks the piano layout, malformed inputs,
byte-exact simulated SRAM writes, golden CRC setup, identity rejection, the three
host commands, their timeout and the reported command byte, and immediate
propagation of each injected register-bus error. REK tests cover delayed and
immediate readiness, all named accepted states including MAX, timeout below
REK and above MAX, last-state preservation on read error, and boot refusing to
complete while stuck at INIT. NORMAL_RUN tests cover immediate and delayed
readiness, the upper bound, timeout below NORMAL_RUN and above MAX, and
preservation of the last state on a bus error. Readback tests cover byte order, timeout and bus-error reporting.
The standalone state-wait and command tests need no firmware fixture. KUnit
does not test physical SPI, IRQs, the first-frame wait in open, panel power,
live framing, or real gestures.

External-power notification
---------------------------

The charger command protocol follows the GPL-2.0-or-later Novatek driver
``p81/nt36532/nt36xxx.c:nvt_set_charge_state`` at the MiCode commit above.
The vendor ``xiaomi/xiaomi_touch_core.c:xiaomi_power_supply_work`` is a
behavior reference only: that file carries Xiaomi's proprietary notice and
none of its code is copied. The notifier/work and restore pattern follows
ianchb's GPL-2.0-or-later adaptation at
``42f3b40c702abbc37c3a04c95af008cd283aeaf3`` and the explicit event-page selection
in https://github.com/blu-sharky/linux-piano at
``453bb567f2fc12423ab3338953ca8938a2f06e16``,
``drivers/input/touchscreen/nt36532e/nt36xxx.c`` (GPL-2.0-or-later).

Only a readable external, system-scope power supply's ONLINE property determines
the state. Batteries, UNKNOWN-type internal charge pumps, device-scope supplies
and USB enumeration do not count. In particular, SC8541 ONLINE is CHG_EN,
not cable presence.
With no readable ONLINE property, the previous state is retained. Notifications
queue freezable work under the existing lifecycle mutex; regmap protects each page/write
transaction from concurrent frame reads. The wire sequence is ``ff 23 88`` then
``d0 53 00`` (online) or ``d0 51 00`` (offline), at register ``0x11c450``.

A closed, disabled or failed stream only caches the current state. System
suspend also drains and gates the supply-query work, because an ONLINE getter
may itself access a bus. The freezable queue excludes supply queries across
the system-wide device PM phase; resume samples the current state again.
Every successful firmware start queues a fresh ONLINE sample after REK, the
first valid frame, NORMAL_RUN and all three BF commands. It resends once even
when the state is unchanged; repeated notifications otherwise send nothing.
Stop drains both workers before panel power can disappear, and teardown removes
the notifier before its final work cancellation. A failed write stops capture
and reports the error to the existing reader/service backoff path, without a
new retry loop. Successful writes log ``THP charger state N sent``.

This aligns the command protocol with the vendor. It does not establish that
charger notifications eliminate a visible display flash; that needs the bounded
owner-attended plug/unplug comparison on this candidate.
