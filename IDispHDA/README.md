# IDispHDA

Sound over DisplayPort and HDMI from Intel's display codec, through AppleHDA, for Raptor Lake-P/H (`8086:51CA`) with the
Tiger Lake framebuffer driver. A Lilu plugin, separate from AppleALC so that the analog codec's kext does not change.

Load it after Lilu, AppleALC and SimpleHDA. `-idisphdaoff` turns it off. `sysctl debug.idisphda` shows what happened.

## The faults it works around

- **The controller looks too early.** AppleHDAController reads STATESTS once at start and does not look again; the display
  codec raises its bit moments later. The controller has a path for a codec named by the framebuffer later on, but only under device
  ID `9D70`/`9D71`, and these controllers run under the stand-in `A348`. The plugin changes the ID in `controllerSpecificFeatures`, the first
  thing `setupAudioController` does before it resets the link and reads the codecs, and puts back the PCI settings of `A348`,
  so the analog codec sees what it always saw. It leaves `probe` alone: another plugin can hold it with a short jump, and
  Lilu panics on a second route over one.
- **AppleHDA's class for these codecs has the wrong node numbers.** It stops at Skylake: converters 2-4, pins 5-7, vendor
  widget 8. The numbers here are converters 3, 5, 7, 9, pins 4, 6, 8, 0xA and up, vendor widget 2, from Apple's
  `AppleGFXHDAFunctionGroup_8086280F` and Linux `sound/hda/codecs/hdmi/intelhdmi.c`.
- **The framebuffer never declares audio for a display run by AGDC.** It ignores the EDID for those, so `audio-codec-info`
  stays empty and `hwEnableAudio` does not run. Its port-to-pin table also follows Ice Lake: port 3 is pin 6 there and pin
  0xA from Tiger Lake on, as Linux has it.
- **The audio link clock is never programmed.** Linux writes `AUD_FREQ_CNTL = 0x8010` on every display from version 12.
- **The codec does not survive the display.** It loses power with it, the audio pin buffer is found closed on wake, and it answers
  again only once the display has audio on, late in its wake. Until then every command to it waits a second and fails, some of them on the framebuffer's wake thread just
  before the mode set, and AppleHDA's restore is lost: the engine refuses to start with `kIOReturnNotReady`. After the first
  timeout the plugin fails those commands at once, for five seconds or until audio is on. It then reads the vendor widget, and
  a codec found reset is replaced through the same late path it first came in by, as an unplugged display's would be. Nothing
  releases the old engine's claim on the display head, and the controller reports a display as new only while its head is free,
  so the plugin frees it and leaves one event in the queue the new driver receives when it starts.
- **The codec does not come back on a running link.** The framebuffer closes the audio pin buffer when the last display with audio
  goes away. After a display disconnected and reconnected that way, the codec stopped answering with the display and its audio on
  again. Linux keeps the buffer open while the audio side wants the codec, so the plugin does too once it has been opened, and with
  the buffer open the codec answers on through a disconnect. If the codec is silent all the same, it gets the wake signal, bit 15 of
  `0x65F10`, and then the controller's own clock stop, which resets the link and tells the drivers to restore their codecs; neither
  has had to run yet. The clock stop does not run by itself on these controllers, whose table of codecs for it names only a few
  display codecs, and the plugin asks for it only with no stream running and if every other codec declares the power states
  AppleHDA restores it for. Two failures in a row and it stops trying until the codec answers again.

## What it does not do

No codec is added until the framebuffer names it and its pins are open. STATESTS is only logged: the controller clears it when
it stops its clock, so a live codec can show nothing there. A silent codec costs one second and then its commands fail at once,
with four attempts in a row at adding it. Each group of routes first checks that the functions use the offsets the code relies on, that the symbols exist and that no
other plugin has routed them, since Lilu panics on the last two; if not, the group stays out. A group also stays inactive
when Lilu runs out of trampoline memory, which `routeMultiple` does not report: it adds 14 routes of about 20 bytes to a pool
of 4096. `test_tables.cpp` checks the numbers; `build.sh` runs it first. `idisppin=<node>` tries another pin without a rebuild.

Based on the plugin structure of NMHDA in [nullmoth/nvidia-macos-driver](https://github.com/nullmoth/nvidia-macos-driver);
the code is new. Upstream AppleALC reached the same conclusion for `A348` on laptops in branch
`patches_for_unsupported_hdmi_audio_a348_a170`.
