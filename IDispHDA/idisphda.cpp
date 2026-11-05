// IDispHDA: sound over DisplayPort and HDMI from Intel's display codec, through AppleHDA.
//
// Four separate faults keep AppleHDA away from the display codec of Tiger Lake and later (8086:2812 and up).
//
// 1. The codec announces itself on the HD Audio link only after the display side has woken its audio link.
//    AppleHDAController reads STATESTS once, at start, and finds address 0 alone; the framebuffer driver starts in the
//    same second, the bit for address 2 rises afterwards and nothing looks at it. The controller has a path for exactly
//    this, in which it waits for a framebuffer that names the codec, but only when its device ID is 8086:9D70 or 9D71.
//    The stand-in ID these controllers receive, 8086:A348, turns that path off.
//
// 2. AppleHDA knows this family of codecs only up to Skylake: converters 2-4, pins 5-7, vendor widget 8. From Ice Lake
//    on the converters are 3, 5, 7 and 9, the pins are 4, 6, 8, 0xA and up, and the vendor widget is 2. Apple's own table
//    for those numbers lives in AppleGFXHDA, which cannot share a controller with an analog codec.
//
// 3. An external display run by AGDC never gets the framebuffer's audio flag from its EDID, so audio-codec-info stays
//    empty and hwEnableAudio never runs, which leaves the audio pin buffer off. The framebuffer's port-to-pin table also
//    follows Ice Lake, where the first Type-C port is pin 6; from Tiger Lake on it is pin 0xA.
//
// 4. The codec loses power with the display and answers again only once the display has audio on, late in its wake. Until
//    then every command to it waits a second and fails, some of them on the framebuffer's own wake thread, and AppleHDA's
//    restore at wake is lost: the converter routing and the vendor widget stay reset and the engine does not start again.
//    The path for a codec that appears late is the one that works, so a codec found reset is replaced through it.
//
// 5. A display that goes away and comes back can leave the codec silent on a running link, even with its audio on again. Linux
//    never closes the audio pin buffer and always resets the link with the codec's power on. Here the buffer stays open, which
//    keeps the codec answering through a disconnect, and a codec silent all the same gets the wake signal and then the
//    controller's own clock stop.
//
// The structure of a Lilu plugin around AppleHDAController follows NMHDA from nullmoth/nvidia-macos-driver; the code
// here is new.
//
// -idisphdaoff turns everything off. debug.idisphda keeps the newest lines of what happened.

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wsign-conversion"
#pragma clang diagnostic ignored "-Wshadow"
#include <Headers/plugin_start.hpp>
#include <Headers/kern_api.hpp>
#include <Headers/kern_util.hpp>
#pragma clang diagnostic pop
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Woverloaded-virtual"
#include <IOKit/pci/IOPCIDevice.h>
#pragma clang diagnostic pop
#include <libkern/c++/OSArray.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <kern/clock.h>
#include <mach/kmod.h>
#include <stdarg.h>
#include <sys/sysctl.h>

#include "idisphda_tables.h"

static char gLog[8192];
static size_t gLogUsed;
static IOSimpleLock *gLogLock;
SYSCTL_STRING(_debug, OID_AUTO, idisphda, CTLFLAG_RD | CTLFLAG_KERN, gLog, sizeof(gLog), "Intel display codec through AppleHDA");

static void report(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void report(const char *fmt, ...) {
	char line[192];
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	SYSLOG("idisp", "%s", line);
	size_t need = strlen(line) + 1;
	if (!gLogLock) return;
	IOSimpleLockLock(gLogLock);
	if (gLogUsed + need >= sizeof(gLog)) {
		// Keep the newer half.
		size_t keep = gLogUsed / 2;
		memmove(gLog, gLog + gLogUsed - keep, keep);
		gLogUsed = keep;
	}
	memcpy(gLog + gLogUsed, line, need - 1);
	gLogUsed += need - 1;
	gLog[gLogUsed++] = '\n';
	gLog[gLogUsed] = 0;
	IOSimpleLockUnlock(gLogLock);
}

template <typename T> static T &field(void *object, size_t offset) {
	return *reinterpret_cast<T *>(static_cast<uint8_t *>(object) + offset);
}

template <typename Function> static Function virtualSlot(void *object, size_t slot) {
	return reinterpret_cast<Function>((*static_cast<void ***>(object))[slot / sizeof(void *)]);
}

// The routes below rely on fixed field offsets and virtual slots. A different build of a binary must leave the system
// as it is, so each function is first checked for the displacements it is known to use.
static bool hasBytes(mach_vm_address_t function, size_t span, const uint8_t *needle, size_t length) {
	auto code = reinterpret_cast<const uint8_t *>(function);
	for (size_t i = 0; i + length <= span; i++)
		if (!memcmp(code + i, needle, length)) return true;
	return false;
}
static bool usesField(mach_vm_address_t function, size_t span, uint32_t displacement) {
	const uint8_t needle[4] {static_cast<uint8_t>(displacement), static_cast<uint8_t>(displacement >> 8),
		static_cast<uint8_t>(displacement >> 16), static_cast<uint8_t>(displacement >> 24)};
	return hasBytes(function, span, needle, sizeof(needle));
}

// Lilu panics when routeMultiple cannot solve a symbol, and when a function another plugin has routed with a short jump is
// routed again. An audio output is not worth a boot that stops, so both are ruled out first. This does not cover a plugin that
// routes the same function later on, so none may.
template <size_t N>
static bool routesAreFree(KernelPatcher &patcher, size_t index, KernelPatcher::RouteRequest (&routes)[N], mach_vm_address_t address,
	size_t size, const char *group) {
	for (auto &route : routes) {
		auto function = patcher.solveSymbol<mach_vm_address_t>(index, route.symbol, address, size);
		patcher.clearError();
		if (!function) {
			report("%s: %s is missing, nothing routed", group, route.symbol);
			return false;
		}
		auto code = reinterpret_cast<const uint8_t *>(function);
		if (!IDisp::plainPrologue(code)) {
			report("%s: %s starts with %02x %02x, nothing routed", group, route.symbol, code[0], code[1]);
			return false;
		}
	}
	return true;
}

// routeMultiple also answers yes when Lilu's memory for trampolines has run out: that function stays as it was, and only its
// empty original gives it away. A group with a hole in it must not act, so the wrappers look at the answer given here.
template <size_t N>
static bool routeGroup(KernelPatcher &patcher, size_t index, KernelPatcher::RouteRequest (&routes)[N], mach_vm_address_t address,
	size_t size, const char *group) {
	if (!routesAreFree(patcher, index, routes, address, size, group)) return false;
	bool installed = patcher.routeMultiple(index, routes, N, address, size);
	const int error = static_cast<int>(patcher.getError());
	patcher.clearError();
	for (auto &route : routes) installed = installed && route.org && *route.org;
	if (installed)
		report("%s routed, image %lu", group, static_cast<unsigned long>(index));
	else
		report("%s: routes incomplete, error %d, left inactive", group, error);
	return installed;
}

static uint32_t readU32(IORegistryEntry *entry, const char *key) {
	auto object = entry->getProperty(key);
	if (auto data = OSDynamicCast(OSData, object))
		if (data->getLength() >= 4) return *static_cast<const uint32_t *>(data->getBytesNoCopy());
	if (auto number = OSDynamicCast(OSNumber, object)) return number->unsigned32BitValue();
	return 0;
}

#pragma mark - AppleHDAController

namespace Controller {
constexpr size_t Registers = 0x88, Provider = 0x98, Codecs = 0x170, IntegratedMask = 0x2D2, VendorID = 0xEF0,
	DeviceID = 0xEF4, WaitingForDisplay = 0x1E90, IdleSupport = 0x1E91;
constexpr unsigned CodecCount = 15;
// The clock stop: allowed by the controller's table, and when the codecs present are exactly those that its own table names.
constexpr size_t ClockStopAllowed = 0x41D, CodecsPresent = 0x2D0, CodecsForClockStop = 0x432;
constexpr size_t SlotTerminate = 0x600, SlotCommandAction = 0xA90;
// Down: the gate keeps framebuffer events in a queue, which the next driver to start receives.
constexpr size_t EventsDeliverable = 0xED0;
// The table of display heads, and the engine that has claimed each one.
constexpr size_t Heads = 0xF08, HeadSize = 0x1A8, HeadCount = 8, HeadFramebuffer = 0, HeadSink = 0x18, HeadsLock = 0x1C58;
}
namespace HDA {
constexpr size_t GCTL = 0x08, STATESTS = 0x0E;
constexpr uint32_t UnsolicitedEnable = 0x100;
}

static mach_vm_address_t orgFeatures, orgFindCodec, orgCodecCommand, orgEventGate;
static void *gController;
static uint32_t gCodecRequests;
static uint64_t gCodecSilentUntil, gCodecSettledAt;
static unsigned gCodecApplications, gLinkResetFailures;
static bool gAudioPinsOpen, gCodecApplied, gCheckCodec, gCodecCommandFailed;
static void *gAudioFramebuffer, *gAudioFramebufferRef;
using SetClockStop = IOReturn (*)(void *, bool);
using DMAIsRunning = bool (*)(void *);
static SetClockStop gSetClockStop;
static DMAIsRunning gDMAIsRunning;
// In the framebuffer's part: its registers.
static bool displayAudioOn();
static void codecWakeSignal(bool on);
// Whether a group's image has been seen, and whether all of that group's routes are really in place.
static bool gControllerSeen, gHDASeen, gFramebufferSeen;
static bool gControllerReady, gHDAReady, gFramebufferReady;

static bool isOurController(IORegistryEntry *pci) {
	if (!pci || !pci->getProperty("hda-gfx")) return false;
	return (readU32(pci, "vendor-id") & 0xFFFF) == 0x8086 && IDisp::lateCodecController(readU32(pci, "device-id") & 0xFFFF);
}

static void engage(void *self) {
	if (gController) return;
	auto vendor = field<uint32_t>(self, Controller::VendorID), device = field<uint32_t>(self, Controller::DeviceID);
	if (vendor != 0x8086 || device != IDisp::StandInDevice) {
		report("controller %04x:%04x left as it is", vendor, device);
		return;
	}
	field<uint32_t>(self, Controller::DeviceID) = IDisp::LateCodecDevice;
	field<uint8_t>(self, Controller::WaitingForDisplay) = 1;
	gController = self;
	report("controller: display codec will be taken from the framebuffer");
}

static void engageIfOurs(void *self) {
	if (gControllerReady && !gController && isOurController(field<IOService *>(self, Controller::Provider))) engage(self);
}

// The ID changes here and not in probe: another plugin can hold probe with a short jump, which rules out a second route.
// This is early enough, since setupAudioController calls this function before it resets the link and reads the codecs, and
// probe's table gives the stand-in ID and the late-codec ID the same masks.
//
// The late-codec ID also selects a different clock-gating setup at start. The analog codec has only ever run with what
// the stand-in ID selects, so the same values go back.
static void wrapFeatures(void *self) {
	engageIfOurs(self);
	FunctionCast(wrapFeatures, orgFeatures)(self);
	if (self != gController) return;
	field<uint8_t>(self, Controller::IdleSupport) = 0;
	if (auto pci = OSDynamicCast(IOPCIDevice, field<IOService *>(self, Controller::Provider))) {
		pci->extendedConfigWrite32(0x44, 0x1B);
		pci->extendedConfigWrite32(0x48, 0);
	}
}

static void *&codecSlot(void *self) {
	return reinterpret_cast<void **>(static_cast<uint8_t *>(self) + Controller::Codecs)[IDisp::CodecAddress];
}

static uint64_t deadlineAfter(unsigned seconds) {
	uint64_t deadline;
	clock_interval_to_deadline(seconds, 1000000000U, &deadline);
	return deadline;
}

static bool codecSilent() {
	if (!gCodecSilentUntil) return false;
	uint64_t now;
	clock_get_uptime(&now);
	if (now < gCodecSilentUntil) return true;
	gCodecSilentUntil = 0;
	return false;
}

// The original walks the whole service plane and takes the first audio-codec-info it meets, which with a second GPU can
// belong to another controller. Only this controller's own GPU counts here. A new codec must also have its pins open.
// STATESTS cannot decide: the controller clears it whenever it stops its clock, so a live codec can show nothing there.
// A command to a silent address costs a second, during which the analog codec waits, so attempts are limited.
static uint32_t displayCodecInfo(void *self) {
	auto pci = field<IOService *>(self, Controller::Provider);
	auto parent = pci ? pci->getParentEntry(gIOServicePlane) : nullptr;
	auto functions = parent ? parent->getChildIterator(gIOServicePlane) : nullptr;
	uint32_t info = 0;
	if (!functions) return 0;
	while (auto object = functions->getNextObject()) {
		auto gpu = OSDynamicCast(IOPCIDevice, object);
		if (!gpu || (readU32(gpu, "vendor-id") & 0xFFFF) != 0x8086 || (readU32(gpu, "class-code") & 0xFF0000) != 0x030000) continue;
		if (auto outputs = gpu->getChildIterator(gIOServicePlane)) {
			while (auto output = outputs->getNextObject()) {
				auto entry = OSDynamicCast(IORegistryEntry, output);
				if (!entry) continue;
				auto value = readU32(entry, "audio-codec-info");
				if (IDisp::codecInfoValid(value)) { info = value; break; }
			}
			outputs->release();
		}
		break;
	}
	functions->release();
	return info;
}

static uint8_t wrapFindCodec(void *self) {
	if (self != gController) return FunctionCast(wrapFindCodec, orgFindCodec)(self);
	const uint32_t info = displayCodecInfo(self);
	if (!info) return 0xFF;
	const unsigned address = info & 0xFF;
	const unsigned bit = 1U << address;
	if (!(field<uint16_t>(self, Controller::IntegratedMask) & bit)) return 0xFF;
	if (reinterpret_cast<void **>(static_cast<uint8_t *>(self) + Controller::Codecs)[address]) {
		gCodecRequests = 0;
		return static_cast<uint8_t>(bit);
	}
	if (!gHDAReady || !gAudioPinsOpen || codecSilent() || gCodecRequests >= IDisp::MaxCodecRequests) return 0xFF;
	gCodecRequests++;
	auto registers = field<volatile uint8_t *>(self, Controller::Registers);
	auto statests = reinterpret_cast<volatile uint16_t *>(registers + HDA::STATESTS);
	unsigned waited = 0;
	while (!(*statests & bit) && waited < 100) {
		IOSleep(5);
		waited += 5;
	}
	report("framebuffer names codec %u, info %08x, STATESTS %04x after %u ms", address, info, *statests, waited);
	return static_cast<uint8_t>(bit);
}

// While the display is dark every command to its codec waits out a one-second timeout, and the framebuffer's wake thread
// sends some of them just before the mode set that would wake the codec. After the first timeout they fail at once, with the
// same result, for a few seconds or until the display turns audio on. The limit in time is for a timeout with nothing wrong.
static IOReturn wrapCodecCommand(void *self, void *codec, uint32_t command, uint32_t *response) {
	const bool display = self == gController && IDisp::commandForCodec(command);
	if (display && codecSilent()) {
		gCodecCommandFailed = true;
		return kIOReturnTimeout;
	}
	const IOReturn result = FunctionCast(wrapCodecCommand, orgCodecCommand)(self, codec, command, response);
	if (display && result == kIOReturnTimeout) {
		gCodecCommandFailed = true;
		gCodecSilentUntil = deadlineAfter(IDisp::SilentCodecSeconds);
		report("display codec silent: its commands fail at once for %u s or until audio is on", IDisp::SilentCodecSeconds);
	}
	return result;
}

// The old engine stays in the table of display heads as the one holding this display: nothing releases it when its codec goes
// away. While it is there the controller never reports the display as new, and it would refuse a new engine's claim.
static void releaseDisplayHead(void *self) {
	auto lock = field<IORecursiveLock *>(self, Controller::HeadsLock);
	IORecursiveLockLock(lock);
	for (size_t i = 0; i < Controller::HeadCount; i++) {
		auto head = static_cast<uint8_t *>(self) + Controller::Heads + i * Controller::HeadSize;
		if (field<void *>(head, Controller::HeadFramebuffer) == gAudioFramebuffer) field<void *>(head, Controller::HeadSink) = nullptr;
	}
	IORecursiveLockUnlock(lock);
}

static IOReturn askCodec(void *self, uint32_t *value) {
	gCodecSilentUntil = 0;
	*value = 0;
	return wrapCodecCommand(self, codecSlot(self), IDisp::codecCommand(IDisp::VendorNode, IDisp::VerbGetVendor, 0), value);
}

// A link reset takes every codec with it, so it waits for an idle link, and it is only for codecs AppleHDA restores afterwards.
static bool linkResetAllowed(void *self) {
	if (!gSetClockStop || !gDMAIsRunning || field<uint8_t>(self, Controller::ClockStopAllowed) != 1) {
		report("display codec: this controller has no clock stop");
		return false;
	}
	if (gDMAIsRunning(self)) {
		report("display codec: sound is playing, link left alone");
		return false;
	}
	auto codecs = reinterpret_cast<void **>(static_cast<uint8_t *>(self) + Controller::Codecs);
	for (unsigned address = 0; address < Controller::CodecCount; address++) {
		if (address == IDisp::CodecAddress || !codecs[address]) continue;
		uint32_t states = 0;
		const auto command = IDisp::commandTo(address, IDisp::FunctionGroupNode, IDisp::VerbGetParameter, IDisp::ParameterPowerStates);
		const IOReturn result = FunctionCast(wrapCodecCommand, orgCodecCommand)(self, codecs[address], command, &states);
		if (result != kIOReturnSuccess || !IDisp::restoredAfterClockStop(states)) {
			report("display codec: codec %u has power states %08x, result %x: link left alone", address, states, result);
			return false;
		}
	}
	return true;
}

// The wake signal first, which touches nothing else. Then the same reset the controller does when it stops the clock on an idle
// link: it saves and restores its own registers and tells the drivers, which restore their codecs. It does not do that here by
// itself, since its table of codecs for it names only a few display codecs, so for this one call it is told they all are.
// Linux keeps the wake signal on during a link reset.
static IOReturn reviveCodec(void *self, uint32_t *value) {
	auto statests = reinterpret_cast<volatile uint16_t *>(field<volatile uint8_t *>(self, Controller::Registers) + HDA::STATESTS);
	const uint16_t before = *statests;
	codecWakeSignal(true);
	codecWakeSignal(false);
	IOReturn result = askCodec(self, value);
	report("display codec: wake signal, STATESTS %04x then %04x, result %x", before, *statests, result);
	if (result == kIOReturnSuccess || !linkResetAllowed(self)) return result;
	auto &forClockStop = field<uint16_t>(self, Controller::CodecsForClockStop);
	const uint16_t saved = forClockStop;
	forClockStop = field<uint16_t>(self, Controller::CodecsPresent);
	codecWakeSignal(true);
	const IOReturn stopped = gSetClockStop(self, true);
	IOSleep(IDisp::LinkResetHoldMs);
	const IOReturn started = gSetClockStop(self, false);
	codecWakeSignal(false);
	forClockStop = saved;
	IOSleep(IDisp::LinkResetSettleMs);
	result = askCodec(self, value);
	report("display codec: link reset, stop %x, start %x, STATESTS %04x, result %x", stopped, started, *statests, result);
	if (result != kIOReturnSuccess) gLinkResetFailures++;
	return result;
}

// Once the display has audio on again, the vendor widget tells whether the codec kept its state. If it was reset, AppleHDA
// believes in a routing that is gone, and a command that failed may have cost it its restore too. So the codec goes away as
// an unplugged display's would, and the gate brings a new one in through the late path. The old driver stack is given a moment
// to stop first: it talks to the same physical codec the new one is about to set up. A codec that does not answer would not get
// through a new driver's setup either, so it is brought back first or the old stack stays.
//
// At boot the mode changes come seconds after the codec, and taking a driver stack apart while it starts is the one way this
// could stop a boot, so nothing happens in that window. Later on a display can go away a second after the stack is built. While
// a stack is starting or inside its window a later event looks again, which is why this first asks whether audio is still on.
static bool replaceCodecIfReset(void *self) {
	if (!gCheckCodec) return false;
	auto &slot = codecSlot(self);
	if (!slot || !gCodecApplications) {
		gCheckCodec = false;
		return false;
	}
	if (!gCodecApplied && !gCodecCommandFailed) return false;
	uint64_t now;
	clock_get_uptime(&now);
	if (now < gCodecSettledAt && (gCodecApplications == 1 || !gCodecCommandFailed)) return false;
	gCheckCodec = false;
	if (!displayAudioOn()) return false;
	uint32_t value = 0;
	IOReturn result = askCodec(self, &value);
	if (result != kIOReturnSuccess && gLinkResetFailures < IDisp::MaxLinkResetsInARow) result = reviveCodec(self, &value);
	if (result != kIOReturnSuccess) {
		report("display codec: no answer with audio on, result %x", result);
		return false;
	}
	gLinkResetFailures = 0;
	if (value == IDisp::VendorEnableAll && !gCodecCommandFailed) return false;
	report("display codec: vendor widget %08x, failed commands %u: replacing it", value, gCodecCommandFailed);
	gCodecApplied = false;
	gCodecCommandFailed = false;
	void *old = slot;
	slot = nullptr;
	virtualSlot<bool (*)(void *, IOOptionBits)>(old, Controller::SlotTerminate)(old, 0);
	static_cast<OSObject *>(old)->release();
	IOSleep(IDisp::CodecReplaceWaitMs);
	releaseDisplayHead(self);
	field<uint8_t>(self, Controller::EventsDeliverable) = 0;
	return true;
}

// While it waits for a display the gate turns unsolicited responses off at every mode change and turns them back on only
// when it finds a codec. On this controller they also carry the analog codec's jack events.
static IOReturn wrapEventGate(void *self, void *ref, void *framebuffer, int event, void *info) {
	engageIfOurs(self);
	bool replaced = false;
	if (self == gController) {
		if (framebuffer == gAudioFramebuffer) gAudioFramebufferRef = ref;
		replaced = replaceCodecIfReset(self);
		// A reset removes the codec if the framebuffer has not named it again yet, and nothing asks for it afterwards.
		if (!codecSlot(self)) field<uint8_t>(self, Controller::WaitingForDisplay) = 1;
	}
	auto result = FunctionCast(wrapEventGate, orgEventGate)(self, ref, framebuffer, event, info);
	// The new driver starts a moment from now and hears about its display from the next event for that display, which may be a
	// long way off. So one waits for it in the gate's queue, as events do when the controller removes a codec itself.
	if (replaced) {
		if (!gAudioFramebufferRef)
			report("display codec replaced: no event seen yet from its framebuffer");
		else if (framebuffer != gAudioFramebuffer || event != IDisp::EventModeDidChange)
			FunctionCast(wrapEventGate, orgEventGate)(self, gAudioFramebufferRef, gAudioFramebuffer, IDisp::EventModeDidChange, nullptr);
		report("display codec replaced: head released, event %d, framebuffer %s", event, framebuffer == gAudioFramebuffer ? "its own" : "another");
	}
	if (self == gController && field<uint8_t>(self, Controller::WaitingForDisplay)) {
		auto registers = field<volatile uint8_t *>(self, Controller::Registers);
		auto gctl = reinterpret_cast<volatile uint32_t *>(registers + HDA::GCTL);
		if (!(*gctl & HDA::UnsolicitedEnable)) *gctl |= HDA::UnsolicitedEnable;
	}
	return result;
}

static bool routeController(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
	auto features = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController26controllerSpecificFeaturesEv", address, size);
	patcher.clearError();
	if (!features) return false;
	auto find = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController21findValidDisplayCodecEv", address, size);
	auto gate = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController20framebufferEventGateEPvP13IOFramebufferiS0_", address, size);
	auto probeLink = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController14probeAudioLinkEv", address, size);
	auto command = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController19executeCodecCommandEP16IOHDACodecDevicejPj", address, size);
	auto release = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController17releaseSinkDeviceEjP9IOService", address, size);
	patcher.clearError();
	if (!find || !gate || !probeLink || !command || !release ||
		!usesField(release, 0x60, Controller::Heads + Controller::HeadSink) || !usesField(release, 0x60, Controller::HeadSize) ||
		!usesField(release, 0x20, Controller::HeadsLock) ||
		!usesField(command, 0x40, Controller::SlotCommandAction) || !usesField(probeLink, 0x140, Controller::SlotTerminate) ||
		!usesField(features, 0x90, Controller::DeviceID) || !usesField(features, 0x90, Controller::WaitingForDisplay) ||
		!usesField(features, 0x90, Controller::IdleSupport) ||
		!usesField(find, 0x180, Controller::IntegratedMask) ||
		!usesField(gate, 0x100, Controller::WaitingForDisplay) || !usesField(gate, 0x100, Controller::DeviceID) ||
		!usesField(gate, 0x100, Controller::EventsDeliverable) ||
		!usesField(probeLink, 0x140, Controller::Codecs)) {
		report("AppleHDAController does not look as expected: nothing routed");
		return true;
	}
	// Called, not routed. Without them a silent codec only gets the wake signal.
	auto clockAllowed = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController34hardwareSupportsClockStopOperationEv", address, size);
	auto clockStop = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController27setControllerClockStopStateEb", address, size);
	auto dmaRunning = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN18AppleHDAController12dmaIsRunningEv", address, size);
	patcher.clearError();
	if (clockAllowed && clockStop && dmaRunning && usesField(clockAllowed, 0x22, Controller::ClockStopAllowed) &&
		usesField(clockAllowed, 0x22, Controller::CodecsPresent) && usesField(clockAllowed, 0x22, Controller::CodecsForClockStop) &&
		usesField(clockStop, 0x120, Controller::CodecsForClockStop)) {
		gSetClockStop = reinterpret_cast<SetClockStop>(clockStop);
		gDMAIsRunning = reinterpret_cast<DMAIsRunning>(dmaRunning);
	} else
		report("AppleHDAController: clock stop does not look as expected, no link reset");
	KernelPatcher::RouteRequest routes[] {
		{"__ZN18AppleHDAController26controllerSpecificFeaturesEv", wrapFeatures, orgFeatures},
		{"__ZN18AppleHDAController21findValidDisplayCodecEv", wrapFindCodec, orgFindCodec},
		{"__ZN18AppleHDAController19executeCodecCommandEP16IOHDACodecDevicejPj", wrapCodecCommand, orgCodecCommand},
		{"__ZN18AppleHDAController20framebufferEventGateEPvP13IOFramebufferiS0_", wrapEventGate, orgEventGate},
	};
	gControllerReady = routeGroup(patcher, index, routes, address, size, "AppleHDAController");
	return true;
}

#pragma mark - AppleHDA

namespace FunctionGroup {
constexpr size_t Codec = 0x10;
constexpr size_t SlotExecuteVerb = 0x868, SlotCanDiscoverPaths = 0x230, SlotLogPathMap = 0x240;
}
namespace Widget {
constexpr size_t NodeID = 0xC, Connections = 0xF0, Flags37 = 0x37, FlagsE8 = 0xE8;
constexpr size_t SlotRunVerb = 0x120, SlotSetPinConfig = 0x318, SlotSelectConnection = 0x478;
}

static mach_vm_address_t orgGroupFactory, orgWidgetFactory, orgApply, orgPathMap, orgWidgetInit, orgPinControl;
static uint32_t gCodecID;

// Both factories pick a class from the codec ID, and the classes for these codecs exist only under a Skylake-era ID.
// That class is close: only the node numbers differ, and the routes below supply them.
static void *wrapGroupFactory(uint32_t *device) {
	if (!gHDAReady || !device || !IDisp::modernDisplayCodec(*device)) return FunctionCast(wrapGroupFactory, orgGroupFactory)(device);
	const uint32_t real = *device;
	gCodecID = real;
	gCodecApplied = false;
	gCodecCommandFailed = false;
	*device = IDisp::ClassCodec;
	auto result = FunctionCast(wrapGroupFactory, orgGroupFactory)(device);
	*device = real;
	report("codec %08x: function group %s", real, result ? "created" : "NOT created");
	return result;
}

static void *wrapWidgetFactory(uint32_t *device) {
	if (!gHDAReady || !device || !IDisp::modernDisplayCodec(*device)) return FunctionCast(wrapWidgetFactory, orgWidgetFactory)(device);
	const uint32_t real = *device;
	*device = IDisp::ClassCodec;
	auto result = FunctionCast(wrapWidgetFactory, orgWidgetFactory)(device);
	*device = real;
	return result;
}

using ExecuteVerb = IOReturn (*)(void *, unsigned, unsigned, unsigned, unsigned *, unsigned);

static unsigned getVerb(void *codec, unsigned node, unsigned verb, unsigned payload) {
	unsigned value = 0;
	auto execute = virtualSlot<ExecuteVerb>(codec, FunctionGroup::SlotExecuteVerb);
	return execute(codec, node, verb, payload, &value, 1) == kIOReturnSuccess ? value : 0xFFFFFFFFU;
}

// Read-only: what the codec shows once the vendor widget has opened all pins, so a failure further on can be read.
static void logTopology(void *codec) {
	const unsigned vendor = getVerb(codec, 0, 0xF00, 0), nodes = getVerb(codec, 1, 0xF00, 4);
	report("codec id %08x, function group 1 nodes %08x", vendor, nodes);
	if (vendor == 0xFFFFFFFFU || nodes == 0xFFFFFFFFU) return;
	const unsigned first = (nodes >> 16) & 0xFF, count = nodes & 0xFF;
	for (unsigned node = first; node < first + count && node < 0x20; node++) {
		const unsigned caps = getVerb(codec, node, 0xF00, 9);
		if (((caps >> 20) & 0xF) == 4)
			report("  pin %02x caps %08x config %08x sense %08x list %08x select %02x device %08x", node, caps,
				getVerb(codec, node, 0xF1C, 0), getVerb(codec, node, 0xF09, 0), getVerb(codec, node, 0xF02, 0),
				getVerb(codec, node, 0xF01, 0) & 0xFF, getVerb(codec, node, 0xF35, 0));
		else
			report("  node %02x caps %08x", node, caps);
	}
}

// The widgets past the first pin stay hidden until the vendor widget is told otherwise, and its node number changed.
static IOReturn wrapApply(void *self) {
	if (!gCodecID) return FunctionCast(wrapApply, orgApply)(self);
	auto codec = field<void *>(self, FunctionGroup::Codec);
	auto execute = virtualSlot<ExecuteVerb>(codec, FunctionGroup::SlotExecuteVerb);
	unsigned value = 0;
	const IOReturn result = execute(codec, IDisp::VendorNode, 0x781, IDisp::VendorEnableAll, &value, 1);
	if (result != kIOReturnSuccess) {
		report("vendor widget %u: enable-all returned %x", IDisp::VendorNode, result);
		return result;
	}
	report("vendor widget %u: enable-all, now %08x", IDisp::VendorNode, getVerb(codec, IDisp::VendorNode, IDisp::VerbGetVendor, 0));
	logTopology(codec);
	gCodecApplied = true;
	gCodecSettledAt = deadlineAfter(gCodecApplications++ ? IDisp::LaterCodecSettleSeconds : IDisp::CodecSettleSeconds);
	return result;
}

static IOReturn wrapPathMap(void *self, OSArray **pathMap) {
	if (!gCodecID) return FunctionCast(wrapPathMap, orgPathMap)(self, pathMap);
	if (!pathMap || !virtualSlot<bool (*)(void *)>(self, FunctionGroup::SlotCanDiscoverPaths)(self)) return kIOReturnUnsupported;
	auto map = OSArray::withCapacity(IDisp::PathCount);
	if (!map) return kIOReturnNoMemory;
	auto addNode = [](OSArray *list, unsigned node) {
		auto entry = OSDictionary::withCapacity(1);
		auto number = OSNumber::withNumber(node, 32);
		const bool ok = entry && number && entry->setObject("NodeID", number) && list->setObject(entry);
		OSSafeReleaseNULL(number);
		OSSafeReleaseNULL(entry);
		return ok;
	};
	for (size_t i = 0; i < IDisp::PathCount; i++) {
		auto path = OSArray::withCapacity(1), set = OSArray::withCapacity(1), nodes = OSArray::withCapacity(2);
		const bool ok = path && set && nodes && addNode(nodes, IDisp::Pins[i].pin) && addNode(nodes, IDisp::Pins[i].converter) &&
			set->setObject(nodes) && path->setObject(set) && map->setObject(path);
		OSSafeReleaseNULL(nodes);
		OSSafeReleaseNULL(set);
		OSSafeReleaseNULL(path);
		if (!ok) {
			map->release();
			return kIOReturnNoMemory;
		}
	}
	*pathMap = map;
	virtualSlot<void (*)(void *, OSArray *)>(self, FunctionGroup::SlotLogPathMap)(self, map);
	return kIOReturnSuccess;
}

static IOReturn widgetRunVerb(void *self, unsigned verb, unsigned payload) {
	unsigned value = 0;
	return virtualSlot<IOReturn (*)(void *, uint16_t, uint16_t, unsigned *)>(self, Widget::SlotRunVerb)(self,
		static_cast<uint16_t>(verb), static_cast<uint16_t>(payload), &value);
}

static IOReturn wrapWidgetInit(void *self, uint8_t node) {
	if (!gCodecID) return FunctionCast(wrapWidgetInit, orgWidgetInit)(self, node);
	if (IDisp::isConverter(node)) {
		static const unsigned quiet[] {0x70D, 0x73E, 0x73F};
		for (auto verb : quiet) {
			const IOReturn result = widgetRunVerb(self, verb, 0);
			if (result != kIOReturnSuccess) report("converter %02x: verb %x returned %x", node, verb, result);
		}
		return kIOReturnSuccess;
	}
	if (auto route = IDisp::pinRoute(node)) {
		auto &connections = field<OSArray *>(self, Widget::Connections);
		OSSafeReleaseNULL(connections);
		connections = OSArray::withCapacity(IDisp::ConverterCount);
		if (connections)
			for (size_t i = 0; i < IDisp::ConverterCount; i++) {
				if (auto number = OSNumber::withNumber(IDisp::Converters[i], 32)) {
					connections->setObject(number);
					number->release();
				}
			}
		virtualSlot<IOReturn (*)(void *, uint16_t)>(self, Widget::SlotSelectConnection)(self, route->converter);
		const IOReturn result = virtualSlot<IOReturn (*)(void *, uint32_t)>(self, Widget::SlotSetPinConfig)(self, route->config);
		if (result != kIOReturnSuccess) {
			report("pin %02x: config returned %x", node, result);
			return result;
		}
		field<uint8_t>(self, Widget::FlagsE8) &= 0xFB;
		field<uint8_t>(self, Widget::Flags37) &= 0x7F;
		return kIOReturnSuccess;
	}
	// The vendor widget, or anything newer than this table: the base class has already read it, and a failure here would
	// lose the whole function group for a node nothing routes through.
	if (node != IDisp::VendorNode) report("node %02x: not in the table", node);
	return kIOReturnSuccess;
}

// The original opens the output amplifier only on pins 5-7.
static IOReturn wrapPinControl(void *self, uint8_t control) {
	const IOReturn result = FunctionCast(wrapPinControl, orgPinControl)(self, control);
	if (!gCodecID || result != kIOReturnSuccess) return result;
	const unsigned node = field<uint16_t>(self, Widget::NodeID);
	if (!IDisp::pinRoute(node) || (node >= 5 && node <= 7)) return result;
	return widgetRunVerb(self, 3, 0xB000 | (control != 0x40 ? 0x80 : 0));
}

static bool routeHDA(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
	auto factory = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN28AppleHDAFunctionGroupFactory27createAppleHDAFunctionGroupEP11DevIdStruct", address, size);
	patcher.clearError();
	if (!factory) return false;
	auto apply = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN30AppleHDAFunctionGroup_8086280740applyCodecSpecificHardwareInitializationEv", address, size);
	auto map = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN30AppleHDAFunctionGroup_8086280727constructPathMapByDiscoveryEPP7OSArray", address, size);
	auto init = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN23AppleHDAWidget_8086280727performWidgetInitializationEh", address, size);
	patcher.clearError();
	if (!apply || !map || !init ||
		!usesField(apply, 0x40, FunctionGroup::SlotExecuteVerb) ||
		!usesField(map, 0x40, FunctionGroup::SlotCanDiscoverPaths) || !usesField(map, 0x300, FunctionGroup::SlotLogPathMap) ||
		!usesField(init, 0x60, Widget::Connections) || !usesField(init, 0x200, Widget::SlotSelectConnection) ||
		!usesField(init, 0x200, Widget::SlotSetPinConfig) || !usesField(init, 0x200, Widget::SlotRunVerb)) {
		report("AppleHDA does not look as expected: nothing routed");
		return true;
	}
	KernelPatcher::RouteRequest routes[] {
		{"__ZN28AppleHDAFunctionGroupFactory27createAppleHDAFunctionGroupEP11DevIdStruct", wrapGroupFactory, orgGroupFactory},
		{"__ZN21AppleHDAWidgetFactory20createAppleHDAWidgetEP11DevIdStruct", wrapWidgetFactory, orgWidgetFactory},
		{"__ZN30AppleHDAFunctionGroup_8086280740applyCodecSpecificHardwareInitializationEv", wrapApply, orgApply},
		{"__ZN30AppleHDAFunctionGroup_8086280727constructPathMapByDiscoveryEPP7OSArray", wrapPathMap, orgPathMap},
		{"__ZN23AppleHDAWidget_8086280727performWidgetInitializationEh", wrapWidgetInit, orgWidgetInit},
		{"__ZN23AppleHDAWidget_8086280713pinSetControlEh", wrapPinControl, orgPinControl},
	};
	gHDAReady = routeGroup(patcher, index, routes, address, size, "AppleHDA");
	return true;
}

#pragma mark - Framebuffer

namespace Framebuffer {
constexpr size_t Index = 0x1DC, Online = 0x1E0, ManagedByAGDC = 0x1E2, Mode = 0x3FD8, PathNoMode = 0x49F0, PathWithMode = 0x4A08,
	HasAudio = 0x424C, Dongle = 0x428A, BaseEDID = 0x428C;
constexpr size_t PathPort = 0x3648, PathPipe = 0x3640, PortConfig = 0x548, ControllerRegisters = 0xC40;
constexpr size_t SlotWriteRegister = 0x130;
constexpr uint32_t PortTypeDisplayPort = 1;
}

static mach_vm_address_t orgSetAudioProperties, orgSetupResources, orgEnableAudio, orgDisableBuffer;
using GetDDCBlock = uint32_t (*)(void *, uint32_t, uint8_t *, uint64_t *, uint32_t, void *);
using ReadRegister = uint32_t (*)(void *, uint64_t);
static GetDDCBlock gGetDDCBlock;
static ReadRegister gReadRegister;
static void *gAudioRegisters;
static uint32_t gAudioPipe;

static bool displayAudioOn() {
	if (!gAudioRegisters) return false;
	const uint32_t output = gReadRegister(gAudioRegisters, IDisp::RegAudioOutputEnable);
	return output != 0xFFFFFFFFU && (output & IDisp::audioOutputBit(gAudioPipe));
}

// As Linux drives it: off first, a millisecond each way.
static void codecWakeSignal(bool on) {
	if (!gAudioRegisters) return;
	auto write = virtualSlot<void (*)(void *, uint64_t, uint32_t)>(gAudioRegisters, Framebuffer::SlotWriteRegister);
	const uint32_t value = gReadRegister(gAudioRegisters, IDisp::RegAudioChicken);
	if (value == 0xFFFFFFFFU) return;
	write(gAudioRegisters, IDisp::RegAudioChicken, value & ~IDisp::AudioCodecWakeSignal);
	IOSleep(2);
	if (!on) return;
	write(gAudioRegisters, IDisp::RegAudioChicken, value | IDisp::AudioCodecWakeSignal);
	IOSleep(2);
}

enum AudioKnowledge : uint8_t { AudioUnknown, AudioAbsent, AudioPresent };
static AudioKnowledge gDisplayAudio[8];

static uint8_t *currentPath(uint8_t *fb) {
	return field<uint8_t *>(fb, field<int32_t>(fb, Framebuffer::Mode) == -1 ? Framebuffer::PathNoMode : Framebuffer::PathWithMode);
}

static const uint32_t *portConfig(uint8_t *path) {
	auto port = path ? field<uint8_t *>(path, Framebuffer::PathPort) : nullptr;
	return port ? field<const uint32_t *>(port, Framebuffer::PortConfig) : nullptr;
}

// A DisplayPort sink that is not behind a dongle, on an output AGDC runs: the case the framebuffer leaves without audio.
static AudioKnowledge askEDID(uint8_t *fb, uint8_t *path) {
	auto config = portConfig(path);
	if (!config || config[2] != Framebuffer::PortTypeDisplayPort || field<uint8_t>(fb, Framebuffer::Dongle)) return AudioAbsent;
	const unsigned extensions = field<uint8_t>(fb, Framebuffer::BaseEDID + 0x7E);
	for (unsigned block = 2; block < extensions + 2 && block <= 4; block++) {
		uint8_t data[128] {};
		uint64_t length = sizeof(data);
		if (gGetDDCBlock(fb, block, data, &length, 0, path) != 0) return AudioUnknown;
		if (IDisp::ctaBlockHasBasicAudio(data)) return AudioPresent;
	}
	return AudioAbsent;
}

static IOReturn wrapSetAudioProperties(uint8_t *fb) {
	if (!gFramebufferReady) return FunctionCast(wrapSetAudioProperties, orgSetAudioProperties)(fb);
	const uint32_t index = field<uint32_t>(fb, Framebuffer::Index);
	uint8_t *path = index && index < arrsize(gDisplayAudio) && field<uint8_t>(fb, Framebuffer::Online) ? currentPath(fb) : nullptr;
	if (path && field<uint8_t>(fb, Framebuffer::ManagedByAGDC)) {
		gDisplayAudio[index] = askEDID(fb, path);
		if (gDisplayAudio[index] == AudioPresent) field<uint8_t>(fb, Framebuffer::HasAudio) = 1;
	}
	const IOReturn result = FunctionCast(wrapSetAudioProperties, orgSetAudioProperties)(fb);
	if (!path) return result;
	auto config = portConfig(path);
	if (!field<uint8_t>(fb, Framebuffer::HasAudio) || !config) {
		report("framebuffer %u: no audio, EDID answer %u", index, gDisplayAudio[index]);
		return result;
	}
	// Whichever way the flag got set, the table the framebuffer used is the wrong one for this generation.
	uint32_t pinOverride = 0;
	uint32_t value = IDisp::codecInfoForPort(config[0]);
	if (PE_parse_boot_argn("idisppin", &pinOverride, sizeof(pinOverride)) && pinOverride) value = IDisp::makeCodecInfo(pinOverride);
	if (value) {
		reinterpret_cast<IOService *>(fb)->setProperty("audio-codec-info", &value, sizeof(value));
		report("framebuffer %u: display has audio, port %u, audio-codec-info %08x", index, config[0], value);
	} else
		report("framebuffer %u: display has audio, port %u has no pin", index, config[0]);
	return result;
}

// Setting the mode copies AGDC's choice over the flag again. Only what the connection change found counts here: the EDID is
// read on that path, as the framebuffer itself does, and not in the middle of a mode set.
static uint64_t wrapSetupResources(uint8_t *fb) {
	const uint64_t result = FunctionCast(wrapSetupResources, orgSetupResources)(fb);
	if (!gFramebufferReady) return result;
	const uint32_t index = field<uint32_t>(fb, Framebuffer::Index);
	if (index && index < arrsize(gDisplayAudio) && gDisplayAudio[index] == AudioPresent && field<uint8_t>(fb, Framebuffer::ManagedByAGDC))
		field<uint8_t>(fb, Framebuffer::HasAudio) = 1;
	return result;
}

// The framebuffer never programs the link clock of the audio side, which firmware leaves for its own needs: Linux sets
// the same value on every display from Tiger Lake on before it wakes the codec. It also enables the pin buffer only for
// the first two pipes.
static uint64_t wrapEnableAudio(uint8_t *controller, uint8_t *fb, uint8_t *path) {
	if (!gFramebufferReady) return FunctionCast(wrapEnableAudio, orgEnableAudio)(controller, fb, path);
	auto registers = field<void *>(controller, Framebuffer::ControllerRegisters);
	auto write = virtualSlot<void (*)(void *, uint64_t, uint32_t)>(registers, Framebuffer::SlotWriteRegister);
	const uint32_t bufferBefore = gReadRegister(registers, IDisp::RegAudioPinBuffer);
	const uint32_t frequency = gReadRegister(registers, IDisp::RegAudioFrequency);
	if (frequency != IDisp::AudioFrequencyGen12) write(registers, IDisp::RegAudioFrequency, IDisp::AudioFrequencyGen12);
	const uint64_t result = FunctionCast(wrapEnableAudio, orgEnableAudio)(controller, fb, path);
	uint32_t buffer = gReadRegister(registers, IDisp::RegAudioPinBuffer);
	if (!(buffer & IDisp::AudioPinBufferEnable)) {
		buffer |= IDisp::AudioPinBufferEnable;
		write(registers, IDisp::RegAudioPinBuffer, buffer);
	}
	gAudioPinsOpen = true;
	gAudioFramebuffer = fb;
	gAudioRegisters = registers;
	gAudioPipe = field<uint32_t>(path, Framebuffer::PathPipe);
	gCodecSilentUntil = 0;
	gCodecRequests = 0;
	gCheckCodec = true;
	const uint32_t pipe = field<uint32_t>(path, Framebuffer::PathPipe);
	report("audio on, framebuffer %u pipe %u: frequency was %08x, pin buffer was %08x now %08x, output %08x, config %08x, misc %08x",
		field<uint32_t>(fb, Framebuffer::Index), pipe, frequency, bufferBefore, gReadRegister(registers, IDisp::RegAudioPinBuffer),
		gReadRegister(registers, IDisp::RegAudioOutputEnable), gReadRegister(registers, IDisp::RegAudioConfig + pipe * 0x100),
		gReadRegister(registers, IDisp::RegAudioMisc + pipe * 0x100));
	return result;
}

// The framebuffer closes the audio pin buffer when the last display with audio goes away, with the link running. Linux keeps the
// buffer open for as long as the audio side wants the codec, so it stays open here.
static void wrapDisableBuffer(void *controller) {
	if (!gFramebufferReady || !gAudioPinsOpen) FunctionCast(wrapDisableBuffer, orgDisableBuffer)(controller);
}

static bool routeFramebuffer(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
	auto properties = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN21AppleIntelFramebuffer18setAudioPropertiesEv", address, size);
	patcher.clearError();
	if (!properties) return false;
	auto resources = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN21AppleIntelFramebuffer21setupDisplayResourcesEv", address, size);
	auto enable = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN31AppleIntelFramebufferController13hwEnableAudioEP21AppleIntelFramebufferP21AppleIntelDisplayPath", address, size);
	auto disableBuffer = patcher.solveSymbol<mach_vm_address_t>(index, "__ZN31AppleIntelFramebufferController20hwDisableAudioBufferEv", address, size);
	gGetDDCBlock = patcher.solveSymbol<GetDDCBlock>(index, "__ZN21AppleIntelFramebuffer13fbGetDDCBlockEjPhPyjP21AppleIntelDisplayPath", address, size);
	gReadRegister = patcher.solveSymbol<ReadRegister>(index, "__ZN31AppleIntelRegisterAccessManager14ReadRegister32Em", address, size);
	patcher.clearError();
	if (!resources || !enable || !disableBuffer || !gGetDDCBlock || !gReadRegister ||
		!usesField(disableBuffer, 0x80, IDisp::RegAudioPinBuffer) || !usesField(disableBuffer, 0x40, IDisp::RegAudioOutputEnable) ||
		!usesField(properties, 0x220, Framebuffer::HasAudio) || !usesField(properties, 0x40, Framebuffer::Online) ||
		!usesField(resources, 0xA0, Framebuffer::ManagedByAGDC) || !usesField(resources, 0xA0, Framebuffer::HasAudio) ||
		!usesField(enable, 0x40, Framebuffer::ControllerRegisters) || !usesField(enable, 0x40, Framebuffer::PathPipe) ||
		!usesField(enable, 0x100, Framebuffer::SlotWriteRegister) || !usesField(enable, 0x100, IDisp::RegAudioPinBuffer)) {
		report("framebuffer does not look as expected: nothing routed");
		return true;
	}
	KernelPatcher::RouteRequest routes[] {
		{"__ZN21AppleIntelFramebuffer18setAudioPropertiesEv", wrapSetAudioProperties, orgSetAudioProperties},
		{"__ZN21AppleIntelFramebuffer21setupDisplayResourcesEv", wrapSetupResources, orgSetupResources},
		{"__ZN31AppleIntelFramebufferController13hwEnableAudioEP21AppleIntelFramebufferP21AppleIntelDisplayPath", wrapEnableAudio, orgEnableAudio},
		{"__ZN31AppleIntelFramebufferController20hwDisableAudioBufferEv", wrapDisableBuffer, orgDisableBuffer},
	};
	gFramebufferReady = routeGroup(patcher, index, routes, address, size, "framebuffer");
	return true;
}

#pragma mark - Plugin

static const char *kPathController[] {"/System/Library/Extensions/AppleHDA.kext/Contents/PlugIns/AppleHDAController.kext/Contents/MacOS/AppleHDAController"};
static const char *kPathHDA[] {"/System/Library/Extensions/AppleHDA.kext/Contents/MacOS/AppleHDA"};
static const char *kPathFramebuffer[] {"/System/Library/Extensions/AppleIntelTGLGraphicsFramebuffer.kext/Contents/MacOS/AppleIntelTGLGraphicsFramebuffer"};

static KernelPatcher::KextInfo kKexts[] {
	{"com.apple.driver.AppleHDAController", kPathController, arrsize(kPathController), {true}, {}, KernelPatcher::KextInfo::Unloaded},
	{"com.apple.driver.AppleHDA", kPathHDA, arrsize(kPathHDA), {true}, {}, KernelPatcher::KextInfo::Unloaded},
	{"com.apple.driver.AppleIntelTGLGraphicsFramebuffer", kPathFramebuffer, arrsize(kPathFramebuffer), {}, {}, KernelPatcher::KextInfo::Unloaded},
};

// Lilu hands every requested image to every callback, and the images SimpleHDA carries can load under another index than
// the one registered, so each image is recognized by a symbol only it has.
static void onImage(void *, KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
	if (!gControllerSeen) gControllerSeen = routeController(patcher, index, address, size);
	if (!gHDASeen) gHDASeen = routeHDA(patcher, index, address, size);
	if (!gFramebufferSeen) gFramebufferSeen = routeFramebuffer(patcher, index, address, size);
}

static void pluginStart() {
	gLogLock = IOSimpleLockAlloc();
	sysctl_register_oid(&sysctl__debug_idisphda);
	report("started");
	lilu.onKextLoadForce(kKexts, arrsize(kKexts), onImage);
}

static const char *bootargOff[] {"-idisphdaoff"};
static const char *bootargDebug[] {"-idisphdadbg"};
static const char *bootargBeta[] {"-idisphdabeta"};

PluginConfiguration ADDPR(config) {
	xStringify(PRODUCT_NAME),
	parseModuleVersion(xStringify(MODULE_VERSION)),
	LiluAPI::AllowNormal,
	bootargOff, arrsize(bootargOff),
	bootargDebug, arrsize(bootargDebug),
	bootargBeta, arrsize(bootargBeta),
	KernelVersion::Sequoia, KernelVersion::Tahoe,
	pluginStart
};

// kmutil places a kext in a kernel collection only when it has a _kmod_info symbol.
extern "C" kern_return_t IDispHDA_kern_start(kmod_info_t *, void *);
extern "C" kern_return_t IDispHDA_kern_stop(kmod_info_t *, void *);
extern "C" { KMOD_EXPLICIT_DECL(org.anv1552.IDispHDA, "1.0.0", IDispHDA_kern_start, IDispHDA_kern_stop) }
