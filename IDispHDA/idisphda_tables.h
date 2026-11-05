// The numbers of the display codec and its registers. No kernel headers: the host test builds this file too.
//
// Sources for the node numbers: Apple's AppleGFXHDAFunctionGroup_8086280F (pins 4, 6, 8, 0xA, 0xB, converters 3, 5, 7, 9,
// vendor widget 2, pin configuration 0x18560010 up) and Linux sound/hda/codecs/hdmi/intelhdmi.c (from Tiger Lake on, port
// N of the display goes to entry N of {4, 6, 8, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF}).
#ifndef idisphda_tables_h
#define idisphda_tables_h

#include <stddef.h>
#include <stdint.h>

namespace IDisp {

constexpr uint32_t StandInDevice = 0xA348, LateCodecDevice = 0x9D70, ClassCodec = 0x80862807;
constexpr unsigned CodecAddress = 2, FunctionGroupNode = 1, VendorNode = 2, VendorEnableAll = 3, VerbGetVendor = 0xF81;
// The controller's event gate asks for the codec, and the function it then calls asks again: four attempts in a row.
constexpr unsigned MaxCodecRequests = 8;
constexpr unsigned SilentCodecSeconds = 5;
constexpr unsigned CodecReplaceWaitMs = 200;
constexpr unsigned CodecSettleSeconds = 15, LaterCodecSettleSeconds = 1;
// The link reset: how long it stays down, how long a codec gets afterwards, and how many in a row may fail before it stops trying.
constexpr unsigned LinkResetHoldMs = 5, LinkResetSettleMs = 20, MaxLinkResetsInARow = 2;
// The power states of a function group, whose top two bits are what AppleHDA restores a codec for after the clock stops.
constexpr unsigned VerbGetParameter = 0xF00, ParameterPowerStates = 0x0F;
constexpr bool restoredAfterClockStop(uint32_t powerStates) { return powerStates >= 0xC0000000U; }
// kIOFBNotifyDisplayModeDidChange.
constexpr int EventModeDidChange = 2;

struct PinRoute {
	uint8_t pin, converter;
	uint32_t config;
};

constexpr uint8_t Converters[] {3, 5, 7, 9};
constexpr size_t ConverterCount = sizeof(Converters) / sizeof(Converters[0]);

constexpr PinRoute Pins[] {
	{0x4, 3, 0x18560010}, {0x6, 5, 0x18560020}, {0x8, 7, 0x18560030}, {0xA, 9, 0x18560040}, {0xB, 3, 0x18560050},
	{0xC, 5, 0x18560060}, {0xD, 7, 0x18560070}, {0xE, 9, 0x18560080}, {0xF, 3, 0x18560090},
};
constexpr size_t PinCount = sizeof(Pins) / sizeof(Pins[0]);
// Apple publishes five paths for this family; AppleHDA has never been asked for more.
constexpr size_t PathCount = 5;

constexpr uint8_t PortPins[] {0x4, 0x6, 0x8, 0xA, 0xB, 0xC, 0xD, 0xE, 0xF};

// AUD_FREQ_CNTL, AUD_PIN_BUF_CTL and the per-transcoder audio registers, as in Linux intel_audio.c and intel_audio_regs.h.
constexpr uint32_t RegAudioFrequency = 0x65900, RegAudioPinBuffer = 0x48414, RegAudioOutputEnable = 0x650C0,
	RegAudioConfig = 0x65000, RegAudioMisc = 0x65010;
constexpr uint32_t AudioFrequencyGen12 = 0x8010, AudioPinBufferEnable = 0x80000000U;
// HSW_AUD_CHICKENBIT and SKL_AUD_CODEC_WAKE_SIGNAL: the display side sends the controller its codec's wake on request.
constexpr uint32_t RegAudioChicken = 0x65F10, AudioCodecWakeSignal = 0x8000;
constexpr uint32_t audioOutputBit(uint32_t pipe) { return 4U << (pipe * 4); }

// A verb as the controller's command path takes it: codec address on top, then the node, the 12-bit verb and its payload.
constexpr uint32_t commandTo(unsigned address, unsigned node, unsigned verb, unsigned payload) {
	return (address << 28) | (node << 20) | (verb << 8) | payload;
}
constexpr uint32_t codecCommand(unsigned node, unsigned verb, unsigned payload) { return commandTo(CodecAddress, node, verb, payload); }
constexpr bool commandForCodec(uint32_t command) { return (command >> 28) == CodecAddress; }

constexpr bool lateCodecController(uint32_t device) { return device == 0x51CA; }

// Everything Linux treats as Tiger Lake's model or newer, in the 8086:28xx range.
constexpr bool modernDisplayCodec(uint32_t id) { return (id >> 16) == 0x8086 && (id & 0xFFFF) >= 0x2812 && (id & 0xFFFF) <= 0x28FF; }

inline bool isConverter(unsigned node) {
	for (size_t i = 0; i < ConverterCount; i++)
		if (Converters[i] == node) return true;
	return false;
}

inline const PinRoute *pinRoute(unsigned node) {
	for (size_t i = 0; i < PinCount; i++)
		if (Pins[i].pin == node) return &Pins[i];
	return nullptr;
}

// Byte 0 the codec address, byte 1 the function group, byte 2 the pin. AppleHDAController refuses a zero group or pin.
constexpr uint32_t makeCodecInfo(unsigned pin) { return (pin << 16) | (FunctionGroupNode << 8) | CodecAddress; }

// Port 0 is the internal panel's on this family, and the framebuffer's own table is right only for an Ice Lake layout.
inline uint32_t codecInfoForPort(unsigned port) {
	if (port == 0 || port >= sizeof(PortPins)) return 0;
	return makeCodecInfo(PortPins[port]);
}

// A function can be routed when it starts with push rbp; mov rbp, rsp; push: Lilu copies those bytes as they are, and none of
// them is relative to rip. A jump in that place means another plugin holds the function, and Lilu panics on a second route
// over a short jump.
constexpr bool plainPrologue(const uint8_t *code) {
	return code[0] == 0x55 && code[1] == 0x48 && code[2] == 0x89 && code[3] == 0xE5 &&
		((code[4] >= 0x50 && code[4] <= 0x57) || (code[4] == 0x41 && code[5] >= 0x50 && code[5] <= 0x57));
}

constexpr bool codecInfoValid(uint32_t info) { return (info & 0xFF) == CodecAddress && (info & 0xFF00) && (info & 0xFF0000); }

inline bool ctaBlockHasBasicAudio(const uint8_t *block) { return block[0] == 0x02 && (block[3] & 0x40); }

}

#endif
