// The display codec's numbers, against the values read from the sources named in idisphda_tables.h.
#include "idisphda_tables.h"
#include <stdio.h>
#include <stdlib.h>

#define CHECK(condition) do { if (!(condition)) { printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; } tests++; } while (0)

int main() {
	int failures = 0, tests = 0;

	// The first Type-C port is display port 3 and pin 0xA, codec address 2, function group 1.
	CHECK(IDisp::codecInfoForPort(3) == 0x000A0102);
	CHECK(IDisp::codecInfoForPort(4) == 0x000B0102);
	CHECK(IDisp::codecInfoForPort(1) == 0x00060102);
	CHECK(IDisp::codecInfoForPort(0) == 0);
	CHECK(IDisp::codecInfoForPort(9) == 0);
	CHECK(IDisp::codecInfoForPort(0xFFFFFFFFU) == 0);

	// AppleHDAController's own acceptance: non-zero group and pin.
	CHECK(IDisp::codecInfoValid(0x000A0102));
	CHECK(!IDisp::codecInfoValid(0));
	CHECK(!IDisp::codecInfoValid(0x00000102));
	CHECK(!IDisp::codecInfoValid(0x000A0002));
	// What a NVIDIA framebuffer publishes must not count as this codec.
	CHECK(!IDisp::codecInfoValid(0x00050100));

	// Linux's Alder Lake-P and Raptor Lake entries, and Tiger Lake's; never the Skylake class's own.
	CHECK(IDisp::modernDisplayCodec(0x8086281C));
	CHECK(IDisp::modernDisplayCodec(0x80862818));
	CHECK(IDisp::modernDisplayCodec(0x80862812));
	CHECK(!IDisp::modernDisplayCodec(0x8086280B));
	CHECK(!IDisp::modernDisplayCodec(0x80862807));
	CHECK(!IDisp::modernDisplayCodec(0x10EC0245));
	CHECK(!IDisp::modernDisplayCodec(0x10DE00AE));

	// Apple's table for this family.
	CHECK(IDisp::pinRoute(0xA) && IDisp::pinRoute(0xA)->converter == 9 && IDisp::pinRoute(0xA)->config == 0x18560040);
	CHECK(IDisp::pinRoute(0x4) && IDisp::pinRoute(0x4)->converter == 3 && IDisp::pinRoute(0x4)->config == 0x18560010);
	CHECK(IDisp::pinRoute(0xB) && IDisp::pinRoute(0xB)->converter == 3 && IDisp::pinRoute(0xB)->config == 0x18560050);
	CHECK(!IDisp::pinRoute(2));
	CHECK(!IDisp::pinRoute(3));
	CHECK(IDisp::isConverter(9) && IDisp::isConverter(3));
	CHECK(!IDisp::isConverter(4) && !IDisp::isConverter(2) && !IDisp::isConverter(0xA));
	// The path map must include the first Type-C port's pin.
	bool typeCInPaths = false;
	for (size_t i = 0; i < IDisp::PathCount; i++) typeCInPaths |= IDisp::Pins[i].pin == 0xA;
	CHECK(typeCInPaths);

	// Linux's AUD_FREQ_GEN12: 8T, no pull clocks, 96 MHz.
	CHECK(IDisp::AudioFrequencyGen12 == ((2U << 14) | (1U << 4)));

	const uint8_t withAudio[128] {0x02, 0x03, 0x23, 0xF1};
	const uint8_t withoutAudio[128] {0x02, 0x03, 0x23, 0xB1};
	const uint8_t otherBlock[128] {0x70, 0x03, 0x23, 0xF1};
	CHECK(IDisp::ctaBlockHasBasicAudio(withAudio));
	CHECK(!IDisp::ctaBlockHasBasicAudio(withoutAudio));
	CHECK(!IDisp::ctaBlockHasBasicAudio(otherBlock));

	// What may be routed: both prologues the routed functions use, and not one somebody has routed already.
	const uint8_t pushR15[] {0x55, 0x48, 0x89, 0xE5, 0x41, 0x57};
	const uint8_t pushRbx[] {0x55, 0x48, 0x89, 0xE5, 0x53, 0x50};
	const uint8_t shortJump[] {0xE9, 0x10, 0x20, 0x30, 0x40, 0x00};
	const uint8_t longJump[] {0xFF, 0x25, 0x00, 0x00, 0x00, 0x00};
	const uint8_t ripRelative[] {0x55, 0x48, 0x89, 0xE5, 0x48, 0x8D};
	const uint8_t ripRelativeR8[] {0x55, 0x48, 0x89, 0xE5, 0x41, 0x8B};
	CHECK(IDisp::plainPrologue(pushR15));
	CHECK(IDisp::plainPrologue(pushRbx));
	CHECK(!IDisp::plainPrologue(shortJump));
	CHECK(!IDisp::plainPrologue(longJump));
	CHECK(!IDisp::plainPrologue(ripRelative));
	CHECK(!IDisp::plainPrologue(ripRelativeR8));

	// The vendor widget's state, as IOHDACodecDevice::executeVerb would compose it; the analog codec's commands stay out.
	CHECK(IDisp::codecCommand(IDisp::VendorNode, IDisp::VerbGetVendor, 0) == 0x202F8100);
	CHECK(IDisp::commandForCodec(0x202F8100));
	CHECK(IDisp::commandForCodec(0x20AF0900));
	CHECK(!IDisp::commandForCodec(0x000F0000));
	CHECK(!IDisp::commandForCodec(0x100F0000));

	// The analog codec's power states as it declares them on this hardware, and a function group that lacks clock stop.
	CHECK(IDisp::commandTo(0, IDisp::FunctionGroupNode, IDisp::VerbGetParameter, IDisp::ParameterPowerStates) == 0x001F000F);
	CHECK(IDisp::restoredAfterClockStop(0xC000001F));
	CHECK(!IDisp::restoredAfterClockStop(0x8000000F));
	CHECK(!IDisp::restoredAfterClockStop(0));
	CHECK(IDisp::audioOutputBit(0) == 0x4);
	CHECK(IDisp::audioOutputBit(1) == 0x40);
	CHECK(IDisp::audioOutputBit(2) == 0x400);

	CHECK(IDisp::lateCodecController(0x51CA));
	CHECK(!IDisp::lateCodecController(0x22EC));

	printf("%d tests, %d failures\n", tests, failures);
	return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
