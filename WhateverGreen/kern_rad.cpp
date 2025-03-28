//
//  kern_rad.cpp
//  WhateverGreen
//
//  Copyright © 2017 vit9696. All rights reserved.
//

#include <Headers/kern_api.hpp>
#include <Headers/kern_iokit.hpp>
#include <Headers/kern_devinfo.hpp>
#include <Headers/kern_time.hpp>
#include <IOKit/IOService.h>

#include <Availability.h>
#include <IOKit/IOPlatformExpert.h>

#include "kern_rad.hpp"

static const char *pathFramebuffer[]		{ "/System/Library/Extensions/AMDFramebuffer.kext/Contents/MacOS/AMDFramebuffer" };
static const char *pathRedeonX6000Framebuffer[]	{ "/System/Library/Extensions/AMDRadeonX6000Framebuffer.kext/Contents/MacOS/AMDRadeonX6000Framebuffer" };
static const char *pathLegacyFramebuffer[]	{ "/System/Library/Extensions/AMDLegacyFramebuffer.kext/Contents/MacOS/AMDLegacyFramebuffer" };
static const char *pathSupport[]			{ "/System/Library/Extensions/AMDSupport.kext/Contents/MacOS/AMDSupport" };
static const char *pathLegacySupport[]		{ "/System/Library/Extensions/AMDLegacySupport.kext/Contents/MacOS/AMDLegacySupport" };
static const char *pathRadeonX3000[]        { "/System/Library/Extensions/AMDRadeonX3000.kext/Contents/MacOS/AMDRadeonX3000" };
static const char *pathRadeonX4000[]        { "/System/Library/Extensions/AMDRadeonX4000.kext/Contents/MacOS/AMDRadeonX4000" };
static const char *pathRadeonX4100[]        { "/System/Library/Extensions/AMDRadeonX4100.kext/Contents/MacOS/AMDRadeonX4100" };
static const char *pathRadeonX4150[]        { "/System/Library/Extensions/AMDRadeonX4150.kext/Contents/MacOS/AMDRadeonX4150" };
static const char *pathRadeonX4200[]        { "/System/Library/Extensions/AMDRadeonX4200.kext/Contents/MacOS/AMDRadeonX4200" };
static const char *pathRadeonX4250[]        { "/System/Library/Extensions/AMDRadeonX4250.kext/Contents/MacOS/AMDRadeonX4250" };
static const char *pathRadeonX5000[]        { "/System/Library/Extensions/AMDRadeonX5000.kext/Contents/MacOS/AMDRadeonX5000" };
static const char *pathRadeonX6000[]        { "/System/Library/Extensions/AMDRadeonX6000.kext/Contents/MacOS/AMDRadeonX6000" };
static const char *patchPolarisController[] { "/System/Library/Extensions/AMD9500Controller.kext/Contents/MacOS/AMD9500Controller" };

static const char *idRadeonX3000New {"com.apple.kext.AMDRadeonX3000"};
static const char *idRadeonX4000New {"com.apple.kext.AMDRadeonX4000"};
static const char *idRadeonX4100New {"com.apple.kext.AMDRadeonX4100"};
static const char *idRadeonX4150New {"com.apple.kext.AMDRadeonX4150"};
static const char *idRadeonX4200New {"com.apple.kext.AMDRadeonX4200"};
static const char *idRadeonX4250New {"com.apple.kext.AMDRadeonX4250"};
static const char *idRadeonX5000New {"com.apple.kext.AMDRadeonX5000"};
static const char *idRadeonX6000New {"com.apple.kext.AMDRadeonX6000"};
static const char *idRadeonX3000Old {"com.apple.AMDRadeonX3000"};
static const char *idRadeonX4000Old {"com.apple.AMDRadeonX4000"};

static KernelPatcher::KextInfo kextRadeonFramebuffer
{ "com.apple.kext.AMDFramebuffer", pathFramebuffer, arrsize(pathFramebuffer), {}, {}, KernelPatcher::KextInfo::Unloaded };
static KernelPatcher::KextInfo kextRadeonLegacyFramebuffer
{ "com.apple.kext.AMDLegacyFramebuffer", pathLegacyFramebuffer, arrsize(pathLegacyFramebuffer), {}, {}, KernelPatcher::KextInfo::Unloaded };
static KernelPatcher::KextInfo kextRadeonSupport
{ "com.apple.kext.AMDSupport", pathSupport, 1, {}, {}, KernelPatcher::KextInfo::Unloaded };
static KernelPatcher::KextInfo kextRadeonLegacySupport
{ "com.apple.kext.AMDLegacySupport", pathLegacySupport, 1, {}, {}, KernelPatcher::KextInfo::Unloaded };
static KernelPatcher::KextInfo kextPolarisController
{ "com.apple.kext.AMD9500Controller", patchPolarisController, 1, {}, {}, KernelPatcher::KextInfo::Unloaded };
static KernelPatcher::KextInfo kextRadeonX6000Framebuffer
{ "com.apple.kext.AMDRadeonX6000Framebuffer", pathRedeonX6000Framebuffer, arrsize(pathRedeonX6000Framebuffer), {}, {}, KernelPatcher::KextInfo::Unloaded };

static KernelPatcher::KextInfo kextRadeonHardware[RAD::MaxRadeonHardware] {
	[RAD::IndexRadeonHardwareX3000] = { idRadeonX3000New, pathRadeonX3000, arrsize(pathRadeonX3000), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX4100] = { idRadeonX4100New, pathRadeonX4100, arrsize(pathRadeonX4100), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX4150] = { idRadeonX4150New, pathRadeonX4150, arrsize(pathRadeonX4150), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX4200] = { idRadeonX4200New, pathRadeonX4200, arrsize(pathRadeonX4200), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX4250] = { idRadeonX4250New, pathRadeonX4250, arrsize(pathRadeonX4250), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX4000] = { idRadeonX4000New, pathRadeonX4000, arrsize(pathRadeonX4000), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX5000] = { idRadeonX5000New, pathRadeonX5000, arrsize(pathRadeonX5000), {}, {}, KernelPatcher::KextInfo::Unloaded },
	[RAD::IndexRadeonHardwareX6000] = { idRadeonX6000New, pathRadeonX6000, arrsize(pathRadeonX6000), {}, {}, KernelPatcher::KextInfo::Unloaded }
};

/**
 *  Power-gating flags
 *  Each symbol corresponds to a bit provided in a radpg argument mask
 */
static const char *powerGatingFlags[] {
	"CAIL_DisableDrmdmaPowerGating",
	"CAIL_DisableGfxCGPowerGating",
	"CAIL_DisableUVDPowerGating",
	"CAIL_DisableVCEPowerGating",
	"CAIL_DisableDynamicGfxMGPowerGating",
	"CAIL_DisableGmcPowerGating",
	"CAIL_DisableAcpPowerGating",
	"CAIL_DisableSAMUPowerGating"
};

RAD *RAD::callbackRAD;

void RAD::init(bool enableNavi10Bkl) {
	callbackRAD = this;

	currentPropProvider.init();
	currentLegacyPropProvider.init();

	force24BppMode = checkKernelArgument("-rad24");
	useCustomAgdpDecision = getKernelVersion() >= KernelVersion::Catalina;

	// Certain displays do not support 32-bit colour output, so we have to force 24-bit.
	if (getKernelVersion() >= KernelVersion::Sierra && force24BppMode) {
		lilu.onKextLoadForce(&kextRadeonFramebuffer);
		// Mojave dropped legacy GPU support (5xxx and 6xxx).
		if (getKernelVersion() < KernelVersion::Mojave)
			lilu.onKextLoadForce(&kextRadeonLegacyFramebuffer);
	}
	
	if (enableNavi10Bkl) {
		lilu.onKextLoadForce(&kextRadeonX6000Framebuffer);
	}

	// Certain GPUs cannot output to DVI at full resolution.
	dviSingleLink = checkKernelArgument("-raddvi");

	// Disabling Metal may be useful for testing
	forceOpenGL = checkKernelArgument("-radgl");

	// Fix accelerator name if requested
	fixConfigName = checkKernelArgument("-radcfg");

	// Broken drivers can still let us boot in vesa mode
	forceVesaMode = checkKernelArgument("-radvesa");
	
	// Fix codec PID to be spoofed PID if requested
	forceCodecInfo = checkKernelArgument("-radcodec");

	// To support overriding connectors and -radvesa mode we need to patch AMDSupport.
	lilu.onKextLoadForce(&kextRadeonSupport);
	// Mojave dropped legacy GPU support (5xxx and 6xxx).
	if (getKernelVersion() < KernelVersion::Mojave)
		lilu.onKextLoadForce(&kextRadeonLegacySupport);
	else
		lilu.onKextLoadForce(&kextPolarisController);

	initHardwareKextMods();

	//FIXME: autodetect?
	uint32_t powerGatingMask = 0;
	PE_parse_boot_argn("radpg", &powerGatingMask, sizeof(powerGatingMask));
	for (size_t i = 0; i < arrsize(powerGatingFlags); i++) {
		if (!(powerGatingMask & (1 << i))) {
			DBGLOG("rad", "not enabling %s", powerGatingFlags[i]);
			powerGatingFlags[i] = nullptr;
		} else {
			DBGLOG("rad", "enabling %s", powerGatingFlags[i]);
		}
	}
}

void RAD::deinit() {

}

void RAD::processKernel(KernelPatcher &patcher, DeviceInfo *info) {
	bool hasAMD = false;
	for (size_t i = 0; i < info->videoExternal.size(); i++) {
		if (info->videoExternal[i].vendor == WIOKit::VendorID::ATIAMD) {
			if (!hasAMD) {
				hasAMD = true;
			}

			if (info->videoExternal[i].video->getProperty("enable-gva-support"))
				enableGvaSupport = true;

			// When injecting values into device properties one cannot specify boolean types.
			// Provide special support for Force_Load_FalconSMUFW.
			auto smufw = OSDynamicCast(OSData, info->videoExternal[i].video->getProperty("Force_Load_FalconSMUFW"));
			if (smufw && smufw->getLength() == 1)
				info->videoExternal[i].video->setProperty("Force_Load_FalconSMUFW",
					*static_cast<const uint8_t *>(smufw->getBytesNoCopy()) ? kOSBooleanTrue : kOSBooleanFalse);
		}
	}

	if (hasAMD) {
		int gva;
		if (PE_parse_boot_argn("radgva", &gva, sizeof(gva)))
			enableGvaSupport = gva != 0;

		KernelPatcher::RouteRequest requests[] {
			KernelPatcher::RouteRequest("__ZN15IORegistryEntry11setPropertyEPKcPvj", wrapSetProperty, orgSetProperty),
			KernelPatcher::RouteRequest("__ZNK15IORegistryEntry11getPropertyEPKc", wrapGetProperty, orgGetProperty),
		};
		patcher.routeMultiple(KernelPatcher::KernelID, requests);

		if (useCustomAgdpDecision && info->firmwareVendor == DeviceInfo::FirmwareVendor::Apple)
			useCustomAgdpDecision = false;
	} else {
		kextRadeonFramebuffer.switchOff();
		kextRadeonLegacyFramebuffer.switchOff();
		kextRadeonSupport.switchOff();
		kextRadeonLegacySupport.switchOff();

		for (size_t i = 0; i < maxHardwareKexts; i++)
			kextRadeonHardware[i].switchOff();
	}
}

void RAD::updatePwmMaxBrightnessFromInternalDisplay() {
	OSDictionary * matching = IOService::serviceMatching("AppleBacklightDisplay");
	if (matching == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null AppleBacklightDisplay");
		return;
	}
	
	OSIterator *iter = IOService::getMatchingServices(matching);
	if (iter == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null matching");
		matching->release();
		return;
	}
	
	IORegistryEntry* display = OSDynamicCast(IORegistryEntry, iter->getNextObject());
	if (display == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null display");
		iter->release();
		matching->release();
		return;
	}
	
	OSDictionary* iodispparm = OSDynamicCast(OSDictionary, display->getProperty("IODisplayParameters"));
	if (iodispparm == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null IODisplayParameters");
		iter->release();
		matching->release();
		return;
	}
	
	OSDictionary* linearbri = OSDynamicCast(OSDictionary, iodispparm->getObject("linear-brightness"));
	if (linearbri == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null linear-brightness");
		iter->release();
		matching->release();
		return;
	}
	
	OSNumber* maxbri = OSDynamicCast(OSNumber, linearbri->getObject("max"));
	if (maxbri == nullptr) {
		DBGLOG("igfx", "isRadeonX6000WiredToInternalDisplay null max");
		iter->release();
		matching->release();
		return;
	}

	callbackRAD->maxPwmBacklightLvl = maxbri->unsigned32BitValue();
	DBGLOG("igfx", "updatePwmMaxBrightnessFromInternalDisplay get max brightness: 0x%x", callbackRAD->maxPwmBacklightLvl);

	iter->release();
	matching->release();
}

typedef int64_t __int64;
typedef int64_t _QWORD;
typedef int32_t _DWORD;
typedef int8_t __int8;

struct dc_stream_state {
	void *sink;
	void *link;
};

#define MAX_SINKS_PER_LINK 4

struct dc_link_settings {
	uint32_t lane_count;
	uint32_t link_rate;
	uint32_t link_spread;
	bool use_link_rate_set;
	uint8_t link_rate_set;
};

struct dc_lane_settings {
	uint32_t VOLTAGE_SWING;
	uint32_t PRE_EMPHASIS;
	uint32_t POST_CURSOR2;
};

struct dc_link_training_overrides {
	uint32_t *voltage_swing;
	uint32_t *pre_emphasis;
	uint32_t *post_cursor2;

	uint16_t *cr_pattern_time;
	uint16_t *eq_pattern_time;
	uint32_t *pattern_for_cr;
	uint32_t *pattern_for_eq;

	uint32_t *downspread;
	bool *alternate_scrambler_reset;
	bool *enhanced_framing;
	bool *mst_enable;
	bool *fec_enable;
};

struct dp_audio_test_data_flags {
	uint8_t test_requested  :1;
	uint8_t disable_video   :1;
};

struct dp_audio_test_data {

	struct dp_audio_test_data_flags flags;
	uint8_t sampling_rate;
	uint8_t channel_count;
	uint8_t pattern_type;
	uint8_t pattern_period[8];
};

union compliance_test_state {
	struct {
		unsigned char STEREO_3D_RUNNING        : 1;
		unsigned char RESERVED                 : 7;
	} bits;
	unsigned char raw;
};

struct graphics_object_id {
	uint32_t  id:8;
	uint32_t  enum_id:4;
	uint32_t  type:4;
	uint32_t  reserved:16; /* for padding. total size should be u32 */
};

union ddi_channel_mapping {
	struct mapping {
		uint8_t lane0:2;	/* Mapping for lane 0 */
		uint8_t lane1:2;	/* Mapping for lane 1 */
		uint8_t lane2:2;	/* Mapping for lane 2 */
		uint8_t lane3:2;	/* Mapping for lane 3 */
	} mapping;
	uint8_t raw;
};

struct device_id {
	uint32_t device_type:16;
	uint32_t enum_id:16;	/* 1 based enum */
	uint16_t raw_device_tag;
};

struct connector_device_tag_info {
	uint32_t acpi_device;
	struct device_id dev_id;
};

union dpcd_rev {
	struct {
		uint8_t MINOR:4;
		uint8_t MAJOR:4;
	} bits;
	uint8_t raw;
};

union max_lane_count {
	struct {
		uint8_t MAX_LANE_COUNT:5;
		uint8_t POST_LT_ADJ_REQ_SUPPORTED:1;
		uint8_t TPS3_SUPPORTED:1;
		uint8_t ENHANCED_FRAME_CAP:1;
	} bits;
	uint8_t raw;
};

union max_down_spread {
	struct {
		uint8_t MAX_DOWN_SPREAD:1;
		uint8_t RESERVED:5;
		uint8_t NO_AUX_HANDSHAKE_LINK_TRAINING:1;
		uint8_t TPS4_SUPPORTED:1;
	} bits;
	uint8_t raw;
};

union dprx_feature {
	struct {
		uint8_t GTC_CAP:1;                             // bit 0: DP 1.3+
		uint8_t SST_SPLIT_SDP_CAP:1;                   // bit 1: DP 1.4
		uint8_t AV_SYNC_CAP:1;                         // bit 2: DP 1.3+
		uint8_t VSC_SDP_COLORIMETRY_SUPPORTED:1;       // bit 3: DP 1.3+
		uint8_t VSC_EXT_VESA_SDP_SUPPORTED:1;          // bit 4: DP 1.4
		uint8_t VSC_EXT_VESA_SDP_CHAINING_SUPPORTED:1; // bit 5: DP 1.4
		uint8_t VSC_EXT_CEA_SDP_SUPPORTED:1;           // bit 6: DP 1.4
		uint8_t VSC_EXT_CEA_SDP_CHAINING_SUPPORTED:1;  // bit 7: DP 1.4
	} bits;
	uint8_t raw;
};

union sink_count {
	struct {
		uint8_t SINK_COUNT:6;
		uint8_t CPREADY:1;
		uint8_t RESERVED:1;
	} bits;
	uint8_t raw;
};

struct dc_dongle_caps {
	/* dongle type (DP converter, CV smart dongle) */
	uint32_t dongle_type;
	bool extendedCapValid;
	/* If dongle_type == DISPLAY_DONGLE_DP_HDMI_CONVERTER,
	indicates 'Frame Sequential-to-lllFrame Pack' conversion capability.*/
	bool is_dp_hdmi_s3d_converter;
	bool is_dp_hdmi_ycbcr422_pass_through;
	bool is_dp_hdmi_ycbcr420_pass_through;
	bool is_dp_hdmi_ycbcr422_converter;
	bool is_dp_hdmi_ycbcr420_converter;
	uint32_t dp_hdmi_max_bpc;
	uint32_t dp_hdmi_max_pixel_clk_in_khz;
};

union dpcd_fec_capability {
	struct {
		uint8_t FEC_CAPABLE:1;
		uint8_t UNCORRECTED_BLOCK_ERROR_COUNT_CAPABLE:1;
		uint8_t CORRECTED_BLOCK_ERROR_COUNT_CAPABLE:1;
		uint8_t BIT_ERROR_COUNT_CAPABLE:1;
		uint8_t RESERVED:4;
	} bits;
	uint8_t raw;
};

union dpcd_dsc_branch_decoder_capabilities {
	struct {
		uint8_t BRANCH_OVERALL_THROUGHPUT_0;
		uint8_t BRANCH_OVERALL_THROUGHPUT_1;
		uint8_t BRANCH_MAX_LINE_WIDTH;
	} fields;
	uint8_t raw[3];
};

struct dpcd_dsc_support {
	uint8_t DSC_SUPPORT		:1;
	uint8_t DSC_PASSTHROUGH_SUPPORT	:1;
	uint8_t RESERVED		:6;
};

struct dpcd_dsc_algorithm_revision {
	uint8_t DSC_VERSION_MAJOR	:4;
	uint8_t DSC_VERSION_MINOR	:4;
};

struct dpcd_dsc_rc_buffer_block_size {
	uint8_t RC_BLOCK_BUFFER_SIZE	:2;
	uint8_t RESERVED		:6;
};

struct dpcd_dsc_slice_capability1 {
	uint8_t ONE_SLICE_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t TWO_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t RESERVED				:1;
	uint8_t FOUR_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t SIX_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t EIGHT_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t TEN_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
	uint8_t TWELVE_SLICES_PER_DP_DSC_SINK_DEVICE	:1;
};

struct dpcd_dsc_line_buffer_bit_depth {
	uint8_t LINE_BUFFER_BIT_DEPTH	:4;
	uint8_t RESERVED		:4;
};

struct dpcd_dsc_block_prediction_support {
	uint8_t BLOCK_PREDICTION_SUPPORT:1;
	uint8_t RESERVED		:7;
};

struct dpcd_maximum_bits_per_pixel_supported_by_the_decompressor {
	uint8_t MAXIMUM_BITS_PER_PIXEL_SUPPORTED_BY_THE_DECOMPRESSOR_LOW	:7;
	uint8_t MAXIMUM_BITS_PER_PIXEL_SUPPORTED_BY_THE_DECOMPRESSOR_HIGH	:7;
	uint8_t RESERVED							:2;
};

struct dpcd_dsc_decoder_color_format_capabilities {
	uint8_t RGB_SUPPORT			:1;
	uint8_t Y_CB_CR_444_SUPPORT		:1;
	uint8_t Y_CB_CR_SIMPLE_422_SUPPORT	:1;
	uint8_t Y_CB_CR_NATIVE_422_SUPPORT	:1;
	uint8_t Y_CB_CR_NATIVE_420_SUPPORT	:1;
	uint8_t RESERVED			:3;
};

struct dpcd_dsc_decoder_color_depth_capabilities {
	uint8_t RESERVED0			:1;
	uint8_t EIGHT_BITS_PER_COLOR_SUPPORT	:1;
	uint8_t TEN_BITS_PER_COLOR_SUPPORT	:1;
	uint8_t TWELVE_BITS_PER_COLOR_SUPPORT	:1;
	uint8_t RESERVED1			:4;
};

struct dpcd_peak_dsc_throughput_dsc_sink {
	uint8_t THROUGHPUT_MODE_0:4;
	uint8_t THROUGHPUT_MODE_1:4;
};

struct dpcd_dsc_slice_capabilities_2 {
	uint8_t SIXTEEN_SLICES_PER_DSC_SINK_DEVICE	:1;
	uint8_t TWENTY_SLICES_PER_DSC_SINK_DEVICE	:1;
	uint8_t TWENTYFOUR_SLICES_PER_DSC_SINK_DEVICE	:1;
	uint8_t RESERVED				:5;
};

struct dpcd_bits_per_pixel_increment{
	uint8_t INCREMENT_OF_BITS_PER_PIXEL_SUPPORTED	:3;
	uint8_t RESERVED				:5;
};
union dpcd_dsc_basic_capabilities {
	struct {
		struct dpcd_dsc_support dsc_support;
		struct dpcd_dsc_algorithm_revision dsc_algorithm_revision;
		struct dpcd_dsc_rc_buffer_block_size dsc_rc_buffer_block_size;
		uint8_t dsc_rc_buffer_size;
		struct dpcd_dsc_slice_capability1 dsc_slice_capabilities_1;
		struct dpcd_dsc_line_buffer_bit_depth dsc_line_buffer_bit_depth;
		struct dpcd_dsc_block_prediction_support dsc_block_prediction_support;
		struct dpcd_maximum_bits_per_pixel_supported_by_the_decompressor maximum_bits_per_pixel_supported_by_the_decompressor;
		struct dpcd_dsc_decoder_color_format_capabilities dsc_decoder_color_format_capabilities;
		struct dpcd_dsc_decoder_color_depth_capabilities dsc_decoder_color_depth_capabilities;
		struct dpcd_peak_dsc_throughput_dsc_sink peak_dsc_throughput_dsc_sink;
		uint8_t dsc_maximum_slice_width;
		struct dpcd_dsc_slice_capabilities_2 dsc_slice_capabilities_2;
		uint8_t reserved;
		struct dpcd_bits_per_pixel_increment bits_per_pixel_increment;
	} fields;
	uint8_t raw[16];
};

struct dpcd_dsc_capabilities {
	union dpcd_dsc_basic_capabilities dsc_basic_caps;
	union dpcd_dsc_branch_decoder_capabilities dsc_branch_decoder_caps;
};

#define MAX_REPEATER_CNT 8

struct dc_lttpr_caps {
	union dpcd_rev revision;
	uint8_t mode;
	uint8_t max_lane_count;
	uint8_t max_link_rate;
	uint8_t phy_repeater_cnt;
	uint8_t max_ext_timeout;
	uint8_t aux_rd_interval[MAX_REPEATER_CNT - 1];
};

struct psr_caps {
	unsigned char psr_version;
	unsigned int psr_rfb_setup_time;
	bool psr_exit_link_training_required;
};

struct dpcd_caps {
	union dpcd_rev dpcd_rev;
	union max_lane_count max_ln_count;
	union max_down_spread max_down_spread;
	union dprx_feature dprx_feature;

	/* valid only for eDP v1.4 or higher*/
	uint8_t edp_supported_link_rates_count;
	uint32_t edp_supported_link_rates[8];

	/* dongle type (DP converter, CV smart dongle) */
	uint32_t dongle_type;
	/* branch device or sink device */
	bool is_branch_dev;
	/* Dongle's downstream count. */
	union sink_count sink_count;
	/* If dongle_type == DISPLAY_DONGLE_DP_HDMI_CONVERTER,
	indicates 'Frame Sequential-to-lllFrame Pack' conversion capability.*/
	struct dc_dongle_caps dongle_caps;

	uint32_t sink_dev_id;
	int8_t sink_dev_id_str[6];
	int8_t sink_hw_revision;
	int8_t sink_fw_revision[2];

	uint32_t branch_dev_id;
	int8_t branch_dev_name[6];
	int8_t branch_hw_revision;
	int8_t branch_fw_revision[2];

	bool allow_invalid_MSA_timing_param;
	bool panel_mode_edp;
	bool dpcd_display_control_capable;
	bool ext_receiver_cap_field_present;
	bool dynamic_backlight_capable_edp;
	union dpcd_fec_capability fec_cap;
	struct dpcd_dsc_capabilities dsc_caps;
	struct dc_lttpr_caps lttpr_caps;
	struct psr_caps psr_caps;

};

union dpcd_sink_ext_caps {
	struct {
		/* 0 - Sink supports backlight adjust via PWM during SDR/HDR mode
		 * 1 - Sink supports backlight adjust via AUX during SDR/HDR mode.
		 */
		uint8_t sdr_aux_backlight_control : 1;
		uint8_t hdr_aux_backlight_control : 1;
		uint8_t reserved_1 : 2;
		uint8_t oled : 1;
		uint8_t reserved : 3;
	} bits;
	uint8_t raw;
};

struct dc;

typedef struct {
	void *remote_sinks[MAX_SINKS_PER_LINK];
	unsigned int sink_count;
	void *local_sink;
	unsigned int link_index;
	uint32_t type;
	uint32_t connector_signal;
	uint32_t irq_source_hpd;
	uint32_t irq_source_hpd_rx;/* aka DP Short Pulse  */
	bool is_hpd_filter_disabled;
	bool dp_ss_off;
	bool link_state_valid;
	bool aux_access_disabled;
	bool sync_lt_in_progress;
	uint32_t lttpr_mode;
	bool is_internal_display;

	/* TODO: Rename. Flag an endpoint as having a programmable mapping to a
	 * DIG encoder. */
	bool is_dig_mapping_flexible;
	bool hpd_status; /* HPD status of link without physical HPD pin. */

	bool edp_sink_present;

	/* caps is the same as reported_link_cap. link_traing use
	 * reported_link_cap. Will clean up.  TODO
	 */
	struct dc_link_settings reported_link_cap;
	struct dc_link_settings verified_link_cap;
	struct dc_link_settings cur_link_settings;
	struct dc_lane_settings cur_lane_setting;
	struct dc_link_settings preferred_link_setting;
	struct dc_link_training_overrides preferred_training_settings;
	struct dp_audio_test_data audio_test_data;

	uint8_t ddc_hw_inst;

	uint8_t hpd_src;

	uint8_t link_enc_hw_inst;
	/* DIG link encoder ID. Used as index in link encoder resource pool.
	 * For links with fixed mapping to DIG, this is not changed after dc_link
	 * object creation.
	 */
	uint32_t eng_id;

	bool test_pattern_enabled;
	union compliance_test_state compliance_test_state;

	void *priv;

	void *ddc;

	bool aux_mode;

	/* Private to DC core */

	struct dc *dc;

	void *ctx;

	void *panel_cntl;
	void *link_enc;
	struct graphics_object_id link_id;
	/* Endpoint type distinguishes display endpoints which do not have entries
	 * in the BIOS connector table from those that do. Helps when tracking link
	 * encoder to display endpoint assignments.
	 */
	uint32_t ep_type;
	union ddi_channel_mapping ddi_channel_mapping;
	struct connector_device_tag_info device_tag;
	struct dpcd_caps dpcd_caps;
	uint32_t dongle_max_pix_clk;
	unsigned short chip_caps;
	unsigned int dpcd_sink_count;
//#if defined(CONFIG_DRM_AMD_DC_HDCP)
//	struct hdcp_caps hdcp_caps;
//#endif
	uint32_t edp_revision;
	union dpcd_sink_ext_caps dpcd_sink_ext_caps;

	/*
	struct psr_settings psr_settings;

	// MST record stream using this link
	struct link_flags {
		bool dp_keep_receiver_powered;
		bool dp_skip_DID2;
		bool dp_skip_reset_segment;
	} wa_flags;
	struct link_mst_stream_allocation_table mst_stream_alloc_table;

	struct dc_link_status link_status;

	struct link_trace link_trace;
	struct gpio *hpd_gpio;
	enum dc_link_fec_state fec_state;
	 */
} my_dc_link_t;

typedef struct {
	void* plane_state;
	struct dc_stream_state *stream;
} my_pipe_ctx_t;

struct panel_cntl_backlight_registers {
	unsigned int BL_PWM_CNTL;
	unsigned int BL_PWM_CNTL2;
	unsigned int BL_PWM_PERIOD_CNTL;
	unsigned int LVTMA_PWRSEQ_REF_DIV_BL_PWM_REF_DIV;
};

struct hw_asic_id {
	uint32_t chip_id;
	uint32_t chip_family;
	uint32_t pci_revision_id;
	uint32_t hw_internal_rev;
	uint32_t vram_type;
	uint32_t vram_width;
	uint32_t feature_flags;
	uint32_t fake_paths_num;
	void *atombios_base_address;
};

struct dc_vram_info {
	unsigned int num_chans;
	unsigned int dram_channel_width_bytes;
};

struct dc_golden_table {
	uint16_t dc_golden_table_ver;
	uint32_t aux_dphy_rx_control0_val;
	uint32_t aux_dphy_tx_control_val;
	uint32_t aux_dphy_rx_control1_val;
	uint32_t dc_gpio_aux_ctrl_0_val;
	uint32_t dc_gpio_aux_ctrl_1_val;
	uint32_t dc_gpio_aux_ctrl_2_val;
	uint32_t dc_gpio_aux_ctrl_3_val;
	uint32_t dc_gpio_aux_ctrl_4_val;
	uint32_t dc_gpio_aux_ctrl_5_val;
};

struct dc_firmware_info {
	struct pll_info {
		uint32_t crystal_frequency; /* in KHz */
		uint32_t min_input_pxl_clk_pll_frequency; /* in KHz */
		uint32_t max_input_pxl_clk_pll_frequency; /* in KHz */
		uint32_t min_output_pxl_clk_pll_frequency; /* in KHz */
		uint32_t max_output_pxl_clk_pll_frequency; /* in KHz */
	} pll_info;

	struct firmware_feature {
		uint32_t memory_clk_ss_percentage;
		uint32_t engine_clk_ss_percentage;
	} feature;

	uint32_t default_display_engine_pll_frequency; /* in KHz */
	uint32_t external_clock_source_frequency_for_dp; /* in KHz */
	uint32_t smu_gpu_pll_output_freq; /* in KHz */
	uint8_t min_allowed_bl_level;
	uint8_t remote_display_config;
	uint32_t default_memory_clk; /* in KHz */
	uint32_t default_engine_clk; /* in KHz */
	uint32_t dp_phy_ref_clk; /* in KHz - DCE12 only */
	uint32_t i2c_engine_ref_clk; /* in KHz - DCE12 only */
	bool oem_i2c_present;
	uint8_t oem_i2c_obj_id;

};

#define NUMBER_OF_UCHAR_FOR_GUID 16
#define MAX_NUMBER_OF_EXT_DISPLAY_PATH 7
#define NUMBER_OF_CSR_M3_ARB 10
#define NUMBER_OF_DISP_CLK_VOLTAGE 4
#define NUMBER_OF_AVAILABLE_SCLK 5

struct i2c_reg_info {
	unsigned char       i2c_reg_index;
	unsigned char       i2c_reg_val;
};

struct edp_info {
	uint16_t edp_backlight_pwm_hz;
	uint16_t edp_ss_percentage;
	uint16_t edp_ss_rate_10hz;
	uint8_t  edp_pwr_on_off_delay;
	uint8_t  edp_pwr_on_vary_bl_to_blon;
	uint8_t  edp_pwr_down_bloff_to_vary_bloff;
	uint8_t  edp_panel_bpc;
	uint8_t  edp_bootup_bl_level;
};

struct integrated_info {
	struct clock_voltage_caps {
		/* The Voltage Index indicated by FUSE, same voltage index
		shared with SCLK DPM fuse table */
		uint32_t voltage_index;
		/* Maximum clock supported with specified voltage index */
		uint32_t max_supported_clk; /* in KHz */
	} disp_clk_voltage[NUMBER_OF_DISP_CLK_VOLTAGE];

	struct display_connection_info {
		struct external_display_path {
			/* A bit vector to show what devices are supported */
			uint32_t device_tag;
			/* 16bit device ACPI id. */
			uint32_t device_acpi_enum;
			/* A physical connector for displays to plug in,
			using object connector definitions */
			struct graphics_object_id device_connector_id;
			/* An index into external AUX/DDC channel LUT */
			uint8_t ext_aux_ddc_lut_index;
			/* An index into external HPD pin LUT */
			uint8_t ext_hpd_pin_lut_index;
			/* external encoder object id */
			struct graphics_object_id ext_encoder_obj_id;
			/* XBAR mapping of the PHY channels */
			union ddi_channel_mapping channel_mapping;

			unsigned short caps;
		} path[MAX_NUMBER_OF_EXT_DISPLAY_PATH];

		uint8_t gu_id[NUMBER_OF_UCHAR_FOR_GUID];
		uint8_t checksum;
	} ext_disp_conn_info; /* exiting long long time */

	struct available_s_clk_list {
		/* Maximum clock supported with specified voltage index */
		uint32_t supported_s_clk; /* in KHz */
		/* The Voltage Index indicated by FUSE for specified SCLK */
		uint32_t voltage_index;
		/* The Voltage ID indicated by FUSE for specified SCLK */
		uint32_t voltage_id;
	} avail_s_clk[NUMBER_OF_AVAILABLE_SCLK];

	uint8_t memory_type;
	uint8_t ma_channel_number;
	uint32_t boot_up_engine_clock; /* in KHz */
	uint32_t dentist_vco_freq; /* in KHz */
	uint32_t boot_up_uma_clock; /* in KHz */
	uint32_t boot_up_req_display_vector;
	uint32_t other_display_misc;
	uint32_t gpu_cap_info;
	uint32_t sb_mmio_base_addr;
	uint32_t system_config;
	uint32_t cpu_cap_info;
	uint32_t max_nb_voltage;
	uint32_t min_nb_voltage;
	uint32_t boot_up_nb_voltage;
	uint32_t ext_disp_conn_info_offset;
	uint32_t csr_m3_arb_cntl_default[NUMBER_OF_CSR_M3_ARB];
	uint32_t csr_m3_arb_cntl_uvd[NUMBER_OF_CSR_M3_ARB];
	uint32_t csr_m3_arb_cntl_fs3d[NUMBER_OF_CSR_M3_ARB];
	uint32_t gmc_restore_reset_time;
	uint32_t minimum_n_clk;
	uint32_t idle_n_clk;
	uint32_t ddr_dll_power_up_time;
	uint32_t ddr_pll_power_up_time;
	/* start for V6 */
	uint32_t pcie_clk_ss_type;
	uint32_t lvds_ss_percentage;
	uint32_t lvds_sspread_rate_in_10hz;
	uint32_t hdmi_ss_percentage;
	uint32_t hdmi_sspread_rate_in_10hz;
	uint32_t dvi_ss_percentage;
	uint32_t dvi_sspread_rate_in_10_hz;
	uint32_t sclk_dpm_boost_margin;
	uint32_t sclk_dpm_throttle_margin;
	uint32_t sclk_dpm_tdp_limit_pg;
	uint32_t sclk_dpm_tdp_limit_boost;
	uint32_t boost_engine_clock;
	uint32_t boost_vid_2bit;
	uint32_t enable_boost;
	uint32_t gnb_tdp_limit;
	/* Start from V7 */
	uint32_t max_lvds_pclk_freq_in_single_link;
	uint32_t lvds_misc;
	uint32_t lvds_pwr_on_seq_dig_on_to_de_in_4ms;
	uint32_t lvds_pwr_on_seq_de_to_vary_bl_in_4ms;
	uint32_t lvds_pwr_off_seq_vary_bl_to_de_in4ms;
	uint32_t lvds_pwr_off_seq_de_to_dig_on_in4ms;
	uint32_t lvds_off_to_on_delay_in_4ms;
	uint32_t lvds_pwr_on_seq_vary_bl_to_blon_in_4ms;
	uint32_t lvds_pwr_off_seq_blon_to_vary_bl_in_4ms;
	uint32_t lvds_reserved1;
	uint32_t lvds_bit_depth_control_val;
	//Start from V9
	unsigned char dp0_ext_hdmi_slv_addr;
	unsigned char dp0_ext_hdmi_reg_num;
	struct i2c_reg_info dp0_ext_hdmi_reg_settings[9];
	unsigned char dp0_ext_hdmi_6g_reg_num;
	struct i2c_reg_info dp0_ext_hdmi_6g_reg_settings[3];
	unsigned char dp1_ext_hdmi_slv_addr;
	unsigned char dp1_ext_hdmi_reg_num;
	struct i2c_reg_info dp1_ext_hdmi_reg_settings[9];
	unsigned char dp1_ext_hdmi_6g_reg_num;
	struct i2c_reg_info dp1_ext_hdmi_6g_reg_settings[3];
	unsigned char dp2_ext_hdmi_slv_addr;
	unsigned char dp2_ext_hdmi_reg_num;
	struct i2c_reg_info dp2_ext_hdmi_reg_settings[9];
	unsigned char dp2_ext_hdmi_6g_reg_num;
	struct i2c_reg_info dp2_ext_hdmi_6g_reg_settings[3];
	unsigned char dp3_ext_hdmi_slv_addr;
	unsigned char dp3_ext_hdmi_reg_num;
	struct i2c_reg_info dp3_ext_hdmi_reg_settings[9];
	unsigned char dp3_ext_hdmi_6g_reg_num;
	struct i2c_reg_info dp3_ext_hdmi_6g_reg_settings[3];
	/* V11 */
	uint32_t dp_ss_control;
	/* V2.1 */
	struct edp_info edp1_info;
	struct edp_info edp2_info;
};

struct dc_bios {
	void *funcs;

	uint8_t *bios;
	uint32_t bios_size;

	uint8_t *bios_local_image;

	void *ctx;
	void *regs;
	struct integrated_info *integrated_info;
	struct dc_firmware_info fw_info;
	bool fw_info_valid;
	struct dc_vram_info vram_info;
	struct dc_golden_table golden_table;
};

struct dmcu_version {
	unsigned int interface_version;
	unsigned int abm_version;
	unsigned int psr_version;
	unsigned int build_version;
};

struct dc_versions {
	const char *dc_ver;
	struct dmcu_version dmcu_version;
};

#define MAX_PLANES 6

struct dc_plane_cap {
	uint32_t type;
	uint32_t blends_with_above : 1;
	uint32_t blends_with_below : 1;
	uint32_t per_pixel_alpha : 1;
	struct {
		uint32_t argb8888 : 1;
		uint32_t nv12 : 1;
		uint32_t fp16 : 1;
		uint32_t p010 : 1;
		uint32_t ayuv : 1;
	} pixel_format_support;
	// max upscaling factor x1000
	// upscaling factors are always >= 1
	// for example, 1080p -> 8K is 4.0, or 4000 raw value
	struct {
		uint32_t argb8888;
		uint32_t nv12;
		uint32_t fp16;
	} max_upscale_factor;
	// max downscale factor x1000
	// downscale factors are always <= 1
	// for example, 8K -> 1080p is 0.25, or 250 raw value
	struct {
		uint32_t argb8888;
		uint32_t nv12;
		uint32_t fp16;
	} max_downscale_factor;
	// minimal width/height
	uint32_t min_width;
	uint32_t min_height;
};

struct rom_curve_caps {
	uint16_t srgb : 1;
	uint16_t bt2020 : 1;
	uint16_t gamma2_2 : 1;
	uint16_t pq : 1;
	uint16_t hlg : 1;
};

struct dpp_color_caps {
	uint16_t dcn_arch : 1; // all DCE generations treated the same
	// input lut is different than most LUTs, just plain 256-entry lookup
	uint16_t input_lut_shared : 1; // shared with DGAM
	uint16_t icsc : 1;
	uint16_t dgam_ram : 1;
	uint16_t post_csc : 1; // before gamut remap
	uint16_t gamma_corr : 1;

	// hdr_mult and gamut remap always available in DPP (in that order)
	// 3d lut implies shaper LUT,
	// it may be shared with MPC - check MPC:shared_3d_lut flag
	uint16_t hw_3d_lut : 1;
	uint16_t ogam_ram : 1; // blnd gam
	uint16_t ocsc : 1;
	uint16_t dgam_rom_for_yuv : 1;
	struct rom_curve_caps dgam_rom_caps;
	struct rom_curve_caps ogam_rom_caps;
};

struct mpc_color_caps {
	uint16_t gamut_remap : 1;
	uint16_t ogam_ram : 1;
	uint16_t ocsc : 1;
	uint16_t num_3dluts : 3; //3d lut always assumes a preceding shaper LUT
	uint16_t shared_3d_lut:1; //can be in either DPP or MPC, but single instance

	struct rom_curve_caps ogam_rom_caps;
};

struct dc_color_caps {
	struct dpp_color_caps dpp;
	struct mpc_color_caps mpc;
};

struct dc_caps {
	uint32_t max_streams;
	uint32_t max_links;
	uint32_t max_audios;
	uint32_t max_slave_planes;
	uint32_t max_slave_yuv_planes;
	uint32_t max_slave_rgb_planes;
	uint32_t max_planes;
	uint32_t max_downscale_ratio;
	uint32_t i2c_speed_in_khz;
	uint32_t i2c_speed_in_khz_hdcp;
	uint32_t dmdata_alloc_size;
	unsigned int max_cursor_size;
	unsigned int max_video_width;
	unsigned int min_horizontal_blanking_period;
	int linear_pitch_alignment;
	bool dcc_const_color;
	bool dynamic_audio;
	bool is_apu;
	bool dual_link_dvi;
	bool post_blend_color_processing;
	bool force_dp_tps4_for_cp2520;
	bool disable_dp_clk_share;
	bool psp_setup_panel_mode;
	bool extended_aux_timeout_support;
	bool dmcub_support;
	uint32_t num_of_internal_disp;
	uint32_t max_dp_protocol_version;
	unsigned int mall_size_per_mem_channel;
	unsigned int mall_size_total;
	unsigned int cursor_cache_size;
	struct dc_plane_cap planes[MAX_PLANES];
	struct dc_color_caps color;
	bool vbios_lttpr_aware;
	bool vbios_lttpr_enable;
};

struct dc_cap_funcs {
	bool (*get_dcc_compression_cap)(const struct dc *dc,
			const struct dc_dcc_surface_param *input,
			struct dc_surface_dcc_cap *output);
};

struct dc_config {
	bool gpu_vm_support;
	bool disable_disp_pll_sharing;
	bool fbc_support;
	bool disable_fractional_pwm;
	bool allow_seamless_boot_optimization;
	bool power_down_display_on_boot;
	bool edp_not_connected;
	bool edp_no_power_sequencing;
	bool force_enum_edp;
	bool forced_clocks;
	bool allow_lttpr_non_transparent_mode;
	bool multi_mon_pp_mclk_switch;
	bool disable_dmcu;
	bool enable_4to1MPC;
	bool allow_edp_hotplug_detection;
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	bool clamp_min_dcfclk;
//#endif
	uint64_t vblank_alignment_dto_params;
	uint8_t  vblank_alignment_max_frame_time_diff;
	bool is_asymmetric_memory;
	bool is_single_rank_dimm;
};

struct dc_bw_validation_profile {
	bool enable;

	unsigned long long total_ticks;
	unsigned long long voltage_level_ticks;
	unsigned long long watermark_ticks;
	unsigned long long rq_dlg_ticks;

	unsigned long long total_count;
	unsigned long long skip_fast_count;
	unsigned long long skip_pass_count;
	unsigned long long skip_fail_count;
};

union mem_low_power_enable_options {
	struct {
		bool vga: 1;
		bool i2c: 1;
		bool dmcu: 1;
		bool dscl: 1;
		bool cm: 1;
		bool mpc: 1;
		bool optc: 1;
	} bits;
	uint32_t u32All;
};

struct dc_debug_options {
	uint32_t visual_confirm;
	bool sanity_checks;
	bool max_disp_clk;
	bool surface_trace;
	bool timing_trace;
	bool clock_trace;
	bool validation_trace;
	bool bandwidth_calcs_trace;
	int max_downscale_src_width;

	/* stutter efficiency related */
	bool disable_stutter;
	bool use_max_lb;
	uint32_t disable_dcc;
	uint32_t pipe_split_policy;
	bool force_single_disp_pipe_split;
	bool voltage_align_fclk;
	bool disable_min_fclk;

	bool disable_dfs_bypass;
	bool disable_dpp_power_gate;
	bool disable_hubp_power_gate;
	bool disable_dsc_power_gate;
	int dsc_min_slice_height_override;
	int dsc_bpp_increment_div;
	bool native422_support;
	bool disable_pplib_wm_range;
	uint32_t pplib_wm_report_mode;
	unsigned int min_disp_clk_khz;
	unsigned int min_dpp_clk_khz;
	int sr_exit_time_dpm0_ns;
	int sr_enter_plus_exit_time_dpm0_ns;
	int sr_exit_time_ns;
	int sr_enter_plus_exit_time_ns;
	int urgent_latency_ns;
	uint32_t underflow_assert_delay_us;
	int percent_of_ideal_drambw;
	int dram_clock_change_latency_ns;
	bool optimized_watermark;
	int always_scale;
	bool disable_pplib_clock_request;
	bool disable_clock_gate;
	bool disable_mem_low_power;
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	bool pstate_enabled;
//#endif
	bool disable_dmcu;
	bool disable_psr;
	bool force_abm_enable;
	bool disable_stereo_support;
	bool vsr_support;
	bool performance_trace;
	bool az_endpoint_mute_only;
	bool always_use_regamma;
	bool recovery_enabled;
	bool avoid_vbios_exec_table;
	bool scl_reset_length10;
	bool hdmi20_disable;
	bool skip_detection_link_training;
	uint32_t edid_read_retry_times;
	bool remove_disconnect_edp;
	unsigned int force_odm_combine; //bit vector based on otg inst
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	unsigned int force_odm_combine_4to1; //bit vector based on otg inst
//	bool disable_z9_mpc;
//#endif
	unsigned int force_fclk_khz;
	bool enable_tri_buf;
	bool dmub_offload_enabled;
	bool dmcub_emulation;
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	bool disable_idle_power_optimizations;
//	unsigned int mall_size_override;
//	unsigned int mall_additional_timer_percent;
//	bool mall_error_as_fatal;
//#endif
	bool dmub_command_table; /* for testing only */
	struct dc_bw_validation_profile bw_val_profile;
	bool disable_fec;
	bool disable_48mhz_pwrdwn;
	/* This forces a hard min on the DCFCLK requested to SMU/PP
	 * watermarks are not affected.
	 */
	unsigned int force_min_dcfclk_mhz;
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	int dwb_fi_phase;
//#endif
	bool disable_timing_sync;
	bool cm_in_bypass;
	int force_clock_mode;/*every mode change.*/

	bool disable_dram_clock_change_vactive_support;
	bool validate_dml_output;
	bool enable_dmcub_surface_flip;
	bool usbc_combo_phy_reset_wa;
	bool disable_dsc;
	bool enable_dram_clock_change_one_display_vactive;
	union mem_low_power_enable_options enable_mem_low_power;
	bool force_vblank_alignment;

	/* Enable dmub aux for legacy ddc */
	bool enable_dmub_aux_for_legacy_ddc;
	bool optimize_edp_link_rate; /* eDP ILR */
	/* force enable edp FEC */
	bool force_enable_edp_fec;
	/* FEC/PSR1 sequence enable delay in 100us */
	uint8_t fec_enable_delay_in100us;
//#if defined(CONFIG_DRM_AMD_DC_DCN)
//	bool disable_z10;
//	bool enable_sw_cntl_psr;
//#endif
};

struct dc_bounding_box_overrides {
	int sr_exit_time_ns;
	int sr_enter_plus_exit_time_ns;
	int urgent_latency_ns;
	int percent_of_ideal_drambw;
	int dram_clock_change_latency_ns;
	int dummy_clock_change_latency_ns;
	/* This forces a hard min on the DCFCLK we use
	 * for DML.  Unlike the debug option for forcing
	 * DCFCLK, this override affects watermark calculations
	 */
	int min_dcfclk_mhz;
};

struct dc_bug_wa {
	bool no_connect_phy_config;
	bool dedcn20_305_wa;
	bool skip_clock_update;
	bool lt_early_cr_pattern;
};

struct dc {
	struct dc_versions versions;
	struct dc_caps caps;
	struct dc_cap_funcs cap_funcs;
	struct dc_config config;
	struct dc_debug_options debug;
	struct dc_bounding_box_overrides bb_overrides;
	struct dc_bug_wa work_arounds;
	/*
	struct dc_context *ctx;
	struct dc_phy_addr_space_config vm_pa_config;

	uint8_t link_count;
	struct dc_link *links[MAX_PIPES * 2];

	struct dc_state *current_state;
	struct resource_pool *res_pool;

	struct clk_mgr *clk_mgr;

	// Display Engine Clock levels
	struct dm_pp_clock_levels sclk_lvls;

	// Inputs into BW and WM calculations.
	struct bw_calcs_dceip *bw_dceip;
	struct bw_calcs_vbios *bw_vbios;
#ifdef CONFIG_DRM_AMD_DC_DCN
	struct dcn_soc_bounding_box *dcn_soc;
	struct dcn_ip_params *dcn_ip;
	struct display_mode_lib dml;
#endif

	// HW functions
	struct hw_sequencer_funcs hwss;
	struct dce_hwseq *hwseq;

	// Require to optimize clocks and bandwidth for added/removed planes
	bool optimized_required;
	bool wm_optimized_required;
#if defined(CONFIG_DRM_AMD_DC_DCN)
	bool idle_optimizations_allowed;
#endif

	// Require to maintain clocks and bandwidth for UEFI enabled HW

	// FBC compressor
	struct compressor *fbc_compressor;

	struct dc_debug_data debug_data;
	struct dpcd_vendor_signature vendor_signature;

	const char *build_id;
	struct vm_helper *vm_helper;
	 */
};

struct dc_context {
	struct dc *dc;

	void *driver_context; /* e.g. amdgpu_device */
	void *perf_trace;
	void *cgs_device;

	uint32_t dce_environment;
	struct hw_asic_id asic_id;

	/* todo: below should probably move to dc.  to facilitate removal
	 * of AS we will store these here
	 */
	uint32_t dce_version;
	struct dc_bios *dc_bios;
	bool created_bios;
	void *gpio_service;
	uint32_t dc_sink_id_count;
	uint32_t dc_stream_id_count;
	uint32_t dc_edp_id_count;
	uint64_t fbc_gpu_addr;
	void *dmub_srv;
};

struct panel_cntl_funcs {
	void (*destroy)(struct panel_cntl **panel_cntl);
	uint32_t (*hw_init)(struct panel_cntl *panel_cntl);
	bool (*is_panel_backlight_on)(struct panel_cntl *panel_cntl);
	bool (*is_panel_powered_on)(struct panel_cntl *panel_cntl);
	void (*store_backlight_level)(struct panel_cntl *panel_cntl);
	void (*driver_set_backlight)(struct panel_cntl *panel_cntl,
			uint32_t backlight_pwm_u16_16);
	uint32_t (*get_current_backlight)(struct panel_cntl *panel_cntl);
};
struct panel_cntl {
	const struct panel_cntl_funcs *funcs;
	struct dc_context *ctx;
	uint32_t inst;
	/* registers setting needs to be saved and restored at InitBacklight */
	struct panel_cntl_backlight_registers stored_backlight_registers;
};
bool g_is_sleep = false;
uint64_t g_last_sleep_time = 0;
static my_dc_link_t* g_dc_link_ptr = NULL;
static mach_vm_address_t orig_dce110_edp_backlight_control;

static mach_vm_address_t orig_dce110_edp_power_control;
static void wrap_dce110_edp_power_control(my_dc_link_t *dc_link, bool power_up) {
	g_dc_link_ptr = dc_link;
	if (orig_dce110_edp_power_control) {
		FunctionCast(wrap_dce110_edp_power_control, orig_dce110_edp_power_control)(dc_link, power_up);
	}
	SYSLOG("igfx", "wrap_dce110_edp_power_control %d end - %p", power_up, g_dc_link_ptr);
}

static char wrap_dce110_edp_backlight_control(my_dc_link_t *that, uint8_t a2) {
	//SYSLOG("igfx", "wrap_dce110_edp_backlight_control start %p:%lu,%lu,%lu,%lu,%lu", that, on_or_off, a3, a4, a5, a6);
	g_dc_link_ptr = that;
	//if (g_dc_link_ptr && g_dc_link_ptr->dc) {
	//	SYSLOG("igfx", "dce110_edp_backlight_control disable_fractional_pwm: %u", g_dc_link_ptr->dc->config.disable_fractional_pwm);
	//}
	char ret = FunctionCast(wrap_dce110_edp_backlight_control, orig_dce110_edp_backlight_control)(that, a2);
	SYSLOG("igfx", "wrap_dce110_edp_backlight_control end - %p, %d", g_dc_link_ptr, ret);
	
	if (g_is_sleep) {
		g_is_sleep = false;
		
		if (g_dc_link_ptr) {
			wrap_dce110_edp_backlight_control(g_dc_link_ptr, false);
			wrap_dce110_edp_power_control(g_dc_link_ptr, false);
			IOSleep(200);
			wrap_dce110_edp_power_control(g_dc_link_ptr, true);
			wrap_dce110_edp_backlight_control(g_dc_link_ptr, true);
		}
	}
	
	return ret;
}

mach_vm_address_t g_x6000fb_address = 0;
size_t g_x6000fb_size = 0;

void search_for_dce_panel_cntl_hw_init() {
	if (g_x6000fb_address == 0 || g_x6000fb_size == 0) {
		return;
	}
	SYSLOG("igfx", "try to find _dce_panel_cntl_hw_init address %p, %zu", (void*)g_x6000fb_address, g_x6000fb_size);
	mach_vm_address_t dpchi_cal = g_x6000fb_address + 0x12DA0F;
	const uint8_t dpchi_expect[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x54, 0x53, 0x48, 0x83, 0xEC, 0x10, 0x48, 0x89, 0xFB, 0x4C, 0x8D };
	unsigned char* dpchi_cal_charp = (unsigned char*)dpchi_cal;
	bool expect_match = true;
	for (int i = 0; i < 20; i++) {
		SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
		if (dpchi_cal_charp[i] != dpchi_expect[i]) {
			SYSLOG("igfx", "_dce_panel_cntl_hw_init dismatch: %d => %x != %x", i, dpchi_cal_charp[i], dpchi_expect[i]);
			expect_match = false;
			//break;
		}
	}
	
	/*if (! expect_match) {
		// try search it
		for (int i = 0; i < 0x200000; i++) {
			dpchi_cal_charp = (unsigned char*)(g_x6000fb_address + i);
			int j = 0;
			for (j = 0; j < 20; j++) {
				//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
				if (dpchi_cal_charp[j] != dpchi_expect[j]) {
					SYSLOG("igfx", "_dce_panel_cntl_hw_init dismatch: %d => %x != %x", i, dpchi_cal_charp[i], dpchi_expect[i]);
					//expect_match = false;
					break;
				}
			}
			if (j == 20) {
				//SYSLOG("igfx", "found _dce_panel_cntl_hw_init address %u", i);
				dpchi_cal = g_x6000fb_address + i;
				expect_match = true;
			}
		}
	}
	
	if (expect_match) {
		SYSLOG("igfx", "found _dce_panel_cntl_hw_init address %p", (void*)dpchi_cal);
	} else {
		SYSLOG("igfx", "can not found _dce_panel_cntl_hw_init address");
	}*/
}

IOReturn RAD::wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute(IOService *framebuffer, IOIndex connectIndex, IOSelect attribute, uintptr_t value) {
	IOReturn ret = FunctionCast(wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute, callbackRAD->orgAMDRadeonX6000AmdRadeonFramebufferSetAttribute)(framebuffer, connectIndex, attribute, value);
	if (attribute != (UInt32)'bklt') {
		return ret;
	}
	
	if (callbackRAD->maxPwmBacklightLvl == 0) {
		DBGLOG("igfx", "wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute zero maxPwmBacklightLvl");
		return 0;
	}
	
	if (callbackRAD->panelCntlPtr == nullptr) {
		DBGLOG("igfx", "wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute null panel cntl");
		static int timmes = 0;
		if (timmes ++ > 20) {
			search_for_dce_panel_cntl_hw_init();
		}
		return 0;
	}
	
	if (callbackRAD->orgDceDriverSetBacklight == nullptr) {
		DBGLOG("igfx", "wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute null orgDcLinkSetBacklightLevel");
		return 0;
	}
	
	// set the backlight of AMD navi10 driver
	callbackRAD->curPwmBacklightLvl = (uint32_t)value;
	uint32_t btlper = callbackRAD->curPwmBacklightLvl*100.0 / callbackRAD->maxPwmBacklightLvl;
	if (btlper < 0) {
		btlper = 0;
	} else if (btlper > 100) {
		btlper = 100;
	}
	
	int pwmval = (int)((btlper / 100.0) * 0xFF) << 8;
	if (pwmval >= 0xFF00) {
		// This is from the dmcu_set_backlight_level function of Linux source
		// ...
		// if (backlight_pwm_u16_16 & 0x10000)
		// 	   backlight_8_bit = 0xFF;
		// else
		// 	   backlight_8_bit = (backlight_pwm_u16_16 >> 8) & 0xFF;
		// ...
		// The max brightness should have 0x10000 bit set
		pwmval = 0x1FF00;
	}
	
	/*if (g_dc_link_ptr) {
		__int64 v10 = 0;
		__int64 v9 = *(_QWORD *)(*(_QWORD *)((__int64)g_dc_link_ptr + 304) + 944LL);
		while ( true ) {
			__int64 stream = *(_QWORD *)(v9 + v10 + 496);
			if ( stream ) {
				if (*(_QWORD *)(stream + 8) == (__int64)g_dc_link_ptr) {
					__int64 pipe_ctx = (v9 + v10 + 488);
					//SYSLOG("igfx", "my_set_backlight_lvl %p:%p, %d, %d", g_dc_link_ptr, pipe_ctx, backlight_pwm_u16_16, ramp);
					_QWORD *a1 = (_QWORD *)pipe_ctx;
					__int64 v5 = *(_QWORD *)(a1[1] + 8LL);
					__int64 v6 = *(_QWORD *)(v5 + 320);
					struct panel_cntl *panel_cntl = (struct panel_cntl *)v6;
					SYSLOG("igfx", "panel_cntl: %p", panel_cntl);
					break;
				} else {
					SYSLOG("igfx", "steam link %lu != %p", *(_QWORD *)(stream + 8), g_dc_link_ptr);
				}
			}
			v10 += 1280LL;
			if ( v10 >= 7680 )
				break;
		}
	} else {
		DBGLOG("igfx", "null g_dc_link_ptr");
	}*/
	
	struct panel_cntl* ppc = (struct panel_cntl*)callbackRAD->panelCntlPtr;
	DBGLOG("igfx", "fb 0x%p, idx: %d, set brightness: 0x%p -> 0x%x, st: %d", framebuffer, connectIndex, callbackRAD->panelCntlPtr, pwmval, ppc->funcs->is_panel_powered_on(ppc));
	callbackRAD->orgDceDriverSetBacklight(callbackRAD->panelCntlPtr, pwmval);
	return 0;
}

uint32_t RAD::wrapDcePanelCntlHwInit(void *panel_cntl) {
	callbackRAD->panelCntlPtr = panel_cntl;
	callbackRAD->updatePwmMaxBrightnessFromInternalDisplay(); // read max brightness value from IOReg
	struct panel_cntl* ppc = (struct panel_cntl*)callbackRAD->panelCntlPtr;
	DBGLOG("igfx", "wrapDcePanelCntlHwInit: %p, bl: %d, pw: %d, bl: %d", panel_cntl, ppc->funcs->is_panel_backlight_on(ppc), ppc->funcs->is_panel_powered_on(ppc), ppc->funcs->get_current_backlight(ppc));
	uint32_t ret = FunctionCast(wrapDcePanelCntlHwInit, callbackRAD->orgDcePanelCntlHwInit)(panel_cntl);
	//if (ppc->funcs->is_panel_backlight_on(ppc) == 0 && ppc->funcs->is_panel_powered_on(ppc) == 1 && g_dc_link_ptr) {
		//wrap_dce110_edp_backlight_control(g_dc_link_ptr, 0);
		//wrap_dce110_edp_power_control(g_dc_link_ptr, false);
	//	g_is_sleep = true;
	//}
	DBGLOG("igfx", "wrapDcePanelCntlHwInit: %p - %u, st: %d", panel_cntl, ret, ppc->funcs->is_panel_powered_on(ppc));
	return ret;
}

IOReturn RAD::wrapAMDRadeonX6000AmdRadeonFramebufferGetAttribute(IOService *framebuffer, IOIndex connectIndex, IOSelect attribute, uintptr_t * value) {
	IOReturn ret = FunctionCast(wrapAMDRadeonX6000AmdRadeonFramebufferGetAttribute,
								callbackRAD->orgAMDRadeonX6000AmdRadeonFramebufferGetAttribute)(framebuffer, connectIndex, attribute, value);
	if (attribute == (UInt32)'bklt') {
		// enable the backlight feature of AMD navi10 driver
		*value = callbackRAD->curPwmBacklightLvl;
		ret = 0;
	}
	return ret;
}

static void dumpHex(const char* tag, void* ptr, size_t size = 128) {
	if (!ptr) {
		SYSLOG("rad", "%s is NULL", tag);
		return;
	}
	uint8_t* data = static_cast<uint8_t*>(ptr);
	for (size_t i = 0; i < size; i += 16) {
		char buf[128];
		int len = snprintf(buf, sizeof(buf), "%s +0x%02zx: ", tag, i);
		for (size_t j = 0; j < 16 && i + j < size; ++j)
			len += snprintf(buf + len, sizeof(buf) - len, "%02X ", data[i + j]);
		SYSLOG("rad", "%s", buf);
	}
}

/*
int RAD::wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval(void* self, unsigned int index, int32_t* outVal) {
	//SYSLOG("rad", "Hooked getVUpdateVBlankInterval - a1: %p, a2: %u, a3: %p", self, index, outVal);
	//if (outVal) {
	//	SYSLOG("rad", "VBlank Interval Value Before: %d", *outVal);
	//}
	if (index >= 6) {
		return 0xE0000001;
	}
	int ret = FunctionCast(wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval,
							   callbackRAD->origAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval)(self, index, outVal);
	//SYSLOG("rad", "getVUpdateVBlankInterval returned: %u, after: %u", ret, *outVal);
	return ret;
}
 */

int RAD::wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval(void* self, unsigned int index, int32_t* outVal) {
	SYSLOG("rad", "[VBlankInterval] Called - self: %p, pipe index: %u, outVal: %p", self, index, outVal);
	if (index >= 6) {
		if (outVal) *outVal = 16666; // fallback to 60Hz (in µs)
		SYSLOG("rad", "[VBlankInterval] Invalid pipe index: %u, defaulted to 60Hz", index);
		return 0; // 或者 0x0 表示成功
	}
	if (index >= 6) {
		//SYSLOG("rad", "[VBlankInterval] Invalid pipe index: %u", index);
		return 0xE0000001;
	}
	int ret = FunctionCast(wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval,
							   callbackRAD->origAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval)(self, index, outVal);
	SYSLOG("rad", "[VBlankInterval] Returned: 0x%X, VBlank Interval: %d", ret, outVal ? *outVal : -1);
	return ret;
}

/*
int64_t RAD::wrapAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming(void* self, char *a2, int64_t a3, double a4, double a5) {
	return FunctionCast(wrapAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming,
							   callbackRAD->origAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming)(self, a2, a3, a4, a5);
	//return kIOReturnSuccess;
}
 */

struct RadeonDetailedTiming {
	uint16_t unknown0;            // +0x00
	uint16_t unknown1;            // +0x02
	uint32_t unknown2;            // +0x04
	uint64_t unknown3;            // +0x08
	uint16_t h_active;            // +0x10
	uint16_t h_blank;             // +0x12
	uint16_t v_active;            // +0x14
	uint16_t v_blank;             // +0x16
	uint64_t reserved0;           // +0x18
	uint64_t reserved1;           // +0x20
	uint64_t reserved2;           // +0x28
	uint64_t reserved3;           // +0x30
	uint64_t reserved4;           // +0x38
	uint64_t reserved5;           // +0x40
	uint32_t pixel_clock;         // +0x54 → 💡 594000000
	uint32_t refresh_rate_raw;    // +0x58
	uint64_t more_unknowns[3];    // +0x5C..0x70
	uint16_t h_sync_offset;       // +0x70
	uint16_t h_sync_pulse;        // +0x72
	uint16_t v_sync_offset;       // +0x74
	uint16_t v_sync_pulse;        // +0x76
	uint16_t h_border;            // +0x78
	uint16_t v_border;            // +0x7A
	uint32_t misc_flags;          // +0x7C
	// ... rest is unknown, including color depth, signal type, etc.
};


int64_t RAD::wrapAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming(void* self, char *a2, int64_t a3, double a4, double a5) {
	// 手动拆分 double，避免在内核中使用 %.2f（禁用 FPU）
	int intHz1 = static_cast<int>(a4);
	int decHz1 = static_cast<int>((a4 - intHz1) * 100);

	int intHz2 = static_cast<int>(a5);
	int decHz2 = static_cast<int>((a5 - intHz2) * 100);

	SYSLOG("rad", "[validateDetailedTiming] Called - self: %p, a2: %p, a3: 0x%llx, a4: %d.%02d Hz, a5: %d.%02d Hz",
		   self, a2, a3, intHz1, decHz1, intHz2, decHz2);

	// === 新增 Pixel Clock 检查 ===
	uint32_t pixelClockHz = *reinterpret_cast<uint32_t *>(a2 + 0x54);
	//if (pixelClockHz == 594000000) {
	//	SYSLOG("rad", "[validateDetailedTiming] Rejecting pixel clock: %u Hz (594MHz)", pixelClockHz);
		//return 0xE00002C7; // kIOReturnUnsupported
	//} else {
		dumpHex("validateDetailedTiming: a2", a2, 204);
	//}
	
	uint16_t h_active = *reinterpret_cast<uint16_t *>(a2);
	if (h_active < 1000) {
		SYSLOG("rad", "[validateDetailedTiming] Rejecting h_active < 1000: %u", h_active);
		//return 0xE00002C7; // kIOReturnUnsupported
	}
	
	if (a4 > 61.0) {
		SYSLOG("rad", "[validateDetailedTiming] Rejecting high refresh rate a4: %.2f Hz", a4);
		return 0xE00002C7; // kIOReturnUnsupported
	}
	if (a5 > 61.0) {
		SYSLOG("rad", "[validateDetailedTiming] Rejecting high refresh rate a5: %.2f Hz", a5);
		return 0xE00002C7; // kIOReturnUnsupported
	}

	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming,
							   callbackRAD->origAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming)(self, a2, a3, a4, a5);

	SYSLOG("rad", "[validateDetailedTiming] Returned: 0x%llx", ret);
	return ret;
}


/*
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet(void* self, unsigned int a2, void* AmdDetailedTimingInformation, void* IOPixelInformation, char a5, int a6) {
	return FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyModeSet)(self, a2, AmdDetailedTimingInformation, IOPixelInformation, a5, a6);
}
 
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet(void* self, unsigned int pipe, void* timing, void* pixelInfo, char a5, int a6) {
	SYSLOG("rad", "[notifyModeSet] Called - self: %p, pipe: %u, timing: %p, pixelInfo: %p, a5: %d, a6: %d",
		   self, pipe, timing, pixelInfo, a5, a6);
	//dumpHex("notifyModeSet: timing", timing);
	//dumpHex("notifyModeSet: pixelInfo", pixelInfo);
	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyModeSet)(self, pipe, timing, pixelInfo, a5, a6);
	SYSLOG("rad", "[notifyModeSet] Returned: 0x%llx", ret);
	return ret;
}*/

void* g_amdLogger = nullptr;

struct AmdDetailedTimingInformation {
	uint32_t unused0[3];         // 0x00
	uint32_t hSyncInterval;      // 0x0C (hsi)
	uint32_t vSyncInterval;      // 0x10 (vsi)
	uint32_t scalerFlags;        // 0x14 (sclf)
	uint32_t hDisplayStart;      // 0x18
	uint32_t hSyncOffset;        // 0x1C
	uint32_t hSyncWidth;         // 0x20
	uint32_t vSyncOffset;        // 0x24
	uint32_t vSyncWidth;         // 0x28
	uint32_t reserved1;          // 0x2C
	uint64_t pixelClockHz;       // 0x30
	uint64_t refreshRateHz;      // 0x38
	uint64_t totalPixels;        // 0x40

	// These are usually display active area and blanking
	uint32_t hActive;            // 0x48
	uint32_t hBlank;             // 0x4C
	uint32_t vActive;            // 0x50
	uint32_t vBlank;             // 0x54

	// Horizontal total = ha + hb, Horizontal front/back porch = hso/hspw
	uint32_t hTotal;             // 0x58
	uint32_t hBackPorch;         // 0x5C
	uint32_t hSyncStart;         // 0x60
	uint32_t hSyncEnd;           // 0x64

	// Vertical timing
	uint32_t vTotal;             // 0x68
	uint32_t vBackPorch;         // 0x6C
	uint32_t vSyncStart;         // 0x70
	uint32_t vSyncEnd;           // 0x74

	// Sync polarity (bitfield?)
	uint8_t  hSyncPolarity;      // 0x78 (used for 'hsp' char, offset 112)
	uint8_t  reservedSync[3];    // 0x79–0x7B

	// Vertical sync polarity (vsp)
	uint8_t  vSyncPolarity;      // 0x7C (used for 'vsp' char, offset 120)
	uint8_t  reservedSync2[3];   // 0x7D–0x7F

	// Border pixels
	uint32_t borderLeft;         // 0x80
	uint32_t borderRight;        // 0x84
	uint32_t borderTop;          // 0x88
	uint32_t borderBottom;       // 0x8C

	// Link info
	uint32_t linkCount;          // 0x90 (offset 128)

	// Pixel encoding, bit depth, colorimetry, dynamic range
	uint16_t pixelEncoding;      // 0x94 (offset 136)
	uint16_t bitsPerComponent;   // 0x96 (offset 138)
	uint16_t colorimetry;        // 0x98 (offset 140)
	uint16_t dynamicRange;       // 0x9A (offset 142)

	// DSC info
	uint16_t dscBitsPerPixel;    // 0x9C (offset 144)
	uint16_t dscSliceHeight;     // 0x9E (offset 146)
	uint16_t dscSliceWidth;      // 0xA0 (offset 148)
};


int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet(
	void* self,
	unsigned int pipe,
	void* timing,
	void* pixelInfo,
	char a5,
	int a6
) {
	SYSLOG("rad", "[notifyModeSet] Called - self: %p, pipe: %u, timing: %p, pixelInfo: %p, a5: %d, a6: %d",
		   self, pipe, timing, pixelInfo, a5, a6);

	auto* t = reinterpret_cast<AmdDetailedTimingInformation*>(timing);
	auto* p = reinterpret_cast<IOPixelInformation*>(pixelInfo);

	if (t) {
		SYSLOG("rad", "[timing] hSyncInterval: %u, vSyncInterval: %u, scalerFlags: 0x%08X",
			   t->hSyncInterval, t->vSyncInterval, t->scalerFlags);

		SYSLOG("rad", "[timing] hDisplayStart: %u, hSyncOffset: %u, hSyncWidth: %u",
			   t->hDisplayStart, t->hSyncOffset, t->hSyncWidth);

		SYSLOG("rad", "[timing] vSyncOffset: %u, vSyncWidth: %u, reserved1: 0x%08X",
			   t->vSyncOffset, t->vSyncWidth, t->reserved1);

		SYSLOG("rad", "[timing] pixelClockHz: %llu, refreshRateHz: %llu, totalPixels: %llu",
			   t->pixelClockHz, t->refreshRateHz, t->totalPixels);

		SYSLOG("rad", "[timing] hActive: %u, hBlank: %u, vActive: %u, vBlank: %u",
			   t->hActive, t->hBlank, t->vActive, t->vBlank);

		SYSLOG("rad", "[timing] hTotal: %u, hBackPorch: %u, hSyncStart: %u, hSyncEnd: %u",
			   t->hTotal, t->hBackPorch, t->hSyncStart, t->hSyncEnd);

		SYSLOG("rad", "[timing] vTotal: %u, vBackPorch: %u, vSyncStart: %u, vSyncEnd: %u",
			   t->vTotal, t->vBackPorch, t->vSyncStart, t->vSyncEnd);

		SYSLOG("rad", "[timing] hSyncPolarity: %u, vSyncPolarity: %u",
			   t->hSyncPolarity, t->vSyncPolarity);

		SYSLOG("rad", "[timing] Borders: L:%u R:%u T:%u B:%u",
			   t->borderLeft, t->borderRight, t->borderTop, t->borderBottom);

		SYSLOG("rad", "[timing] linkCount: %u", t->linkCount);

		SYSLOG("rad", "[timing] PixelEncoding: %u, BitsPerComponent: %u, Colorimetry: %u, DynamicRange: %u",
			   t->pixelEncoding, t->bitsPerComponent, t->colorimetry, t->dynamicRange);

		SYSLOG("rad", "[timing] DSC: Bpp: %u, SliceHeight: %u, SliceWidth: %u",
			   t->dscBitsPerPixel, t->dscSliceHeight, t->dscSliceWidth);
	}

	if (p) {
		SYSLOG("rad", "[pixelInfo] bytesPerRow: %u, bytesPerPlane: %u, bitsPerPixel: %u, pixelType: %u",
			   p->bytesPerRow, p->bytesPerPlane, p->bitsPerPixel, p->pixelType);

		SYSLOG("rad", "[pixelInfo] componentCount: %u, bitsPerComponent: %u, pixelFormat: %u, flags: 0x%08X",
			   p->componentCount, p->bitsPerComponent, p->pixelFormat, p->flags);

		SYSLOG("rad", "[pixelInfo] activeWidth: %u, activeHeight: %u",
			   p->activeWidth, p->activeHeight);

		for (int i = 0; i < 8; ++i) {
			SYSLOG("rad", "[pixelInfo] componentMasks[%d]: low = 0x%08X, high = 0x%08X",
				   i, p->componentMasks[i * 2], p->componentMasks[i * 2 + 1]);
		}
	}
	
	if (g_amdLogger != nullptr) {
		callbackRAD->wrapAMDRadeonX6000AmdLoggerlogTiming(g_amdLogger, 0, 1, timing);
	}

	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyModeSet)(self, pipe, timing, pixelInfo, a5, a6);

	SYSLOG("rad", "[notifyModeSet] Returned: 0x%llx", ret);
	return ret;
}


/*
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyModeValidation(void* self, unsigned int a2, void* AmdDetailedTimingInformation, void* AmdFbDisplayPath) {
	return FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyModeValidation,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyModeValidation)(self, a2, AmdDetailedTimingInformation, AmdFbDisplayPath);
}
 */

struct AmdTimingInfo {
	uint32_t resolutionCode;      // 0x00 可能是某种 mode ID 或 encoded 分辨率
	uint64_t reserved0[4];        // 0x04–0x1F，未解码
	uint32_t refreshRateNum;      // 0x20 = 6720 (示例)
	uint32_t refreshRateDen;      // 0x24 = 3780
	uint32_t flags;               // 0x28 = 1
	uint64_t pixelClockHz;        // 0x2C = 1045230000 (0x3E4CF1B0)？
	uint64_t tbd1;                // 0x34
	uint64_t tbd2;                // 0x3C
};

int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyModeValidation(void* self, unsigned int pipe, void* timing, void* displayPath) {
	SYSLOG("rad", "[notifyModeValidation] Called - self: %p, pipe: %u, timing: %p, displayPath: %p", self, pipe, timing, displayPath);
	AmdTimingInfo *timingPtr = (AmdTimingInfo *)timing;
	uint64_t pixelClock = timingPtr->pixelClockHz;
	//if (pixelClock == 594000000) {
	//	SYSLOG("rad", "mute 594!!");
	//} else {
		dumpHex("notifyModeValidation: timing", timing);
		dumpHex("notifyModeValidation: displayPath", displayPath);
	//}
	if (pixelClock > 600000000) {
		SYSLOG("rad", "Blocking mode due to high refresh pixelClock: %llu", pixelClock);
		return 0xE00002C7; // kIOReturnUnsupported
	}
	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyModeValidation,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyModeValidation)(self, pipe, timing, displayPath);
	SYSLOG("rad", "[notifyModeValidation] Returned: 0x%llx", ret);
	return ret;
}

/*
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesoverrideTimingRange(void* self, void* AGDCFBOverrideTimingRange_t, int64_t a3) {
	return FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesoverrideTimingRange,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesoverrideTimingRange)(self, AGDCFBOverrideTimingRange_t, a3);
}
 */
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesoverrideTimingRange(void* self, void* timingRange, int64_t size) {
	SYSLOG("rad", "[overrideTimingRange] Called - self: %p, timingRange: %p, size: 0x%llx", self, timingRange, size);
	dumpHex("overrideTimingRange", timingRange, static_cast<size_t>(size > 64 ? 64 : size));
	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesoverrideTimingRange,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicesoverrideTimingRange)(self, timingRange, size);
	SYSLOG("rad", "[overrideTimingRange] Returned: 0x%llx", ret);
	return ret;
}

struct AGDCFBSetEDIDData_t {
	uint32_t fbIndex;               // a2[0]：帧缓冲编号
	uint32_t edidSize;              // a2[1]：EDID 数据大小，最大 1024（0x400）
	uint8_t  edidData[1024];        // a2[2...]: EDID 原始数据（典型为 128/256 字节）
	// uint8_t reserved[0];         // 由于总长度正好为 1032 字节，无需 padding
};
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicessetFbEdid(void* self, void* AGDCFBSetEDIDData_t_ptr, u_long a3) {
	auto* edidData = reinterpret_cast<uint8_t*>(AGDCFBSetEDIDData_t_ptr);
	uint32_t fbIndex = *reinterpret_cast<uint32_t*>(edidData);
	uint32_t edidSize = *reinterpret_cast<uint32_t*>(edidData + 4);

	SYSLOG("rad", "[setFbEdid] Called - self: %p, fbIndex: %u, edidSize: %u, a3: %lu",
		   self, fbIndex, edidSize, a3);

	// 动态打印 a3 字节，每行 16 字节
	const size_t totalBytes = static_cast<size_t>(a3);
	const size_t bytesPerLine = 16;
	char lineBuffer[128];

	for (size_t i = 0; i < totalBytes; i += bytesPerLine) {
		size_t len = snprintf(lineBuffer, sizeof(lineBuffer), "[setFbEdid] +%04zx: ", i);
		for (size_t j = 0; j < bytesPerLine && (i + j) < totalBytes; ++j) {
			len += snprintf(lineBuffer + len, sizeof(lineBuffer) - len, "%02X ", edidData[i + j]);
		}
		lineBuffer[sizeof(lineBuffer) - 1] = '\0'; // 保险终止
		SYSLOG("rad", "%s", lineBuffer);
	}

	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdAgdcServicessetFbEdid,
							   callbackRAD->origAMDRadeonX6000AmdAgdcServicessetFbEdid)(self, AGDCFBSetEDIDData_t_ptr, a3);
	SYSLOG("rad", "[setFbEdid] Returned: 0x%llx", ret);
	return ret;
}

// AMDRadeonX6000_AmdAgdcServices::setFbEdid(AGDCFBSetEDIDData_t *,ulong)
// __int64 __fastcall AMDRadeonX6000_AmdAgdcServices::setFbEdid(__int64 a1, _DWORD *a2, __int64 a3)


/*
bool wrapNotifyLinkChangeTmp(void *atiDeviceControl, kAGDCRegisterLinkControlEvent_t event, void *eventData, uint32_t eventFlags) {
	auto ret = FunctionCast(wrapNotifyLinkChange, callbackRAD->orgNotifyLinkChange)(atiDeviceControl, event, eventData, eventFlags);

	if (event == kAGDCValidateDetailedTiming) {
		auto cmd = static_cast<AGDCValidateDetailedTiming_t *>(eventData);
		DBGLOG("rad", "AGDCValidateDetailedTiming %u -> %d (%u)", cmd->framebufferIndex, ret, cmd->modeStatus);
		// While we have this condition below, the only actual value we get is ret = true, cmd->modeStatus = 0.
		// This is because AGDP is disabled, and starting from 10.15.1b2 AMDFramebuffer no longer accepts 0 in
		// __ZN14AMDFramebuffer22validateDetailedTimingEPvy
		if (ret == false || cmd->modeStatus < 1 || cmd->modeStatus > 3) {
			cmd->modeStatus = 2;
			ret = true;
		}
	}

	return ret;
}
 
 __int64 __fastcall AMDRadeonX6000_AmdAgdcServices::notifyLinkChange(__int64 a1, unsigned int a2, unsigned __int8 a3)
 {
   unsigned int v3; // r14d
   __int64 v5; // rax
   __int64 v6; // r15
   const char *v7; // rax
   __int64 v8; // rdx

   v5 = (*(__int64 (__fastcall **)(_QWORD))(**(_QWORD **)(a1 + 280) + 2336LL))(*(_QWORD *)(a1 + 280));
   if ( (*(unsigned __int8 (__fastcall **)(__int64, __int64))(*(_QWORD *)v5 + 280LL))(v5, 3LL) )
   {
	 v6 = *(_QWORD *)(a1 + 288);
	 v3 = 0;
	 v7 = (const char *)(*(__int64 (__fastcall **)(__int64, _QWORD))(*(_QWORD *)a1 + 904LL))(a1, 0LL);
	 if ( v6 )
	   (*(void (**)(__int64, __int64, __int64, const char *, ...))(*(_QWORD *)v6 + 288LL))(
		 v6,
		 11LL,
		 2LL,
		 "[AGDC] %s::%s() ??? DEBUG: Hotplug notification supressed.\n",
		 v7,
		 "notifyLinkChange");
	 else
	   MEMORY[0x1011A](
		 "[%s][AGDC] %s::%s() ??? DEBUG: Hotplug notification supressed.\n",
		 (const char *)(a1 + 304),
		 v7,
		 "notifyLinkChange");
	 return v3;
   }
   LOBYTE(v3) = 1;
   if ( a2 > 3 )
	 return v3;
   v8 = 0LL;
   if ( a3 != 0xFF )
	 v8 = a3 + 1LL;
   return (*(__int64 (__fastcall **)(__int64, _QWORD, __int64, _QWORD))(*(_QWORD *)a1 + 2224LL))(
			a1,
			dword_C20D4F0[a2],
			v8,
			0LL);
 }

int64_t AMDRadeonX6000_AmdAgdcServices_notifyLinkChange(void* self, unsigned int eventIndex, uint8_t param) {
	// 定义返回值变量，初始化为 1（成功）
	int result = 1;

	// 获取设备对象 device = this->device (偏移 +0x118)
	void* device = *(void**)((char*)self + 0x118);

	// 获取 device->getPowerGatingState()
	void* powerGatingObj = ((int64_t (**)(void*))(*(int64_t**)device + 2336LL))(device);

	// 调用 powerGatingObj->isPowerGated(3)
	bool isGated = ((bool (*)(void*, int64_t))(*(int64_t*)powerGatingObj + 280LL))(powerGatingObj, 3LL);

	if (isGated) {
		// 打印 "Hotplug notification suppressed" 日志
		void* logger = *(void**)((char*)self + 0x120);
		const char* className = ((const char* (*)(void*, uint64_t))(*(int64_t**)self + 904LL))(self, 0);

		if (logger) {
			((void (*)(void*, int64_t, int64_t, const char*, ...))(*(int64_t**)logger + 288LL))(
				logger, 11, 2, "[AGDC] %s::%s() ??? DEBUG: Hotplug notification supressed.\n",
				className, "notifyLinkChange");
		} else {
			// fallback 打印方式
			printf("[%s][AGDC] %s::%s() ??? DEBUG: Hotplug notification supressed.\n",
				   (char*)self + 0x130, className, "notifyLinkChange");
		}

		return 0;  // 被电源管理屏蔽，什么都不做
	}

	// 若事件类型大于支持的范围，直接返回 1
	if (eventIndex > 3)
		return result;

	// 将 param（如 modeStatus - 1）转换为 driver 参数
	int64_t arg2 = (param != 0xFF) ? (param + 1) : 0;

	// eventMap[eventIndex] 就是 dword_C20D4F0[eventIndex]
	static const uint32_t eventMap[] = {
		0,   // LinkInsert
		10,  // ValidateDetailedTiming
		2,   // LinkChange
		4    // LinkFramebuffer
	};

	uint32_t realEvent = eventMap[eventIndex];

	// 最终调用 AGDC 的处理函数（偏移 +0x8B0）
	return ((int64_t (*)(void*, uint32_t, int64_t, uint32_t))(*(int64_t**)self + 2224LL))(
		self, realEvent, arg2, 0);
}
*/
 
int64_t RAD::wrapAMDRadeonX6000AmdAgdcServicesnotifyLinkChange(void* self, unsigned int AmdLinkChangeEvent, uint8_t modeStatusMinusOne) {
	SYSLOG("rad", "notifyLinkChange called with event=%u, param=%u", AmdLinkChangeEvent, modeStatusMinusOne);
	// AGDCValidateDetailedTiming 事件编号通常是 1
	//if (AmdLinkChangeEvent == 1) {
	//	uint8_t modeStatus = modeStatusMinusOne + 1;
	//	if (modeStatus < 1 || modeStatus > 3) {
	//		SYSLOG("rad", "Invalid modeStatus %u detected, fixing to 2 (param = 1)", modeStatus);
	//		modeStatusMinusOne = 1; // 让 modeStatus = 2
	//	}
	//}

	return FunctionCast(wrapAMDRadeonX6000AmdAgdcServicesnotifyLinkChange, callbackRAD->origAMDRadeonX6000AmdAgdcServicesnotifyLinkChange)(self, AmdLinkChangeEvent, modeStatusMinusOne);
}

static mach_vm_address_t Orig_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts;
static int64_t Wrap_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts(__int64 a1, __int64 a2) {
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000_AmdInterruptManager_sleepInterrupts"" start " "%lld:%lld", a1, a2);
	uint64_t currNs = getCurrentTimeNs();
	if (currNs - g_last_sleep_time >= 1000000000) {
		g_is_sleep = true;
		g_last_sleep_time = getCurrentTimeNs();
	}
	int64_t ret = FunctionCast(Wrap_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts, Orig_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts)(a1, a2);
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000_AmdInterruptManager_sleepInterrupts"" end - " "%lld", ret);
	return ret;
}

static mach_vm_address_t Orig_AMDRadeonX6000_AmdLogger_setDebugLevel;
static int64_t Wrap_AMDRadeonX6000_AmdLogger_setDebugLevel(__int64 a1, int a2) {
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000_AmdLogger_setDebugLevel"" start " "%lld:%d", a1, a2);
	g_amdLogger = (void*)a1;
	a2 = 9999;
	RAD::callbackRAD->wrapAMDRadeonX6000AmdLoggersetAllLogging((void*)a1, 1);
	int64_t ret = FunctionCast(Wrap_AMDRadeonX6000_AmdLogger_setDebugLevel, Orig_AMDRadeonX6000_AmdLogger_setDebugLevel)(a1, a2);
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000_AmdLogger_setDebugLevel"" end - " "%lld", ret);
	return ret;
}

int64_t RAD::wrapAMDRadeonX6000AmdLoggersetAllLogging(void* AMDRadeonX6000_AmdLogger, unsigned int a2) {
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000AmdLoggersetAllLogging"" start " "%p:%d", AMDRadeonX6000_AmdLogger, a2);
	int64_t ret = FunctionCast(wrapAMDRadeonX6000AmdLoggersetAllLogging, callbackRAD->origAMDRadeonX6000AmdLoggersetAllLogging)(AMDRadeonX6000_AmdLogger, 1);
	SYSLOG("igfx", "Wrap_""AMDRadeonX6000AmdLoggersetAllLogging"" end - " "%lld", ret);
	return ret;
}

// AMDRadeonX6000_AmdLogger::logTiming(LogType,LogSeverity,AmdDetailedTimingInformation const*)	__text	000000000BFE8702	000002A9	00000000		R	.	.	.	.	.	.	.	.	.
// void __fastcall AMDRadeonX6000_AmdLogger::logTiming(__int64 a1, __int64 a2, __int64 a3, __int64 a4)
void RAD::wrapAMDRadeonX6000AmdLoggerlogTiming(void* self, __int64 LogType, __int64 LogSeverity, void* AmdDetailedTimingInformation) {
	SYSLOG("igfx", "wrapAMDRadeonX6000AmdLoggerlogTiming"" start " "%p:%lld:%lld:%p", self, LogType, LogSeverity, AmdDetailedTimingInformation);
	FunctionCast(wrapAMDRadeonX6000AmdLoggerlogTiming, callbackRAD->origAMDRadeonX6000AmdLoggerlogTiming)(self, LogType, LogSeverity, AmdDetailedTimingInformation);
	SYSLOG("igfx", "wrapAMDRadeonX6000AmdLoggerlogTiming"" end");
}

static mach_vm_address_t Orig_my_dm_logger_write;
static char g_buffer[81920] = {0};
static int64_t Wrap_my_dm_logger_write(int64_t a1, unsigned int a2, const char *a3, ...) {
	va_list args;
	va_start(args, a3);
	vsnprintf(g_buffer, 8192, a3, args);
	SYSLOG("igfx", "dm log, type: %d, msg: %s", a2, g_buffer);
	va_end(args);
	return 0;
}

// AMDRadeonX6000_AmdAgdcServices::notifyModeSet(uint,AmdDetailedTimingInformation *,IOPixelInformation *,uchar,bool)
// __ZNK30AMDRadeonX6000_AmdAgdcServices13notifyModeSetEjP28AmdDetailedTimingInformationP18IOPixelInformationhb
// __int64 __fastcall AMDRadeonX6000_AmdAgdcServices::notifyModeSet(char *a1, unsigned int a2, unsigned int *a3, __int64 a4, char a5, int a6)

// AMDRadeonX6000_AmdAgdcServices::notifyModeValidation(uint,AmdDetailedTimingInformation *,AmdFbDisplayPath *)
// __ZN30AMDRadeonX6000_AmdAgdcServices20notifyModeValidationEjP28AmdDetailedTimingInformationP16AmdFbDisplayPath
// __int64 __fastcall AMDRadeonX6000_AmdAgdcServices::notifyModeValidation(char *a1, unsigned int a2, __int64 a3, __int64 a4)

// AMDRadeonX6000_AmdAgdcServices::overrideTimingRange(AGDCFBOverrideTimingRange_t *,ulong)
// __ZN30AMDRadeonX6000_AmdAgdcServices19overrideTimingRangeEP27AGDCFBOverrideTimingRange_tm
// __int64 __fastcall AMDRadeonX6000_AmdAgdcServices::overrideTimingRange(__int64 a1, _DWORD *a2, __int64 a3)

// AMDRadeonX6000_AmdRadeonFramebuffer::validateDetailedTiming(void *,ulong long)
// __ZN35AMDRadeonX6000_AmdRadeonFramebuffer22validateDetailedTimingEPvy
// __int64 __fastcall AMDRadeonX6000_AmdRadeonFramebuffer::validateDetailedTiming(AMDRadeonX6000_AmdValidatedTimingList **this, char *a2, __int64 a3, double a4, double a5)

bool RAD::processKext(KernelPatcher &patcher, size_t index, mach_vm_address_t address, size_t size) {
	if (kextRadeonX6000Framebuffer.loadIndex == index) {
		SYSLOG("igfx", "RAD::processKext, index: %lu, address: %p, size: %lu", index, (void*)address, size);
		g_x6000fb_address = address;
		g_x6000fb_size = size;
		if (getKernelVersion() >= KernelVersion::Monterey) {
			KernelPatcher::RouteRequest requests[] = {
				{"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer25setAttributeForConnectionEijm", wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute, orgAMDRadeonX6000AmdRadeonFramebufferSetAttribute},
				{"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer25getAttributeForConnectionEijPm", wrapAMDRadeonX6000AmdRadeonFramebufferGetAttribute, orgAMDRadeonX6000AmdRadeonFramebufferGetAttribute},
				//DBG{"__ZNK30AMDRadeonX6000_AmdAgdcServices16notifyLinkChangeE18AmdLinkChangeEventh", wrapAMDRadeonX6000AmdAgdcServicesnotifyLinkChange,
				//	origAMDRadeonX6000AmdAgdcServicesnotifyLinkChange},
				// 这个是错的{"__ZN39AMDRadeonX6000_AmdRadeonControllerNavi224getVUpdateVBlankIntervalEiP22_VUpdateVBlankInterval", wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval,
				//	origAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval},
				{"__ZN38AMDRadeonX6000_AmdRadeonControllerNavi24getVUpdateVBlankIntervalEiP22_VUpdateVBlankInterval", wrapAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval,
					origAMDRadeonX6000AmdRadeonControllerNavi2getVUpdateVBlankInterval},
				//DBG{"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer22validateDetailedTimingEPvy", wrapAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming,
				//	origAMDRadeonX6000AmdRadeonFramebuffervalidateDetailedTiming},
				//DBG{"__ZN24AMDRadeonX6000_AmdLogger13setDebugLevelE11LogSeverity", Wrap_AMDRadeonX6000_AmdLogger_setDebugLevel, Orig_AMDRadeonX6000_AmdLogger_setDebugLevel},
				//DBG{"__ZN24AMDRadeonX6000_AmdLogger13setAllLoggingEb", wrapAMDRadeonX6000AmdLoggersetAllLogging, origAMDRadeonX6000AmdLoggersetAllLogging},
				//DBG{"__ZNK24AMDRadeonX6000_AmdLogger9logTimingE7LogType11LogSeverityPK28AmdDetailedTimingInformation", wrapAMDRadeonX6000AmdLoggerlogTiming,
				//	origAMDRadeonX6000AmdLoggerlogTiming},
				//{"__ZN34AMDRadeonX6000_AmdInterruptManager15sleepInterruptsEv", Wrap_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts, Orig_AMDRadeonX6000_AmdInterruptManager_sleepInterrupts},
				//DBG{"__ZNK30AMDRadeonX6000_AmdAgdcServices13notifyModeSetEjP28AmdDetailedTimingInformationP18IOPixelInformationhb", wrapAMDRadeonX6000AmdAgdcServicesnotifyModeSet,
				//	origAMDRadeonX6000AmdAgdcServicesnotifyModeSet},
				//DBG{"__ZN30AMDRadeonX6000_AmdAgdcServices9setFbEdidEP19AGDCFBSetEDIDData_tm", wrapAMDRadeonX6000AmdAgdcServicessetFbEdid,
				//	origAMDRadeonX6000AmdAgdcServicessetFbEdid},
				//DBG{"__ZN30AMDRadeonX6000_AmdAgdcServices20notifyModeValidationEjP28AmdDetailedTimingInformationP16AmdFbDisplayPath", wrapAMDRadeonX6000AmdAgdcServicesnotifyModeValidation,
				//	origAMDRadeonX6000AmdAgdcServicesnotifyModeValidation},
				//DBG{"__ZN30AMDRadeonX6000_AmdAgdcServices19overrideTimingRangeEP27AGDCFBOverrideTimingRange_tm", wrapAMDRadeonX6000AmdAgdcServicesoverrideTimingRange,
				//	origAMDRadeonX6000AmdAgdcServicesoverrideTimingRange},
			};

			if (!patcher.routeMultiple(index, requests, address, size, true, true))
				SYSLOG("igfx", "Failed to route redeon x6000 gpu tracing.");
			
			SYSLOG("igfx", "route _dce_panel_cntl_hw_init");
			mach_vm_address_t dpchi_cal = address + 0x12CB94; // For Monterey
			if (getKernelVersion() >= KernelVersion::Sequoia) {
				dpchi_cal = address + 0x12DA0F; // For Sequoia
			} else if (getKernelVersion() >= KernelVersion::Sonoma) {
				dpchi_cal = address + 0x12D99F; // For Sonoma
			} else if (getKernelVersion() >= KernelVersion::Ventura) {
				dpchi_cal = address + 0x12EC5F; // For Ventura
			}
			
			// Verify the expect code of the destinate address, otherwise will not call to prevent kernel panic
			const uint8_t dpchi_expect_sequoia[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x54, 0x53, 0x48, 0x83, 0xEC, 0x10, 0x48, 0x89, 0xFB, 0x4C, 0x8D };
			const uint8_t dpchi_expect_other[]   = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x50, 0x49, 0x89, 0xFD, 0x4C, 0x8D, 0x45 };
			unsigned char* dpchi_expect = (unsigned char*)dpchi_expect_other;
			if (getKernelVersion() >= KernelVersion::Sonoma) {
				dpchi_expect = (unsigned char*)dpchi_expect_sequoia;
			}
			unsigned char* dpchi_cal_charp = (unsigned char*)dpchi_cal;
			bool expect_match = true;
			for (int i = 0; i < 20; i++) {
				//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
				if (dpchi_cal_charp[i] != dpchi_expect[i]) {
					SYSLOG("igfx", "_dce_panel_cntl_hw_init dismatch: %d => %x != %x", i, dpchi_cal_charp[i], dpchi_expect[i]);
					expect_match = false;
					break;
				}
			}
			
			if (! expect_match) {
				// try search it
				for (int i = 0; i < 0x200000; i++) {
					dpchi_cal_charp = (unsigned char*)(address + i);
					int j = 0;
					for (j = 0; j < 20; j++) {
						//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
						if (dpchi_cal_charp[j] != dpchi_expect[j]) {
							//SYSLOG("igfx", "_dce_panel_cntl_hw_init dismatch: %d => %x != %x", i, dpchi_cal_charp[i], dpchi_expect[i]);
							//expect_match = false;
							break;
						}
					}
					if (j == 20) {
						SYSLOG("igfx", "found _dce_panel_cntl_hw_init address %u", i);
						dpchi_cal = address + i;
						expect_match = true;
					}
				}
			}
			
			if (expect_match) {
				orgDcePanelCntlHwInit = patcher.routeFunction(dpchi_cal, reinterpret_cast<mach_vm_address_t>(wrapDcePanelCntlHwInit), true);
				if (patcher.getError() == KernelPatcher::Error::NoError) {
					DBGLOG("igfx", "routed _dce_panel_cntl_hw_init");
				} else {
					SYSLOG("igfx", "failed to route _dce_panel_cntl_hw_init %d", patcher.getError());
					patcher.clearError();
				}
				
				mach_vm_address_t ddsb_cal = address + 0x12CFC9;
				if (getKernelVersion() >= KernelVersion::Sequoia) {
					ddsb_cal = address + 0x12DE48;
				} else if (getKernelVersion() >= KernelVersion::Sonoma) {
					ddsb_cal = address + 0x12DDD8;
				} else if (getKernelVersion() >= KernelVersion::Ventura) {
					ddsb_cal = address + 0x12F094;
				}
				SYSLOG("igfx", "got Monterey _dce_driver_set_backlight address: %p", ddsb_cal);
				const uint8_t ddsb_expect_sequoia[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x50, 0x41, 0x89, 0xF6, 0x48, 0x89, 0xFB };
				const uint8_t ddsb_expect_other[] =   { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x50, 0x41, 0x89, 0xF7, 0x49, 0x89, 0xFE };
				unsigned char* ddsb_expect = (unsigned char*)ddsb_expect_other;
				if (getKernelVersion() >= KernelVersion::Sonoma) {
					ddsb_expect = (unsigned char*)ddsb_expect_sequoia;
				}
				unsigned char* ddsb_cal_charp = (unsigned char*)ddsb_cal;
				expect_match = true;
				for (int i = 0; i < 20; i++) {
					//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
					if (ddsb_cal_charp[i] != ddsb_expect[i]) {
						SYSLOG("igfx", "_dce_driver_set_backlight dismatch: %d => %x != %x", i, ddsb_cal_charp[i], ddsb_expect[i]);
						expect_match = false;
						break;
					}
				}
				
				if (! expect_match) {
					// try search
					for (int i = 0; i < 0x200000; i++) {
						ddsb_cal_charp = (unsigned char*)(address + i);
						int j = 0;
						for (j = 0; j < 20; j++) {
							//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
							if (ddsb_cal_charp[j] != ddsb_expect[j]) {
								//SYSLOG("igfx", "_dce_panel_cntl_hw_init dismatch: %d => %x != %x", i, dpchi_cal_charp[i], dpchi_expect[i]);
								//expect_match = false;
								break;
							}
						}
						if (j == 20) {
							SYSLOG("igfx", "found _dce_driver_set_backlight address %u", i);
							ddsb_cal = address + i;
							expect_match = true;
						}
					}
				}
				
				
				if (expect_match) {
					orgDceDriverSetBacklight = reinterpret_cast<t_DceDriverSetBacklight>(ddsb_cal);
				}
			}
			
			/*
			SYSLOG("igfx", "route _dm_logger_write");
			//mach_vm_address_t dlw_cal = address + 0x169117;
			mach_vm_address_t dlw_cal = address + 0x169B83; // sequoia
			const uint8_t dlw_expect[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x48, 0x81, 0xEC, 0x88, 0x04, 0x00, 0x00 };
			unsigned char* dlw_cal_charp = (unsigned char*)dlw_cal;
			expect_match = true;
			for (int i = 0; i < 20; i++) {
				//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
				if (dlw_cal_charp[i] != dlw_expect[i]) {
					SYSLOG("igfx", "_dm_logger_write dismatch: %d => %x != %x", i, dlw_cal_charp[i], dlw_expect[i]);
					expect_match = false;
					break;
				}
			}
			if (expect_match) {
				Orig_my_dm_logger_write = patcher.routeFunction(dlw_cal, reinterpret_cast<mach_vm_address_t>(Wrap_my_dm_logger_write), true);
				if (patcher.getError() == KernelPatcher::Error::NoError) {
					DBGLOG("igfx", "routed _dm_logger_write");
				} else {
					SYSLOG("igfx", "failed to route _dm_logger_write %d", patcher.getError());
					patcher.clearError();
				}
			}
			 */
			
			/*
			SYSLOG("igfx", "route _dce110_edp_power_control");
			mach_vm_address_t depc_cal = address + 0x1B04B9;
			const uint8_t depc_expect[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x48, 0x83, 0xEC, 0x48, 0x4C, 0x8B, 0xB7 };
			unsigned char* depc_cal_charp = (unsigned char*)depc_cal;
			expect_match = true;
			for (int i = 0; i < 20; i++) {
				//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
				if (depc_cal_charp[i] != depc_expect[i]) {
					SYSLOG("igfx", "_dce110_edp_power_control dismatch: %d => %x != %x", i, depc_cal_charp[i], depc_expect[i]);
					expect_match = false;
					break;
				}
			}
			if (expect_match) {
				orig_dce110_edp_power_control = patcher.routeFunction(depc_cal, reinterpret_cast<mach_vm_address_t>(wrap_dce110_edp_power_control), true);
				if (patcher.getError() == KernelPatcher::Error::NoError) {
					DBGLOG("igfx", "routed _dce110_edp_power_control");
				} else {
					SYSLOG("igfx", "failed to route _dce110_edp_power_control %d", patcher.getError());
					patcher.clearError();
				}
			}
			
			SYSLOG("igfx", "route _dce110_edp_backlight_control");
			mach_vm_address_t debc_cal = address + 0x1B0926;
			const uint8_t debc_expect[] = { 0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56, 0x41, 0x55, 0x41, 0x54, 0x53, 0x48, 0x83, 0xEC, 0x48, 0x4C, 0x8B, 0xAF };
			unsigned char* debc_cal_charp = (unsigned char*)debc_cal;
			expect_match = true;
			for (int i = 0; i < 20; i++) {
				//SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
				if (debc_cal_charp[i] != debc_expect[i]) {
					SYSLOG("igfx", "_dce110_edp_backlight_control dismatch: %d => %x != %x", i, debc_cal_charp[i], debc_expect[i]);
					expect_match = false;
					break;
				}
			}
			if (expect_match) {
				orig_dce110_edp_backlight_control = patcher.routeFunction(debc_cal, reinterpret_cast<mach_vm_address_t>(wrap_dce110_edp_backlight_control), true);
				if (patcher.getError() == KernelPatcher::Error::NoError) {
					DBGLOG("igfx", "routed _dce110_edp_backlight_control");
				} else {
					SYSLOG("igfx", "failed to route _dce110_edp_backlight_control %d", patcher.getError());
					patcher.clearError();
				}
			}*/
		} else {
			KernelPatcher::RouteRequest requests[] = {
				{"_dce_panel_cntl_hw_init", wrapDcePanelCntlHwInit, orgDcePanelCntlHwInit},
				{"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer25setAttributeForConnectionEijm", wrapAMDRadeonX6000AmdRadeonFramebufferSetAttribute, orgAMDRadeonX6000AmdRadeonFramebufferSetAttribute},
				{"__ZN35AMDRadeonX6000_AmdRadeonFramebuffer25getAttributeForConnectionEijPm", wrapAMDRadeonX6000AmdRadeonFramebufferGetAttribute, orgAMDRadeonX6000AmdRadeonFramebufferGetAttribute},
				//{"_dc_link_bandwidth_kbps", wrapDcLinkBandwidthKbps, orgDcLinkBandwidthKbps},
			};
			
			mach_vm_address_t dpchi_cal = address + 0x124F56;
			unsigned char* dpchi_cal_charp = (unsigned char*)dpchi_cal;
			for (int i = 0; i < 20; i++) {
				SYSLOG("igfx", "%d => %x", i, dpchi_cal_charp[i]);
			}
			mach_vm_address_t ddsb_cal = address + 0x124B21;
			unsigned char* ddsb_cal_charp = (unsigned char*)ddsb_cal;
			for (int i = 0; i < 20; i++) {
				SYSLOG("igfx", "%d => %x", i, ddsb_cal_charp[i]);
			}

			if (!patcher.routeMultiple(index, requests, address, size, true, true))
				SYSLOG("igfx", "Failed to route redeon x6000 gpu tracing.");
			
			mach_vm_address_t ddsb = patcher.solveSymbol(index, "_dce_driver_set_backlight");
			SYSLOG("igfx", "got Bigsur _dce_driver_set_backlight address: %p", ddsb);
			orgDceDriverSetBacklight = reinterpret_cast<t_DceDriverSetBacklight>(ddsb);
		}
		//mach_vm_address_t ddsb_cal = address + 0x124F56;
		//SYSLOG("igfx", "got _dce_driver_set_backlight address: %p, %p", ddsb, ddsb_cal);
		//orgDceDriverSetBacklight = reinterpret_cast<t_DceDriverSetBacklight>(ddsb);
		if (patcher.getError() != KernelPatcher::Error::NoError) {
			SYSLOG("igfx", "failed to resolve _dce_driver_set_backlight");
			patcher.clearError();
			return false;
		}
	}
	
	if (kextRadeonFramebuffer.loadIndex == index) {
		if (force24BppMode)
			process24BitOutput(patcher, kextRadeonFramebuffer, address, size);
		return true;
	}

	if (kextRadeonLegacyFramebuffer.loadIndex == index) {
		if (force24BppMode)
			process24BitOutput(patcher, kextRadeonLegacyFramebuffer, address, size);
		return true;
	}

	if (kextRadeonSupport.loadIndex == index) {
		processConnectorOverrides(patcher, address, size, true);

		if (getKernelVersion() > KernelVersion::Mojave ||
			(getKernelVersion() == KernelVersion::Mojave && getKernelMinorVersion() >= 5)) {
			KernelPatcher::RouteRequest request("__ZN13ATIController8TestVRAME13PCI_REG_INDEXb", doNotTestVram);
			patcher.routeMultiple(index, &request, 1, address, size);
		}

		if (useCustomAgdpDecision) {
			KernelPatcher::RouteRequest request("__ZN16AtiDeviceControl16notifyLinkChangeE31kAGDCRegisterLinkControlEvent_tmj", wrapNotifyLinkChange, orgNotifyLinkChange);
			patcher.routeMultiple(index, &request, 1, address, size);
		}

		return true;
	}

	if (kextRadeonLegacySupport.loadIndex == index) {
		processConnectorOverrides(patcher, address, size, false);
		return true;
	}

	if (kextPolarisController.loadIndex == index) {
		KernelPatcher::RouteRequest request("__ZN17AMD9500Controller23findProjectByPartNumberEP20ControllerProperties", findProjectByPartNumber);
		patcher.routeMultiple(index, &request, 1, address, size);
	}

	for (size_t i = 0; i < maxHardwareKexts; i++) {
		if (kextRadeonHardware[i].loadIndex == index) {
			processHardwareKext(patcher, i, address, size);
			return true;
		}
	}

	return false;
}

void RAD::initHardwareKextMods() {
	// Decide on kext amount present for optimal performance.
	// 10.15+   has X4000, X5000, and X6000
	// 10.14+   has X4000 and X5000
	// 10.13.4+ has X3000, X4000, and X5000
	if (getKernelVersion() >= KernelVersion::Catalina)
		maxHardwareKexts = MaxRadeonHardwareCatalina;
	else if (getKernelVersion() >= KernelVersion::Mojave)
		maxHardwareKexts = MaxRadeonHardwareMojave;
	else if (getKernelVersion() == KernelVersion::HighSierra && getKernelMinorVersion() >= 5)
		maxHardwareKexts = MaxRadeonHardwareModernHighSierra;

	// 10.13.4 fixed black screen issues
	if (maxHardwareKexts != MaxRadeonHardware) {
		for (size_t i = 0; i < MaxGetFrameBufferProcs; i++)
			getFrameBufferProcNames[IndexRadeonHardwareX4000][i] = nullptr;

		// We have nothing to do for these kexts on recent systems
		if (!fixConfigName && !forceOpenGL && !forceCodecInfo) {
			// X4000 kext is not included in this list as we need to fix GVA properties for most of its GPUs
			kextRadeonHardware[IndexRadeonHardwareX5000].switchOff();
			kextRadeonHardware[IndexRadeonHardwareX6000].switchOff();
		}
	}

	if (getKernelVersion() < KernelVersion::Catalina) {
		kextRadeonHardware[IndexRadeonHardwareX6000].switchOff();
	}

	if (getKernelVersion() < KernelVersion::HighSierra) {
		// Versions before 10.13 do not support X4250 and X5000
		kextRadeonHardware[IndexRadeonHardwareX4250].switchOff();
		kextRadeonHardware[IndexRadeonHardwareX5000].switchOff();

		// Versions before 10.13 have legacy X3000 and X4000 IDs
		kextRadeonHardware[IndexRadeonHardwareX3000].id = idRadeonX3000Old;
		kextRadeonHardware[IndexRadeonHardwareX4000].id = idRadeonX4000Old;

		bool preSierra = getKernelVersion() < KernelVersion::Sierra;

		if (preSierra) {
			// Versions before 10.12 do not support X4100
			kextRadeonHardware[IndexRadeonHardwareX4100].switchOff();
		}

		if (preSierra || (getKernelVersion() == KernelVersion::Sierra && getKernelMinorVersion() < 7)) {
			// Versions before 10.12.6 do not support X4150, X4200
			kextRadeonHardware[IndexRadeonHardwareX4150].switchOff();
			kextRadeonHardware[IndexRadeonHardwareX4200].switchOff();
		}
	}

	lilu.onKextLoadForce(kextRadeonHardware, maxHardwareKexts);
}

void RAD::process24BitOutput(KernelPatcher &patcher, KernelPatcher::KextInfo &info, mach_vm_address_t address, size_t size) {
	auto bitsPerComponent = patcher.solveSymbol<int *>(info.loadIndex, "__ZL18BITS_PER_COMPONENT", address, size);
	if (bitsPerComponent) {
		while (bitsPerComponent && *bitsPerComponent) {
			if (*bitsPerComponent == 10) {
				auto ret = MachInfo::setKernelWriting(true, KernelPatcher::kernelWriteLock);
				if (ret == KERN_SUCCESS) {
					DBGLOG("rad", "fixing BITS_PER_COMPONENT");
					*bitsPerComponent = 8;
					MachInfo::setKernelWriting(false, KernelPatcher::kernelWriteLock);
				} else {
					SYSLOG("rad", "failed to disable write protection for BITS_PER_COMPONENT");
				}
			}
			bitsPerComponent++;
		}
	} else {
		SYSLOG("rad", "failed to find BITS_PER_COMPONENT");
		patcher.clearError();
	}

	DBGLOG("rad", "fixing pixel types");

	KernelPatcher::LookupPatch pixelPatch {
		&info,
		reinterpret_cast<const uint8_t *>("--RRRRRRRRRRGGGGGGGGGGBBBBBBBBBB"),
		reinterpret_cast<const uint8_t *>("--------RRRRRRRRGGGGGGGGBBBBBBBB"),
		32, 2
	};

	patcher.applyLookupPatch(&pixelPatch);
	if (patcher.getError() != KernelPatcher::Error::NoError) {
		SYSLOG("rad", "failed to patch RGB mask for 24-bit output");
		patcher.clearError();
	}
}

void RAD::processConnectorOverrides(KernelPatcher &patcher, mach_vm_address_t address, size_t size, bool modern) {
	if (modern) {
		if (getKernelVersion() >= KernelVersion::HighSierra) {
			KernelPatcher::RouteRequest requests[] {
				KernelPatcher::RouteRequest("__ZN14AtiBiosParser116getConnectorInfoEP13ConnectorInfoRh", wrapGetConnectorsInfoV1, orgGetConnectorsInfoV1),
				KernelPatcher::RouteRequest("__ZN14AtiBiosParser216getConnectorInfoEP13ConnectorInfoRh", wrapGetConnectorsInfoV2, orgGetConnectorsInfoV2),
				KernelPatcher::RouteRequest("__ZN14AtiBiosParser126translateAtomConnectorInfoERN30AtiObjectInfoTableInterface_V117AtomConnectorInfoER13ConnectorInfo",
											wrapTranslateAtomConnectorInfoV1, orgTranslateAtomConnectorInfoV1),
				KernelPatcher::RouteRequest("__ZN14AtiBiosParser226translateAtomConnectorInfoERN30AtiObjectInfoTableInterface_V217AtomConnectorInfoER13ConnectorInfo",
											wrapTranslateAtomConnectorInfoV2, orgTranslateAtomConnectorInfoV2),
				KernelPatcher::RouteRequest("__ZN13ATIController5startEP9IOService", wrapATIControllerStart, orgATIControllerStart)
			};
			patcher.routeMultiple(kextRadeonSupport.loadIndex, requests, address, size);
		} else {
			KernelPatcher::RouteRequest requests[] {
				KernelPatcher::RouteRequest("__ZN23AtiAtomBiosDceInterface17getConnectorsInfoEP13ConnectorInfoRh", wrapGetConnectorsInfoV1, orgGetConnectorsInfoV1),
				KernelPatcher::RouteRequest("__ZN13ATIController5startEP9IOService", wrapATIControllerStart, orgATIControllerStart),
			};
			patcher.routeMultiple(kextRadeonSupport.loadIndex, requests, address, size);

			orgGetAtomObjectTableForType = reinterpret_cast<t_getAtomObjectTableForType>(patcher.solveSymbol(kextRadeonSupport.loadIndex,
																											 "__ZN20AtiAtomBiosUtilities25getAtomObjectTableForTypeEhRh", address, size));
			if (!orgGetAtomObjectTableForType) {
				SYSLOG("rad", "failed to find AtiAtomBiosUtilities::getAtomObjectTableForType");
				patcher.clearError();
			}
		}
	} else {
		KernelPatcher::RouteRequest requests[] {
			KernelPatcher::RouteRequest("__ZN23AtiAtomBiosDceInterface17getConnectorsInfoEP13ConnectorInfoRh", wrapLegacyGetConnectorsInfo, orgLegacyGetConnectorsInfo),
			KernelPatcher::RouteRequest("__ZN19AMDLegacyController5startEP9IOService", wrapLegacyATIControllerStart, orgLegacyATIControllerStart),
		};
		patcher.routeMultiple(kextRadeonLegacySupport.loadIndex, requests, address, size);

		orgLegacyGetAtomObjectTableForType = patcher.solveSymbol<t_getAtomObjectTableForType>(kextRadeonLegacySupport.loadIndex,
																							  "__ZN20AtiAtomBiosUtilities25getAtomObjectTableForTypeEhRh", address, size);
		if (!orgLegacyGetAtomObjectTableForType) {
			SYSLOG("rad", "failed to find AtiAtomBiosUtilities::getAtomObjectTableForType");
			patcher.clearError();
		}
	}
}

void RAD::processHardwareKext(KernelPatcher &patcher, size_t hwIndex, mach_vm_address_t address, size_t size) {
	auto getFrame = getFrameBufferProcNames[hwIndex];
	auto &hardware = kextRadeonHardware[hwIndex];

	// Fix boot and wake to black screen
	for (size_t j = 0; j < MaxGetFrameBufferProcs && getFrame[j] != nullptr; j++) {
		auto getFB = patcher.solveSymbol(hardware.loadIndex, getFrame[j], address, size);
		if (getFB) {
			// Initially it was discovered that the only problematic register is PRIMARY_SURFACE_ADDRESS_HIGH (0x1A07).
			// This register must be nulled to solve most of the issues.
			// Depending on the amount of connected screens PRIMARY_SURFACE_ADDRESS (0x1A04) may not be null.
			// However, as of AMD Vega drivers in 10.13 DP1 both of these registers are now ignored.
			// Furthermore, there are no (extra) issues from just returning 0 in framebuffer base address.

			// xor rax, rax
			// ret
			uint8_t ret[] {0x48, 0x31, 0xC0, 0xC3};
			patcher.routeBlock(getFB, ret, sizeof(ret));
			if (patcher.getError() == KernelPatcher::Error::NoError) {
				DBGLOG("rad", "patched %s", getFrame[j]);
			} else {
				SYSLOG("rad", "failed to patch %s code %d", getFrame[j], patcher.getError());
				patcher.clearError();
			}
		} else {
			SYSLOG("rad", "failed to find %s code %d", getFrame[j], patcher.getError());
			patcher.clearError();
		}
	}

	// Fix reported Accelerator name to support WhateverName.app
	// Also fix GVA properties for X4000.
	if (fixConfigName || hwIndex == IndexRadeonHardwareX4000) {
		KernelPatcher::RouteRequest request(populateAccelConfigProcNames[hwIndex], wrapPopulateAccelConfig[hwIndex], orgPopulateAccelConfig[hwIndex]);
		patcher.routeMultiple(hardware.loadIndex, &request, 1, address, size);
	}

	// Enforce OpenGL support if requested
	if (forceOpenGL) {
		DBGLOG("rad", "disabling Metal support");
		uint8_t find1[] {0x4D, 0x65, 0x74, 0x61, 0x6C, 0x53, 0x74, 0x61};
		uint8_t find2[] {0x4D, 0x65, 0x74, 0x61, 0x6C, 0x50, 0x6C, 0x75};
		uint8_t repl1[] {0x50, 0x65, 0x74, 0x61, 0x6C, 0x53, 0x74, 0x61};
		uint8_t repl2[] {0x50, 0x65, 0x74, 0x61, 0x6C, 0x50, 0x6C, 0x75};

		KernelPatcher::LookupPatch antimetal[] {
			{&hardware, find1, repl1, sizeof(find1), 2},
			{&hardware, find2, repl2, sizeof(find1), 2}
		};

		for (auto &p : antimetal) {
			patcher.applyLookupPatch(&p);
			patcher.clearError();
		}
	}

	// Patch AppleGVA support for non-supported models
	if (forceCodecInfo && getHWInfoProcNames[hwIndex] != nullptr) {
		KernelPatcher::RouteRequest request(getHWInfoProcNames[hwIndex], wrapGetHWInfo[hwIndex], orgGetHWInfo[hwIndex]);
		patcher.routeMultiple(hardware.loadIndex, &request, 1, address, size);
	}
}

void RAD::mergeProperty(OSDictionary *props, const char *name, OSObject *value) {
	// The only type we could make from device properties is data.
	// To be able to override other types we do a conversion here.
	auto data = OSDynamicCast(OSData, value);
	if (data) {
		// It is hard to make a boolean even from ACPI, so we make a hack here:
		// 1-byte OSData with 0x01 / 0x00 values becomes boolean.
		auto val = static_cast<const uint8_t *>(data->getBytesNoCopy());
		auto len = data->getLength();
		if (val && len == sizeof(uint8_t)) {
			if (val[0] == 1) {
				props->setObject(name, kOSBooleanTrue);
				DBGLOG("rad", "prop %s was merged as kOSBooleanTrue", name);
				return;
			} else if (val[0] == 0) {
				props->setObject(name, kOSBooleanFalse);
				DBGLOG("rad", "prop %s was merged as kOSBooleanFalse", name);
				return;
			}
		}

		// Consult the original value to make a decision
		auto orgValue = props->getObject(name);
		if (val && orgValue) {
			DBGLOG("rad", "prop %s has original value", name);
			if (len == sizeof(uint32_t) && OSDynamicCast(OSNumber, orgValue)) {
				auto num = *reinterpret_cast<const uint32_t *>(val);
				auto osnum = OSNumber::withNumber(num, 32);
				if (osnum) {
					DBGLOG("rad", "prop %s was merged as number %u", name, num);
					props->setObject(name, osnum);
					osnum->release();
				}
				return;
			} else if (len > 0 && val[len-1] == '\0' && OSDynamicCast(OSString, orgValue)) {
				auto str = reinterpret_cast<const char *>(val);
				auto osstr = OSString::withCString(str);
				if (osstr) {
					DBGLOG("rad", "prop %s was merged as string %s", name, str);
					props->setObject(name, osstr);
					osstr->release();
				}
				return;
			}
		} else {
			DBGLOG("rad", "prop %s has no original value", name);
		}
	}

	// Default merge as is
	props->setObject(name, value);
	DBGLOG("rad", "prop %s was merged", name);
}

void RAD::mergeProperties(OSDictionary *props, const char *prefix, IOService *provider) {
	// Should be ok, but in case there are issues switch to dictionaryWithProperties();
	auto dict = provider->getPropertyTable();
	if (dict) {
		auto iterator = OSCollectionIterator::withCollection(dict);
		if (iterator) {
			OSSymbol *propname;
			size_t prefixlen = strlen(prefix);
			while ((propname = OSDynamicCast(OSSymbol, iterator->getNextObject())) != nullptr) {
				auto name = propname->getCStringNoCopy();
				if (name && propname->getLength() > prefixlen && !strncmp(name, prefix, prefixlen)) {
					auto prop = dict->getObject(propname);
					if (prop)
						mergeProperty(props, name + prefixlen, prop);
					else
						DBGLOG("rad", "prop %s was not merged due to no value", name);
				} else {
					//DBGLOG("rad", "prop %s does not match %s prefix", safeString(name), prefix);
				}
			}

			iterator->release();
		} else {
			SYSLOG("rad", "prop merge failed to iterate over properties");
		}
	} else {
		SYSLOG("rad", "prop merge failed to get properties");
	}

	if (!strcmp(prefix, "CAIL,")) {
		for (size_t i = 0; i < arrsize(powerGatingFlags); i++) {
			if (powerGatingFlags[i] && props->getObject(powerGatingFlags[i])) {
				DBGLOG("rad", "cail prop merge found %s, replacing", powerGatingFlags[i]);
				auto num = OSNumber::withNumber(1, 32);
				if (num) {
					props->setObject(powerGatingFlags[i], num);
					num->release();
				}
			}
		}
	}
}

void RAD::applyPropertyFixes(IOService *service, uint32_t connectorNum) {
	if (service && getKernelVersion() >= KernelVersion::HighSierra) {
		// Starting with 10.13.2 this is important to fix sleep issues due to enforced 6 screens
		if (!service->getProperty("CFG,CFG_FB_LIMIT")) {
			DBGLOG("rad", "setting fb limit to %u", connectorNum);
			service->setProperty("CFG_FB_LIMIT", connectorNum, 32);
		}

		// In the past we set CFG_USE_AGDC to false, which caused visual glitches and broken multimonitor support.
		// A better workaround is to disable AGDP just like we do globally.
	}
}

void RAD::updateConnectorsInfo(void *atomutils, t_getAtomObjectTableForType gettable, IOService *ctrl, RADConnectors::Connector *connectors, uint8_t *sz) {
	if (atomutils) {
		DBGLOG("rad", "getConnectorsInfo found %u connectors", *sz);
		RADConnectors::print(connectors, *sz);
	}

	// Check if the user wants to override automatically detected connectors
	auto cons = ctrl->getProperty("connectors");
	if (cons) {
		auto consData = OSDynamicCast(OSData, cons);
		if (consData) {
			auto consPtr = consData->getBytesNoCopy();
			auto consSize = consData->getLength();

			uint32_t consCount;
			if (WIOKit::getOSDataValue(ctrl, "connector-count", consCount)) {
				*sz = consCount;
				DBGLOG("rad", "getConnectorsInfo got size override to %u", *sz);
			}

			if (consPtr && consSize > 0 && *sz > 0 && RADConnectors::valid(consSize, *sz)) {
				RADConnectors::copy(connectors, *sz, static_cast<const RADConnectors::Connector *>(consPtr), consSize);
				DBGLOG("rad", "getConnectorsInfo installed %u connectors", *sz);
				applyPropertyFixes(ctrl, *sz);
			} else {
				DBGLOG("rad", "getConnectorsInfo conoverrides have invalid size %u for %u num", consSize, *sz);
			}
		} else {
			DBGLOG("rad", "getConnectorsInfo conoverrides have invalid type");
		}
	} else {
		if (atomutils) {
			DBGLOG("rad", "getConnectorsInfo attempting to autofix connectors");
			uint8_t sHeader = 0, displayPathNum = 0, connectorObjectNum = 0;
			auto baseAddr = static_cast<uint8_t *>(gettable(atomutils, AtomObjectTableType::Common, &sHeader)) - sizeof(uint32_t);
			auto displayPaths = static_cast<AtomDisplayObjectPath *>(gettable(atomutils, AtomObjectTableType::DisplayPath, &displayPathNum));
			auto connectorObjects = static_cast<AtomConnectorObject *>(gettable(atomutils, AtomObjectTableType::ConnectorObject, &connectorObjectNum));
			if (displayPathNum == connectorObjectNum)
				autocorrectConnectors(baseAddr, displayPaths, displayPathNum, connectorObjects, connectorObjectNum, connectors, *sz);
			else
				DBGLOG("rad", "getConnectorsInfo found different displaypaths %u and connectors %u", displayPathNum, connectorObjectNum);
		}

		applyPropertyFixes(ctrl, *sz);

		// Prioritise connectors, since it may cause black screen on e.g. R9 370
		const uint8_t *senseList = nullptr;
		uint8_t senseNum = 0;
		auto priData = OSDynamicCast(OSData, ctrl->getProperty("connector-priority"));
		if (priData) {
			senseList = static_cast<const uint8_t *>(priData->getBytesNoCopy());
			senseNum = static_cast<uint8_t>(priData->getLength());
			DBGLOG("rad", "getConnectorInfo found %u senses in connector-priority", senseNum);
			reprioritiseConnectors(senseList, senseNum, connectors, *sz);
		} else {
			DBGLOG("rad", "getConnectorInfo leaving unchaged priority");
		}
	}

	DBGLOG("rad", "getConnectorsInfo resulting %u connectors follow", *sz);
	RADConnectors::print(connectors, *sz);
}

void RAD::autocorrectConnectors(uint8_t *baseAddr, AtomDisplayObjectPath *displayPaths, uint8_t displayPathNum, AtomConnectorObject *connectorObjects,
								uint8_t connectorObjectNum, RADConnectors::Connector *connectors, uint8_t sz) {
	for (uint8_t i = 0; i < displayPathNum; i++) {
		if (!isEncoder(displayPaths[i].usGraphicObjIds)) {
			DBGLOG("rad", "autocorrectConnectors not encoder %X at %u", displayPaths[i].usGraphicObjIds, i);
			continue;
		}

		uint8_t txmit = 0, enc = 0;
		if (!getTxEnc(displayPaths[i].usGraphicObjIds, txmit, enc))
			continue;

		uint8_t sense = getSenseID(baseAddr + connectorObjects[i].usRecordOffset);
		if (!sense) {
			DBGLOG("rad", "autocorrectConnectors failed to detect sense for %u connector", i);
			continue;
		}

		DBGLOG("rad", "autocorrectConnectors found txmit %02X enc %02X sense %02X for %u connector", txmit, enc, sense, i);

		autocorrectConnector(getConnectorID(displayPaths[i].usConnObjectId), sense, txmit, enc, connectors, sz);
	}
}

void RAD::autocorrectConnector(uint8_t connector, uint8_t sense, uint8_t txmit, uint8_t enc, RADConnectors::Connector *connectors, uint8_t sz) {
	// This function attempts to fix the following issues:
	//
	// 1. Incompatible DVI transmitter on 290X, 370 and probably some other models
	// In this case a correct transmitter is detected by AtiAtomBiosDce60::getPropertiesForEncoderObject, however, later
	// in AtiAtomBiosDce60::getPropertiesForConnectorObject for DVI DL and TITFP513 this value is conjuncted with 0xCF,
	// which makes it wrong: 0x10 -> 0, 0x11 -> 1. As a result one gets black screen when connecting multiple displays.
	// getPropertiesForEncoderObject takes usGraphicObjIds and getPropertiesForConnectorObject takes usConnObjectId

	if (callbackRAD->dviSingleLink) {
		if (connector != CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_I &&
			connector != CONNECTOR_OBJECT_ID_DUAL_LINK_DVI_D &&
			connector != CONNECTOR_OBJECT_ID_LVDS) {
			DBGLOG("rad", "autocorrectConnector found unsupported connector type %02X", connector);
			return;
		}

		auto fixTransmit = [](auto &con, uint8_t idx, uint8_t sense, uint8_t txmit) {
			if (con.sense == sense) {
				if (con.transmitter != txmit && (con.transmitter & 0xCF) == con.transmitter) {
					DBGLOG("rad", "autocorrectConnector replacing txmit %02X with %02X for %u connector sense %02X",
						   con.transmitter, txmit, idx, sense);
					con.transmitter = txmit;
				}
				return true;
			}
			return false;
		};

		bool isModern = RADConnectors::modern();
		for (uint8_t j = 0; j < sz; j++) {
			if (isModern) {
				auto &con = (&connectors->modern)[j];
				if (fixTransmit(con, j, sense, txmit))
					break;
			} else {
				auto &con = (&connectors->legacy)[j];
				if (fixTransmit(con, j, sense, txmit))
					break;
			}
		}
	} else {
		DBGLOG("rad", "autocorrectConnector use -raddvi to enable dvi autocorrection");
	}
}

void RAD::reprioritiseConnectors(const uint8_t *senseList, uint8_t senseNum, RADConnectors::Connector *connectors, uint8_t sz) {
	static constexpr uint32_t typeList[] {
		RADConnectors::ConnectorLVDS,
		RADConnectors::ConnectorDigitalDVI,
		RADConnectors::ConnectorHDMI,
		RADConnectors::ConnectorDP,
		RADConnectors::ConnectorVGA
	};
	static constexpr uint8_t typeNum {static_cast<uint8_t>(arrsize(typeList))};

	bool isModern = RADConnectors::modern();
	uint16_t priCount = 1;
	// Automatically detected connectors have equal priority (0), which often results in black screen
	// This allows to change this firstly by user-defined list, then by type list.
	//TODO: priority is ignored for 5xxx and 6xxx GPUs, should we manually reorder items?
	for (uint8_t i = 0; i < senseNum + typeNum + 1; i++) {
		for (uint8_t j = 0; j < sz; j++) {
			auto reorder = [&](auto &con) {
				if (i == senseNum + typeNum) {
					if (con.priority == 0)
						con.priority = priCount++;
				} else if (i < senseNum) {
					if (con.sense == senseList[i]) {
						DBGLOG("rad", "reprioritiseConnectors setting priority of sense %02X to %u by sense", con.sense, priCount);
						con.priority = priCount++;
						return true;
					}
				} else {
					if (con.priority == 0 && con.type == typeList[i-senseNum]) {
						DBGLOG("rad", "reprioritiseConnectors setting priority of sense %02X to %u by type", con.sense, priCount);
						con.priority = priCount++;
					}
				}
				return false;
			};

			if ((isModern && reorder((&connectors->modern)[j])) ||
				(!isModern && reorder((&connectors->legacy)[j])))
				break;
		}
	}
}

void RAD::setGvaProperties(IOService *accelService) {
	auto codecStr = OSDynamicCast(OSString, accelService->getProperty("IOGVACodec"));
	if (codecStr == nullptr) {
		DBGLOG("rad", "updating X4000 accelerator IOGVACodec to VCE");
		accelService->setProperty("IOGVACodec", "VCE");
	} else {
		auto codec = codecStr->getCStringNoCopy();
		DBGLOG("rad", "X4000 accelerator IOGVACodec is already set to %s", safeString(codec));
		if (codec != nullptr && strncmp(codec, "AMD", strlen("AMD")) == 0) {
			bool needsDecode = accelService->getProperty("IOGVAHEVCDecode") == nullptr;
			bool needsEncode = accelService->getProperty("IOGVAHEVCEncode") == nullptr;
			if (needsDecode) {
				OSObject *VTMaxDecodeLevel = OSNumber::withNumber(153, 32);
				OSString *VTMaxDecodeLevelKey  = OSString::withCString("VTMaxDecodeLevel");
				OSDictionary *VTPerProfileDetailsInner = OSDictionary::withCapacity(1);
				OSDictionary *VTPerProfileDetails = OSDictionary::withCapacity(3);
				OSString *VTPerProfileDetailsKey1 = OSString::withCString("1");
				OSString *VTPerProfileDetailsKey2 = OSString::withCString("2");
				OSString *VTPerProfileDetailsKey3 = OSString::withCString("3");

				OSArray *VTSupportedProfileArray = OSArray::withCapacity(3);
				OSNumber *VTSupportedProfileArray1 = OSNumber::withNumber(1, 32);
				OSNumber *VTSupportedProfileArray2 = OSNumber::withNumber(2, 32);
				OSNumber *VTSupportedProfileArray3 = OSNumber::withNumber(3, 32);

				OSDictionary *IOGVAHEVCDecodeCapabilities = OSDictionary::withCapacity(2);
				OSString *VTPerProfileDetailsKey = OSString::withCString("VTPerProfileDetails");
				OSString *VTSupportedProfileArrayKey = OSString::withCString("VTSupportedProfileArray");

				if (VTMaxDecodeLevel != nullptr && VTMaxDecodeLevelKey != nullptr && VTPerProfileDetailsInner != nullptr &&
					VTPerProfileDetails != nullptr && VTPerProfileDetailsKey1 != nullptr && VTPerProfileDetailsKey2 != nullptr &&
					VTPerProfileDetailsKey3 != nullptr && VTSupportedProfileArrayKey != nullptr && VTSupportedProfileArray1 != nullptr &&
					VTSupportedProfileArray2 != nullptr && VTSupportedProfileArray3 != nullptr && VTSupportedProfileArray != nullptr &&
					VTPerProfileDetailsKey != nullptr && IOGVAHEVCDecodeCapabilities != nullptr) {
					VTPerProfileDetailsInner->setObject(VTMaxDecodeLevelKey, VTMaxDecodeLevel);
					VTPerProfileDetails->setObject(VTPerProfileDetailsKey1, VTPerProfileDetailsInner);
					VTPerProfileDetails->setObject(VTPerProfileDetailsKey2, VTPerProfileDetailsInner);
					VTPerProfileDetails->setObject(VTPerProfileDetailsKey3, VTPerProfileDetailsInner);

					VTSupportedProfileArray->setObject(VTSupportedProfileArray1);
					VTSupportedProfileArray->setObject(VTSupportedProfileArray2);
					VTSupportedProfileArray->setObject(VTSupportedProfileArray3);

					IOGVAHEVCDecodeCapabilities->setObject(VTPerProfileDetailsKey, VTPerProfileDetails);
					IOGVAHEVCDecodeCapabilities->setObject(VTSupportedProfileArrayKey, VTSupportedProfileArray);

					accelService->setProperty("IOGVAHEVCDecode", "1");
					accelService->setProperty("IOGVAHEVCDecodeCapabilities", IOGVAHEVCDecodeCapabilities);

					DBGLOG("rad", "recovering IOGVAHEVCDecode");
				} else {
					SYSLOG("rad", "allocation failure in IOGVAHEVCDecode");
				}

				OSSafeReleaseNULL(VTMaxDecodeLevel);
				OSSafeReleaseNULL(VTMaxDecodeLevelKey);
				OSSafeReleaseNULL(VTPerProfileDetailsInner);
				OSSafeReleaseNULL(VTPerProfileDetails);
				OSSafeReleaseNULL(VTPerProfileDetailsKey1);
				OSSafeReleaseNULL(VTPerProfileDetailsKey2);
				OSSafeReleaseNULL(VTPerProfileDetailsKey3);
				OSSafeReleaseNULL(VTSupportedProfileArrayKey);
				OSSafeReleaseNULL(VTSupportedProfileArray1);
				OSSafeReleaseNULL(VTSupportedProfileArray2);
				OSSafeReleaseNULL(VTSupportedProfileArray3);
				OSSafeReleaseNULL(VTSupportedProfileArray);
				OSSafeReleaseNULL(VTPerProfileDetailsKey);
				OSSafeReleaseNULL(IOGVAHEVCDecodeCapabilities);
			}

			if (needsEncode) {
				OSObject *VTMaxEncodeLevel = OSNumber::withNumber(153, 32);
				OSString *VTMaxEncodeLevelKey  = OSString::withCString("VTMaxEncodeLevel");

				OSDictionary *VTPerProfileDetailsInner = OSDictionary::withCapacity(1);
				OSDictionary *VTPerProfileDetails = OSDictionary::withCapacity(1);
				OSString *VTPerProfileDetailsKey1 = OSString::withCString("1");

				OSArray *VTSupportedProfileArray = OSArray::withCapacity(1);
				OSNumber *VTSupportedProfileArray1 = OSNumber::withNumber(1, 32);

				OSDictionary *IOGVAHEVCEncodeCapabilities = OSDictionary::withCapacity(4);
				OSString *VTPerProfileDetailsKey = OSString::withCString("VTPerProfileDetails");
				OSString *VTQualityRatingKey = OSString::withCString("VTQualityRating");
				OSNumber *VTQualityRating = OSNumber::withNumber(50, 32);
				OSString *VTRatingKey = OSString::withCString("VTRating");
				OSNumber *VTRating = OSNumber::withNumber(350, 32);
				OSString *VTSupportedProfileArrayKey = OSString::withCString("VTSupportedProfileArray");

				if (VTMaxEncodeLevel != nullptr && VTMaxEncodeLevelKey != nullptr && VTPerProfileDetailsInner != nullptr &&
					VTPerProfileDetails != nullptr && VTPerProfileDetailsKey1 != nullptr && VTSupportedProfileArrayKey != nullptr &&
					VTSupportedProfileArray1 != nullptr && VTSupportedProfileArray != nullptr && VTPerProfileDetailsKey != nullptr &&
					VTQualityRatingKey != nullptr && VTQualityRating != nullptr && VTRatingKey != nullptr && VTRating != nullptr &&
					IOGVAHEVCEncodeCapabilities != nullptr) {

					VTPerProfileDetailsInner->setObject(VTMaxEncodeLevelKey, VTMaxEncodeLevel);
					VTPerProfileDetails->setObject(VTPerProfileDetailsKey1, VTPerProfileDetailsInner);
					VTSupportedProfileArray->setObject(VTSupportedProfileArray1);

					IOGVAHEVCEncodeCapabilities->setObject(VTPerProfileDetailsKey, VTPerProfileDetails);
					IOGVAHEVCEncodeCapabilities->setObject(VTQualityRatingKey, VTQualityRating);
					IOGVAHEVCEncodeCapabilities->setObject(VTRatingKey, VTRating);
					IOGVAHEVCEncodeCapabilities->setObject(VTSupportedProfileArrayKey, VTSupportedProfileArray);

					accelService->setProperty("IOGVAHEVCEncode", "1");
					accelService->setProperty("IOGVAHEVCEncodeCapabilities", IOGVAHEVCEncodeCapabilities);

					DBGLOG("rad", "recovering IOGVAHEVCEncode");
				} else {
					SYSLOG("rad", "allocation failure in IOGVAHEVCEncode");
				}

				OSSafeReleaseNULL(VTMaxEncodeLevel);
				OSSafeReleaseNULL(VTMaxEncodeLevelKey);
				OSSafeReleaseNULL(VTPerProfileDetailsInner);
				OSSafeReleaseNULL(VTPerProfileDetails);
				OSSafeReleaseNULL(VTPerProfileDetailsKey1);
				OSSafeReleaseNULL(VTSupportedProfileArrayKey);
				OSSafeReleaseNULL(VTSupportedProfileArray1);
				OSSafeReleaseNULL(VTSupportedProfileArray);
				OSSafeReleaseNULL(VTPerProfileDetailsKey);
				OSSafeReleaseNULL(VTQualityRatingKey);
				OSSafeReleaseNULL(VTQualityRating);
				OSSafeReleaseNULL(VTRatingKey);
				OSSafeReleaseNULL(VTRating);
				OSSafeReleaseNULL(IOGVAHEVCEncodeCapabilities);
			}
		}
	}
}

void RAD::updateAccelConfig(size_t hwIndex, IOService *accelService, const char **accelConfig) {
	if (accelService && accelConfig) {
		if (fixConfigName) {
			auto gpuService = accelService->getParentEntry(gIOServicePlane);

			if (gpuService) {
				auto model = OSDynamicCast(OSData, gpuService->getProperty("model"));
				if (model) {
					auto modelStr = static_cast<const char *>(model->getBytesNoCopy());
					if (modelStr) {
						if (modelStr[0] == 'A' && ((modelStr[1] == 'M' && modelStr[2] == 'D') ||
												   (modelStr[1] == 'T' && modelStr[2] == 'I')) && modelStr[3] == ' ') {
							modelStr += 4;
						}

						DBGLOG("rad", "updateAccelConfig found gpu model %s", modelStr);
						*accelConfig = modelStr;
					} else {
						DBGLOG("rad", "updateAccelConfig found null gpu model");
					}
				} else {
					DBGLOG("rad", "updateAccelConfig failed to find gpu model");
				}

			} else {
				DBGLOG("rad", "updateAccelConfig failed to find accelerator parent");
			}
		}

		if (enableGvaSupport && hwIndex == IndexRadeonHardwareX4000) {
			setGvaProperties(accelService);
		}
	}
}

bool RAD::wrapSetProperty(IORegistryEntry *that, const char *aKey, void *bytes, unsigned length) {
	if (length > 10 && aKey && reinterpret_cast<const uint32_t *>(aKey)[0] == 'edom' && reinterpret_cast<const uint16_t *>(aKey)[2] == 'l') {
		DBGLOG("rad", "SetProperty caught model %u (%.*s)", length, length, static_cast<char *>(bytes));
		if (*static_cast<uint32_t *>(bytes) == ' DMA' || *static_cast<uint32_t *>(bytes) == ' ITA' || *static_cast<uint32_t *>(bytes) == 'edaR') {
			if (FunctionCast(wrapGetProperty, callbackRAD->orgGetProperty)(that, aKey)) {
				DBGLOG("rad", "SetProperty ignored setting %s to %s", aKey, static_cast<char *>(bytes));
				return true;
			}
			DBGLOG("rad", "SetProperty missing %s, fallback to %s", aKey, static_cast<char *>(bytes));
		}
	}

	return FunctionCast(wrapSetProperty, callbackRAD->orgSetProperty)(that, aKey, bytes, length);
}

OSObject *RAD::wrapGetProperty(IORegistryEntry *that, const char *aKey) {
	auto obj = FunctionCast(wrapGetProperty, callbackRAD->orgGetProperty)(that, aKey);
	auto props = OSDynamicCast(OSDictionary, obj);

	if (props && aKey) {
		const char *prefix {nullptr};
		auto provider = OSDynamicCast(IOService, that->getParentEntry(gIOServicePlane));
		if (provider) {
			if (aKey[0] == 'a') {
				if (!strcmp(aKey, "aty_config"))
					prefix = "CFG,";
				else if (!strcmp(aKey, "aty_properties"))
					prefix = "PP,";
			} else if (aKey[0] == 'c' && !strcmp(aKey, "cail_properties")) {
				prefix = "CAIL,";
			}

			if (prefix) {
				DBGLOG("rad", "GetProperty discovered property merge request for %s", aKey);
				auto rawProps = props->copyCollection();
				if (rawProps) {
					auto newProps = OSDynamicCast(OSDictionary, rawProps);
					if (newProps) {
						callbackRAD->mergeProperties(newProps, prefix, provider);
						that->setProperty(aKey, newProps);
						obj = newProps;
					}
					rawProps->release();
				}

			}
		}
	}

	return obj;
}

uint32_t RAD::wrapGetConnectorsInfoV1(void *that, RADConnectors::Connector *connectors, uint8_t *sz) {
	uint32_t code = FunctionCast(wrapGetConnectorsInfoV1, callbackRAD->orgGetConnectorsInfoV1)(that, connectors, sz);
	auto props = callbackRAD->currentPropProvider.get();

	if (code == 0 && sz && props && *props) {
		if (getKernelVersion() >= KernelVersion::HighSierra)
			callbackRAD->updateConnectorsInfo(nullptr, nullptr, *props, connectors, sz);
		else
			callbackRAD->updateConnectorsInfo(static_cast<void **>(that)[1], callbackRAD->orgGetAtomObjectTableForType, *props, connectors, sz);
	} else {
		DBGLOG("rad", "getConnectorsInfoV1 failed %X or undefined %d", code, props == nullptr);
	}

	return code;
}

uint32_t RAD::wrapGetConnectorsInfoV2(void *that, RADConnectors::Connector *connectors, uint8_t *sz) {
	uint32_t code = FunctionCast(wrapGetConnectorsInfoV2, callbackRAD->orgGetConnectorsInfoV2)(that, connectors, sz);
	auto props = callbackRAD->currentPropProvider.get();

	if (code == 0 && sz && props && *props)
		callbackRAD->updateConnectorsInfo(nullptr, nullptr, *props, connectors, sz);
	else
		DBGLOG("rad", "getConnectorsInfoV2 failed %X or undefined %d", code, props == nullptr);

	return code;
}

uint32_t RAD::wrapLegacyGetConnectorsInfo(void *that, RADConnectors::Connector *connectors, uint8_t *sz) {
	uint32_t code = FunctionCast(wrapLegacyGetConnectorsInfo, callbackRAD->orgLegacyGetConnectorsInfo)(that, connectors, sz);
	auto props = callbackRAD->currentLegacyPropProvider.get();

	if (code == 0 && sz && props && *props)
		callbackRAD->updateConnectorsInfo(static_cast<void **>(that)[1], callbackRAD->orgLegacyGetAtomObjectTableForType, *props, connectors, sz);
	else
		DBGLOG("rad", "legacy getConnectorsInfo failed %X or undefined %d", code, props == nullptr);

	return code;
}

uint32_t RAD::wrapTranslateAtomConnectorInfoV1(void *that, RADConnectors::AtomConnectorInfo *info, RADConnectors::Connector *connector) {
	uint32_t code = FunctionCast(wrapTranslateAtomConnectorInfoV1, callbackRAD->orgTranslateAtomConnectorInfoV1)(that, info, connector);

	if (code == 0 && info && connector) {
		RADConnectors::print(connector, 1);

		uint8_t sense = getSenseID(info->i2cRecord);
		if (sense) {
			DBGLOG("rad", "translateAtomConnectorInfoV1 got sense id %02X", sense);

			// We need to extract usGraphicObjIds from info->hpdRecord, which is of type ATOM_SRC_DST_TABLE_FOR_ONE_OBJECT:
			// struct ATOM_SRC_DST_TABLE_FOR_ONE_OBJECT {
			//   uint8_t ucNumberOfSrc;
			//   uint16_t usSrcObjectID[ucNumberOfSrc];
			//   uint8_t ucNumberOfDst;
			//   uint16_t usDstObjectID[ucNumberOfDst];
			// };
			// The value we need is in usSrcObjectID. The structure is byte-packed.

			uint8_t ucNumberOfSrc = info->hpdRecord[0];
			for (uint8_t i = 0; i < ucNumberOfSrc; i++) {
				auto usSrcObjectID = *reinterpret_cast<uint16_t *>(info->hpdRecord + sizeof(uint8_t) + i * sizeof(uint16_t));
				DBGLOG("rad", "translateAtomConnectorInfoV1 checking %04X object id", usSrcObjectID);
				if (((usSrcObjectID & OBJECT_TYPE_MASK) >> OBJECT_TYPE_SHIFT) == GRAPH_OBJECT_TYPE_ENCODER) {
					uint8_t txmit = 0, enc = 0;
					if (getTxEnc(usSrcObjectID, txmit, enc))
						callbackRAD->autocorrectConnector(getConnectorID(info->usConnObjectId), getSenseID(info->i2cRecord), txmit, enc, connector, 1);
					break;
				}
			}


		} else {
			DBGLOG("rad", "translateAtomConnectorInfoV1 failed to detect sense for translated connector");
		}
	}

	return code;
}

uint32_t RAD::wrapTranslateAtomConnectorInfoV2(void *that, RADConnectors::AtomConnectorInfo *info, RADConnectors::Connector *connector) {
	uint32_t code = FunctionCast(wrapTranslateAtomConnectorInfoV2, callbackRAD->orgTranslateAtomConnectorInfoV2)(that, info, connector);

	if (code == 0 && info && connector) {
		RADConnectors::print(connector, 1);

		uint8_t sense = getSenseID(info->i2cRecord);
		if (sense) {
			DBGLOG("rad", "translateAtomConnectorInfoV2 got sense id %02X", sense);
			uint8_t txmit = 0, enc = 0;
			if (getTxEnc(info->usGraphicObjIds, txmit, enc))
				callbackRAD->autocorrectConnector(getConnectorID(info->usConnObjectId), getSenseID(info->i2cRecord), txmit, enc, connector, 1);
		} else {
			DBGLOG("rad", "translateAtomConnectorInfoV2 failed to detect sense for translated connector");
		}
	}

	return code;
}

bool RAD::wrapATIControllerStart(IOService *ctrl, IOService *provider) {
	DBGLOG("rad", "starting controller " PRIKADDR, CASTKADDR(current_thread()));
	if (callbackRAD->forceVesaMode) {
		DBGLOG("rad", "disabling video acceleration on request");
		return false;
	}

	callbackRAD->currentPropProvider.set(provider);
	bool r = FunctionCast(wrapATIControllerStart, callbackRAD->orgATIControllerStart)(ctrl, provider);
	DBGLOG("rad", "starting controller done %d " PRIKADDR, r, CASTKADDR(current_thread()));
	callbackRAD->currentPropProvider.erase();

	return r;
}

bool RAD::wrapLegacyATIControllerStart(IOService *ctrl, IOService *provider) {
	DBGLOG("rad", "starting legacy controller " PRIKADDR, CASTKADDR(current_thread()));
	if (callbackRAD->forceVesaMode) {
		DBGLOG("rad", "disabling legacy video acceleration on request");
		return false;
	}

	callbackRAD->currentLegacyPropProvider.set(provider);
	bool r = FunctionCast(wrapLegacyATIControllerStart, callbackRAD->orgLegacyATIControllerStart)(ctrl, provider);
	DBGLOG("rad", "starting legacy legacy controller done %d " PRIKADDR, r, CASTKADDR(current_thread()));
	callbackRAD->currentLegacyPropProvider.erase();

	return r;
}

IOReturn RAD::findProjectByPartNumber(IOService *ctrl, void *properties) {
	// Drivers have predefined framebuffers for the following models:
	// 113-4E353BU, 113-4E3531U, 113-C94002A1XTA
	// Despite this looking sane, at least with Sapphire 113-4E353BU-O50 (RX 580) these framebuffers break connectors.
	return kIOReturnNotFound;
}

bool RAD::doNotTestVram(IOService *ctrl, uint32_t reg, bool retryOnFail) {
	// Based on vladie's patch description:
	// TestVRAM fills memory with 0xaa55aa55 bytes (it's magenta pixels visible onscreen),
	// and it tries to test too much of address space, writing this bytes to framebuffer memory.
	// If you have verbose mode enabled (as i have), there is a possibility that framebuffer
	// will scroll during this test, and TestVRAM will write 0xaa55aa55, but read 0x00000000
	// (because magenta-colored pixels are scrolled up) causing kernel panic.
	//
	// Here we just do not do video memory testing for simplicity.
	return true;
}

bool RAD::wrapNotifyLinkChange(void *atiDeviceControl, kAGDCRegisterLinkControlEvent_t event, void *eventData, uint32_t eventFlags) {
	auto ret = FunctionCast(wrapNotifyLinkChange, callbackRAD->orgNotifyLinkChange)(atiDeviceControl, event, eventData, eventFlags);

	if (event == kAGDCValidateDetailedTiming) {
		auto cmd = static_cast<AGDCValidateDetailedTiming_t *>(eventData);
		DBGLOG("rad", "AGDCValidateDetailedTiming %u -> %d (%u)", cmd->framebufferIndex, ret, cmd->modeStatus);
		// While we have this condition below, the only actual value we get is ret = true, cmd->modeStatus = 0.
		// This is because AGDP is disabled, and starting from 10.15.1b2 AMDFramebuffer no longer accepts 0 in
		// __ZN14AMDFramebuffer22validateDetailedTimingEPvy
		if (ret == false || cmd->modeStatus < 1 || cmd->modeStatus > 3) {
			cmd->modeStatus = 2;
			ret = true;
		}
	}

	return ret;
}

void RAD::updateGetHWInfo(IOService *accelVideoCtx, void *hwInfo) {
	IOService *accel, *pciDev;
	accel = OSDynamicCast(IOService, accelVideoCtx->getParentEntry(gIOServicePlane));
	if (accel == NULL) {
		SYSLOG("rad", "getHWInfo: no parent found for accelVideoCtx!");
		return;
	}
	pciDev = OSDynamicCast(IOService, accel->getParentEntry(gIOServicePlane));
	if (pciDev == NULL) {
		SYSLOG("rad", "getHWInfo: no parent found for accel!");
		return;
	}
	uint16_t &org = getMember<uint16_t>(hwInfo, 0x4);
	uint32_t dev = org;
	if (!WIOKit::getOSDataValue(pciDev, "codec-device-id", dev)) {
		// fallback to device-id only if we do not have codec-device-id
		WIOKit::getOSDataValue(pciDev, "device-id", dev);
	}
	DBGLOG("rad", "getHWInfo: original PID: 0x%04X, replaced PID: 0x%04X", org, dev);
	org = static_cast<uint16_t>(dev);
}
