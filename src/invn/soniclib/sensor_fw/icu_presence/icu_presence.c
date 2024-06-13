/*! \file icu_presence.c
 *
 * \brief Chirp ICU Presence Detection firmware interface
 *
 * This file contains function definitions to interface a specific sensor firmware
 * package to SonicLib, including the main initialization routine for the firmware.
 * That routine initializes various fields within the \a ch_dev_t device descriptor
 * and specifies the proper functions to implement SonicLib API calls.  Those may
 * either be common implementations or firmware-specific routines located in this file.
 */

/*
 Copyright (c) 2019-2022, Chirp Microsystems.  All rights reserved.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
 */

#include <invn/soniclib/soniclib.h>
#include "icu_presence.h"
#include <invn/soniclib/details/ch_common.h>
#include <invn/icu_interface/presence/structs.h>
#include <invn/icu_interface/presence/config_helper.h>

static const ch_api_funcs_t api_funcs = {
		.set_num_samples      = ch_common_set_num_samples,
		.get_range            = NULL,
		.get_amplitude        = NULL,
		.get_iq_data          = ch_common_get_iq_data,
		.get_amplitude_data   = NULL,
		.mm_to_samples        = ch_common_mm_to_samples,
		.set_data_output      = NULL,
		.set_target_interrupt = ch_common_set_target_interrupt,
		.get_target_interrupt = ch_common_get_target_interrupt,
		.set_sample_window    = NULL,
		.get_amplitude_avg    = NULL,
		.set_tx_length        = ch_common_set_tx_length,
		.get_tx_length        = ch_common_get_tx_length,
		.algo_specific_api    = NULL,
};

static const ch_calib_funcs_t calib_funcs = {
		.prepare_pulse_timer = ch_common_prepare_pulse_timer,
		.store_pt_result     = ch_common_store_pt_result,
		.store_op_freq       = ch_common_store_op_freq,
		.store_bandwidth     = ch_common_store_bandwidth,
		.store_scalefactor   = ch_common_store_scale_factor,
		.get_locked_state    = ch_common_get_locked_state,
};

static fw_info_t self = {
		.api_funcs   = &api_funcs,
		.calib_funcs = &calib_funcs,
		/* This firmware can only restart - clock cal etc. must be done previously */
		.fw_includes_sensor_init     = 0,
		.fw_includes_tx_optimization = 0,
		.freqCounterCycles           = ICU_COMMON_FREQCOUNTERCYCLES,
		.freqLockValue               = ICU_COMMON_READY_FREQ_LOCKED,
		.max_num_thresholds          = 0,
		.oversample                  = 0, /* This firmware does not use oversampling */
};

uint8_t icu_presence_init(ch_dev_t *dev_ptr, fw_info_t **fw_info) {
	(void)dev_ptr;

	/* Init firmware-specific function pointers */
	self.fw_text              = icu_presence_fw_text;
	self.fw_text_size         = icu_presence_text_size;
	self.fw_vec               = icu_presence_fw_vec;
	self.fw_vec_size          = icu_presence_vec_size;
	self.fw_version_string    = icu_presence_version;
	self.ram_init             = get_ram_icu_presence_init_ptr();
	self.get_fw_ram_init_size = get_icu_presence_fw_ram_init_size;
	self.get_fw_ram_init_addr = get_icu_presence_fw_ram_init_addr;

	*fw_info = &self;

	return 0;
}

void icu_presence_generate_default_user_config(IcuPresenceUserConfig *user_config) {
	icu_interface_presence_generate_default_user_config(user_config);
}

uint8_t icu_presence_generate_config(const IcuPresenceSensorConfig *sensor_config,
                                     const IcuPresenceUserConfig *user_config, IcuPresenceConfig *algo_config) {
	return icu_interface_presence_generate_config(sensor_config, user_config, algo_config);
}
