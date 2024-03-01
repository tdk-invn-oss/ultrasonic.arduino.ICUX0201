/*! \file icu_presence.h
 *
 * \brief Internal definitions for the Chirp ICU Presence Detection sensor firmware.
 *
 * This file contains various definitions and values for use with the icu_presence
 * sensor firmware.
 *
 * You should not need to edit this file or call the driver functions directly.  Doing so
 * will reduce your ability to benefit from future enhancements and releases from Chirp.
 *
 */

/*
 * Copyright (c) 2016-2022, Chirp Microsystems.  All rights reserved.
 *
 * Chirp Microsystems CONFIDENTIAL
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You can contact the authors of this program by email at support@chirpmicro.com
 * or by mail at 2560 Ninth Street, Suite 220A, Berkeley, CA 94710.
 */

#ifndef ICU_PRESENCE_H_
#define ICU_PRESENCE_H_

#include <stdint.h>
#include <invn/soniclib/details/icu.h>
#include <invn/soniclib/soniclib.h>

#include <invn/icu_interface/presence/structs.h>

#define ICU_PRESENCE_MAX_SAMPLES (IQ_SAMPLES_MAX)  // from shasta_external_regs.h

extern const char *icu_presence_version;  // version string in fw .c file
extern const uint8_t icu_presence_fw_text[];
extern const uint8_t icu_presence_fw_vec[];
extern const uint16_t icu_presence_text_size;
extern const uint16_t icu_presence_vec_size;

uint16_t get_icu_presence_fw_ram_init_addr(void);
uint16_t get_icu_presence_fw_ram_init_size(void);

const unsigned char *get_ram_icu_presence_init_ptr(void);

uint8_t icu_presence_init(ch_dev_t *dev_ptr, fw_info_t **fw_info);

/**
 * \brief Helper function to generate algo config structure
 * To generate recommended algo config, user should provide sensor_config and user_config
 * \param[in] sensor_config: pointer to structure that includes information related to sensor information and its
 * measurement information e.g. fop, number or samples param[in] user_config: pointer to structure that includes high
 * level setting to configure the presence detection (e.g. sensitivity) \retval 0 Success. \retval 1 Error: sensor odr
 * cannot be zero. \retval 2 Error: sensor cic_odr must be within [2,6]. \retval 4 Error: sensor pulse length cannot be
 * zero. \ingroup IcuPresence
 */
uint8_t icu_presence_generate_config(const IcuPresenceSensorConfig *sensor_config,
                                     const IcuPresenceUserConfig *user_config, IcuPresenceConfig *algo_config);

/*!
 * \brief Get default configuration structure
 * \param[in,out] user_config pointer IcuPresenceUserConfig structure to update with default parameters
 * \ingroup IcuPresence
 */
void icu_presence_generate_default_user_config(IcuPresenceUserConfig *user_config);

#endif /* ICU_PRESENCE_H_ */
