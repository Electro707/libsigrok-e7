#ifndef LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_EXTRA_H
#define LIBSIGROK_HARDWARE_DREAMSOURCELAB_DSLOGIC_EXTRA_H

#include <glib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "protocol.h"

SR_PRIV uint16_t ds_trigger_get_mask0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);
SR_PRIV uint16_t ds_trigger_get_value0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);
SR_PRIV uint16_t ds_trigger_get_edge0(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);
SR_PRIV uint16_t ds_trigger_get_mask1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);
SR_PRIV uint16_t ds_trigger_get_value1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);
SR_PRIV uint16_t ds_trigger_get_edge1(uint16_t stage, uint16_t msc, uint16_t lsc, gboolean qutr_mode, gboolean half_mode);

#endif
