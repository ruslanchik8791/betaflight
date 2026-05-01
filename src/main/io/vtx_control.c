#include <stdbool.h>
#include <stdint.h>
#include <drivers/vtx_table.h>

#include "platform.h"

#if defined(USE_VTX_CONTROL) && defined(USE_VTX_COMMON)

#include "common/maths.h"
#include "config/config_eeprom.h"
#include "drivers/buttons.h"
#include "drivers/light_led.h"
#include "drivers/time.h"
#include "drivers/vtx_common.h"
#include "config/config.h"
#include "fc/runtime_config.h"
#include "io/spektrum_vtx_control.h"
#include "io/vtx.h"
#include "io/vtx_control.h"
#include "osd/osd.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(vtxConfig_t, vtxConfig, PG_VTX_CONFIG, 1);

PG_RESET_TEMPLATE(vtxConfig_t, vtxConfig,
    .halfDuplex = true
);

static uint8_t locked = 0;

void vtxControlInit(void)
{
}

void vtxControlInputPoll(void)
{
#if defined(USE_SPEKTRUM_VTX_CONTROL)
    spektrumVtxControl();
#endif
}

static void vtxUpdateBandAndChannel(uint8_t bandStep, uint8_t channelStep)
{
    if (vtxCommonDevice()) {
        vtxSettingsConfigMutable()->band += bandStep;
        vtxSettingsConfigMutable()->channel += channelStep;
    }
}

void vtxIncrementBand(void)
{
    vtxUpdateBandAndChannel(+1, 0);
}

void vtxDecrementBand(void)
{
    vtxUpdateBandAndChannel(-1, 0);
}

void vtxIncrementChannel(void)
{
    vtxUpdateBandAndChannel(0, +1);
}

void vtxDecrementChannel(void)
{
    vtxUpdateBandAndChannel(0, -1);
}

void vtxUpdateActivatedChannel(void)
{
    if (vtxCommonDevice()) {
        static uint8_t lastIndex = -1;

        for (uint8_t index = 0; index < MAX_CHANNEL_ACTIVATION_CONDITION_COUNT; index++) {
            const vtxChannelActivationCondition_t *vtxChannelActivationCondition = &vtxConfig()->vtxChannelActivationConditions[index];

            if (isRangeActive(vtxChannelActivationCondition->auxChannelIndex, &vtxChannelActivationCondition->range)
                && index != lastIndex) {
                lastIndex = index;

                if (vtxChannelActivationCondition->band > 0) {
                    vtxSettingsConfigMutable()->band = vtxChannelActivationCondition->band;
                }
                if (vtxChannelActivationCondition->channel > 0) {
                    vtxSettingsConfigMutable()->channel = vtxChannelActivationCondition->channel;
                }
                if (vtxChannelActivationCondition->power > 0) {
                    vtxSettingsConfigMutable()->power = vtxChannelActivationCondition->power;
                }
                break;
            }
        }
    }
}

void vtxCycleBandOrChannel(const uint8_t bandStep, const uint8_t channelStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t band = 0, channel = 0;
        if (!vtxCommonGetBandAndChannel(vtxDevice, &band, &channel)) {
            return;
        }

        int newChannel = channel + channelStep;
        if (newChannel > vtxTableChannelCount) {
            newChannel = 1;
        } else if (newChannel < 1) {
            newChannel = vtxTableChannelCount;
        }

        int newBand = band + bandStep;
        if (newBand > vtxTableBandCount) {
            newBand = 1;
        } else if (newBand < 1) {
            newBand = vtxTableBandCount;
        }

        vtxSettingsConfigMutable()->band = newBand;
        vtxSettingsConfigMutable()->channel = newChannel;
    }
}

void vtxCyclePower(const uint8_t powerStep)
{
    const vtxDevice_t *vtxDevice = vtxCommonDevice();
    if (vtxDevice) {
        uint8_t power = 0;
        if (!vtxCommonGetPowerIndex(vtxDevice, &power)) {
            return;
        }

        int newPower = power + powerStep;
        if (newPower >= vtxTablePowerLevels) {
            newPower = 1;
        } else if (newPower < 0) {
            newPower = vtxTablePowerLevels;
        }

        vtxSettingsConfigMutable()->power = newPower;
    }
}

void handleVTXControlButton(void)
{
#if defined(USE_VTX_RTC6705) && defined(BUTTON_A_PIN)
    bool buttonWasPressed = false;
    const timeMs_t start = millis();
    timeMs_t ledToggleAt = start;
    bool ledEnabled = false;
    uint8_t flashesDone = 0;
    uint8_t actionCounter = 0;
    bool buttonHeld;

    while ((buttonHeld = buttonAPressed())) {
        const timeMs_t end = millis();
        int32_t diff = cmp32(end, start);
        if (diff > 25 && diff <= 1000) actionCounter = 4;
        else if (diff > 1000 && diff <= 3000) actionCounter = 3;
        else if (diff > 3000 && diff <= 5000) actionCounter = 2;
        else if (diff > 5000) actionCounter = 1;

        if (actionCounter) {
            diff = cmp32(ledToggleAt, end);
            if (diff < 0) {
                ledEnabled = !ledEnabled;
                const uint8_t updateDuration = 60;
                ledToggleAt = end + updateDuration;
                if (ledEnabled) LED1_ON; else LED1_OFF;
                if (!ledEnabled) flashesDone++;
                if (flashesDone == actionCounter) {
                    ledToggleAt += (1000 - ((flashesDone * updateDuration) * 2));
                    flashesDone = 0;
                }
            }
            buttonWasPressed = true;
        }
    }

    if (!buttonWasPressed) return;
    LED1_OFF;

    switch (actionCounter) {
    case 4: vtxCycleBandOrChannel(0, +1); break;
    case 3: vtxCycleBandOrChannel(+1, 0); break;
    case 2: vtxCyclePower(+1); break;
    case 1: saveConfigAndNotify(); break;
    }
#endif
}
#endif
