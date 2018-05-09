/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "extended_audio.h"




static struct voiceconfig_page * const mp = &pagemem.u.voiceconfig_page;
static struct voiceconfig_obj * const gui = &gui_objs.u.voiceconfig;

static u16 current_selected = 0;

enum {
    VOICE_SRC_SWITCH = 0,
    VOICE_SRC_AUX,
    VOICE_SRC_TIMER,
    VOICE_SRC_TELEMETRY,
    VOICE_SRC_MIXER,
};

static u8 voiceconfig_getsrctype (u8 idx)
{
    switch(PAGE_GetID()) {
        case PAGEID_VOICESWITCH:
            if (idx < NUM_SWITCHES)
                return VOICE_SRC_SWITCH;
#if NUM_AUX_KNOBS
                if (idx < NUM_SWITCHES + NUM_AUX_KNOBS * 2)
                    return VOICE_SRC_AUX;
#endif
            break;
        case PAGEID_VOICETELTIM:
            if (idx < NUM_TIMERS)
                return VOICE_SRC_TIMER;
            if (idx < NUM_TIMERS + TELEM_NUM_ALARMS)
                return VOICE_SRC_TELEMETRY;
            break;
    }
    return VOICE_SRC_MIXER;
}

const char *voiceglobal_str_cb(guiObject_t *obj, const void *data)
{
    (void)obj;
    int idx = (long)data;
    switch (idx) {
        case 0: strcpy(tempstring, _tr("Battery")); break;
        case 1: strcpy(tempstring, _tr("Binding")); break;
        case 2: strcpy(tempstring, _tr("Inactivity")); break;
        case 3: strcpy(tempstring, _tr("Timer warn")); break;
        case 4:
        case 5:
        case 6:
        case 7: snprintf(tempstring, sizeof(tempstring), _tr("Timer %d"), idx - 3); break;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13: snprintf(tempstring, sizeof(tempstring), _tr("Telem %d"), idx - 7); break;
    }
    return tempstring;

}

static const char *voiceglobal_cb(guiObject_t *obj, int dir, void *data)
{
    (void) obj;
    int idx = (long)data;
    struct CustomVoice *vpt;
    vpt = NULL;

    switch (idx) {
        case 0:
            vpt = &Model.voice.switches[idx];
            break;
        case 1:
            vpt = &Model.voice.timer[idx];
            break;
        case 2:
            vpt = &Model.voice.telemetry[idx - NUM_TIMERS];
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            break;
        case 8:
        case 9:
        case 10:
        case 11:
        case 12:
        case 13:
            break;
        }
    if (dir == -1 && vpt->music == CUSTOM_ALARM_ID) // set to none below 1
        vpt->music = 0;
    if (dir == 1 && vpt->music == 0) // set to CUSTOM_ALARM_ID when currently none
        vpt->music = CUSTOM_ALARM_ID - 1;
    GUI_TextSelectEnablePress((guiTextSelect_t *)obj, vpt->music);

    if (vpt->music == 0) {
        return strcpy(tempstring, _tr("None"));
    }
    vpt->music = GUI_TextSelectHelper(vpt->music - CUSTOM_ALARM_ID + 1, //Relabling so voice in menu starts with 1
        1, voice_map_entries - CUSTOM_ALARM_ID, dir, 1, 10, NULL) + CUSTOM_ALARM_ID - 1;
    snprintf(tempstring, 5, "%d", vpt->music - CUSTOM_ALARM_ID + 1);
    return tempstring;
}

const char *voiceconfig_str_cb(guiObject_t *obj, const void *data)
{
    (void)obj;
    int idx = (long)data;
    switch(voiceconfig_getsrctype(idx)) {
        case VOICE_SRC_SWITCH:
            INPUT_SourceName(tempstring,idx + INP_HAS_CALIBRATION + 1);
            break;
#if NUM_AUX_KNOBS
        case VOICE_SRC_AUX:
            if ((idx - NUM_SWITCHES) % 2) {
                INPUT_SourceName(tempstring,(idx - NUM_SWITCHES -1) / 2 + NUM_STICKS + 1);
                strcat(tempstring, " ");
                strcat(tempstring, _tr("up"));
            }
            else {
                INPUT_SourceName(tempstring,(idx - NUM_SWITCHES) / 2 + NUM_STICKS + 1);
                strcat(tempstring," ");
                strcat(tempstring, _tr("down"));
            }
            break;
#endif
        case VOICE_SRC_TIMER:
            snprintf(tempstring, sizeof(tempstring), _tr("Timer %d"),
                idx + 1);
            break;
        case VOICE_SRC_TELEMETRY:
            snprintf(tempstring, sizeof(tempstring), _tr("Telem %d"),
                idx - (NUM_TIMERS) + 1);
            break;
        case VOICE_SRC_MIXER:
            INPUT_SourceName(tempstring,
                    (idx + INP_HAS_CALIBRATION + NUM_SWITCHES + 1));
            break;
    }
    return tempstring;
}

static void voice_test_cb(guiObject_t *obj, void *data)
{
    (void) obj;
    u8 idx = (long)data;
    switch(voiceconfig_getsrctype(idx)) {
        case VOICE_SRC_SWITCH:
            if (Model.voice.switches[idx - VOICE_SRC_SWITCH].music)
                MUSIC_Play(Model.voice.switches[idx].music);
            break;
#if NUM_AUX_KNOBS
        case VOICE_SRC_AUX:
            if (Model.voice.aux[idx - VOICE_SRC_AUX].music)
                MUSIC_Play(Model.voice.aux[idx - NUM_SWITCHES].music);
            break;
#endif
        case VOICE_SRC_TIMER:
            if (Model.voice.timer[idx - VOICE_SRC_TIMER].music)
                MUSIC_Play(Model.voice.timer[idx].music);
            break;
        case VOICE_SRC_TELEMETRY:
            if (Model.voice.telemetry[idx - VOICE_SRC_TELEMETRY].music)
                MUSIC_Play(Model.voice.telemetry[idx - NUM_TIMERS].music);
            break;
        case VOICE_SRC_MIXER:
            if (Model.voice.mixer[idx - VOICE_SRC_MIXER].music)
                MUSIC_Play(Model.voice.mixer[idx].music);
            break;
    }
}

static const char *voicelbl_cb(guiObject_t *obj, const void *data)
{
    (void) obj;
    int idx = (long)data;
    switch (voiceconfig_getsrctype(idx)) {
        case VOICE_SRC_SWITCH:
            if (Model.voice.switches[idx].music)
                return voice_map[Model.voice.switches[idx].music].label;
            break;
#if NUM_AUX_KNOBS
        case VOICE_SRC_AUX:
            if(Model.voice.aux[idx - NUM_SWITCHES].music)
                return voice_map[Model.voice.aux[idx - NUM_SWITCHES].music].label;
            break;
#endif
        case VOICE_SRC_TIMER:
            if (Model.voice.timer[idx].music)
                return voice_map[Model.voice.timer[idx].music].label;
            break;
        case VOICE_SRC_TELEMETRY:
            if (Model.voice.telemetry[idx - NUM_TIMERS].music)
                return voice_map[Model.voice.telemetry[idx - NUM_TIMERS].music].label;
            break;
        case VOICE_SRC_MIXER:
            if (Model.voice.mixer[idx].music)
                return voice_map[Model.voice.mixer[idx].music].label;
            break;
    }
    return strcpy(tempstring, "");
}

static const char *voiceid_cb(guiObject_t *obj, int dir, void *data)
{
    (void) obj;
    int idx = (long)data;
    int cur_row = idx - GUI_ScrollableCurrentRow(&gui->scrollable);
    struct CustomVoice *vpt;
    vpt = NULL;

    switch (voiceconfig_getsrctype(idx)) {
        case VOICE_SRC_SWITCH:
            vpt = &Model.voice.switches[idx];
            break;
#if NUM_AUX_KNOBS
        case VOICE_SRC_AUX:
            vpt = &Model.voice.aux[idx - NUM_SWITCHES];
            break;
#endif
        case VOICE_SRC_TIMER:
            vpt = &Model.voice.timer[idx];
            break;
        case VOICE_SRC_TELEMETRY:
            vpt = &Model.voice.telemetry[idx - NUM_TIMERS];
            break;
        case VOICE_SRC_MIXER:
            vpt = &Model.voice.mixer[idx];
            break;
    }
    if (dir == -1 && vpt->music == CUSTOM_ALARM_ID) // set to none below 1
        vpt->music = 0;
    if (dir == 1 && vpt->music == 0) // set to CUSTOM_ALARM_ID when currently none
        vpt->music = CUSTOM_ALARM_ID - 1;
    GUI_TextSelectEnablePress((guiTextSelect_t *)obj, vpt->music);

    if (vpt->music == 0) {
        GUI_Redraw(&gui->voicelbl[cur_row]);
        return strcpy(tempstring, _tr("None"));
    }
    vpt->music = GUI_TextSelectHelper(vpt->music - CUSTOM_ALARM_ID + 1, //Relabling so voice in menu starts with 1
        1, voice_map_entries - CUSTOM_ALARM_ID, dir, 1, 10, NULL) + CUSTOM_ALARM_ID - 1;
    snprintf(tempstring, 5, "%d", vpt->music - CUSTOM_ALARM_ID + 1);
    GUI_Redraw(&gui->voicelbl[cur_row]);
    return tempstring;
}
