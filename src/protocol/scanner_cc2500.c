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

#ifdef MODULAR
    // Allows the linker to properly relocate
    #define SCANNER_CC2500_Cmds PROTO_Cmds
    #pragma long_calls
#endif

#include "common.h"
#include "interface.h"
#include "rftools.h"
#include "telemetry.h"
#include "stdlib.h"

#ifdef PROTO_HAS_CC2500
#if SUPPORT_SCANNER

#ifdef MODULAR
    struct Scanner Scanner;
#endif

#define MIN_RADIOCHANNEL    0x00
#define MAX_RADIOCHANNEL    0x80  // testing only, max is 0xFF
#define CHANNEL_LOCK_TIME   100  // with precalibration channel requires  only 90 usec for synthesizer to settle
#define AVERAGE_INTVL       100
#define RSSI_OFFSET         72  // for true dBm values

static int averages, channel, scan_state;
static u8 calibration[MAX_RADIOCHANNEL];
static u8 calibration_fscal2, calibration_fscal3;

enum ScanStates {
    SCAN_CHANNEL_CHANGE = 0,
    SCAN_GET_RSSI,
};

static void cc2500_init()
{
    /* Initialize CC2500 chip */
    CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x00);  // packet automation control
    CC2500_WriteReg(CC2500_08_PKTCTRL0, 0x00);  // packet automation control
    CC2500_WriteReg(CC2500_0A_CHANNR,   0x00);  // channel number
    CC2500_WriteReg(CC2500_0B_FSCTRL1,  0x08);  // frequency synthesizer control
    CC2500_WriteReg(CC2500_0C_FSCTRL0,  0x00);  // frequency synthesizer control
    CC2500_WriteReg(CC2500_0D_FREQ2,    0x5C);  // frequency control word, high byte
    CC2500_WriteReg(CC2500_0E_FREQ1,    0x4E);  // frequency control word, middle byte
    CC2500_WriteReg(CC2500_0F_FREQ0,    0xDE);  // frequency control word, low byte
    CC2500_WriteReg(CC2500_10_MDMCFG4,  0x86);  // modem configuration
    CC2500_WriteReg(CC2500_11_MDMCFG3,  0x83);  // modem configuration
    CC2500_WriteReg(CC2500_12_MDMCFG2,  0x00);  // modem configuration FSK for better sensitivity
    CC2500_WriteReg(CC2500_13_MDMCFG1,  0x23);  // modem configuration
    CC2500_WriteReg(CC2500_14_MDMCFG0,  0xA4);  // modem configuration
    CC2500_WriteReg(CC2500_15_DEVIATN,  0x44);  // modem deviation setting 38.085938
    CC2500_WriteReg(CC2500_17_MCSM1,    0x0F);  // always stay in RX mode
    CC2500_WriteReg(CC2500_18_MCSM0,    0x08);  // disable auto-calibration
    CC2500_WriteReg(CC2500_19_FOCCFG,   0x16);  // frequency offset compensation configuration
    CC2500_WriteReg(CC2500_1A_BSCFG,    0x6C);  // bit synchronization configuration
    CC2500_WriteReg(CC2500_1B_AGCCTRL2, 0x03);  // agc control
    CC2500_WriteReg(CC2500_1C_AGCCTRL1, 0x40);  // agc control
    CC2500_WriteReg(CC2500_1D_AGCCTRL0, 0x91);  // agc control
    CC2500_WriteReg(CC2500_21_FREND1,   0x56);  // front end rx configuration
    CC2500_WriteReg(CC2500_22_FREND0,   0x10);  // front end tx configuration
    CC2500_WriteReg(CC2500_23_FSCAL3,   0xA9);  // frequency synthesizer calibration
    CC2500_WriteReg(CC2500_24_FSCAL2,   0x0A);  // frequency synthesizer calibration
    CC2500_WriteReg(CC2500_25_FSCAL1,   0x00);  // frequency synthesizer calibration
    CC2500_WriteReg(CC2500_26_FSCAL0,   0x11);  // frequency synthesizer calibration
    CC2500_WriteReg(CC2500_2C_TEST2,    0x88);  // various test settings
    CC2500_WriteReg(CC2500_2D_TEST1,    0x31);  // various test settings
    CC2500_WriteReg(CC2500_2E_TEST0,    0x0B);  // various test settings
    CC2500_WriteReg(CC2500_3E_PATABLE,  0xfe);
    CC2500_SetTxRxMode(RX_EN);  // Receive mode
    CC2500_Strobe(CC2500_SIDLE);
    CC2500_Strobe(CC2500_SRX);
    usleep(1000);  // wait for RX to activate
}

static void _calibrate()
{
    for (int c = 0; c < MAX_RADIOCHANNEL; c++) {
        CLOCK_ResetWatchdog();
        CC2500_Strobe(CC2500_SIDLE);
        CC2500_WriteReg(CC2500_0A_CHANNR, c);
        CC2500_Strobe(CC2500_SCAL);
        usleep(900);
        calibration[c] = CC2500_ReadReg(CC2500_25_FSCAL1);
    }
    calibration_fscal3 = CC2500_ReadReg(CC2500_23_FSCAL3);  // only needs to be done once
    calibration_fscal2 = CC2500_ReadReg(CC2500_24_FSCAL2);  // only needs to be done once
    CC2500_Strobe(CC2500_SIDLE);
}

static void _scan_next()
{
    CC2500_WriteReg(CC2500_23_FSCAL3, calibration_fscal3);
    CC2500_WriteReg(CC2500_24_FSCAL2, calibration_fscal2);
    CC2500_WriteReg(CC2500_25_FSCAL1, calibration[channel]);
    CC2500_WriteReg(CC2500_0A_CHANNR, Scanner.chan_min + channel);
    /*
    switch (Scanner.attenuator) {
        case 0: CYRF_WriteRegister(CYRF_06_RX_CFG, 0x4A); break;  // LNA on, ATT off
        case 1: CYRF_WriteRegister(CYRF_06_RX_CFG, 0x0A); break;  // LNA off, ATT off
        default:  CYRF_WriteRegister(CYRF_06_RX_CFG, 0x2A); break;  // LNA off, no ATT on
    }
    */
}

static u8 _scan_rssi()
{
    CC2500_Strobe(CC2500_SFRX);
    //while (CC2500_ReadReg(CC2500_38_PKTSTATUS) & (CC2500_STATE_RX | CC2500_STATE_CALIBRATE)) {
        usleep(10);
    //}
#ifdef EMULATOR
    return rand32() % 0xFF;
#else
    u8 rssi = CC2500_ReadReg(CC2500_34_RSSI);  // 0.5 db/count, RSSI value read from the RSSI status register is a 2’s complement number
    u8 rssi_rel;

    if (rssi >= 128) {
        rssi_rel = rssi - 128;  // relative power levels 0-127 (equals -137 to -72 dBm)
    } else {
        rssi_rel = rssi + 128;  // relativ power levels 128-255 (equals -73 to -10 dBm)
    }
    return rssi_rel;
#endif
}

static u16 scan_cb()
{
    int rssi_value;
    switch (scan_state) {
        case SCAN_CHANNEL_CHANGE:
            averages = 0;
            channel++;
            //CC2500_Strobe(CC2500_SFRX);
            if (channel == (Scanner.chan_max - Scanner.chan_min + 1))
                channel = 0;
            if (Scanner.averaging)
                Scanner.rssi_peak[channel] = 0;  // Reset value for peak mode scans after channel change
            _scan_next();
            scan_state = SCAN_GET_RSSI;
            CC2500_Strobe(CC2500_SRX);
            return CHANNEL_LOCK_TIME;
        case SCAN_GET_RSSI:
            rssi_value = _scan_rssi();
            Scanner.rssi[channel] = (rssi_value + 9 * Scanner.rssi[channel]) / 10;  // fast exponential smoothing with alpha 0.1
            if (rssi_value > Scanner.rssi_peak[channel])
                    Scanner.rssi_peak[channel] = rssi_value;
            averages++;
            if (averages < (abs(Scanner.averaging)))
                return AVERAGE_INTVL + rand32() % 10;  // make measurements slightly random in time
            scan_state = SCAN_CHANNEL_CHANGE;
    }
    return 100;
}

static void initialize()
{
    CLOCK_StopTimer();
    Scanner.chan_min = MIN_RADIOCHANNEL;
    Scanner.chan_max = MAX_RADIOCHANNEL;
    averages = 0;
    channel = 0;
    scan_state = SCAN_CHANNEL_CHANGE;
    memset(Scanner.rssi, 0, sizeof(Scanner.rssi));  // clear old rssi values
    memset(Scanner.rssi_peak, 0, sizeof(Scanner.rssi_peak));  // clear old rssi peak values
    CC2500_Reset();
    cc2500_init();
    _calibrate();  // precalibrate frequency registers for faster channel switching
    CLOCK_StartTimer(1250, scan_cb);
}

uintptr_t SCANNER_CC2500_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
        case PROTOCMD_INIT:  initialize(); return 0;
        case PROTOCMD_DEINIT:
        case PROTOCMD_RESET:
            CLOCK_StopTimer();
            return (CC2500_Reset() ? 1 : -1);
        case PROTOCMD_CHECK_AUTOBIND: return 0;
        case PROTOCMD_BIND: return 0;
        case PROTOCMD_NUMCHAN: return 16;
        case PROTOCMD_DEFAULT_NUMCHAN: return 1;
        case PROTOCMD_CURRENT_ID:  return 0;
        case PROTOCMD_GETOPTIONS:
            return 0;
        case PROTOCMD_SETOPTIONS:
            break;
        case PROTOCMD_TELEMETRYSTATE:
            return PROTO_TELEM_UNSUPPORTED;
        case PROTOCMD_CHANNELMAP:
            return UNCHG;
        default: break;
    }
    return 0;
}

#endif  // SUPPORT_SCANNER
#endif  // PROTO_HAS_CC2500
