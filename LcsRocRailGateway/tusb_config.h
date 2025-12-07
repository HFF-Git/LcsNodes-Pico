


#pragma once

// --- TinyUSB configuration for Channel 1 only ---

#define CFG_TUD_ENDPOINT0_SIZE    64

// Enable CDC
#define CFG_TUD_CDC               1

// Increase to 2 if you want more CDC ports (but we use 1 here)
#define CFG_TUD_CDC_EP_BUFSIZE    64
#define CFG_TUD_CDC_RX_BUFSIZE    256
#define CFG_TUD_CDC_TX_BUFSIZE    256

// No need for MSC, HID, MIDI, etc.
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

#define CFG_TUSB_OS               OPT_OS_PICO

// ------------------ REQUIRED ------------------
// Tell TinyUSB which RHPort is used and in what mode
// On Pico, use RHPort 0 in device mode
#define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE
#define CFG_TUSB_RHPORT1_MODE     0   // not used
