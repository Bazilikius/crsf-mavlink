#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// Board-specific TinyUSB Configuration

#ifndef CFG_TUD_DESC_LEN
#define CFG_TUD_DESC_LEN         64
#endif

// Enable Device stack
#define CFG_TUD_ENABLED          1

// Enable dual CDC ports
#define CFG_TUD_CDC              2

// CDC FIFO size
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256

#ifdef __cplusplus
 }
#endif

#endif // _TUSB_CONFIG_H_
