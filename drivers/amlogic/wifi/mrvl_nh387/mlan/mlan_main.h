/** @file mlan_main.h
 *
 *  @brief This file defines the private and adapter data
 *  structures and declares global function prototypes used
 *  in MLAN module.
 *
 *  Copyright (C) 2008-2010, Marvell International Ltd. 
 *  All Rights Reserved
 */

/******************************************************
Change log:
    10/13/2008: initial version
******************************************************/

#ifndef _MLAN_MAIN_H_
#define _MLAN_MAIN_H_

#ifdef DEBUG_LEVEL1
extern t_void(*print_callback) (IN t_void * pmoal_handle,
                                IN t_u32 level, IN t_s8 * pformat, IN ...);
/** Log debug message */
#ifdef __GNUC__
#ifdef DEBUG_LEVEL2
#define PRINTM(level, pformat, args...) \
do {                                    \
    if (print_callback) \
        print_callback(MNULL, level, pformat, ## args); \
} while (0)
#else
#define PRINTM(level, pformat, args...) \
do {                                    \
    if ((print_callback) && (level != MINFO) && (level != MWARN)) \
        print_callback(MNULL, level, pformat, ## args); \
} while (0)
#endif
#else
#ifdef DEBUG_LEVEL2
#define PRINTM(level, pformat, ...) \
do {                                    \
    if (print_callback)  \
        print_callback(MNULL, level, pformat, __VA_ARGS__); \
} while (0)
#else
#define PRINTM(level, pformat, ...) \
do {                                    \
    if ((print_callback) && (level != MINFO) && (level != MWARN)) \
        print_callback(MNULL, level, pformat, __VA_ARGS__); \
} while (0)
#endif
#endif /* __GNUC__ */

/** Max hex dump data length */
#define MAX_DATA_DUMP_LEN	48

/** Debug hexdump for level-1 debugging */
#define DBG_HEXDUMP(level,x,y,z)   \
do {                \
    if (print_callback)  \
        print_callback(MNULL, MHEX_DUMP | level, x, y, z); \
} while (0)
#else
/** Log debug message */
#define PRINTM(level, pformat, args...) do {} while (0)

/** Debug hexdump for level-1 debugging */
#define DBG_HEXDUMP(level,x,y,z) do {} while (0)
#endif /* DEBUG_LEVEL1 */

#ifdef DEBUG_LEVEL2
/** Log entry point for debugging */
#define ENTER()     \
do {                \
        PRINTM(MENTRY, "Enter: %s\n", __FUNCTION__);   \
} while (0)

/** Log exit point for debugging */
#define LEAVE()     \
do {                \
        PRINTM(MENTRY, "Leave: %s\n", __FUNCTION__);   \
} while (0)

/** Hexdump for level-2 debugging */
#define HEXDUMP(x,y,z)   \
do {                \
    if (print_callback)  \
        print_callback(MNULL, MHEX_DUMP | MINFO, x, y, z); \
} while (0)
#else
/** Log entry point for debugging */
#define ENTER() do {} while (0)

/** Log exit point for debugging */
#define LEAVE() do {} while (0)

/** Hexdump for debugging */
#define HEXDUMP(x,y,z) do {} while (0)
#endif /* DEBUG_LEVEL2 */

/** Find minimum */
#ifndef MIN
#define MIN(a,b)		((a) < (b) ? (a) : (b))
#endif

/** Find maximum */
#ifndef MAX
#define MAX(a,b)		((a) > (b) ? (a) : (b))
#endif

#ifdef memset
#undef memset
#endif
/** Memset routine */
#define memset(adapter, s, c, len) \
  adapter->callbacks.moal_memset(adapter->pmoal_handle, s, c, len)

#ifdef memmove
#undef memmove
#endif
/** Memmove routine */
#define memmove(adapter, dest, src, len) \
  adapter->callbacks.moal_memmove(adapter->pmoal_handle, dest, src, len)

#ifdef memcpy
#undef memcpy
#endif
/** Memcpy routine */
#define memcpy(adapter, to, from, len) \
  adapter->callbacks.moal_memcpy(adapter->pmoal_handle, to, from, len)

#ifdef memcmp
#undef memcmp
#endif
/** Memcmp routine */
#define memcmp(adapter, s1, s2, len) \
  adapter->callbacks.moal_memcmp(adapter->pmoal_handle, s1, s2, len)

/** Find number of elements */
#ifndef NELEMENTS
#define NELEMENTS(x)    (sizeof(x)/sizeof(x[0]))
#endif

/** SWAP: swap t_u8 */
#define SWAP_U8(a,b)	{t_u8 t; t=a; a=b; b=t;}

/** SWAP: swap t_u8 */
#define SWAP_U16(a,b)	{t_u16 t; t=a; a=b; b=t;}

/** MLAN MNULL pointer */
#define MNULL                           (0)

/** 16 bits byte swap */
#define swap_byte_16(x) \
((t_u16)((((t_u16)(x) & 0x00ffU) << 8) | \
         (((t_u16)(x) & 0xff00U) >> 8)))

/** 32 bits byte swap */
#define swap_byte_32(x) \
((t_u32)((((t_u32)(x) & 0x000000ffUL) << 24) | \
         (((t_u32)(x) & 0x0000ff00UL) <<  8) | \
         (((t_u32)(x) & 0x00ff0000UL) >>  8) | \
         (((t_u32)(x) & 0xff000000UL) >> 24)))

/** 64 bits byte swap */
#define swap_byte_64(x) \
((t_u64)((t_u64)(((t_u64)(x) & 0x00000000000000ffULL) << 56) | \
         (t_u64)(((t_u64)(x) & 0x000000000000ff00ULL) << 40) | \
         (t_u64)(((t_u64)(x) & 0x0000000000ff0000ULL) << 24) | \
         (t_u64)(((t_u64)(x) & 0x00000000ff000000ULL) <<  8) | \
         (t_u64)(((t_u64)(x) & 0x000000ff00000000ULL) >>  8) | \
         (t_u64)(((t_u64)(x) & 0x0000ff0000000000ULL) >> 24) | \
         (t_u64)(((t_u64)(x) & 0x00ff000000000000ULL) >> 40) | \
         (t_u64)(((t_u64)(x) & 0xff00000000000000ULL) >> 56) ))

#ifdef BIG_ENDIAN_SUPPORT
/** Convert ulong n/w to host */
#define mlan_ntohl(x) x
/** Convert host ulong to n/w */
#define mlan_htonl(x) x
/** Convert n/w to host */
#define mlan_ntohs(x)  x
/** Convert host to n/w */
#define mlan_htons(x)  x
/** Convert from 16 bit little endian format to CPU format */
#define wlan_le16_to_cpu(x) swap_byte_16(x)
/** Convert from 32 bit little endian format to CPU format */
#define wlan_le32_to_cpu(x) swap_byte_32(x)
/** Convert from 64 bit little endian format to CPU format */
#define wlan_le64_to_cpu(x) swap_byte_64(x)
/** Convert to 16 bit little endian format from CPU format */
#define wlan_cpu_to_le16(x) swap_byte_16(x)
/** Convert to 32 bit little endian format from CPU format */
#define wlan_cpu_to_le32(x) swap_byte_32(x)
/** Convert to 64 bit little endian format from CPU format */
#define wlan_cpu_to_le64(x) swap_byte_64(x)

/** Convert TxPD to little endian format from CPU format */
#define endian_convert_TxPD(x);                                         \
    {                                                                   \
        (x)->tx_pkt_length = wlan_cpu_to_le16((x)->tx_pkt_length);      \
        (x)->tx_pkt_offset = wlan_cpu_to_le16((x)->tx_pkt_offset);      \
        (x)->tx_pkt_type   = wlan_cpu_to_le16((x)->tx_pkt_type);        \
        (x)->tx_control    = wlan_cpu_to_le32((x)->tx_control);         \
    }

/** Convert RxPD from little endian format to CPU format */
#define endian_convert_RxPD(x);                                         \
    {                                                                   \
        (x)->rx_pkt_length = wlan_le16_to_cpu((x)->rx_pkt_length);      \
        (x)->rx_pkt_offset = wlan_le16_to_cpu((x)->rx_pkt_offset);      \
        (x)->rx_pkt_type   = wlan_le16_to_cpu((x)->rx_pkt_type);        \
        (x)->seq_num       = wlan_le16_to_cpu((x)->seq_num);            \
    }
#else
/** Convert ulong n/w to host */
#define mlan_ntohl(x) swap_byte_32(x)
/** Convert host ulong to n/w */
#define mlan_htonl(x) swap_byte_32(x)
/** Convert n/w to host */
#define mlan_ntohs(x) swap_byte_16(x)
/** Convert host to n/w */
#define mlan_htons(x) swap_byte_16(x)
/** Do nothing */
#define wlan_le16_to_cpu(x) x
/** Do nothing */
#define wlan_le32_to_cpu(x) x
/** Do nothing */
#define wlan_le64_to_cpu(x) x
/** Do nothing */
#define wlan_cpu_to_le16(x) x
/** Do nothing */
#define wlan_cpu_to_le32(x) x
/** Do nothing */
#define wlan_cpu_to_le64(x) x

/** Convert TxPD to little endian format from CPU format */
#define endian_convert_TxPD(x)  do {} while (0)
/** Convert RxPD from little endian format to CPU format */
#define endian_convert_RxPD(x)  do {} while (0)
#endif /* BIG_ENDIAN_SUPPORT */

/** Global moal_assert_callback */
extern t_void(*assert_callback) (IN t_void * pmoal_handle, IN t_u32 cond);

/** Assertion */
#define MASSERT(cond)                   \
do {                                    \
    if (!(cond)) {                      \
        PRINTM(MFATAL, "ASSERT: %s: %i\n", __FUNCTION__, __LINE__); \
        if (assert_callback) {          \
            assert_callback(MNULL, (t_u32)(cond)); \
        } else {                        \
            do {} while(1);             \
        }                               \
    }                                   \
} while(0)

/** Header alignment */
#define HEADER_ALIGNMENT                8

/** Upload size */
#define WLAN_UPLD_SIZE                  (2312)

/** Maximum event buffer size */
#define MAX_EVENT_SIZE                  1024

#ifdef STA_SUPPORT
/** Maximum buffer size for ARP filter */
#define ARP_FILTER_MAX_BUF_SIZE     	68
#endif /* STA_SUPPORT */

/** 60 seconds */
#define MRVDRV_TIMER_60S                60000
/** 10 seconds */
#define MRVDRV_TIMER_10S                10000
/** 5 seconds */
#define MRVDRV_TIMER_5S                 5000
/** 1 second */
#define MRVDRV_TIMER_1S                 1000

/** Maximum size of multicast list */
#define MRVDRV_MAX_MULTICAST_LIST_SIZE  32
/** Maximum size of channel */
#define MRVDRV_MAX_CHANNEL_SIZE         14
/** Maximum length of SSID */
#define MRVDRV_MAX_SSID_LENGTH          32
/** WEP list macros & data structures */
/** Size of key buffer in bytes */
#define MRVL_KEY_BUFFER_SIZE_IN_BYTE    16
/** Maximum length of WPA key */
#define MRVL_MAX_KEY_WPA_KEY_LENGTH     32

/** Default listen interval */
#define MLAN_DEFAULT_LISTEN_INTERVAL    10

/** Maximum number of region codes */
#define MRVDRV_MAX_REGION_CODE          8

/** Default factor for calculating beacon average */
#define DEFAULT_BCN_AVG_FACTOR          8
/** Default factor for calculating data average */
#define DEFAULT_DATA_AVG_FACTOR         8

/** The first valid channel for use */
#define FIRST_VALID_CHANNEL          0xff
/** Default Ad-Hoc channel */
#define DEFAULT_AD_HOC_CHANNEL       6
/** Default Ad-Hoc channel A */
#define DEFAULT_AD_HOC_CHANNEL_A     36

/** Number of WEP keys */
#define MRVL_NUM_WEP_KEY                (4)

/** Default multiple DTIM */
#define MRVDRV_DEFAULT_MULTIPLE_DTIM        1

/** Default beacon missing timeout */
#define DEFAULT_BCN_MISS_TIMEOUT            5

/**
 *  Maximum buffer space for beacons retrieved from scan responses
 */
#define MAX_SCAN_BEACON_BUFFER          8000

/**
 * @brief Buffer pad space for newly allocated beacons/probe responses
 *
 * Beacons are typically 6 bytes longer than an equivalent probe response.
 *  For each scan response stored, allocate an extra byte pad at the end to
 *  allow easy expansion to store a beacon in the same memory a probe response
 *  previously contained
 */
#define SCAN_BEACON_ENTRY_PAD          6

/** Scan time specified in the channel TLV for each channel for passive scans */
#define MRVDRV_PASSIVE_SCAN_CHAN_TIME       200

/** Scan time specified in the channel TLV for each channel for active scans */
#define MRVDRV_ACTIVE_SCAN_CHAN_TIME        200

/** Scan time specified in the channel TLV for each channel for specific scans */
#define MRVDRV_SPECIFIC_SCAN_CHAN_TIME      110

/**
 * Max total scan time in milliseconds
 * The total scan time should be less than scan command timeout value (10s)
 */
#define MRVDRV_MAX_TOTAL_SCAN_TIME     (MRVDRV_TIMER_10S - MRVDRV_TIMER_1S)

/** Offset for GTK as it has version to skip past for GTK */
#define RSN_GTK_OUI_OFFSET 2

/** If OUI is not found */
#define MLAN_OUI_NOT_PRESENT 0
/** If OUI is found */
#define MLAN_OUI_PRESENT 1

/** RF antenna select 1 */
#define RF_ANTENNA_1		0x1
/** RF antenna auto select */
#define RF_ANTENNA_AUTO		0xFFFF

/** Is cmd_resp, event or data packet received? */
#define IS_CARD_RX_RCVD(adapter) (adapter->cmd_resp_received || \
                                  adapter->event_received || \
                                  adapter->data_received)

/** Type command */
#define MLAN_TYPE_CMD			1
/** Type data */
#define MLAN_TYPE_DATA			0
/** Type event */
#define MLAN_TYPE_EVENT			3

/** Maximum numbfer of registers to read for multiple port */
#define MAX_MP_REGS			64
/** Maximum port */
#define MAX_PORT			16

/** Multi port aggregation packet limit */
#define SDIO_MP_AGGR_DEF_PKT_LIMIT       (8)

#ifdef SDIO_MULTI_PORT_TX_AGGR
/** Multi port TX aggregation buffer size */
#define SDIO_MP_TX_AGGR_DEF_BUF_SIZE        (8192)      /* 8K */
#endif /* SDIO_MULTI_PORT_TX_AGGR */

#ifdef SDIO_MULTI_PORT_RX_AGGR
/** Multi port RX aggregation buffer size */
#define SDIO_MP_RX_AGGR_DEF_BUF_SIZE        (16384)     /* 16K */
#endif /* SDIO_MULTI_PORT_RX_AGGR */

/** Hight threashold to stop data traffic */
#define  RX_HIGH_THRESHOLD           60
/** Low threashold to restart data traffic */
#define  RX_LOW_THRESHOLD            15

/** Debug command number */
#define DBG_CMD_NUM	5

/** Max bitmap rate size */
#define MAX_BITMAP_RATES_SIZE 10

/** Info for debug purpose */
typedef struct _wlan_dbg
{
    /** Number of host to card command failures */
    t_u32 num_cmd_host_to_card_failure;
    /** Number of host to card sleep confirm failures */
    t_u32 num_cmd_sleep_cfm_host_to_card_failure;
    /** Number of host to card Tx failures */
    t_u32 num_tx_host_to_card_failure;
    /** Number of deauthentication events */
    t_u32 num_event_deauth;
    /** Number of disassosiation events */
    t_u32 num_event_disassoc;
    /** Number of link lost events */
    t_u32 num_event_link_lost;
    /** Number of deauthentication commands */
    t_u32 num_cmd_deauth;
    /** Number of association comamnd successes */
    t_u32 num_cmd_assoc_success;
    /** Number of association command failures */
    t_u32 num_cmd_assoc_failure;
    /** Number of Tx timeouts */
    t_u32 num_tx_timeout;
    /** Number of command timeouts */
    t_u32 num_cmd_timeout;
    /** Timeout command ID */
    t_u16 timeout_cmd_id;
    /** Timeout command action */
    t_u16 timeout_cmd_act;
    /** List of last command IDs */
    t_u16 last_cmd_id[DBG_CMD_NUM];
    /** List of last command actions */
    t_u16 last_cmd_act[DBG_CMD_NUM];
    /** Last command index */
    t_u16 last_cmd_index;
    /** List of last command response IDs */
    t_u16 last_cmd_resp_id[DBG_CMD_NUM];
    /** Last command response index */
    t_u16 last_cmd_resp_index;
    /** List of last events */
    t_u16 last_event[DBG_CMD_NUM];
    /** Last event index */
    t_u16 last_event_index;
} wlan_dbg;

/** Hardware status codes */
typedef enum _WLAN_HARDWARE_STATUS
{
    WlanHardwareStatusReady,
    WlanHardwareStatusInitializing,
    WlanHardwareStatusInitdone,
    WlanHardwareStatusReset,
    WlanHardwareStatusClosing,
    WlanHardwareStatusNotReady
} WLAN_HARDWARE_STATUS;

/** WLAN_802_11_POWER_MODE */
typedef enum _WLAN_802_11_POWER_MODE
{
    Wlan802_11PowerModeCAM,
    Wlan802_11PowerModePSP
} WLAN_802_11_POWER_MODE;

/** tx param */
typedef struct _mlan_tx_param
{
    /** next packet length */
    t_u32 next_pkt_len;
} mlan_tx_param;

/** PS_STATE */
typedef enum _PS_STATE
{
    PS_STATE_AWAKE,
    PS_STATE_PRE_SLEEP,
    PS_STATE_SLEEP_CFM,
    PS_STATE_SLEEP
} PS_STATE;

/** Minimum flush timer for win size of 1 is 50 ms */
#define MIN_FLUSH_TIMER_MS 50
/** Tx BA stream table */
typedef struct _TxBAStreamTbl TxBAStreamTbl;

/** Add BA parameter data structure */
typedef struct
{
    /** Window size for initiator */
    t_u32 tx_win_size;
    /** Window size for receiver */
    t_u32 rx_win_size;
    /** Block ack timeout */
    t_u32 timeout;
} add_ba_param_t;

/** Tx aggregation data structure */
typedef struct _txAggr_t
{
    /** AMPDU user */
    t_u8 ampdu_user;
    /** AMPDU AP */
    t_u8 ampdu_ap;
    /** AMSDU */
    t_u8 amsdu;
} tx_aggr_t;

/** RA list table */
typedef struct _raListTbl raListTbl;

/** RA list table */
struct _raListTbl
{
    /** Pointer to previous node */
    raListTbl *pprev;
    /** Pointer to next node */
    raListTbl *pnext;
    /** Buffer list head */
    mlan_list_head buf_head;
    /** RA list buffer */
    t_u8 ra[MLAN_MAC_ADDR_LENGTH];

    /** total size of packets in RA list */
    t_u32 total_pkts_size;
        /** packets received */
    t_u16 packet_count;
        /** packet count threshold to setup BA */
    t_u8 ba_packet_threshold;
        /** is 11n enabled */
    t_u32 is_11n_enabled;
};

/** TID table */
typedef struct _tidTbl
{
    /** RA list head */
    mlan_list_head ra_list;
    /** Current RA list */
    raListTbl *ra_list_curr;
} tid_tbl_t;

/** Highest priority setting for a packet (uses voice AC) */
#define WMM_HIGHEST_PRIORITY  7
/** HIGHEST priority TID  */
#define HIGH_PRIO_TID   7
/** LEAST priority TID    */
#define LOW_PRIO_TID    0

/** Struct of WMM DESC */
typedef struct _wmm_desc
{
    /** TID table */
    tid_tbl_t tid_tbl_ptr[MAX_NUM_TID];
    /** Packets out */
    t_u32 packets_out[MAX_NUM_TID];
    /** Spin lock to protect ra_list */
    t_void *ra_list_spinlock;

    /** AC status */
    WmmAcStatus_t ac_status[MAX_AC_QUEUES];
    /** AC downgraded values */
    mlan_wmm_ac_e ac_down_graded_vals[MAX_AC_QUEUES];

    /** Max driver packet delay sent to the firmware for expiry eval */
    t_u32 drv_pkt_delay_max;

    /** WMM queue priority table */
    t_u8 queue_priority[MAX_AC_QUEUES];
    /** User priority packet transmission control */
    t_u32 user_pri_pkt_tx_ctrl[WMM_HIGHEST_PRIORITY + 1];       /* UP: 0 to 7 */

    /** Number of transmit packets queued */
    mlan_scalar tx_pkts_queued;
    /** Tracks highest priority with a packet queued */
    mlan_scalar highest_queued_prio;
} wmm_desc_t;

/** Security structure */
typedef struct _wlan_802_11_security_t
{
    /** WPA enabled flag */
    t_u8 wpa_enabled;
    /** E-Supplicant enabled flag */
    t_u8 ewpa_enabled;
    /** WPA2 enabled flag */
    t_u8 wpa2_enabled;
    /** WAPI enabled flag */
    t_u8 wapi_enabled;
    /** WAPI key on flag */
    t_u8 wapi_key_on;
    /** WEP status */
    WLAN_802_11_WEP_STATUS wep_status;
    /** Authentication mode */
    t_u32 authentication_mode;
    /** Encryption mode */
    t_u32 encryption_mode;
} wlan_802_11_security_t;

/** Current Basic Service Set State Structure */
typedef struct
{
    /** BSS descriptor */
    BSSDescriptor_t bss_descriptor;
    /** WMM enable? */
    t_u8 wmm_enabled;
    /** Uapsd enable?*/
    t_u8 wmm_uapsd_enabled;
    /** Band */
    t_u8 band;
    /** Number of rates supported */
    t_u32 num_of_rates;
    /** Supported rates*/
    t_u8 data_rates[WLAN_SUPPORTED_RATES];
} current_bss_params_t;

/** Sleep_params */
typedef struct _sleep_params_t
{
    /** Sleep parameter error */
    t_u16 sp_error;
    /** Sleep parameter offset */
    t_u16 sp_offset;
    /** Sleep parameter stable time */
    t_u16 sp_stable_time;
    /** Sleep parameter calibration control */
    t_u8 sp_cal_control;
    /** Sleep parameter external sleep clock */
    t_u8 sp_ext_sleep_clk;
    /** Sleep parameter reserved */
    t_u16 sp_reserved;
} sleep_params_t;

/** Sleep_period */
typedef struct sleep_period_t
{
    /** Sleep period */
    t_u16 period;
    /** Reserved */
    t_u16 reserved;
} sleep_period_t;

/** mrvl_wep_key_t */
typedef struct _mrvl_wep_key_t
{
    /** Length */
    t_u32 length;
    /** WEP key index */
    t_u32 key_index;
    /** WEP key length */
    t_u32 key_length;
    /** WEP keys */
    t_u8 key_material[MRVL_KEY_BUFFER_SIZE_IN_BYTE];
} mrvl_wep_key_t;

/** Maximum number of region channel */
#define MAX_REGION_CHANNEL_NUM  2

/** Chan-Freq-TxPower mapping table*/
typedef struct _chan_freq_power_t
{
    /** Channel Number */
    t_u16 channel;
    /** Frequency of this Channel */
    t_u32 freq;
    /** Max allowed Tx power level */
    t_u16 max_tx_power;
    /** TRUE:channel unsupported;  FLASE:supported */
    t_u8 unsupported;
} chan_freq_power_t;

/** Region-band mapping table */
typedef struct _region_chan_t
{
    /** TRUE if this entry is valid */
    t_u8 valid;
    /** Region code for US, Japan ... */
    t_u8 region;
    /** Band B/G/A, used for BAND_CONFIG cmd */
    t_u8 band;
    /** Actual No. of elements in the array below */
    t_u8 num_cfp;
    /** chan-freq-txpower mapping table */
    chan_freq_power_t *pcfp;
} region_chan_t;

#ifdef STA_SUPPORT
/** State of 11d */
typedef enum
{
    DISABLE_11D = 0,
    ENABLE_11D = 1,
} state_11d_t;

#define DEFAULT_11D_STATE   DISABLE_11D

/** Domain regulatory information */
typedef struct _wlan_802_11d_domain_reg
{
    /** Country Code */
    t_u8 country_code[COUNTRY_CODE_LEN];
    /** No. of subband */
    t_u8 no_of_sub_band;
    /** Subband data */
    IEEEtypes_SubbandSet_t sub_band[MRVDRV_MAX_SUBBAND_802_11D];
} wlan_802_11d_domain_reg_t;

/** Data for state machine */
typedef struct _wlan_802_11d_state
{
    /** True for enabling 11D */
    state_11d_t enable_11d;
    /** True for user enabling 11D */
    state_11d_t user_enable_11d;
} wlan_802_11d_state_t;
#endif

/** Vendor specific configuration IE */
typedef struct _vendor_spec_cfg_ie
{
    /** Bit 0-2: scan/assoc/ad-hoc */
    t_u16 mask;
    /** Optional, 0/1: gen_ie/vs_ie */
    t_u16 flag;
    /** Information element */
    t_u8 ie[MLAN_MAX_VSIE_LEN];
} vendor_spec_cfg_ie;

/** Data structure for WPS information */
typedef struct
{
    /** Session enable flag */
    t_u8 session_enable;
} wps_t;

/** mlan_operations data structure */
typedef struct _mlan_operations
{
    /** cmd init handler */
    mlan_status(*init_cmd) (IN t_void * priv, IN t_u8 first_sta);
    /** ioctl handler */
    mlan_status(*ioctl) (t_void * adapter, pmlan_ioctl_req pioctl_req);
    /** cmd handler */
    mlan_status(*prepare_cmd) (IN t_void * priv,
                               IN t_u16 cmd_no,
                               IN t_u16 cmd_action,
                               IN t_u32 cmd_oid,
                               IN t_void * pioctl_buf,
                               IN t_void * pdata_buf, IN t_void * pcmd_buf);
    /** cmdresp handler */
    mlan_status(*process_cmdresp) (IN t_void * priv,
                                   IN t_u16 cmdresp_no,
                                   IN t_void * pcmd_buf, IN t_void * pioctl);
    /** rx handler */
    mlan_status(*process_rx_packet) (IN t_void * adapter,
                                     IN pmlan_buffer pmbuf);
    /** event handler */
    mlan_status(*process_event) (IN t_void * priv);
    /** txpd handler */
    t_void *(*process_txpd) (IN t_void * priv, IN pmlan_buffer pmbuf);
    /** BSS role */
    mlan_bss_role bss_role;
} mlan_operations;

/** Private structure for MLAN */
typedef struct _mlan_private
{
    /** Pointer to mlan_adapter */
    struct _mlan_adapter *adapter;
    /** BSS index */
    t_u8 bss_index;
    /** BSS type */
    t_u8 bss_type;
    /** BSS role */
    t_u8 bss_role;
    /** BSS Priority */
    t_u8 bss_priority;
    /** BSS number */
    t_u8 bss_num;
    /** Frame type */
    t_u8 frame_type;
    /** MAC address information */
    t_u8 curr_addr[MLAN_MAC_ADDR_LENGTH];
    /** Media connection status */
    t_u8 media_connected;

    /** Current packet filter */
    t_u16 curr_pkt_filter;
    /** Infrastructure mode */
    t_u32 bss_mode;

    /** Tx packet control */
    t_u32 pkt_tx_ctrl;

    /** Tx power level */
    t_u16 tx_power_level;
    /** Maximum Tx power level */
    t_u8 max_tx_power_level;
    /** Minimum Tx power level */
    t_u8 min_tx_power_level;
    /** Tx rate */
    t_u8 tx_rate;
#ifdef STA_SUPPORT
    /** tx ht_info */
    t_u8 tx_htinfo;
    /** rxpd_htinfo */
    t_u8 rxpd_htinfo;
#endif                          /* STA_SUPPORT */
#ifdef UAP_SUPPORT
    /** UAP 11n flag */
    t_u8 is_11n_enabled;
    /** packet forward control */
    t_u8 pkt_fwd;
#endif                          /* UAP_SUPPORT */
    /** Rx PD rate */
    t_u8 rxpd_rate;
    /** Rate bitmap */
    t_u16 rate_bitmap;
    /** Bitmap rates */
    t_u16 bitmap_rates[MAX_BITMAP_RATES_SIZE];
    /** Data rate */
    t_u32 data_rate;
    /** Automatic data rate flag */
    t_u8 is_data_rate_auto;
    /** Factor for calculating beacon average */
    t_u16 bcn_avg_factor;
    /** Factor for calculating data average */
    t_u16 data_avg_factor;
    /** Last data RSSI */
    t_s16 data_rssi_last;
    /** Last data Noise Floor */
    t_s16 data_nf_last;
    /** Average data RSSI */
    t_s16 data_rssi_avg;
    /** Averag data Noise Floor */
    t_s16 data_nf_avg;
    /** Last beacon RSSI */
    t_s16 bcn_rssi_last;
    /** Last beacon Noise Floor */
    t_s16 bcn_nf_last;
    /** Average beacon RSSI */
    t_s16 bcn_rssi_avg;
    /** Average beacon Noise Floor */
    t_s16 bcn_nf_avg;

    /** Attempted BSS descriptor */
    BSSDescriptor_t *pattempted_bss_desc;
    /** Prevous SSID */
    mlan_802_11_ssid prev_ssid;
    /** Previous BSSID */
    t_u8 prev_bssid[MLAN_MAC_ADDR_LENGTH];

    /** Current SSID/BSSID related parameters*/
    current_bss_params_t curr_bss_params;

    /** Beacon period */
    t_u16 beacon_period;
    /** Listen interval */
    t_u16 listen_interval;
    /** ATIM window */
    t_u16 atim_window;

    /** AdHoc channel */
    t_u8 adhoc_channel;
    /** AdHoc link sensed flag */
    t_u8 adhoc_is_link_sensed;
    /** AdHoc G rate */
    t_u8 adhoc_state;
    /** AdHoc autoselect */
    t_u8 adhoc_auto_sel;
    /** Received CHANNEL_SWITCH_ANN event */
    t_u8 recvd_chanswann_event;

    /** Security related */
    /** Encryption parameter */
    wlan_802_11_security_t sec_info;
    /** WEP keys */
    mrvl_wep_key_t wep_key[MRVL_NUM_WEP_KEY];
    /** Current WEP key index */
    t_u16 wep_key_curr_index;
    /** EWPA query 0: disable, 1: enable */
    t_u8 ewpa_query;
    /** Encryption Key*/
    t_u8 wpa_ie[256];
    /** WPA IE length */
    t_u8 wpa_ie_len;
    /** GTK set flag */
    t_u8 wpa_is_gtk_set;
    /** AES key material */
    HostCmd_DS_802_11_KEY_MATERIAL aes_key;
    /** WAPI IE */
    t_u8 wapi_ie[256];
    /** WAPI IE length */
    t_u8 wapi_ie_len;
#ifdef UAP_SUPPORT
    /** Pointer to the station table */
    mlan_list_head sta_list;
#endif                          /* UAP_SUPPORT */

    /** MGMT IE */
    custom_ie mgmt_ie[MAX_MGMT_IE_INDEX];
    /** Advanced Encryption Standard */
    t_u8 adhoc_aes_enabled;
    /** WMM required */
    t_u8 wmm_required;
    /** WMM enabled */
    t_u8 wmm_enabled;
    /** WMM qos info */
    t_u8 wmm_qosinfo;
    /** WMM related variable*/
    wmm_desc_t wmm;

    /** Pointer to the Transmit BA stream table*/
    mlan_list_head tx_ba_stream_tbl_ptr;
    /** Pointer to the priorities for AMSDU/AMPDU table*/
    tx_aggr_t aggr_prio_tbl[MAX_NUM_TID];
    /** Pointer to the priorities for AMSDU/AMPDU table*/
    t_u8 addba_reject[MAX_NUM_TID];
    /** Struct to store ADDBA parameters */
    add_ba_param_t add_ba_param;
    /* last rx_seq */
    t_u16 rx_seq[MAX_NUM_TID];
    /** Pointer to the Receive Reordering table*/
    mlan_list_head rx_reorder_tbl_ptr;
    /** Lock for Rx packets */
    t_void *rx_pkt_lock;

#ifdef STA_SUPPORT
    /** Buffer to store the association response for application retrieval */
    t_u8 assoc_rsp_buf[MRVDRV_ASSOC_RSP_BUF_SIZE];
    /** Length of the data stored in assoc_rsp_buf */
    t_u32 assoc_rsp_size;

    /** Generic IEEE IEs passed from the application to be inserted into the
     *    association request to firmware 
     */
    t_u8 gen_ie_buf[MRVDRV_GENIE_BUF_SIZE];
    /** Length of the data stored in gen_ie_buf */
    t_u8 gen_ie_buf_len;
    /** Vendor specific configuration IEs */
    vendor_spec_cfg_ie vs_ie[MLAN_MAX_VSIE_NUM];

    t_u8 *pcurr_bcn_buf;
    t_u32 curr_bcn_size;
    t_void *curr_bcn_buf_lock;

    /** WPS */
    wps_t wps;
#endif                          /* STA_SUPPORT */
    /** function table */
    mlan_operations ops;

    /** Port Control mode */
    t_u8 port_ctrl_mode;

    /** Port open flag */
    t_u8 port_open;

    /** Port open flag state at time of association attempt */
    t_u8 prior_port_status;

    /** Scan block flag */
    t_u8 scan_block;

} mlan_private, *pmlan_private;

/** BA stream status */
typedef enum
{
    BA_STREAM_NOT_SETUP = 0,
    BA_STREAM_SETUP_INPROGRESS,
    BA_STREAM_SETUP_COMPLETE
} baStatus_e;

/** Tx BA stream table */
struct _TxBAStreamTbl
{
    /** TxBAStreamTbl previous node */
    TxBAStreamTbl *pprev;
    /** TxBAStreamTbl next node */
    TxBAStreamTbl *pnext;
    /** TID */
    int tid;
    /** RA */
    t_u8 ra[MLAN_MAC_ADDR_LENGTH];
    /** BA stream status */
    baStatus_e ba_status;
};

/** RX reorder table */
typedef struct _RxReorderTbl RxReorderTbl;

typedef struct
{
    /** Timer for flushing */
    t_void *timer;
    /** Timer set flag */
    t_u8 timer_is_set;
    /** RxReorderTbl ptr */
    RxReorderTbl *ptr;
    /** Priv pointer */
    mlan_private *priv;
} reorder_tmr_cnxt_t;

/** RX reorder table */
struct _RxReorderTbl
{
    /** RxReorderTbl previous node */
    RxReorderTbl *pprev;
    /** RxReorderTbl next node */
    RxReorderTbl *pnext;
    /** TID */
    int tid;
    /** TA */
    t_u8 ta[MLAN_MAC_ADDR_LENGTH];
    /** Start window */
    int start_win;
    /** Window size */
    int win_size;
    /** Pointer to pointer to RxReorderTbl */
    t_void **rx_reorder_ptr;
    /** Timer context */
    reorder_tmr_cnxt_t timer_context;
};

/** BSS priority node */
typedef struct _mlan_bssprio_node mlan_bssprio_node;

/** BSS priority node */
struct _mlan_bssprio_node
{
    /** Pointer to previous node */
    mlan_bssprio_node *pprev;
    /** Pointer to next node */
    mlan_bssprio_node *pnext;
    /** Pointer to priv */
    pmlan_private priv;
};

/** BSS priority table */
typedef struct _mlan_bssprio_tbl mlan_bssprio_tbl;

/** BSS priority table */
struct _mlan_bssprio_tbl
{
    /** BSS priority list head */
    mlan_list_head bssprio_head;
    /** Current priority node */
    mlan_bssprio_node *bssprio_cur;
};

/** cmd_ctrl_node */
typedef struct _cmd_ctrl_node cmd_ctrl_node;

/** _cmd_ctrl_node */
struct _cmd_ctrl_node
{
    /** Pointer to previous node */
    cmd_ctrl_node *pprev;
    /** Pointer to next node */
    cmd_ctrl_node *pnext;
    /** Pointer to priv */
    pmlan_private priv;
    /** Command OID for sub-command use */
    t_u32 cmd_oid;
    /** Command flag */
    t_u32 cmd_flag;
    /** Pointer to mlan_buffer */
    mlan_buffer *cmdbuf;
    /** Pointer to mlan_buffer */
    mlan_buffer *respbuf;
    /** Command parameter */
    t_void *pdata_buf;
    /** Pointer to mlan_ioctl_req if command is from IOCTL */
    t_void *pioctl_buf;
    /** pre_allocated mlan_buffer for cmd */
    mlan_buffer *pmbuf;
};

#ifdef UAP_SUPPORT
/** station node */
typedef struct _sta_node sta_node;

/** station node*/
struct _sta_node
{
    /** previous node */
    sta_node *pprev;
    /** next node */
    sta_node *pnext;
    /** station mac address */
    t_u8 mac_addr[MLAN_MAC_ADDR_LENGTH];
    /** 11n flag */
    t_u8 is_11n_enabled;
    /** AMPDU STA */
    t_u8 ampdu_sta[MAX_NUM_TID];
    /** last rx_seq */
    t_u16 rx_seq[MAX_NUM_TID];
    t_u8 wapi_key_on;
};
#endif /* UAP_SUPPORT */

/** 802.11h State information kept in the 'mlan_adapter' of the driver */
typedef struct
{
    t_u32 is_11h_enabled;   /**< Enables/disables 11h in the driver (adhoc start) */
    t_u32 is_11h_active;    /**< Indicates whether 11h is active in the firmware */
    t_u32 tx_disabled;      /**< Set when driver receives a STOP TX event from fw */

    /** Minimum TX Power capability sent to FW for 11h use and fw power control */
    t_s8 min_tx_power_capability;

    /** Maximum TX Power capability sent to FW for 11h use and fw power control */
    t_s8 max_tx_power_capability;

    /** User provisioned local power constraint sent in association requests */
    t_s8 usr_def_power_constraint;

    /** Quiet IE */
    IEEEtypes_Quiet_t quiet_ie;

} wlan_11h_state_t;

/**
 * @brief Driver measurement state held in 'mlan_adapter' structure
 *  
 *  Used to record a measurement request that the driver is pending on 
 *    the result (received measurement report).
 */
typedef struct
{
    /**
     * Dialog token of a pending measurement request/report.  Used to
     *   block execution while waiting for the specific dialog token
     */
    t_u8 meas_rpt_pend_on;

    /**
     * Measurement report received from the firmware that we were pending on
     */
    HostCmd_DS_MEASUREMENT_REPORT meas_rpt_returned;

} wlan_meas_state_t;

#ifdef SDIO_MULTI_PORT_TX_AGGR
/** data structure for SDIO MPA TX */
typedef struct _sdio_mpa_tx
{
        /** allocated buf for tx aggreation */
    t_u8 *head_ptr;
        /** multiport tx aggregation buffer pointer */
    t_u8 *buf;
        /** multiport tx aggregation buffer length */
    t_u32 buf_len;
        /** multiport tx aggregation packet count */
    t_u32 pkt_cnt;
        /** multiport tx aggregation ports */
    t_u16 ports;
        /** multiport tx aggregation starting port */
    t_u16 start_port;
        /** multiport tx aggregation enable/disable flag */
    t_u8 enabled;
        /** multiport tx aggregation buffer size */
    t_u32 buf_size;
        /** multiport tx aggregation pkt aggr limit */
    t_u32 pkt_aggr_limit;
} sdio_mpa_tx;
#endif

#ifdef SDIO_MULTI_PORT_RX_AGGR
/** data structure for SDIO MPA RX */
typedef struct _sdio_mpa_rx
{
        /** allocated buf for rx aggreation */
    t_u8 *head_ptr;
        /** multiport rx aggregation buffer pointer */
    t_u8 *buf;
        /** multiport rx aggregation buffer length */
    t_u32 buf_len;
        /** multiport rx aggregation packet count */
    t_u32 pkt_cnt;
        /** multiport rx aggregation ports */
    t_u16 ports;
        /** multiport rx aggregation starting port */
    t_u16 start_port;

        /** multiport rx aggregation mbuf array */
    pmlan_buffer mbuf_arr[SDIO_MP_AGGR_DEF_PKT_LIMIT];
        /** multiport rx aggregation pkt len array */
    t_u32 len_arr[SDIO_MP_AGGR_DEF_PKT_LIMIT];

        /** multiport rx aggregation enable/disable flag */
    t_u8 enabled;
        /** multiport rx aggregation buffer size */
    t_u32 buf_size;
        /** multiport rx aggregation pkt aggr limit */
    t_u32 pkt_aggr_limit;
} sdio_mpa_rx;
#endif /* SDIO_MULTI_PORT_RX_AGGR */

/** mlan_init_para structure */
typedef struct _mlan_init_para
{
#ifdef MFG_CMD_SUPPORT
    /** MFG mode */
    t_u32 mfg_mode;
#endif
    /** SDIO interrupt mode (0: INT_MODE_SDIO, 1: INT_MODE_GPIO) */
    t_u32 int_mode;
    /** GPIO interrupt pin number */
    t_u32 gpio_pin;
#ifdef SDIO_MULTI_PORT_TX_AGGR
    /** SDIO MPA Tx */
    t_u32 mpa_tx_cfg;
#endif
#ifdef SDIO_MULTI_PORT_RX_AGGR
    /** SDIO MPA Rx */
    t_u32 mpa_rx_cfg;
#endif
    /** Auto deep sleep */
    t_u32 auto_ds;
    /** IEEE PS mode */
    t_u32 ps_mode;
    /** Max Tx buffer size */
    t_u32 max_tx_buf;
#if defined(STA_SUPPORT)
    /** 802.11d configuration */
    t_u32 cfg_11d;
#endif
} mlan_init_para, *pmlan_init_para;

/** Adapter data structure for MLAN */
typedef struct _mlan_adapter
{
    /** MOAL handle structure */
    t_void *pmoal_handle;
    /** Private pointer */
    pmlan_private priv[MLAN_MAX_BSS_NUM];
    /** Priority table for bss */
    mlan_bssprio_tbl bssprio_tbl[MLAN_MAX_BSS_NUM];
    /** Callback table */
    mlan_callbacks callbacks;
    /** Init parameters */
    mlan_init_para init_para;

    /** mlan_lock for init/shutdown */
    t_void *pmlan_lock;
    /** main_proc_lock for main_process */
    t_void *pmain_proc_lock;
    /** mlan_processing */
    t_u32 mlan_processing;
    /** Max tx buf size */
    t_u16 max_tx_buf_size;
    /** Tx buf size */
    t_u16 tx_buf_size;
    /** current tx buf size in fw */
    t_u16 curr_tx_buf_size;
    /** IO port */
    t_u32 ioport;

    /** STATUS variables */
    WLAN_HARDWARE_STATUS hw_status;
    /** PnP SUPPORT */
    t_u8 surprise_removed;

    /** Radio on flag */
    t_u16 radio_on;

    /** Firmware release number */
    t_u32 fw_release_number;

    /** Number of antenna used */
    t_u16 number_of_antenna;

    /** Firmware capability information */
    t_u32 fw_cap_info;
    /** pint_lock for interrupt handling */
    t_void *pint_lock;
    /** Interrupt status */
    t_u8 sdio_ireg;
    /** SDIO multiple port read bitmap */
    t_u16 mp_rd_bitmap;
    /** SDIO multiple port write bitmap */
    t_u16 mp_wr_bitmap;
    /** SDIO end port from txbufcfg */
    t_u16 mp_end_port;
    /** SDIO port mask calculated based on txbufcfg end port */
    t_u16 mp_data_port_mask;
    /** Current available port for read */
    t_u8 curr_rd_port;
    /** Current available port for write */
    t_u8 curr_wr_port;
    /** Array to store values of SDIO multiple port group registers */
    t_u8 *mp_regs;
    /** allocated buf to read SDIO multiple port group registers */
    t_u8 *mp_regs_buf;
    /** Array to store data transfer eligibility based on tid (QoS-over-SDIO) */
    t_u8 tx_eligibility[MAX_NUM_TID];

#ifdef SDIO_MULTI_PORT_TX_AGGR
        /** data structure for SDIO MPA TX */
    sdio_mpa_tx mpa_tx;
#endif                          /* SDIO_MULTI_PORT_TX_AGGR */

#ifdef SDIO_MULTI_PORT_RX_AGGR
        /** data structure for SDIO MPA RX */
    sdio_mpa_rx mpa_rx;
#endif                          /* SDIO_MULTI_PORT_RX_AGGR */

    /** SDIO interrupt mode (0: INT_MODE_SDIO, 1: INT_MODE_GPIO) */
    t_u32 int_mode;
    /** GPIO interrupt pin number */
    t_u32 gpio_pin;

    /** Event cause */
    t_u32 event_cause;
    /** Event buffer */
    pmlan_buffer pmlan_buffer_event;
    /** Upload length */
    t_u32 upld_len;
    /** Upload buffer*/
    t_u8 upld_buf[WLAN_UPLD_SIZE];
    /** Data sent: 
     *       TRUE - Data is sent to fw, no Tx Done received
     *       FALSE - Tx done received for previous Tx
     */
    t_u8 data_sent;
    /** CMD sent: 
     *       TRUE - CMD is sent to fw, no CMD Done received
     *       FALSE - CMD done received for previous CMD
     */
    t_u8 cmd_sent;
    /** CMD Response received: 
     *       TRUE - CMD is response is received from fw, and yet to process
     *       FALSE - No cmd response to process
     */
    t_u8 cmd_resp_received;
    /** Event received: 
     *       TRUE - Event received from fw, and yet to process
     *       FALSE - No events to process
     */
    t_u8 event_received;

    /** Data received:
     *       TRUE - Event received from fw, and yet to process
     *       FALSE - No events to process
     */
    t_u8 data_received;

    /** Command-related variables */
    /** Command sequence number */
    t_u16 seq_num;
    /** Command controller nodes */
    cmd_ctrl_node *cmd_pool;
    /** Current Command */
    cmd_ctrl_node *curr_cmd;
    /** mlan_lock for command */
    t_void *pmlan_cmd_lock;
    /** Number of command timeouts */
    t_u32 num_cmd_timeout;
    /** Last init fw command id */
    t_u16 last_init_cmd;
    /** Command timer */
    t_void *pmlan_cmd_timer;
    /** Command timer set flag */
    t_u8 cmd_timer_is_set;

    /** Command Queues */
    /** Free command buffers */
    mlan_list_head cmd_free_q;
    /** Pending command buffers */
    mlan_list_head cmd_pending_q;
    /** Command queue for scanning */
    mlan_list_head scan_pending_q;
    /** mlan_processing */
    t_u32 scan_processing;

    /** Region code */
    t_u16 region_code;
    /** Region Channel data */
    region_chan_t region_channel[MAX_REGION_CHANNEL_NUM];
#ifdef STA_SUPPORT
    /** FSM variable for 11d support */
    wlan_802_11d_state_t state_11d;
    /** Universal Channel data */
    region_chan_t universal_channel[MAX_REGION_CHANNEL_NUM];
    /** Parsed region channel */
    parsed_region_chan_11d_t parsed_region_chan;
    /** 11D and Domain Regulatory Data */
    wlan_802_11d_domain_reg_t domain_reg;
#endif                          /* STA_SUPPORT */
    /** FSM variable for 11h support */
    wlan_11h_state_t state_11h;
    /** FSM variable for MEAS support */
    wlan_meas_state_t state_meas;
    /** Scan table */
    BSSDescriptor_t *pscan_table;

    /** Number of records in the scan table */
    t_u32 num_in_scan_table;
    /** Scan probes */
    t_u16 scan_probes;

    /** Scan type */
    t_u8 scan_type;
    /** Scan mode */
    t_u32 scan_mode;
    /** Specific scan time */
    t_u16 specific_scan_time;
    /** Active scan time */
    t_u16 active_scan_time;
    /** Passive scan time */
    t_u16 passive_scan_time;

    /** Beacon buffer */
    t_u8 bcn_buf[MAX_SCAN_BEACON_BUFFER];
    /** Pointer to valid beacon buffer end */
    t_u8 *pbcn_buf_end;

    /** F/W supported bands */
    t_u8 fw_bands;
    /** User selected band to start adhoc network */
    t_u8 adhoc_start_band;
    /** User selected bands */
    t_u8 config_bands;
    /** Pointer to channel list last sent to the firmware for scanning */
    ChanScanParamSet_t *pscan_channels;

    /** Tx lock flag */
    t_u8 tx_lock_flag;

    /** sleep_params_t */
    sleep_params_t sleep_params;
    /** sleep_period_t (Enhanced Power Save) */
    sleep_period_t sleep_period;

    /** Power Save mode */
    /**
      * Wlan802_11PowerModeCAM = disable
      * Wlan802_11PowerModePSP = enable
      */
    t_u16 ps_mode;
    /** Power Save state */
    t_u32 ps_state;
    /** Need to wakeup flag */
    t_u8 need_to_wakeup;

    /** Multiple DTIM */
    t_u16 multiple_dtim;
    /** Local listen interval */
    t_u16 local_listen_interval;
    /** Null packet interval */
    t_u16 null_pkt_interval;

    /** Power save confirm sleep command buffer */
    pmlan_buffer psleep_cfm;
    /** Beacon miss timeout */
    t_u16 bcn_miss_time_out;

    /** AdHoc awake period */
    t_u16 adhoc_awake_period;

    /** Deep Sleep flag */
    t_u8 is_deep_sleep;
    /** Auto Deep Sleep enabled at init time */
    t_u8 init_auto_ds;

        /** delay null pkt flag */
    t_u8 delay_null_pkt;
    /** Delay to PS in milliseconds */
    t_u16 delay_to_ps;
    /** Enhanced PS mode */
    t_u16 enhanced_ps_mode;
    /** Device wakeup required flag */
    t_u8 pm_wakeup_card_req;

    /** Gen NULL pkg */
    t_u16 gen_null_pkt;

    /** PPS/UAPSD mode flag */
    t_u16 pps_uapsd_mode;
    /** Number of wakeup tries */
    t_u32 pm_wakeup_fw_try;

    /** Host Sleep configured flag */
    t_u8 is_hs_configured;
    /** Host Sleep configuration */
    hs_config_param hs_cfg;
    /** Host Sleep activated flag */
    t_u8 hs_activated;
    /** Event body */
    t_u8 event_body[MAX_EVENT_SIZE];
    /** 802.11n device capabilities */
    t_u32 hw_dot_11n_dev_cap;
    /** Device support for MIMO abstraction of MCSs */
    t_u8 hw_dev_mcs_support;
    /** 802.11n Device Capabilities */
    t_u32 usr_dot_11n_dev_cap;
    /** MIMO abstraction of MCSs supported by device */
    t_u8 usr_dev_mcs_support;
#ifdef STA_SUPPORT
    /** Enable 11n support for adhoc start */
    t_u8 adhoc_11n_enabled;
    /** Adhoc Secondary Channel Offset */
    t_u8 chan_offset;
#endif                          /* STA_SUPPORT */

#ifdef MFG_CMD_SUPPORT
    t_u32 mfg_mode;
#endif
    /** Debug */
    wlan_dbg dbg;

    /** RX pending for forwarding packets */
    t_u16 pending_bridge_pkts;
    /** RX data pause sent or not */
    t_u16 rx_pause_flag;

#ifdef STA_SUPPORT
    /** ARP filter buffer */
    t_u8 arp_filter[ARP_FILTER_MAX_BUF_SIZE];
    /** ARP filter buffer size */
    t_u32 arp_filter_size;
#endif                          /* STA_SUPPORT */

    /** Bypass TX queue */
    mlan_list_head bypass_txq;

} mlan_adapter, *pmlan_adapter;

/** Ethernet packet type for EAPOL */
#define MLAN_ETHER_PKT_TYPE_EAPOL	(0x888E)
/** Ethernet packet type offset */
#define MLAN_ETHER_PKT_TYPE_OFFSET  (12)

mlan_status wlan_init_lock_list(IN pmlan_adapter pmadapter);
t_void wlan_free_lock_list(IN pmlan_adapter pmadapter);
mlan_status wlan_init_timer(IN pmlan_adapter pmadapter);
t_void wlan_free_timer(IN pmlan_adapter pmadapter);

/* Function prototype */
/** Download firmware */
mlan_status wlan_dnld_fw(IN pmlan_adapter pmadapter, IN pmlan_fw_image pmfw);

/** Initialize firmware */
mlan_status wlan_init_fw(IN pmlan_adapter pmadapter);

/** Initialize firmware complete */
mlan_status wlan_init_fw_complete(IN pmlan_adapter pmadapter);

/** Shutdown firmware */
mlan_status wlan_shutdown_fw(IN pmlan_adapter pmadapter);

/** Shutdown firmware complete */
mlan_status wlan_shutdown_fw_complete(IN pmlan_adapter pmadapter);

/** Receive event */
mlan_status wlan_recv_event(pmlan_private priv,
                            mlan_event_id event_id, t_void * pmevent);

/** Initialize mlan_adapter structure */
t_void wlan_init_adapter(IN pmlan_adapter pmadapter);

/** Deletes the BSS priority table */
t_void wlan_delete_bsspriotbl(pmlan_private priv);

/** Initialize mlan_private structure */
t_void wlan_init_priv(IN pmlan_private priv);

/** Process event */
mlan_status wlan_process_event(pmlan_adapter pmadapter);

/** Prepare command */
mlan_status wlan_prepare_cmd(IN pmlan_private priv,
                             IN t_u16 cmd_no,
                             IN t_u16 cmd_action,
                             IN t_u32 cmd_oid,
                             IN t_void * pioctl_buf, IN t_void * pdata_buf);

/** cmd timeout handler */
t_void wlan_cmd_timeout_func(t_void * FunctionContext);
/** process host cmd */
mlan_status wlan_misc_ioctl_host_cmd(IN pmlan_adapter pmadapter,
                                     IN pmlan_ioctl_req pioctl_req);
/** process init/shutdown cmd*/
mlan_status wlan_misc_ioctl_init_shutdown(IN pmlan_adapter pmadapter,
                                          IN pmlan_ioctl_req pioctl_req);
/** process debug info */
mlan_status wlan_get_info_debug_info(IN pmlan_adapter pmadapter,
                                     IN pmlan_ioctl_req pioctl_req);

mlan_status wlan_set_ewpa_mode(mlan_private * priv,
                               mlan_ds_passphrase * psec_pp);
mlan_status wlan_find_bss(mlan_private * pmpriv, pmlan_ioctl_req pioctl_req);

/** Free adapter */
t_void wlan_free_adapter(pmlan_adapter pmadapter);
/** Allocate command buffer */
mlan_status wlan_alloc_cmd_buffer(IN mlan_adapter * pmadapter);
/** Free command buffer */
mlan_status wlan_free_cmd_buffer(IN mlan_adapter * pmadapter);
/** Request command lock */
t_void wlan_request_cmd_lock(mlan_adapter * pmadapter);
/** Release command lock */
t_void wlan_release_cmd_lock(mlan_adapter * pmadapter);
/**Cancel pending command */
void wlan_cancel_all_pending_cmd(pmlan_adapter pmadapter);
/**Cancel pending ioctl */
void wlan_cancel_pending_ioctl(pmlan_adapter pmadapter,
                               pmlan_ioctl_req pioctl_req);

/** Insert command to free queue */
t_void wlan_insert_cmd_to_free_q(IN mlan_adapter * pmadapter,
                                 IN cmd_ctrl_node * pcmd_node);

/** Insert command to pending queue */
t_void wlan_insert_cmd_to_pending_q(IN mlan_adapter * pmadapter,
                                    IN cmd_ctrl_node * pcmd_node,
                                    IN t_u32 addtail);

/** Execute next command */
mlan_status wlan_exec_next_cmd(mlan_adapter * pmadapter);
/** Proecess command response */
mlan_status wlan_process_cmdresp(mlan_adapter * pmadapter);
/** Handle received packet, has extra handling for aggregate packets */
mlan_status wlan_handle_rx_packet(pmlan_adapter pmadapter, pmlan_buffer pmbuf);
/** Process transmission */
mlan_status wlan_process_tx(pmlan_private priv, pmlan_buffer pmbuf,
                            mlan_tx_param * tx_param);
/** Transmit a null data packet */
mlan_status wlan_send_null_packet(pmlan_private priv, t_u8 flags);

#if defined(SDIO_MULTI_PORT_TX_AGGR) || defined(SDIO_MULTI_PORT_RX_AGGR)
mlan_status wlan_alloc_sdio_mpa_buffers(IN mlan_adapter * pmadapter,
                                        t_u32 mpa_tx_buf_size,
                                        t_u32 mpa_rx_buf_size);

mlan_status wlan_free_sdio_mpa_buffers(IN mlan_adapter * pmadapter);
#endif

/** Process write data complete */
mlan_status wlan_write_data_complete(pmlan_adapter pmlan_adapter,
                                     pmlan_buffer pmbuf, mlan_status status);
/** Process receive packet complete */
mlan_status wlan_recv_packet_complete(t_void * pmlan_adapter,
                                      pmlan_buffer pmbuf, mlan_status status);
/** Clean Tx Rx queues */
t_void wlan_clean_txrx(pmlan_private priv);

t_void wlan_add_buf_bypass_txqueue(mlan_adapter * pmadapter,
                                   pmlan_buffer pmbuf);
t_void wlan_process_bypass_tx(mlan_adapter * pmadapter);
t_void wlan_cleanup_bypass_txq(mlan_adapter * pmadapter);
t_u8 wlan_bypass_tx_list_empty(mlan_adapter * pmadapter);

/** Check if this is the last packet */
t_u8 wlan_check_last_packet_indication(pmlan_private priv);

/** function to allocate a mlan_buffer */
pmlan_buffer wlan_alloc_mlan_buffer(mlan_adapter * pmadapter, t_u32 data_len,
                                    t_u32 head_room, t_u32 malloc_flag);
/** function to free a mlan_buffer */
t_void wlan_free_mlan_buffer(mlan_adapter * pmadapter, pmlan_buffer pmbuf);

/** Check Power Save condition */
t_void wlan_check_ps_cond(mlan_adapter * pmadapter);

/** handle command for enhanced power save mode */
mlan_status wlan_cmd_enh_power_mode(pmlan_private pmpriv,
                                    IN HostCmd_DS_COMMAND * cmd,
                                    IN t_u16 cmd_action,
                                    IN t_u16 ps_bitmap, IN t_void * pdata_buf);
/** handle command resp for enhanced power save mode */
mlan_status wlan_ret_enh_power_mode(IN pmlan_private pmpriv,
                                    IN HostCmd_DS_COMMAND * resp,
                                    IN mlan_ioctl_req * pioctl_buf);

/** Process sleep confirm command response */
void wlan_process_sleep_confirm_resp(pmlan_adapter pmadapter, t_u8 * pbuf,
                                     t_u32 len);

/** Perform hs related activities on receving the power up interrupt */
void wlan_process_hs_config(pmlan_adapter pmadapter);

mlan_status wlan_pm_reset_card(pmlan_adapter adapter);
mlan_status wlan_pm_wakeup_card(pmlan_adapter pmadapter);

mlan_status wlan_pm_ioctl_hscfg(pmlan_adapter pmadapter,
                                pmlan_ioctl_req pioctl_req);

t_void wlan_host_sleep_activated_event(pmlan_private priv, t_u8 activated);
/** Handles the command response of hs_cfg */
mlan_status wlan_ret_802_11_hs_cfg(IN pmlan_private pmpriv,
                                   IN HostCmd_DS_COMMAND * resp,
                                   IN mlan_ioctl_req * pioctl_buf);
/** Sends HS_WAKEUP event to applications */
t_void wlan_host_sleep_wakeup_event(pmlan_private priv);

#ifdef STA_SUPPORT
/** Process received packet */
mlan_status wlan_process_rx_packet(pmlan_adapter pmadapter, pmlan_buffer pmbuf);
/** ioctl handler for station mode */
mlan_status mlan_sta_ioctl(t_void * adapter, pmlan_ioctl_req pioctl_req);

/** cmd handler for station mode */
mlan_status mlan_sta_prepare_cmd(IN t_void * priv,
                                 IN t_u16 cmd_no,
                                 IN t_u16 cmd_action,
                                 IN t_u32 cmd_oid,
                                 IN t_void * pioctl_buf,
                                 IN t_void * pdata_buf, IN t_void * pcmd_buf);

/** cmdresp handler for station mode */
mlan_status mlan_process_sta_cmdresp(IN t_void * priv,
                                     IN t_u16 cmdresp_no,
                                     IN t_void * pcmd_buf, IN t_void * pioctl);

/** rx handler for station mode */
mlan_status mlan_process_sta_rx_packet(IN t_void * adapter,
                                       IN pmlan_buffer pmbuf);

/** event handler for station mode */
mlan_status mlan_process_sta_event(IN t_void * priv);

/** fill txpd for station mode */
t_void *mlan_process_sta_txpd(IN t_void * priv, IN pmlan_buffer pmbuf);

/** send init cmd to firmware for station mode */
mlan_status mlan_sta_init_cmd(IN t_void * priv, IN t_u8 first_sta);

/** Scan for networks */
mlan_status wlan_scan_networks(IN mlan_private * pmpriv,
                               IN t_void * pioctl_buf,
                               IN const wlan_user_scan_cfg * puser_scan_in);

/** Scan for specific SSID */
mlan_status wlan_scan_specific_ssid(IN mlan_private * pmpriv,
                                    IN t_void * pioctl_buf,
                                    IN mlan_802_11_ssid * preq_ssid);

/** Scan command handler */
mlan_status wlan_cmd_802_11_scan(IN pmlan_private pmpriv,
                                 IN HostCmd_DS_COMMAND * pcmd,
                                 IN t_void * pdata_buf);

/** Queue scan command handler */
t_void wlan_queue_scan_cmd(IN mlan_private * pmpriv,
                           IN cmd_ctrl_node * pcmd_node);

/** Handler for scan command response */
mlan_status wlan_ret_802_11_scan(IN pmlan_private pmpriv,
                                 IN HostCmd_DS_COMMAND * resp,
                                 IN t_void * pioctl_buf);

/** check network compatibility */
t_s32 wlan_is_network_compatible(IN mlan_private * pmpriv,
                                 IN t_u32 index, IN t_u32 mode);

/** Find an SSID in a list */
t_s32 wlan_find_ssid_in_list(IN pmlan_private pmpriv,
                             IN mlan_802_11_ssid * ssid,
                             IN t_u8 * bssid, IN t_u32 mode);

/** Find a BSSID in a list */
t_s32 wlan_find_bssid_in_list(IN mlan_private * pmpriv,
                              IN t_u8 * bssid, IN t_u32 mode);

/** Find best network */
mlan_status wlan_find_best_network(IN mlan_private * pmpriv,
                                   OUT mlan_ssid_bssid * preq_ssid_bssid);

/** Compare two SSIDs */
t_s32 wlan_ssid_cmp(IN pmlan_adapter pmadapter,
                    IN mlan_802_11_ssid * ssid1, IN mlan_802_11_ssid * ssid2);

/** Associate */
mlan_status wlan_associate(IN mlan_private * pmpriv,
                           IN t_void * pioctl_buf,
                           IN BSSDescriptor_t * pBSSDesc);

/** Associate command handler */
mlan_status wlan_cmd_802_11_associate(IN mlan_private * pmpriv,
                                      IN HostCmd_DS_COMMAND * cmd,
                                      IN t_void * pdata_buf);

/** Handler for association command response */
mlan_status wlan_ret_802_11_associate(IN mlan_private * pmpriv,
                                      IN HostCmd_DS_COMMAND * resp,
                                      IN t_void * pioctl_buf);

/** Reset connected state */
t_void wlan_reset_connect_state(pmlan_private priv);

t_void wlan_2040_coex_event(pmlan_private pmpriv);

/** convert band to radio type */
t_u8 wlan_band_to_radio_type(IN t_u8 band);

/** Disconnect */
mlan_status wlan_disconnect(IN mlan_private * pmpriv,
                            IN mlan_ioctl_req * pioctl_req,
                            IN mlan_802_11_mac_addr * mac);

/** Ad-Hoc start */
mlan_status wlan_adhoc_start(IN mlan_private * pmpriv,
                             IN t_void * pioctl_buf,
                             IN mlan_802_11_ssid * padhoc_ssid);

/** Ad-Hoc join */
mlan_status wlan_adhoc_join(IN mlan_private * pmpriv,
                            IN t_void * pioctl_buf,
                            IN BSSDescriptor_t * pBSSDesc);

/** Ad-Hoc start command handler */
mlan_status wlan_cmd_802_11_ad_hoc_start(IN mlan_private * pmpriv,
                                         IN HostCmd_DS_COMMAND * cmd,
                                         IN t_void * pdata_buf);

/** Ad-Hoc command handler */
mlan_status wlan_cmd_802_11_ad_hoc_join(IN mlan_private * pmpriv,
                                        IN HostCmd_DS_COMMAND * cmd,
                                        IN t_void * pdata_buf);

/** Handler for Ad-Hoc commands */
mlan_status wlan_ret_802_11_ad_hoc(IN mlan_private * pmpriv,
                                   IN HostCmd_DS_COMMAND * resp,
                                   IN t_void * pioctl_buf);

/** Handler for bgscan query commands */
mlan_status wlan_cmd_802_11_bg_scan_query(IN mlan_private * pmpriv,
                                          IN HostCmd_DS_COMMAND * pcmd,
                                          IN t_void * pdata_buf);

/* 802.11D related functions */
/** Initialize 11D */
t_void wlan_11d_init(mlan_adapter * pmadapter);
/** Enable 11D */
mlan_status wlan_11d_enable(mlan_private * pmpriv, t_void * pioctl_buf,
                            state_11d_t flag);
/** Set 11D universal table */
mlan_status wlan_11d_set_universaltable(mlan_private * pmpriv, t_u8 band);
/** Get 11D state */
state_11d_t wlan_11d_get_state(mlan_private * pmpriv);
/** Clear 11D region table */
mlan_status wlan_11d_clear_parsedtable(mlan_private * pmpriv);
/** Create 11D country information for downloading */
mlan_status wlan_11d_create_dnld_countryinfo(mlan_private * pmpriv, t_u8 band);
/** Get scan type from 11D info */
t_u8 wlan_11d_get_scan_type(pmlan_adapter pmadapter, t_u8 band, t_u8 chan,
                            parsed_region_chan_11d_t * parsed_region_chan);
/** Command handler for 11D country info */
mlan_status wlan_cmd_802_11d_domain_info(mlan_private * pmpriv,
                                         HostCmd_DS_COMMAND * pcmd,
                                         t_u16 cmd_action);
/** Handler for 11D country info command response */
mlan_status wlan_ret_802_11d_domain_info(mlan_private * pmpriv,
                                         HostCmd_DS_COMMAND * resp);
/** Parse 11D country info */
mlan_status wlan_11d_parse_dnld_countryinfo(mlan_private * pmpriv,
                                            BSSDescriptor_t * pBSSDesc);
/** Prepare 11D domain information for download */
mlan_status wlan_11d_prepare_dnld_domain_info_cmd(mlan_private * pmpriv);
/** Parse 11D country information  */
mlan_status wlan_11d_parse_domain_info(pmlan_adapter pmadapter,
                                       IEEEtypes_CountryInfoFullSet_t *
                                       country_info, t_u8 band,
                                       parsed_region_chan_11d_t *
                                       parsed_region_chan);

/** Convert channel to frequency */
t_u32 wlan_11d_chan_2_freq(pmlan_adapter pmadapter, t_u8 chan, t_u8 band);
/** Get Channel-Frequency-Power by band and channel */
chan_freq_power_t *wlan_get_cfp_by_band_and_channel(pmlan_adapter pmadapter,
                                                    t_u8 band, t_u16 channel,
                                                    region_chan_t *
                                                    region_channel);
/** Find Channel-Frequency-Power by band and channel */
chan_freq_power_t *wlan_find_cfp_by_band_and_channel(mlan_adapter * pmadapter,
                                                     t_u8 band, t_u16 channel);
/** Find Channel-Frequency-Power by band and frequency */
chan_freq_power_t *wlan_find_cfp_by_band_and_freq(mlan_adapter * pmadapter,
                                                  t_u8 band, t_u32 freq);
/** Get region Channel-Frequency-Power table */
chan_freq_power_t *wlan_get_region_cfp_table(pmlan_adapter pmadapter,
                                             t_u8 region, t_u8 band,
                                             int *cfp_no);
/** Get Tx power of channel from Channel-Frequency-Power */
t_u8 wlan_get_txpwr_of_chan_from_cfp(mlan_private * pmpriv, t_u8 channel);
/** Convert index into data rate */
t_u32 wlan_index_to_data_rate(pmlan_adapter pmadapter, t_u8 index,
                              t_u8 ht_info);
t_u32 wlan_find_freq_from_band_chan(t_u8, t_u8);
/** Set region table */
mlan_status wlan_set_regiontable(mlan_private * pmpriv, t_u8 region, t_u8 band);
/** Append the vendor specific TLV */
int wlan_cmd_append_vsie_tlv(mlan_private * pmpriv, t_u16 vsie_mask,
                             t_u8 ** ppbuffer);

/* Rate related functions */
/** Convert index into data rate */
t_u32 wlan_index_to_data_rate(pmlan_adapter pmadapter, t_u8 index,
                              t_u8 ht_info);
/** Get active data rates */
t_u32 wlan_get_active_data_rates(mlan_private * pmpriv,
                                 WLAN_802_11_RATES rates);
/** Get supported data rates */
t_u32 wlan_get_supported_rates(mlan_private * pmpriv, WLAN_802_11_RATES rates);
/** Convert data rate to index */
t_u8 wlan_data_rate_to_index(pmlan_adapter pmadapter, t_u32 rate);
/** Check if rate is auto */
t_u8 wlan_is_rate_auto(mlan_private * pmpriv);
/** Get rate index */
int wlan_get_rate_index(pmlan_adapter pmadapter, t_u16 * rateBitmap, int size);
/** Region code index table */
extern t_u16 region_code_index[MRVDRV_MAX_REGION_CODE];

/*  Save a beacon buffer of the current bss descriptor */
t_void wlan_save_curr_bcn(IN mlan_private * pmpriv);
/*  Restore a beacon buffer of the current bss descriptor */
t_void wlan_restore_curr_bcn(IN mlan_private * pmpriv);
/*  Free a beacon buffer of the current bss descriptor */
t_void wlan_free_curr_bcn(IN mlan_private * pmpriv);

#endif /* STA_SUPPORT */

#ifdef UAP_SUPPORT
/** get station node */
sta_node *wlan_get_station_entry(mlan_private * priv, t_u8 * mac);
/** delete station list */
t_void wlan_delete_station_list(pmlan_private priv);
/** delete station entry */
t_void wlan_delete_station_entry(mlan_private * priv, t_u8 * mac);
/** add station entry */
sta_node *wlan_add_station_entry(mlan_private * priv, t_u8 * mac);
/** process uap rx packet */
mlan_status wlan_process_uap_rx_packet(IN mlan_private * priv,
                                       IN pmlan_buffer pmbuf);
#endif /* UAP_SUPPORT */

#ifdef UAP_SUPPORT
/* process the recevied packet and bridge the packet */
mlan_status wlan_uap_recv_packet(IN mlan_private * priv, IN pmlan_buffer pmbuf);
#endif /* UAP_SUPPORT */

mlan_status wlan_misc_ioctl_custom_ie_list(IN pmlan_adapter pmadapter,
                                           IN pmlan_ioctl_req pioctl_req);

/** 
 *  @brief RA based queueing
 *   
 *  @param priv                 A pointer to mlan_private structure
 *
 *  @return 	   	        MTRUE or MFALSE
 */
static INLINE t_u8
queuing_ra_based(pmlan_private priv)
{
    /* 
     * Currently we assume if we are in Infra, then DA=RA. This might not be
     * true in the future
     */
    if ((priv->bss_mode == MLAN_BSS_MODE_INFRA) &&
        (GET_BSS_ROLE(priv) == MLAN_BSS_ROLE_STA))
        return MFALSE;

    return MTRUE;
}

/** 
 *  @brief Copy Rates
 *   
 *  @param dest                 A pointer to Dest Buf
 *  @param pos		        The position for copy
 *  @param src		        A pointer to Src Buf
 *  @param len                  The len of Src Buf
 *
 *  @return 	   	        Number of Rates copied
 */
static INLINE t_u32
wlan_copy_rates(t_u8 * dest, t_u32 pos, t_u8 * src, int len)
{
    int i;

    for (i = 0; i < len && src[i]; i++, pos++) {
        if (pos >= sizeof(WLAN_802_11_RATES))
            break;
        dest[pos] = src[i];
    }

    return pos;
}

/** 
 *  @brief strlen
 *   
 *  @param str		        A pointer to string
 *
 *  @return 	   	        Length of string
 */
static INLINE t_u32
wlan_strlen(const t_s8 * str)
{
    t_u32 i;

    for (i = 0; str[i] != 0; i++) {
    }
    return i;
}

/** delay unit */
typedef enum _delay_unit
{
    USEC,
    MSEC,
    SEC,
} t_delay_unit;

/** delay function */
t_void wlan_delay_func(mlan_adapter * pmadapter, t_u32 delay, t_delay_unit u);

/** delay function wrapper */
#define wlan_delay(p, n)   wlan_delay_func(p, n, SEC)
/** delay function wrapper */
#define wlan_mdelay(p, n)  wlan_delay_func(p, n, MSEC)
/** delay function wrapper */
#define wlan_udelay(p, n)  wlan_delay_func(p, n, USEC)

/** Function to check if any command is pending in the queue */
#define IS_COMMAND_PENDING(pmadapter) ((cmd_ctrl_node *)util_peek_list(pmadapter->pmoal_handle, \
                                       &pmadapter->cmd_pending_q,\
                                       pmadapter->callbacks.moal_spin_lock,\
                                       pmadapter->callbacks.moal_spin_unlock))

/** Get BSS number from priv */
#define GET_BSS_NUM(priv)   (priv)->bss_num
/** 
 *  @brief This function returns priv based on the BSS num and BSS type
 *  
 *  @param handle    A pointer to mlan_adapter
 *  @param bss_num   BSS number
 *  @param bss_type  BSS type
 *
 *  @return          Pointer to mlan_private
 */
static INLINE mlan_private *
wlan_get_priv_by_id(mlan_adapter * pmadapter, t_u32 bss_num, t_u32 bss_type)
{
    int i;

    for (i = 0; i < MLAN_MAX_BSS_NUM; i++) {
        if (pmadapter->priv[i]) {
            if ((pmadapter->priv[i]->bss_num == bss_num) &&
                (pmadapter->priv[i]->bss_type == bss_type))
                break;
        }
    }
    return ((i < MLAN_MAX_BSS_NUM) ? pmadapter->priv[i] : MNULL);
}

/** 
 *  @brief This function returns first available priv
 *  based on the BSS role
 *  
 *  @param handle    A pointer to mlan_adapter
 *  @param bss_role  BSS role or MLAN_BSS_ROLE_ANY
 *
 *  @return          Pointer to mlan_private
 */
static INLINE mlan_private *
wlan_get_priv(mlan_adapter * pmadapter, mlan_bss_role bss_role)
{
    int i;

    for (i = 0; i < MLAN_MAX_BSS_NUM; i++) {
        if (pmadapter->priv[i]) {
            if (bss_role == MLAN_BSS_ROLE_ANY ||
                GET_BSS_ROLE(pmadapter->priv[i]) == bss_role)
                break;
        }
    }
    return ((i < MLAN_MAX_BSS_NUM) ? pmadapter->priv[i] : MNULL);
}

#endif /* !_MLAN_MAIN_H_ */
