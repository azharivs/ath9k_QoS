// ************ In the name of Allah ************
/*
 * Author: Mohammad H. Daei
 *         MHDaei@Gmail.com
 *
 * Note: MTS stands for MAC802.11 TX Scheduler
 */

#ifndef _TX_SCHED_H
#define _TX_SCHED_H

/**
 * DOC: New Service Policy
 *
 * For defining a new service policy, you should design and implement
 * available functions in the tx_sched_ops  according to your demands
 * and then provide the created ops for TX Scheduler.
 *
 * If you need to define  a new function, you'd better define it as a
 * static function.
 */

/* Forward declaration of structs defined in ieee80211_i.h */
struct txq_info;

/* Maximum Hardware Queue Depth */
#define MTS_MAX_HW_QDEPTH 				2
#define MTS_TSID_SUPPORT				false

#define IEEE80211_SEQ_MAX				4096

#define MTS_MAX_AMPDU_LIMIT				65532
#define MTS_MAX_BA						64
#define MTS_MAX_RATES					4
#define MTS_DEFAULT_BEACON_INT			100

#define MTS_FACTOR						17							/* 2^17 = 131072 */
#define MTS_AVG_RATE_FACTOR				10							/* 2^10 = 1024 */
#define MTS_MAX_LOG_FACTOR				MTS_FACTOR
#define MTS_MIN_DIV_FACTOR				10							/* 2^10 = 1024 */

#define MTS_LN_2						22713						/* (ln 2) << MTS_LN_2_FACTOR */
#define MTS_LN_2_FACTOR					15

#define MTS_DEFAULT_TXQ_DELAY_BOUND		50000000					/* in Nanoseconds */
#define MTS_DEFAULT_TXQ_DELAY_VIOLATE	((1 << MTS_FACTOR) / 100)	/* 0.01 */
#define MTS_INIT_TIME_ALLOWANCE			0							/* in Nanoseconds */
#define MTS_DEFAULT_ARRIVAL_MAX_DUR		1000000000ULL				/* in Nanoseconds */

/* Statistics */
#define MTS_INIT_QEMPTY_PROB			(1 << (MTS_FACTOR - 7))		/* 0.0078 */
#define MTS_DEF_QEMPTY_WEIGHT			7							/* 2^7 = 128 */

#define MTS_INIT_AVG_PKT_QLEN			0							/* in Packets */
#define MTS_DEF_AVG_PKT_WEIGHT			7							/* 2^7 = 128 */

#define MTS_INIT_AVG_BYTE_QLEN			0							/* in Bytes */
#define MTS_DEF_AVG_BYTE_WEIGHT			7							/* 2^7 = 128 */

#define MTS_INIT_AVG_SRV_TIME			(3125 << (MTS_FACTOR + 6))	/* in Nanoseconds */
#define MTS_DEF_AVG_SRV_WEIGHT			7							/* 2^7 = 128 */

#define MTS_INIT_AVG_PKT_RATE			0							/* in Packets per Second */
#define MTS_DEF_AVG_PKT_RATE_WEIGHT		9							/* 2^7 = 128 */

#define MTS_INIT_AVG_BYTE_RATE			0							/* in Bytes per Second */
#define MTS_DEF_AVG_BYTE_RATE_WEIGHT	9							/* 2^7 = 128 */

/* Just for testing: */
#define MTS_SP_MAX_AMPDU_LIMIT 			60000

#define MTS_OFDM_PLCP_BITS				22

/* TX time calculation */
#define MTS_L_STF						8
#define MTS_L_LTF						8
#define MTS_L_SIG						4
#define MTS_HT_SIG						8
#define MTS_HT_STF						4
#define MTS_HT_LTF(_strm)				((_strm) << 2)
#define MTS_HT_RC_2_STREAMS(_rc)		((((_rc) & 0x78) >> 3) + 1)

#define MTS_TIME_SYMBOLS(_time)			((_time) >> 2)
#define MTS_TIME_SYMBOLS_SGI(_time)		(((_time) * 5 - 4) / 18)
#define MTS_SYMBOL_TIME(_nsym)			((_nsym) << 2)
#define MTS_SYMBOL_TIME_SGI(_nsym)		((((_nsym) * 18) + 4) / 5)

#define MTS_AGGR_DELIM_SZ          		4
#define MTS_AGGR_MINPLEN           		256 		/* in bytes, minimum packet length */
#define MTS_TXKEYIX_INVALID				((u8)-1)

/* Helper macro for safe invoking */
#define SAFE_INVOKE(_expect, _func, args...)				\
do {														\
	if (_expect(_func))										\
		_func(args);										\
} while (0)

/* Increment with wrap-around */
#define INCRMNT(_var, _max_size)							\
do {														\
	(_var)++;												\
	(_var) &= ((_max_size) - 1);							\
} while (0)

/* Addition with wrap-around */
#define ADDITION(_var1, _var2, _max_size)					\
(((_var1) + (_var2)) & ((_max_size) - 1))

/* Subtraction */
#define SUBTRCT(_var1, _var2, _max_size)					\
(((_var1) - (_var2)) & ((_max_size) - 1))

/* Standard number of delimiters based on frame length alone */
#define MTS_AGGR_GET_NDELIM(_len)							\
(((_len) >= MTS_AGGR_MINPLEN) ? 0 :							\
    DIV_ROUND_UP(MTS_AGGR_MINPLEN -							\
    (_len), MTS_AGGR_DELIM_SZ))

#define MTS_PADBYTES(_len) ((4 - ((_len) % 4)) % 4)

/* enum tx_scheduler_flags - TX Scheduler flags
 *
 * These flags are used with the @flags member of struct
 * mac80211_tx_scheduler
 *
 * @SCHEDULED_TASKLET: The Tasklet is already scheduled for execution.
 */
enum tx_scheduler_flags {
	SCHEDULED_TASKLET,

	/* The other bits are optional */
};

/* enum tx_scheduler_per_ac_flags - TX Scheduler per AC flags
 *
 * These flags are used with the @flags member of struct
 * mac80211_tx_scheduler.ac
 *
 * @MTS_SENDING: Driver software queue is not empty.
 * @DRIVER_READY: Driver is ready to receive new packets.
 * @DEIVER_HOLD: Driver has been held for sending burst packets.
 */
enum tx_scheduler_per_ac_flags {
	SCHEDULE,
	CHANGE_STATE,

	MTS_SENDING,
	UNKNOWN_RETX,
	DRIVER_READY,

	DRIVER_HOLD,
	RETX_STATUS,
	HAS_STATUS,
	RET_NOT_HOLD,
	COLLISION,

	MTS_BEACON_WAKE,

	/* The other bits are optional */
};

enum tx_info_flags {
	RETRANSMIT,
	SET_RATES,
	LEGACY_RATE,

	/* The other bits are optional */
};

enum sched_txqi_flags {
	SCHEDULER_TXQ_STOP,
	SCHEDULER_TXQ_AMPDU,
};

/* Types of wakes
 */
enum wake_type {
	NEW_PKT_WAKE,
	START_TXQ_WAKE,
	STOP_TXQ_WAKE,
	PS_WAKEUP_WAKE,
	UNKNOWN_WAKE,
};

enum mts_reports {
	MTS_OK,
	MTS_ERROR,
};

struct mts_decimal_num
{
	s64 internal;
	u16 factor;
};

/* Exponentially weighted moving average (EWMA) */
struct mts_ewma {

	unsigned long long internal;
	u8 factor;
	u8 weight;

};

struct mts_skb_arrival_info {

	ktime_t arrival_time;
	u16	length;

	struct list_head list;
};

struct mts_buff {
	struct list_head list;

	u32 frame_len;

	struct sk_buff *skb;
};

/*
 * struct sched_tx_info - TX Scheduler transmit information
 *
 * @nframes: The number of frames that are sent burstly.
 * @list: List entry in a TX info list.
 */
struct sched_tx_info {

	struct sched_txq_info *stxqi;
	struct ieee80211_tx_rate rates[MTS_MAX_RATES];

	unsigned long flags;

	struct list_head skbs;
	u16 nframes;
	u32 len;

	ktime_t service_time;

	/* Optional members */
	void *sp_data;

	/* Keep last */
	struct list_head list;
};

EFINE_EVENT(local_only_evt, drv_cancel_remain_on_channel,
	TP_PROTO(struct ieee80211_local *local),
	TP_ARGS(local)
);

TRACE_EVENT(drv_set_ringparam,
	TP_PROTO(struct ieee80211_local *local, u32 tx, u32 rx),

	TP_ARGS(local, tx, rx),

	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, tx)
		__field(u32, rx)
	),

	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->tx = tx;
		__entry->rx = rx;
	),

	TP_printk(
		LOCAL_PR_FMT " tx:%d rx %d",
		LOCAL_PR_ARG, __entry->tx, __entry->rx
	)
);

TRACE_EVENT(drv_get_ringparam,
	TP_PROTO(struct ieee80211_local *local, u32 *tx, u32 *tx_max,
		 u32 *rx, u32 *rx_max),

	TP_ARGS(local, tx, tx_max, rx, rx_max),

	TP_STRUCT__entry(
		LOCAL_ENTRY
		__field(u32, tx)
		__field(u32, tx_max)
		__field(u32, rx)
		__field(u32, rx_max)
	),

	TP_fast_assign(
		LOCAL_ASSIGN;
		__entry->tx = *tx;
		__entry->tx_max = *tx_max;
		__entry->rx = *rx;
		__entry->rx_max = *rx_max;
	),

	TP_printk(
		LOCAL_PR_FMT " tx:%d tx_max %d rx %d rx_max %d",
		LOCAL_PR_ARG,
		__entry->tx, __entry->tx_max, __entry->rx, __entry->rx_max
	)
);

DEFINE_EVENT(local_only_evt, drv_tx_frames_pending,
	TP_PROTO(struct ieee80211_local *local),
	TP_ARGS(local)
);

DEFINE_EVENT(local_only_evt, drv_offchannel_tx_cancel_wait,
	TP_PROTO(struct ieee80211_local *local),
	TP_ARGS(local)
);


/* struct mac80211_tx_scheduler - TX Scheduler information and data
 *
 * The main data structures and information related to the TX Scheduler
 * that are accessible by service policies are defined in this DS.
 */
struct mac80211_tx_scheduler {
	struct ieee80211_local *local;

	struct tasklet_struct scheduler_tasklet;

	//int done_skb_counter;
	spinlock_t ready_lock;

};

/*
* @ac_num: The AC for this queue
* @npkt: The number of available packets in the TXQ
* @len: The total number of TXQ bytes
* @last_skb: Pointer to the last added packet
*/
struct sched_txq_info {

	unsigned long flags;
	struct txq_info *txqi;

	u8 ac_num;

	atomic_t npkt;
	atomic_t len;

	u16 seqno_start;
	u16 seqno_next;

	struct sk_buff *last_skb;

	ktime_t last_pkt_arrival_time;

	ktime_t	delay_bound;
	struct mts_decimal_num delay_violate;

	struct stxqi_statistics {

		struct mts_ewma empty_pr;

		struct mts_ewma avg_pkt_len;
		struct mts_ewma avg_byte_len;

		struct mts_ewma avg_service_time;

		struct mts_ewma avg_pkt_rate;
		struct mts_ewma avg_byte_rate;

		struct list_head arrival_list;
		u32 arrival_list_npkt;
		u64 arrival_list_len;

	} statistics;

	/* Optional members */
	ktime_t time_allowance;

	/* TODO: Change to pkt list */
	void *sp_prv;

	void *sp_data;
};

struct sched_skb_data {

	struct sched_txq_info *stxqi;
	u16 seqno;
};

struct timer_data {

	struct mac80211_tx_scheduler *mts;
	u8 ac_num;
};

/* The structure should not be accessed directly
 * but only through the helper functions.
 *
 * @ampdu_density: Minimum A-MPDU spacing
 * @real_agg_len: Total length without physical headers,
 * padding, delimiter and etc.
 */
struct mts_aggr_limit_check {

	struct mac80211_tx_scheduler *mts;
	u32 max_bytes;
	struct ieee80211_tx_rate rate;
	u8 ampdu_density;

	bool non_agg;
	u16 pad_bytes;
};

/* struct tx_sched_ops - Callbacks from TX Scheduler to Service Policy
 *
 * This structure contains  various callbacks that the service policy
 * must handle.
 *
 * TX Scheduler will call these functions according to conditions.
 *
 * @init: Called when the service policy is loaded in order to
 *  initialize service policy.
 *  Can be NULL, if not required.
 *
 * @wake_tx_queue: Called when new packets have been added to the
 *  queue.
 *
 * @scheduler: The main decisions are made in this function.
 *  Must be atomic.
 *
 * @drv_ready: Called when driver is ready.
 *  Can be NULL, if not required.
 *
 * @exit: Called when the service policy is unloaded.
 */
struct tx_sched_ops {
	void (*init)(struct mac80211_tx_scheduler *mts);

	void (*debugfs_hw)(struct mac80211_tx_scheduler *mts);

	void (*debugfs_txq)(struct mac80211_tx_scheduler *mts,
			struct sched_txq_info *stxqi, struct dentry *txq_dir);

	void (*wake)(struct mac80211_tx_scheduler *mts,
			struct sched_txq_info *stxqi, enum wake_type type);

	void (*scheduler)(struct mac80211_tx_scheduler *mts, u8 ac_num);

	bool (*hold)(struct mac80211_tx_scheduler *mts, u8 ac_num,
			bool complete);

	void (*tx_done)(struct mts_ac_info *aci,
			struct sched_tx_info *stxi);

	void (*init_txq)(struct mac80211_tx_scheduler *mts,
				struct sched_txq_info *stxqi);

	void (*purge_txq)(struct mac80211_tx_scheduler *mts,
			struct sched_txq_info *stxqi, bool free);

	void (*beacon) (struct mac80211_tx_scheduler *mts);

	void (*change_bi) (struct mac80211_tx_scheduler *mts);

	void (*exit)(struct mac80211_tx_scheduler *mts);

	bool beacon_wake;

	const char *name;
};

// ===================================================================
//                       Common Data Structures
// ===================================================================
/* Please define commonly used Data Structures for service policies,
 * right here to avoid redundancy.
 */

struct pkt_txq_entry {

	struct sk_buff *skb;

	struct sched_txq_info *stxqi;

	unsigned long arrival_JFs;
	unsigned int len;

	bool dont_aggr;

	struct list_head list;
};

struct pkt_info_entry {

	struct sk_buff *skb;

	/* Arrival Time */
	ktime_t tstamp;

	unsigned int len;
	bool dont_aggr;

	struct list_head list;
};

struct txq_entry {
	/* Must be first */
	struct list_head pkt_list;

	struct sched_txq_info *stxqi;

	/* First packet arrival time */
	ktime_t first_tstamp;

	struct list_head list;
};

// ===================================================================

/* This must be called once for each hardware device.
 *
 * This function will initialize  the TX Scheduler and initializing
 * function of each service policy must be called in this function.
 *
 * @local: every Local has its own TX Scheduler therefore the Local
 * value is sent to the function.
 *
 * @ops: The received OPS from the driver for checking and modifying.
 */
void tx_scheduler_init (struct ieee80211_local *local,
		const struct ieee80211_ops *ops);

void tx_scheduler_debugfs_hw_add(struct ieee80211_local *local);

void tx_scheduler_debugfs_sta_add(struct sta_info *sta);

void tx_scheduler_debugfs_sta_remove(struct sta_info *sta);

/* By calling this function, Driver informs TX Scheduler that has
 * placed previous packets in HW queue, so it's ready to receive
 * new packets.
 */
bool tx_scheduler_drv_get_rates (struct ieee80211_vif *vif,
	    struct ieee80211_sta *sta, struct sk_buff *skb,
	    struct ieee80211_tx_rate *rates, int *max_rates);

u16 ieee80211_select_queue_80211(struct ieee80211_sub_if_data *sdata,
				 struct sk_buff *skb,
				 struct ieee80211_hdr *hdr);
u16 ieee80211_select_queue(struct ieee80211_sub_if_data *sdata,
			   struct sk_buff *skb);
void ieee80211_set_qos_hdr(struct ieee80211_sub_if_data *sdata,
			   struct sk_buff *skb);

/* mac80211 calls this function for sending wake signal to
 * TX Scheduler
 */
void tx_scheduler_wake (struct ieee80211_local *local,
		struct txq_info *txqi, enum wake_type type);

void tx_scheduler_tx_done (struct ieee80211_local *local,
		struct sk_buff *skb);

void tx_scheduler_init_txq (struct ieee80211_local *local,
		struct txq_info *txqi);

void tx_scheduler_purge_txq (struct ieee80211_local *local,
		struct txq_info *txqi, bool free);

void tx_scheduler_beacon_tim (struct ieee80211_local *local,
		struct ieee80211_vif *vif);

void tx_scheduler_change_bi (struct ieee80211_local *local,
		struct ieee80211_bss_conf *bss_conf);

#endif /* _TX_SCHED_H */
