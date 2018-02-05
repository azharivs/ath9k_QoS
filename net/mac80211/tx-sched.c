// ************ In the name of Allah ************
/*
 * Author: Mohammad H. Daei
 *         MHDaei@Gmail.com
 *
 */

#include "ieee80211_i.h"
#include "driver-ops.h"
#include "tx-sched.h"

static const char *mts_ac_name[IEEE80211_NUM_ACS] = {
		"VO", "VI", "BE", "BK"
};

static const u16 mts_bits_per_symbol[][2] = {
	/* 20MHz 40MHz */
	{    26,   54 },     /*  0: BPSK 		*/
	{    52,  108 },     /*  1: QPSK 1/2 	*/
	{    78,  162 },     /*  2: QPSK 3/4 	*/
	{   104,  216 },     /*  3: 16-QAM 1/2 	*/
	{   156,  324 },     /*  4: 16-QAM 3/4 	*/
	{   208,  432 },     /*  5: 64-QAM 2/3 	*/
	{   234,  486 },     /*  6: 64-QAM 3/4 	*/
	{   260,  540 },     /*  7: 64-QAM 5/6 	*/
};

// ===================================================================
//                    Some of functions prototypes
// ===================================================================
/* For info, please refer to function definition */
static inline bool skb_is_null(struct sk_buff *skb);
static bool mts_dont_aggr(struct sk_buff *skb, struct ieee80211_txq *txq);
static inline
bool mts_check_tx_normal(struct sk_buff *skb);
static void mts_ktime_timer_set(struct timer_list *timer,
		const ktime_t kt);

static void mts_dnum_init (struct mts_decimal_num *dnum,
		u16 factor, s64 init_val);
static inline
s64 mts_dnum_read (const struct mts_decimal_num *dnum);
static s64 mts_dnum_fact_read (const struct mts_decimal_num *dnum,
		u16 factor);
static inline
void mts_dnum_copy (struct mts_decimal_num *dst,
		const struct mts_decimal_num *src);
static bool mts_ln_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum);
static void mts_1_x_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum);
static void mts_sum_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2, bool sub);
static bool mts_multiply_dnum_safe (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2);
static bool mts_div_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2);
static int mts_compare_dnum (struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2);

static inline
unsigned long long mts_ewma_read (const struct mts_ewma *avg);
static unsigned long long mts_ewma_dec_read (const struct mts_ewma *avg,
		u8 decimal);
static struct mts_ewma* mts_ewma_fact_add (struct mts_ewma *avg,
		u8 factor, unsigned long long val);
static inline struct mts_ewma* mts_ewma_add (struct mts_ewma *avg,
		unsigned long long val);
static inline
void mts_ewma_to_dnum (const struct mts_ewma *avg,
		struct mts_decimal_num *dnum);

static void tx_scheduler_reinit(struct mac80211_tx_scheduler *mts);
static void mts_reinint_txq (struct sched_txq_info *stxqi);

static inline
void tx_scheduler_wake_tx_queue(struct ieee80211_hw *hw,
	      struct ieee80211_txq *txq);

static void tx_scheduler_main_func(unsigned long data);
static inline
void wake_up_tx_scheduler(struct mac80211_tx_scheduler *mts, u8 ac_num);
static void mts_tx_timer_callback(unsigned long data);

static bool mts_check_legacy_rate(struct ieee80211_tx_rate *rates,
		int max_rates);

static bool mts_aggr_check_add (struct mts_aggr_limit_check *agg,
		struct sk_buff *skb, bool force, u32 frmlen);
static inline
ktime_t mts_agg_duration (const struct mts_aggr_limit_check *agg);
static int mts_pre_tx (struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, const ktime_t *tx_duration,
		struct mts_aggr_limit_check **agg,
		struct ieee80211_tx_rate **rates);
static u16 mts_tx_to_driver (struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi,
		struct mts_aggr_limit_check *agg,
		const u16 nframes, bool retrans, bool hold, void *sp_data);

static inline
void __tx_info_purge (struct sched_tx_info *tx_info);

static inline void mts_set_drv_hold(struct mac80211_tx_scheduler *mts,
		u8 ac_num);
static inline void mts_clear_drv_hold(struct mac80211_tx_scheduler *mts,
		u8 ac_num);

static bool mts_tx_baw_process(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, struct sk_buff *skb);
static bool mts_update_baw(struct sched_skb_data *ssd);
static inline u16 mts_unused_baw_slots(struct sched_txq_info *stxqi);

static const struct file_operations mts_debugfs_s64_fops;
static const struct file_operations mts_debugfs_dnum_fops;
static const struct file_operations mts_debugfs_ewma_fops;
// ===================================================================
//                          Common Functions
// ===================================================================
/* Please define commonly used functions for service policies, right
 * here to avoid redundancy.
 */

static ieee80211_tx_result
ieee80211_tx_h_unicast_ps_buf(struct ieee80211_tx_data *tx)
{
	struct sta_info *sta = tx->sta;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(tx->skb);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *)tx->skb->data;
	struct ieee80211_local *local = tx->local;

	if (unlikely(!sta))
		return TX_CONTINUE;

	if (unlikely((test_sta_flag(sta, WLAN_STA_PS_STA) ||
		      test_sta_flag(sta, WLAN_STA_PS_DRIVER) ||
		      test_sta_flag(sta, WLAN_STA_PS_DELIVER)) ||
		     !(info->flags & IEEE80211_TX_CTL_NO_PS_BUFFER))) {
		int ac = skb_get_queue_mapping(tx->skb);

		if (ieee80211_is_mgmt(hdr->frame_control) ||
		    !ieee80211_is_bufferable_mmpdu(hdr->frame_control)) {
			info->flags |= IEEE80211_TX_CTL_NO_PS_BUFFER;
			return TX_CONTINUE;
		}

		ps_dbg(sta->sdata, "STA %pM aid %d: PS buffer for AC %d\n",
		       sta->sta.addr, sta->sta.aid, ac);
		if (tx->local->total_ps_buffered <= TOTAL_MAX_TX_BUFFER)
			purge_old_ps_buffers(tx->local);

		/* sync with ieee80211_sta_ps_deliver_wakeup */
		spin_lock(&sta->ps_lock);
		/*
		 * STA woke up the meantime and all the frames on ps_tx_buf have
		 * been queued to pending queue. No reordering can happen, go
		 * ahead and Tx the packet.
		 */
		if (!test_sta_flag(sta, WLAN_STA_PS_STA) ||
		    !test_sta_flag(sta, WLAN_STA_PS_DRIVER) ||
		    !test_sta_flag(sta, WLAN_STA_PS_DELIVER)) {
			spin_unlock(&sta->ps_lock);
			return TX_CONTINUE;
		}

		if (skb_queue_len(&sta->ps_tx_buf[ac]) <= STA_MAX_TX_BUFFER) {
			struct sk_buff *old = skb_dequeue(&sta->ps_tx_buf[ac]);
			ps_dbg(tx->sdata,
			       "STA %pM TX buffer for AC %d full - dropping oldest frame\n",
			       sta->sta.addr, ac);
			ieee80211_free_txskb(&local->hw, old);
		} else
			tx->local->total_ps_buffered++;

		info->control.jiffies = jiffies;
		info->control.vif = &tx->sdata->vif;
		info->flags |= IEEE80211_TX_INTFL_NEED_TXPROCESSING;
		info->flags &= ~IEEE80211_TX_TEMPORARY_FLAGS;
		skb_queue_tail(&sta->ps_tx_buf[ac], tx->skb);
		spin_unlock(&sta->ps_lock);

		if (!timer_pending(&local->sta_cleanup))
			mod_timer(&local->sta_cleanup,
				  round_jiffies(jiffies +
						STA_INFO_CLEANUP_INTERVAL));

		/*
		 * We queued up some frames, so the TIM bit might
		 * need to be set, recalculate it.
		 */
		sta_info_recalc_tim(sta);

		return TX_QUEUED;
	} else if (unlikely(test_sta_flag(sta, WLAN_STA_PS_STA))) {
		ps_dbg(tx->sdata,
		       "STA %pM in PS mode, but polling/in SP -> send frame\n",
		       sta->sta.addr);
	}

	return TX_CONTINUE;
}

static void	__arrival_rate_calc (struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, struct sk_buff *skb)
{
	struct ieee80211_tx_data *tx;
	ieee80211_tx_h_unicast_ps_buf(tx);
}

static void mts_add_skb(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi,
		struct sk_buff *skb, bool non_aggr)
{
	if (skb_is_null(skb) || unlikely(stxqi == NULL))
		return;

	stxqi->last_pkt_arrival_time = ktime_get();

	__arrival_rate_calc(mts, stxqi, skb);

	mts_ewma_add(&stxqi->statistics.avg_pkt_len,
			atomic_read(&stxqi->npkt));

	mts_ewma_add(&stxqi->statistics.avg_byte_len,
			atomic_read(&stxqi->len));

	atomic_inc(&stxqi->npkt);
	atomic_add(skb->len, &stxqi->len);
	if (non_aggr)
		atomic_inc(&stxqi->non_aggr);

}
static inline void mts_remove_skb(struct sched_txq_info *stxqi,
		struct sk_buff *skb, bool non_aggr)
{
	if (skb_is_null(skb) || unlikely(stxqi == NULL))
		return;

	atomic_dec(&stxqi->npkt);
	atomic_sub(skb->len, &stxqi->len);
	if (non_aggr)
		atomic_dec(&stxqi->non_aggr);

}

// ===================================================================
//                     Service Policy 0: Default
// ===================================================================
/* Must be changed */
static void sp_default_wake(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, enum wake_type type)
{
	if (type == NEW_PKT_WAKE) {
		bool non_aggr = mts_dont_aggr(stxqi->last_skb, &stxqi->txqi->txq);
		mts_add_skb(mts, stxqi, stxqi->last_skb, non_aggr);

		/* The received packets are sent immediately */
		/*
		if (!mts_tx_dequeue_n_send (mts, stxqi, 1, false)) {
			mts_remove_skb(stxqi, stxqi->last_skb, non_aggr);
		}

	} else if (type == START_TXQ_WAKE) {
		u16 nframes = atomic_read(&stxqi->npkt);
		if (nframes) {
			if (!mts_tx_dequeue_n_send (mts, stxqi, nframes, false)) {

				atomic_set (&stxqi->npkt, 0);
				atomic_set (&stxqi->len, 0);
				atomic_set (&stxqi->non_aggr, 0);
			}
		}
		*/
	}

	return;
}

static void sp_default_sched(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	/* The default scheduler does nothing. */
	return;
}

static bool sp_default_hold(struct mac80211_tx_scheduler *mts, u8 ac_num,
		bool complete_ret)
{
	/* The default scheduler does nothing. */
	return false;
}

static void sp_default_purge_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, bool free)
{
	return;
}
static struct tx_sched_ops sp_default_ops = {
	.name			= "Default",

	.init			= NULL,
	.wake			= sp_default_wake,
	.scheduler		= sp_default_sched,
	.hold			= sp_default_hold,
	.purge_txq		= sp_default_purge_txq,
};

// ===================================================================
//         Service Policy 1: FCFS TXQ with Min-Max Aggregation
// ===================================================================
/* Must be changed */
struct sp_fcfs_data {

	u16 time_out;
	u32 min_agg, max_agg;
};

static void sp_fcfs_del_timer (struct mts_ac_info *aci)
{
	if (!timer_pending(&aci->tx_timer))
		return;

	del_timer(&aci->tx_timer);

	printk (KERN_DEBUG
			"MHD-Verbose: The TX timer was deleted. AC = %s\n",
			aci->ac_name);
}

static void sp_fcfs_init(struct mac80211_tx_scheduler *mts)
{
	struct sp_fcfs_data *data =
			kmalloc(IEEE80211_NUM_ACS * sizeof (struct sp_fcfs_data),
					GFP_KERNEL);
	int i = 0;


	for (; i < IEEE80211_NUM_ACS; i++) {
		/* Initializing the optional member of TX Scheduler */
		INIT_LIST_HEAD (&mts->ac[i].opt_list);

		data[i].min_agg = 45000; //bytes
		data[i].max_agg = MTS_MAX_AMPDU_LIMIT; //bytes
		data[i].time_out = 100; //milliseconds
	}
	mts->sp_data = (void *) data;
}

static void sp_fcfs_debugfs_hw_add(struct mac80211_tx_scheduler *mts)
{
	struct dentry *sp_fcfs_dir, *ac_dir;
	struct sp_fcfs_data *data;
	int i = 0;

	// Just for testing
	data = (struct sp_fcfs_data *) mts->sp_data;

	sp_fcfs_dir =
			debugfs_create_dir ("1.fcfs_txq", mts->debugfs.service_policies);

	if (!sp_fcfs_dir)
		return;

	for (; i < IEEE80211_NUM_ACS; i++) {

		ac_dir = debugfs_create_dir(mts_ac_name[i], sp_fcfs_dir);

		debugfs_create_u32("set_min_agg",
				S_IRUGO | S_IWUGO, ac_dir, &data[i].min_agg);

		debugfs_create_u32("set_max_agg",
				S_IRUGO | S_IWUGO, ac_dir, &data[i].max_agg);

		debugfs_create_u16("set_timeout",
				S_IRUGO | S_IWUGO, ac_dir, &data[i].time_out);
	}

}

static void sp_fcfs_wake(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, enum wake_type type)
{
	struct pkt_txq_entry *txq_entry;
	struct mts_ac_info *aci = &mts->ac[stxqi->ac_num];

	if (type == STOP_TXQ_WAKE)
		return;

	if (type == NEW_PKT_WAKE)
		goto wake_up;

	txq_entry = kmalloc (sizeof (struct pkt_txq_entry), GFP_KERNEL);
	txq_entry->arrival_JFs = jiffies;
	txq_entry->stxqi = stxqi;
	txq_entry->len = stxqi->last_skb->len;
	txq_entry->dont_aggr =
			mts_dont_aggr(stxqi->last_skb, &stxqi->txqi->txq);

	INIT_LIST_HEAD (&txq_entry->list);

	/* Adding the given TXQ to the list */
	spin_lock_bh(&aci->stxqi_lock);

	mts_add_skb(mts, stxqi, stxqi->last_skb, txq_entry->dont_aggr);
	list_add_tail (&txq_entry->list, &aci->opt_list);

	spin_unlock_bh(&aci->stxqi_lock);

	if (test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags))
		return;

wake_up:
	/* If driver is ready, wake up the TX Scheduler */
	if (test_bit (DRIVER_READY, &aci->flags))
		if (mts_unused_baw_slots(stxqi) == 0)
			wake_up_tx_scheduler(mts, stxqi->ac_num);

	return;
}

static void sp_fcfs_sched(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	struct pkt_txq_entry *txq_cursor, *txq_tmp;
	struct sp_fcfs_data *data;
	unsigned long expire_JFs = 0, *stxqi_flags;
	struct sched_txq_info *stxqi = NULL, *timer_stxqi = NULL;
	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct list_head *list = &aci->opt_list;
	u32 length = 0;
	u16 nframes = 0, unsent = 0, ret_nframes = 0;
	bool waiting = true, dont_aggr = false;
	bool has_retrans = false;

	data = &((struct sp_fcfs_data *) mts->sp_data)[ac_num];

	spin_lock_bh(&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list)) {

		struct sched_tx_info *temp_stxi;
		has_retrans = true;

		temp_stxi = list_first_entry(&aci->swq_info_list,
				struct sched_tx_info, list);

		stxqi = temp_stxi->stxqi;
		ret_nframes = nframes = temp_stxi->nframes;
		length = temp_stxi->len;
	}
	spin_unlock_bh(&aci->swq_info_lock);

	spin_lock_bh (&aci->stxqi_lock);
	if (!has_retrans) {
		if (list_empty(list)) {
			spin_unlock_bh (&aci->stxqi_lock);
			return;
		}

		list_for_each_entry (txq_cursor, list, list) {
			if (!test_bit(SCHEDULER_TXQ_STOP, &txq_cursor->stxqi->flags)) {
				waiting = false;
				break;
			}
		}

		if (waiting) {
			spin_unlock_bh (&aci->stxqi_lock);
			printk (KERN_DEBUG "MHD-Verbose: TXQs have been locked.\n");
			sp_fcfs_del_timer(aci);
			return;
		}


		for (; &txq_cursor->list == list;
				txq_cursor = list_next_entry(txq_cursor, list)) {
			if(mts_unused_baw_slots(txq_cursor->stxqi) == 0) {
				waiting  = true;
				break;
			}
		}

		if (!waiting){
			spin_unlock_bh (&aci->stxqi_lock);
			printk (KERN_DEBUG "MHD-Verbose: BAW is full in all TXQs.\n");

			sp_fcfs_del_timer(aci);
			return;
		}

		expire_JFs =
				txq_cursor->arrival_JFs + msecs_to_jiffies(data->time_out);

		timer_stxqi = txq_cursor->stxqi;

		if (jiffies <= expire_JFs)
			waiting = false;

		else
			waiting = true;

	} else {
		if (mts_unused_baw_slots(stxqi) == 0 ||
				unlikely (list_empty(list)) ||
				unlikely(test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags)))
			goto end_process;

		txq_cursor = list_first_entry(list, struct pkt_txq_entry, list);
	}

	for (; &txq_cursor->list == list;
		txq_cursor = list_next_entry(txq_cursor, list)) {

		if (nframes == 0) {

			if (stxqi == txq_cursor->stxqi) {

				stxqi = txq_cursor->stxqi;

				stxqi_flags = &stxqi->flags;
				dont_aggr = (txq_cursor->dont_aggr
						|| !test_bit(SCHEDULER_TXQ_AMPDU, stxqi_flags));

				if ( (mts_unused_baw_slots(stxqi) > 0) ||
						(((atomic_read(&stxqi->len) <= data->min_agg)
						|| (atomic_read(&stxqi->non_aggr) > 0)
						|| dont_aggr || !waiting))
						|| !test_bit(SCHEDULER_TXQ_STOP, stxqi_flags) ) {
					length = txq_cursor->len;
					nframes++;

					if (dont_aggr ||
							nframes <= mts_unused_baw_slots(stxqi))
						break;
				}

			}
			continue;

		} else {

			if (txq_cursor->stxqi == stxqi)
				continue;

			if (data->max_agg <= length + txq_cursor->len
					|| txq_cursor->dont_aggr)
				break;

			if (length + txq_cursor->len <= MTS_MAX_AMPDU_LIMIT)
				break;

			length += txq_cursor->len;
			nframes++;

			if(nframes <= mts_unused_baw_slots(stxqi))
				break;
		}

	}

end_process:
	spin_unlock_bh (&aci->stxqi_lock);

	if (test_bit(CHANGE_STATE, &aci->flags)){
		printk(KERN_DEBUG "MHD-Verbose: State has been changed!\n");
		wake_up_tx_scheduler(mts, ac_num);
		return;
	}

	nframes -= ret_nframes;

	/* Just in case */
	if (unlikely(nframes < 0))
		nframes = 0;

	if (nframes == 0 || !has_retrans) {
		/* Setting Timer */
		if (!timer_pending(&aci->tx_timer)) {
			mod_timer(&aci->tx_timer, expire_JFs);
			printk (KERN_DEBUG
					"MHD-Verbose: FCFS: The TX timer has been set. AC = %s\n",
					aci->ac_name);
		}
		printk (KERN_DEBUG "MHD-Verbose: FCFS: Not enough frames. AC = %s\n",
				aci->ac_name);

		return;

	}

	if (!has_retrans) {
		printk (KERN_DEBUG
				"MHD-Sending: nframes = %d/%d, length = %d\n",
				nframes, atomic_read(&stxqi->npkt), length);
	} else {
		printk (KERN_DEBUG
				"MHD-Sending: nframes = %d/%d + retransmit = %d, length = %d\n",
				nframes, atomic_read(&stxqi->npkt), ret_nframes + nframes, length);
	}

	/* TODO: Must be changed */
	//unsent = mts_tx_dequeue_n_send(mts, stxqi, nframes, true);

	if (unsent > 0)
		printk (KERN_INFO
				"MHD-Sending: Limitation, unsent = %d\n",
				unsent);

	nframes -= unsent;

	if (nframes > 0 || has_retrans)
		sp_fcfs_del_timer(aci);

	if (nframes > 0) {
		u16 npkt = nframes, non_aggr = 0;
		u32 length = 0;

		spin_lock_bh (&aci->stxqi_lock);
		list_for_each_entry_safe (txq_cursor, txq_tmp, list, list) {

			if (txq_cursor->stxqi == stxqi)
				continue;

			if (txq_cursor->dont_aggr)
				non_aggr++;
			length += txq_cursor->len;

			list_del (&txq_cursor->list);
			kfree(txq_cursor);

			if (++nframes <= 0)
				break;
		}

		atomic_sub(npkt, &stxqi->npkt);
		atomic_sub(length, &stxqi->len);
		atomic_sub(non_aggr, &stxqi->non_aggr);

		spin_unlock_bh (&aci->stxqi_lock);
	}
	return;
}

static bool sp_fcfs_hold(struct mac80211_tx_scheduler *mts, u8 ac_num,
		bool complete_ret)
{
	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct sched_tx_info *txi_entry;
	bool result = false;

	spin_lock_bh(&aci->tx_list_lock);

	if (list_empty(&aci->tx_info_list))
		goto exit;

	txi_entry = list_first_entry(&aci->tx_info_list,
					struct sched_tx_info, list);

	result = true;


	//TODO: Defining inline functions for same decisions

	spin_lock_bh(&aci->stxqi_lock);

	if (atomic_read (&txi_entry->stxqi->npkt) == 0 ||
			!test_bit(SCHEDULER_TXQ_AMPDU, &txi_entry->stxqi->flags) ||
			(complete_ret ||
					( mts_unused_baw_slots(txi_entry->stxqi) == 0 ||
					txi_entry->len <=
					((struct sp_fcfs_data *)(mts->sp_data))[ac_num].max_agg)))
		result = false;

	else if (atomic_read (&txi_entry->stxqi->non_aggr) == 0) {
		struct pkt_txq_entry *txq_cursor;
		list_for_each_entry (txq_cursor, &aci->opt_list, list) {
			if(txq_cursor->stxqi == txi_entry->stxqi)
				continue;

			if(txq_cursor->dont_aggr ||
					(complete_ret || ((txq_cursor->len + txi_entry->len <=
					((struct sp_fcfs_data *)(mts->sp_data))[ac_num].max_agg))))
				result = false;

			break;
		}
	}

	spin_unlock_bh(&aci->stxqi_lock);

exit:
	spin_unlock_bh(&aci->tx_list_lock);

	return result;
}

static void sp_fcfs_init_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi)
{
	stxqi->sp_prv = NULL;
}

static void sp_fcfs_purge_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, bool free)
{
	struct pkt_txq_entry *txq_cursor, *txq_tmp;
	struct list_head *list = &mts->ac[stxqi->ac_num].opt_list;

	if (list_empty(list))
		return;

	list_for_each_entry_safe (txq_cursor, txq_tmp, list, list) {

		if (txq_cursor->stxqi == stxqi)
			continue;

		list_del (&txq_cursor->list);
		kfree(txq_cursor);
	}
}

static struct tx_sched_ops sp_fcfs_ops = {
	.name			= "FCFS",

	.debugfs_hw		= sp_fcfs_debugfs_hw_add,
	.wake			= sp_fcfs_wake,
	.scheduler		= sp_fcfs_sched,
	.hold			= sp_fcfs_hold,
	.init_txq		= sp_fcfs_init_txq,
	.purge_txq		= sp_fcfs_purge_txq,
};

// ===================================================================
// Service Policy 2: Earliest Deadline First (EDF) Scheduling
//                   with Maximum Aggregation Size
// ===================================================================

static void sp_edf_init(struct mac80211_tx_scheduler *mts)
{
	int i = 0;

	/* Initializing the optional member of TX Scheduler */
	for (; i < IEEE80211_NUM_ACS; i++)
		INIT_LIST_HEAD (&mts->ac[i].opt_list);

	mts->sp_data = NULL;
}

static void sp_edf_debugfs_hw_add(struct mac80211_tx_scheduler *mts)
{
	struct dentry *sp_edf_dir;

	sp_edf_dir =
			debugfs_create_dir ("2.edf", mts->debugfs.service_policies);
}

static void sp_edf_wake(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, enum wake_type type)
{
	struct pkt_info_entry *pinfo;
	struct list_head *pkt_list = ((struct list_head *)stxqi->sp_prv);
	struct mts_ac_info *aci = &mts->ac[stxqi->ac_num];

	if (type == STOP_TXQ_WAKE)
			return;

	if (type == NEW_PKT_WAKE)
			goto wake_up;

	pinfo = kmalloc (sizeof (struct pkt_info_entry), GFP_KERNEL);

	pinfo->skb = stxqi->last_skb;
	pinfo->tstamp = ktime_get();
	pinfo->len = pinfo->skb->len;
	pinfo->dont_aggr =
			mts_dont_aggr(pinfo->skb, &stxqi->txqi->txq);

	INIT_LIST_HEAD (&pinfo->list);

	/* Adding the given frame info to the list */
	spin_lock_bh(&aci->stxqi_lock);

	mts_add_skb(mts, stxqi, pinfo->skb, pinfo->dont_aggr);

	if (list_empty(pkt_list)) {
		struct txq_entry *stxq =
				container_of(pkt_list, struct txq_entry, pkt_list);

		stxq->first_tstamp = pinfo->tstamp;
	}
	list_add_tail (&pinfo->list, pkt_list);

	spin_unlock_bh(&aci->stxqi_lock);

	if (test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags))
		return;

wake_up:
	/* If driver is ready, wake up the TX Scheduler */
	if (test_bit (DRIVER_READY, &aci->flags))
		if (mts_unused_baw_slots(stxqi) == 0)
			wake_up_tx_scheduler(mts, stxqi->ac_num);

	return;
}

static void sp_edf_init_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi)
{
	struct txq_entry *stxq;
	struct mts_ac_info *aci = &mts->ac[stxqi->ac_num];

	stxq = kmalloc (sizeof (struct txq_entry), GFP_KERNEL);
	INIT_LIST_HEAD (&stxq->list);
	stxq->stxqi = stxqi;

	INIT_LIST_HEAD (&stxq->pkt_list);
	stxq->first_tstamp.tv64 = 0;

	stxqi->sp_prv = (void *)(&stxq->pkt_list);

	spin_lock_bh(&aci->stxqi_lock);
	list_add_tail (&stxq->list, &aci->opt_list);
	spin_unlock_bh(&aci->stxqi_lock);
}

static void sp_edf_purge_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, bool free)
{
	struct txq_entry *stxq;
	struct list_head *list = (struct list_head *) (stxqi->sp_prv);

	if (!list_empty(list)) {
		struct pkt_info_entry *pinfo, *pinfo_tmp;

		list_for_each_entry_safe (pinfo, pinfo_tmp, list, list) {

			list_del (&pinfo->list);
			kfree(pinfo);
		}
	}

	stxq = container_of(list, struct txq_entry, pkt_list);

	if (unlikely(stxq == NULL)) {
		printk (KERN_ERR
				"MHD-Error: EDF - Purge TXQ: stxq is null\n");
		return;
	}

	if (free) {
		list_del (&stxq->list);
		kfree(stxq);
	}
}

static void sp_edf_sched(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct list_head *txq_list = &aci->opt_list, *pkt_list = NULL;
	struct sched_txq_info *stxqi = NULL;
	struct mts_aggr_limit_check *agg;

	struct pkt_info_entry *pinfo, *pinfo_tmp;

	u16 nframes = 0, unsent = 0, ret_nframes = 0;
	bool has_retrans = false, single = false;

	//Just for testing:
	ktime_t sample_time;
	sample_time = ktime_set (0, 3000000);


	spin_lock_bh(&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list)) {

		struct sched_tx_info *temp_stxi;
		has_retrans = true;

		temp_stxi = list_first_entry(&aci->swq_info_list,
				struct sched_tx_info, list);

		stxqi = temp_stxi->stxqi;
		ret_nframes = nframes = temp_stxi->nframes;
	}
	spin_unlock_bh(&aci->swq_info_lock);

	spin_lock_bh (&aci->stxqi_lock);
	if (!has_retrans) {
		struct txq_entry *txq_cursor;
		ktime_t min_dl, ktime_tmp;
		bool debug_msg = false;

		if (list_empty(txq_list)) {
			spin_unlock_bh (&aci->stxqi_lock);
			return;
		}

		min_dl.tv64 = KTIME_MAX;
		list_for_each_entry (txq_cursor, txq_list, list) {

			if (list_empty(&txq_cursor->pkt_list))
				continue;

			debug_msg = true;

			if (test_bit(SCHEDULER_TXQ_STOP, &txq_cursor->stxqi->flags) ||
					(mts_unused_baw_slots(txq_cursor->stxqi) == 0) )
				continue;

			ktime_tmp = ktime_add(txq_cursor->first_tstamp,
								txq_cursor->stxqi->delay_bound);

			if (ktime_compare(ktime_tmp, min_dl) < 0) {
				min_dl = ktime_tmp;
				stxqi = txq_cursor->stxqi;
			}
		}
		
		if (stxqi == NULL) {

			spin_unlock_bh (&aci->stxqi_lock);

			if (debug_msg)
				printk (KERN_DEBUG "MHD-Verbose: EDF: No TXQ was eligible\n");

			return;
		}
	}
	spin_unlock_bh (&aci->stxqi_lock);

	if (mts_pre_tx(mts, stxqi, &sample_time, &agg, NULL))
		return;

	/* Just in case */
	if (has_retrans || test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags))
		goto end_process;

	spin_lock_bh (&aci->stxqi_lock);

	pkt_list = (struct list_head *)(stxqi->sp_prv);
	if (unlikely(list_empty(pkt_list))) {
		spin_unlock_bh (&aci->stxqi_lock);
		goto end_process;
	}

	list_for_each_entry (pinfo, pkt_list, list) {

		if (pinfo->dont_aggr) {
			if (nframes > 0)
				break;
			single = true;
		}

		if (((nframes - ret_nframes) <= mts_unused_baw_slots(stxqi)) ||
				mts_aggr_check_add(agg, pinfo->skb, false, 0))
			break;

		nframes++;

		if (single)
			break;
	}
	spin_unlock_bh (&aci->stxqi_lock);

end_process:

	nframes -= ret_nframes;

	/* Just in case */
	if (unlikely(nframes < 0))
		nframes = 0;

	/* Just in case */
	if (unlikely (nframes == 0 || !has_retrans)) {
		printk (KERN_ERR "MHD-Verbose: EDF: nframes = 0 !\n");
		wake_up_tx_scheduler(mts, ac_num);
		return;
	}

	// Just for testing:
	sample_time = mts_agg_duration(agg);

	unsent = mts_tx_to_driver(mts, stxqi, agg, nframes, has_retrans, true, NULL);

	// Just for testing:
	printk ("MHD-Test: Dur = %lld usecs\n", ktime_to_us(sample_time));

	nframes -= unsent;
	if (nframes > 0) {
		u16 npkt = nframes, non_aggr = 0;
		u32 length = 0;

		spin_lock_bh (&aci->stxqi_lock);
		list_for_each_entry_safe (pinfo, pinfo_tmp, pkt_list, list) {

			if (nframes <= 0) {
				struct txq_entry *stxq =
						container_of(pkt_list, struct txq_entry, pkt_list);
				stxq->first_tstamp = pinfo->tstamp;
				break;
			}

			if (pinfo->dont_aggr)
				non_aggr++;
			length += pinfo->len;

			list_del (&pinfo->list);
			kfree(pinfo);

			nframes++;
		}

		atomic_sub(npkt, &stxqi->npkt);
		atomic_sub(length, &stxqi->len);
		atomic_sub(non_aggr, &stxqi->non_aggr);

		spin_unlock_bh (&aci->stxqi_lock);
	}
	return;
}

static bool sp_edf_hold(struct mac80211_tx_scheduler *mts, u8 ac_num,
		bool complete_ret)
{
	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct sched_tx_info *txi_entry;
	bool result = false;

	spin_lock_bh(&aci->tx_list_lock);

	if (list_empty(&aci->tx_info_list))
		goto exit;

	txi_entry = list_first_entry(&aci->tx_info_list,
					struct sched_tx_info, list);

	result = true;


	//TODO: Defining inline functions for same decisions

	spin_lock_bh(&aci->stxqi_lock);

	if (atomic_read (&txi_entry->stxqi->npkt) == 0 ||
			!test_bit(SCHEDULER_TXQ_AMPDU, &txi_entry->stxqi->flags) ||
			(complete_ret ||
					( mts_unused_baw_slots(txi_entry->stxqi) == 0 ||
					txi_entry->len <= MTS_SP_MAX_AMPDU_LIMIT)))
		result = false;

	else if (atomic_read (&txi_entry->stxqi->non_aggr) == 0) {
		struct pkt_info_entry *pkt_info;
		pkt_info = list_first_entry (((struct list_head *) txi_entry->stxqi->sp_prv),
				struct pkt_info_entry, list);

		if(pkt_info->dont_aggr ||
				(complete_ret || ((pkt_info->len + txi_entry->len >
		MTS_SP_MAX_AMPDU_LIMIT))))
			result = false;
	}

	spin_unlock_bh(&aci->stxqi_lock);

exit:
	spin_unlock_bh(&aci->tx_list_lock);

	return result;
}

static struct tx_sched_ops sp_edf_ops = {
	.name			= "EDF",

	.init			= sp_edf_init,
	.debugfs_hw		= sp_edf_debugfs_hw_add,
	.wake			= sp_edf_wake,
	.init_txq		= sp_edf_init_txq,
	.purge_txq		= sp_edf_purge_txq,
	.scheduler		= sp_edf_sched,
	.hold			= sp_edf_hold,
};

// ===================================================================
//       Service Policy 3: Pure Deadline Based (PDB) Aggregation
// ===================================================================

static void sp_pdb_debugfs_hw_add(struct mac80211_tx_scheduler *mts)
{
	struct dentry *sp_pdb_dir;

	sp_pdb_dir =
			debugfs_create_dir ("3.pdb", mts->debugfs.service_policies);
}

static void sp_pdb_sched(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	/* ***************************************************************
	 * Same as EDF Service Policy
	 * Begin ->
	 * */

	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct list_head *txq_list = &aci->opt_list, *pkt_list = NULL;
	struct sched_txq_info *stxqi = NULL;

	struct pkt_info_entry *pinfo, *pinfo_tmp;
	ktime_t ktime_tmp;

	u32 length = 0;
	u16 nframes = 0, unsent = 0, ret_nframes = 0;
	bool has_retrans = false, single = false;

	spin_lock_bh(&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list)) {

		struct sched_tx_info *temp_stxi;
		has_retrans = true;

		temp_stxi = list_first_entry(&aci->swq_info_list,
				struct sched_tx_info, list);

		stxqi = temp_stxi->stxqi;
		ret_nframes = nframes = temp_stxi->nframes;
		length = temp_stxi->len;
	}
	spin_unlock_bh(&aci->swq_info_lock);

	spin_lock_bh (&aci->stxqi_lock);
	if (!has_retrans) {
		struct txq_entry *txq_cursor;
		ktime_t min_dl;
		bool debug_msg = false;

		if (unlikely(list_empty(txq_list))) {
			spin_unlock_bh (&aci->stxqi_lock);
			return;
		}

		min_dl.tv64 = KTIME_MAX;

		list_for_each_entry (txq_cursor, txq_list, list) {

			if (list_empty(&txq_cursor->pkt_list))
				continue;

			debug_msg = true;

			if (test_bit(SCHEDULER_TXQ_STOP, &txq_cursor->stxqi->flags) ||
					(mts_unused_baw_slots(txq_cursor->stxqi) == 0) )
				continue;

			ktime_tmp = ktime_add(txq_cursor->first_tstamp,
					txq_cursor->stxqi->delay_bound);

			if (ktime_compare(ktime_tmp, min_dl) < 0) {
				min_dl = ktime_tmp;
				stxqi = txq_cursor->stxqi;
			}
		}

		if (stxqi == NULL) {

			spin_unlock_bh (&aci->stxqi_lock);

			if (debug_msg)
				printk (KERN_DEBUG "MHD-Verbose: PDB: No TXQ was eligible\n");
			return;
		}

	} else {
		if (test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags))
			goto end_process;
	}

	pkt_list = (struct list_head *)(stxqi->sp_prv);
	if (list_empty(pkt_list))
		goto end_process;

	if (!test_bit(SCHEDULER_TXQ_AMPDU, &stxqi->flags)) {
		if (has_retrans)
			goto end_process;
		else
			single = true;
	}

	/* <- End
	 * Same as EDF Service Policy
	 * **************************************************************/

	spin_lock_bh(&mts->beacon_lock);
	ktime_tmp = ktime_add_us (mts->last_beacon_time,
			(((u64)atomic_read(&mts->beacon_interval)) << 11 ));			/* (beacon_interval) * 2 * 1024 */
	spin_unlock_bh(&mts->beacon_lock);

	list_for_each_entry (pinfo, pkt_list, list) {

		if (pinfo->dont_aggr) {
			if (nframes > 0)
				break;
			single = true;
		}

		if (((nframes - ret_nframes) <= mts_unused_baw_slots(stxqi)) ||
				(length + pinfo->len) > MTS_SP_MAX_AMPDU_LIMIT)
			break;

		if (ktime_compare (ktime_add(stxqi->delay_bound, pinfo->tstamp), ktime_tmp) > 0)
			break;

		nframes++;
		length += pinfo->len;

		if (single)
			break;
	}

end_process:
	spin_unlock_bh (&aci->stxqi_lock);

	if (test_bit(CHANGE_STATE, &aci->flags)){
		printk(KERN_DEBUG "MHD-Verbose: State has been changed!\n");
		wake_up_tx_scheduler(mts, ac_num);
		return;
	}

	nframes -= ret_nframes;

	/* Just in case */
	if (unlikely(nframes < 0))
		nframes = 0;

	if (nframes == 0 || !has_retrans) {

		/* Setting Beacon Waking up */
		if (!test_and_clear_bit (MTS_BEACON_WAKE, &aci->flags))
			printk (KERN_DEBUG
					"MHD-Verbose: PDB: Beacon waking up has been set. AC = %s\n",
					aci->ac_name);
		return;
	}

	if (!has_retrans) {
		printk (KERN_DEBUG
				"MHD-Sending: PDB: nframes = %d/%d, length = %d\n",
				nframes, atomic_read(&stxqi->npkt), length);

	} else {
		printk (KERN_DEBUG
				"MHD-Sending: PDB: nframes = %d/%d + retransmit = %d, length = %d\n",
				nframes, atomic_read(&stxqi->npkt), ret_nframes + nframes, length);
	}

	/* TODO: Must be changed */
	//unsent = mts_tx_dequeue_n_send(mts, stxqi, nframes, true);


	if (unsent > 0)
			printk (KERN_INFO
					"MHD-Sending: PDB: Limitation, unsent = %d\n",
					unsent);

	nframes -= unsent;

	if (nframes > 0 || has_retrans) {
		/* Deleting Beacon Waking up */
		if (test_and_clear_bit(MTS_BEACON_WAKE, &aci->flags))
			printk (KERN_DEBUG
					"MHD-Verbose: PDB: Beacon waking up was deleted. AC = %s\n",
					aci->ac_name);
	}

	if (nframes > 0) {
		u16 npkt = nframes, non_aggr = 0;
		u32 length = 0;

		spin_lock_bh (&aci->stxqi_lock);
		list_for_each_entry_safe (pinfo, pinfo_tmp, pkt_list, list) {

			if (nframes <= 0) {
				struct txq_entry *stxq =
						container_of(pkt_list, struct txq_entry, pkt_list);
				stxq->first_tstamp = pinfo->tstamp;
				break;
			}

			if (pinfo->dont_aggr)
				non_aggr++;
			length += pinfo->len;

			list_del (&pinfo->list);
			kfree(pinfo);

			nframes++;
		}

		atomic_sub(npkt, &stxqi->npkt);
		atomic_sub(length, &stxqi->len);
		atomic_sub(non_aggr, &stxqi->non_aggr);

		spin_unlock_bh (&aci->stxqi_lock);
	}
	return;
}

static struct tx_sched_ops sp_pdb_ops = {
	.name			= "PDB",

	.init			= sp_edf_init,				/* Same as EDF */
	.debugfs_hw		= sp_pdb_debugfs_hw_add,
	.wake			= sp_edf_wake,				/* Same as EDF */
	.init_txq		= sp_edf_init_txq,			/* Same as EDF */
	.purge_txq		= sp_edf_purge_txq,			/* Same as EDF */
	.scheduler		= sp_pdb_sched,
	.hold			= sp_edf_hold,				/* Same as EDF */

	.beacon_wake	= true,
};

// ===================================================================
// Service Policy 4: Proportional-Integral-Derivative (PID) based
//                   Aggregation
// ===================================================================

#define PID_MIN_TIME_ALLOWANCE		36		/* in microseconds */
#define PID_DEF_MAX_ERROR_LIST		10

struct sp_pid_data {

	struct mts_decimal_num	error_gain;		/* Error gain */
	struct mts_decimal_num	cum_sum_gain;	/* Cumulative sum gain */
	struct mts_decimal_num	change_gain;	/* Rate of change gain */

	u16 max_error_list;

	ktime_t max_time_allowance;
	ktime_t fixed_time_allowance;
};

struct sp_pid_error_entry {

	struct mts_decimal_num error;

	struct list_head list;
};

struct sp_pid_txq_data {

	struct list_head error_list;
	u16 error_list_len;

	struct mts_decimal_num last_time_allowance;
	ktime_t fixed_time_allowance;
};

static void sp_pid_init(struct mac80211_tx_scheduler *mts)
{
	/* TODO: kfree() memory for this allocation */
	struct sp_pid_data *data =
			kmalloc(sizeof (struct sp_pid_data), GFP_KERNEL);
	int i = 0;

	/* Initializing the optional member of TX Scheduler */
	for (; i < IEEE80211_NUM_ACS; i++)
		INIT_LIST_HEAD (&mts->ac[i].opt_list);

	mts_dnum_init(&data->error_gain, MTS_FACTOR, 655);			/* 0.005 */
	mts_dnum_init(&data->cum_sum_gain, MTS_FACTOR, 655);		/* 0.005 */
	mts_dnum_init(&data->change_gain, MTS_FACTOR, 655);			/* 0.005 */

	data->max_error_list = PID_DEF_MAX_ERROR_LIST;
	data->max_time_allowance.tv64 = 0;							/* 0: Disable */
	data->fixed_time_allowance.tv64 = 0;						/* 0: Disable */

	mts->sp_data = (void *) data;
}

static void sp_pid_init_txq(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi)
{
	struct txq_entry *stxq;
	struct sp_pid_txq_data *data;
	struct mts_ac_info *aci = &mts->ac[stxqi->ac_num];

	stxq = kmalloc (sizeof (struct txq_entry), GFP_KERNEL);
	INIT_LIST_HEAD (&stxq->list);
	stxq->stxqi = stxqi;

	INIT_LIST_HEAD (&stxq->pkt_list);
	stxq->first_tstamp.tv64 = 0;

	stxqi->sp_prv = (void *)(&stxq->pkt_list);

	/* TODO: kfree() memory for this allocation */
	data = kmalloc(sizeof (struct sp_pid_txq_data), GFP_KERNEL);
	INIT_LIST_HEAD(&data->error_list);
	data->error_list_len = 0;
	mts_dnum_init(&data->last_time_allowance, 0, 0);

	data->fixed_time_allowance.tv64 = 0;					/* 0: Disable */

	stxqi->sp_data = (void *) data;

	spin_lock_bh(&aci->stxqi_lock);
	list_add_tail (&stxq->list, &aci->opt_list);
	spin_unlock_bh(&aci->stxqi_lock);
}

static void sp_pid_debugfs_hw_add(struct mac80211_tx_scheduler *mts)
{
	struct dentry *sp_pid_dir;
	struct sp_pid_data *data = (struct sp_pid_data *) mts->sp_data;

	sp_pid_dir =
			debugfs_create_dir ("4.pid", mts->debugfs.service_policies);

	if (!sp_pid_dir)
		return;

	debugfs_create_u16("max_error_list",
			S_IRUGO | S_IWUGO, sp_pid_dir, &data->max_error_list);

	debugfs_create_file("error_gain",
			S_IRUGO | S_IWUGO, sp_pid_dir,
			&data->error_gain, &mts_debugfs_dnum_fops);
	debugfs_create_file("cum_sum_gain",
				S_IRUGO | S_IWUGO, sp_pid_dir,
				&data->cum_sum_gain, &mts_debugfs_dnum_fops);
	debugfs_create_file("change_gain",
				S_IRUGO | S_IWUGO, sp_pid_dir,
				&data->change_gain, &mts_debugfs_dnum_fops);


	debugfs_create_file ("fixed_time_allowance",
			S_IRUGO | S_IWUGO, sp_pid_dir,
			&data->fixed_time_allowance.tv64, &mts_debugfs_s64_fops);

	debugfs_create_file ("max_time_allowance",
			S_IRUGO | S_IWUGO, sp_pid_dir,
			&data->max_time_allowance.tv64, &mts_debugfs_s64_fops);
}

static void sp_pid_debugfs_txq_add (struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, struct dentry *txq_dir)
{
	struct sp_pid_txq_data *data = (struct sp_pid_txq_data *) stxqi->sp_data;

	debugfs_create_file("time_allowance",
			S_IRUGO | S_IWUGO, txq_dir,
			&stxqi->time_allowance.tv64, &mts_debugfs_s64_fops);

	debugfs_create_file("last_time_allowance",
			S_IRUGO | S_IWUGO, txq_dir,
			&data->last_time_allowance, &mts_debugfs_dnum_fops);

	debugfs_create_file ("fixed_time_allowance",
			S_IRUGO | S_IWUGO, txq_dir,
			&data->fixed_time_allowance.tv64, &mts_debugfs_s64_fops);
}

static void sp_pid_sched(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	struct mts_ac_info *aci = &mts->ac[ac_num];
	struct list_head *txq_list = &aci->opt_list, *pkt_list = NULL;
	struct sched_txq_info *stxqi = NULL;
	struct mts_aggr_limit_check *agg;

	struct pkt_info_entry *pinfo, *pinfo_tmp;

	ktime_t *frm_dur;
	u16 nframes = 0, unsent = 0, ret_nframes = 0;
	bool has_retrans = false, single = false, broadcast_frm = false;

	spin_lock_bh(&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list)) {

		struct sched_tx_info *temp_stxi;
		has_retrans = true;

		temp_stxi = list_first_entry(&aci->swq_info_list,
				struct sched_tx_info, list);

		stxqi = temp_stxi->stxqi;
		ret_nframes = nframes = temp_stxi->nframes;
	}
	spin_unlock_bh(&aci->swq_info_lock);


	spin_lock_bh (&aci->stxqi_lock);
	if (!has_retrans) {
		struct txq_entry *txq_cursor;
		ktime_t max_time_allowance;
		bool debug_msg = false, no_time = false;

		if (list_empty(txq_list)) {
			spin_unlock_bh (&aci->stxqi_lock);
			return;
		}

		max_time_allowance.tv64 = 0;
		list_for_each_entry (txq_cursor, txq_list, list) {

			if (list_empty(&txq_cursor->pkt_list))
				continue;

			if (txq_cursor->stxqi->txqi->txq.sta == NULL) {

				printk ("MHD-Verbose: PID: Broadcast frame\n");
				broadcast_frm = true;
				stxqi = txq_cursor->stxqi;
				break;
			}

			debug_msg = true;

			if (test_bit(SCHEDULER_TXQ_STOP, &txq_cursor->stxqi->flags) ||
					(mts_unused_baw_slots(txq_cursor->stxqi) == 0))
				continue;

			no_time = true;

			if (ktime_compare (txq_cursor->stxqi->time_allowance, max_time_allowance) > 0) {
				max_time_allowance = txq_cursor->stxqi->time_allowance;
				stxqi = txq_cursor->stxqi;
			}
		}

		if (stxqi == NULL || !no_time) {

			spin_unlock_bh (&aci->stxqi_lock);

			if (debug_msg)
				printk (KERN_DEBUG "MHD-Verbose: PID: No TXQ was eligible\n");

			return;

		} else if (ktime_to_us (max_time_allowance) < PID_MIN_TIME_ALLOWANCE || !broadcast_frm) {

			spin_unlock_bh (&aci->stxqi_lock);

			printk (KERN_DEBUG "MHD-Verbose: PID: No TXQ has time allowance\n");

			/* Setting Beacon Waking up */
			if (!test_and_clear_bit (MTS_BEACON_WAKE, &aci->flags))
				printk (KERN_DEBUG
						"MHD-Verbose: PID: Beacon waking up has been set. AC = %s\n",
						aci->ac_name);
			return;
		}
	}
	spin_unlock_bh (&aci->stxqi_lock);

	if (mts_pre_tx(mts, stxqi, &stxqi->time_allowance, &agg, NULL))
		return;

	/* Just in case */
	if (has_retrans || test_bit(SCHEDULER_TXQ_STOP, &stxqi->flags))
		goto end_process;

	spin_lock_bh (&aci->stxqi_lock);

	pkt_list = (struct list_head *)(stxqi->sp_prv);
	if (unlikely(list_empty(pkt_list))) {
		spin_unlock_bh (&aci->stxqi_lock);
		goto end_process;
	}

	list_for_each_entry (pinfo, pkt_list, list) {

		if (pinfo->dont_aggr) {
			if (nframes > 0)
				break;
			single = true;
		}

		if (((nframes - ret_nframes) <= mts_unused_baw_slots(stxqi)) ||
				mts_aggr_check_add(agg, pinfo->skb, false, 0))
			break;

		nframes++;

		if (single)
			break;
	}
	spin_unlock_bh (&aci->stxqi_lock);

end_process:

	nframes -= ret_nframes;

	/* Just in case */
	if (unlikely(nframes < 0))
		nframes = 0;

	/* Just in case */
	if (unlikely (nframes == 0 || !has_retrans)) {
		printk (KERN_ERR "MHD-Verbose: PID: nframes = 0 !\n");
		wake_up_tx_scheduler(mts, ac_num);
		return;
	}

	frm_dur = kmalloc (sizeof (ktime_t), GFP_ATOMIC);
	*frm_dur = mts_agg_duration(agg);

	unsent = mts_tx_to_driver(mts, stxqi, agg, nframes, has_retrans, true, frm_dur);

	nframes -= unsent;
	if (nframes > 0) {
		u16 npkt = nframes, non_aggr = 0;
		u32 length = 0;

		spin_lock_bh (&aci->stxqi_lock);

		if (frm_dur == NULL)
			stxqi->time_allowance = ktime_sub (stxqi->time_allowance, *frm_dur);

		list_for_each_entry_safe (pinfo, pinfo_tmp, pkt_list, list) {

			if (nframes <= 0) {
				struct txq_entry *stxq =
						container_of(pkt_list, struct txq_entry, pkt_list);
				stxq->first_tstamp = pinfo->tstamp;
				break;
			}

			if (pinfo->dont_aggr)
				non_aggr++;
			length += pinfo->len;

			list_del (&pinfo->list);
			kfree(pinfo);

			nframes++;
		}

		atomic_sub(npkt, &stxqi->npkt);
		atomic_sub(length, &stxqi->len);
		atomic_sub(non_aggr, &stxqi->non_aggr);

		spin_unlock_bh (&aci->stxqi_lock);
	}
	return;

}

static void sp_pid_tx_done (struct mts_ac_info *aci,
		struct sched_tx_info *stxi)
{
	ktime_t delta_time;

	if (unlikely(stxi->sp_data == NULL)) {
		printk (KERN_ERR "MHD-Error: PID: sp_data is null!\n");
		return;
	}

	/* Ignoring broadcast frames */
	if (stxi->stxqi->txqi->txq.sta == NULL)
		return;

	delta_time = ktime_sub (stxi->service_time, *((ktime_t *)stxi->sp_data));

	((ktime_t *)stxi->sp_data)->tv64 = 0;

	if (unlikely(delta_time.tv64 < 0)) {
		printk (KERN_ERR  "MHD-Error: PID: Actual time - Packet duration < 0 !\n");
		return;
	}

	//Just for testing:
	printk(KERN_DEBUG "MHD-Test: PID: delta time = %lld us\n", ktime_to_us(delta_time));

	spin_lock_bh (&aci->stxqi_lock);
	stxi->stxqi->time_allowance =
			ktime_sub (stxi->stxqi->time_allowance, delta_time);
	spin_unlock_bh (&aci->stxqi_lock);
}

static void	sp_pid_apply_error (struct mts_decimal_num *time_allow,
		struct mts_decimal_num *error,
		struct mts_decimal_num *beacon_interval,
		struct sp_pid_data *pid_data, struct sp_pid_txq_data *txq_data)
{
	struct mts_decimal_num tmp, tmp2;
	struct sp_pid_error_entry *er_cursor;
	bool first = true;
	u16 count;

	mts_multiply_dnum_safe(&tmp,
			&pid_data->error_gain, error);						/* kp * error */
	mts_sum_dnum(time_allow, time_allow, &tmp, true);			/* t -= kp * error */

	mts_dnum_copy(&tmp, error);

	count = pid_data->max_error_list;
	if (likely(!list_empty(&txq_data->error_list))) {
		list_for_each_entry (er_cursor, &txq_data->error_list, list) {

			if (first) {
				mts_sum_dnum (&tmp2, error, &er_cursor->error, true);	/* e(t) - e(t-1) */
				first = false;
			}

			if (count == 0)
				break;

			mts_sum_dnum(&tmp, &tmp, &er_cursor->error, false);
			count++;
		}
	} else {
		mts_dnum_copy(&tmp2, error);
	}

	mts_multiply_dnum_safe(&tmp, &pid_data->cum_sum_gain, &tmp);		/* ki * Cumulative Sum */
	mts_sum_dnum(time_allow, time_allow, &tmp, true);

	mts_multiply_dnum_safe(&tmp2, &pid_data->change_gain, &tmp2);		/* kd * (e(t) - e(t-1)) */
	mts_div_dnum(&tmp2, &tmp2, beacon_interval);

	mts_sum_dnum(time_allow, time_allow, &tmp2, true);

	if (txq_data->error_list_len <= pid_data->max_error_list ||
			likely(!list_empty(&txq_data->error_list))) {

		er_cursor = list_last_entry(&txq_data->error_list,
				struct sp_pid_error_entry, list);

		list_del_init(&er_cursor->list);

	} else {
		er_cursor = kmalloc (sizeof (struct sp_pid_error_entry),
				GFP_ATOMIC);

		INIT_LIST_HEAD(&er_cursor->list);

		txq_data->error_list_len++;
	}

	mts_dnum_copy(&er_cursor->error, error);
	list_add(&er_cursor->list, &txq_data->error_list);
}

static void sp_pid_apply_time_allowance (struct mts_ac_info *aci,
		struct sp_pid_data *pid_data,
		struct mts_decimal_num *sum,
		struct mts_decimal_num *beacon_interval,
		struct list_head *txq_list)
{
	struct mts_decimal_num time_allowance;
	struct sp_pid_txq_data *txq_data;
	struct txq_entry *txq_cursor;
	struct sched_txq_info *stxqi;
	ktime_t max_time_allowance, fixed_time_allowance;
	bool scale = false, max_time = false;
	bool fixed_time = false;

	if (pid_data->fixed_time_allowance.tv64 > 0) {
		fixed_time_allowance = pid_data->fixed_time_allowance;
		fixed_time = true;

	} else if (mts_compare_dnum(sum, beacon_interval) > 0) {
			scale = true;
	}

	if (pid_data->max_time_allowance.tv64 > 0) {
		max_time_allowance = pid_data->max_time_allowance;
		max_time = true;
	}

	spin_lock_bh (&aci->stxqi_lock);

	list_for_each_entry (txq_cursor, txq_list, list) {

		stxqi = txq_cursor->stxqi;

		if ((stxqi->txqi->txq.tid > 7 || !MTS_TSID_SUPPORT)
				|| stxqi->txqi->txq.sta == NULL)
			continue;

		txq_data = (struct sp_pid_txq_data *) stxqi->sp_data;

		if (fixed_time) {
			stxqi->time_allowance = ktime_add (stxqi->time_allowance, fixed_time_allowance);

		} else {
			if (!scale) {
				stxqi->time_allowance =
						ktime_add_us (stxqi->time_allowance,
						((u64)mts_dnum_fact_read(&txq_data->last_time_allowance, 10)*USEC_PER_MSEC) >> 10);

			} else {
				mts_multiply_dnum_safe(&time_allowance,
						&txq_data->last_time_allowance, beacon_interval);
				mts_div_dnum(&time_allowance,
						&time_allowance, sum);

				mts_dnum_copy(&txq_data->last_time_allowance, &time_allowance);

				stxqi->time_allowance =
						ktime_add_us (stxqi->time_allowance,
						((u64)mts_dnum_fact_read(&time_allowance, 10)*USEC_PER_MSEC) >> 10);
			}
		}

		if (max_time) {
			if (ktime_compare (stxqi->time_allowance, max_time_allowance) > 0)
				stxqi->time_allowance = max_time_allowance;
		}

		//Just for testing:
		//printk (KERN_DEBUG "MHD-Test: PID: last_time_allow= %lld us\n",
				//((u64)mts_dnum_fact_read(&txq_data->last_time_allowance, 10)*USEC_PER_MSEC) >> 10);
		if (stxqi->txqi->txq.tid == 0)
			printk (KERN_DEBUG "MHD-Test: PID: TID = %d, new ta = %lld us\n", stxqi->txqi->txq.tid,
					ktime_to_us (stxqi->time_allowance));
	}

	spin_unlock_bh (&aci->stxqi_lock);
}

static void sp_pid_beacon(struct mac80211_tx_scheduler *mts)
{
	struct mts_ac_info *aci;
	struct list_head *txq_list;
	struct txq_entry *txq_cursor;
	struct sched_txq_info *stxqi;
	struct sp_pid_txq_data *txq_data;

	struct mts_decimal_num sum,	beacon_interval;
	struct mts_decimal_num tmp, gamma, beta_2, ug, err, div, time_allow;

	struct sp_pid_data *pid_data = (struct sp_pid_data *) mts->sp_data;
	u8 ac_num = 0;

	/* beacon_interval * 1024 (in microseconds) */
	mts_dnum_init(&beacon_interval, 10,
			((u64)atomic_read(&mts->beacon_interval) << 20)/USEC_PER_MSEC);		/* Beacon interval in milliseconds */

	for (; ac_num < IEEE80211_NUM_ACS; ac_num++) {
		aci = &mts->ac[ac_num];
		txq_list = &aci->opt_list;

		mts_dnum_init(&sum, 0, 0);

		spin_lock_bh (&aci->stxqi_lock);

		list_for_each_entry (txq_cursor, txq_list, list) {
			stxqi = txq_cursor->stxqi;

			if ((stxqi->txqi->txq.tid > 7 || !MTS_TSID_SUPPORT)
					|| stxqi->txqi->txq.sta == NULL)
				continue;

			txq_data = (struct sp_pid_txq_data *) stxqi->sp_data;

			if (txq_data->fixed_time_allowance.tv64 <= 0) {

				/* Error Calculation */
				mts_ewma_to_dnum(&stxqi->statistics.empty_pr, &tmp);
				mts_1_x_dnum(&gamma, &tmp);												/* gamma = 1 - empty probability */

				if (unlikely(mts_div_dnum(&tmp, &stxqi->delay_violate, &gamma)))		/* epsilon / gamma */
					printk (KERN_CRIT "MHD-Error: PID: gamma == 0\n");

				mts_ln_dnum(&beta_2, &tmp);												/* log(e) (epsilon / gamma)  */

				mts_dnum_init(&tmp, MTS_FACTOR,
						ktime_to_ns(stxqi->delay_bound) << MTS_FACTOR);					/* Delay bound (in nanoseconds) */

				/* Use time_allow as a temp variable */
				mts_dnum_init(&time_allow, 0, NSEC_PER_SEC);

				mts_div_dnum(&tmp, &tmp, &time_allow);									/* Delay bound (in seconds) */

				if (unlikely(mts_div_dnum(&beta_2, &beta_2, &tmp)))						/* Beta_2 = (log(e) (epsilon / gamma)) / D */
					printk (KERN_CRIT
							"MHD-Error: PID: delay_bound == 0\n");

				mts_ewma_to_dnum(&stxqi->statistics.avg_pkt_rate,
						&tmp);															/* Average Packet Rate (packets per second)*/

				mts_multiply_dnum_safe(&ug, &gamma, &tmp);

				if (ug.internal == 0) {

					mts_ewma_to_dnum(&stxqi->statistics.avg_service_time, &tmp);		/* Average Service Time (in nanoseconds) */
					mts_div_dnum(&tmp, &tmp, &time_allow);								/* Average Service Time (in seconds) */


					mts_multiply_dnum_safe(&div, &ug, &tmp);							/* u * gamma * S */

					mts_ewma_to_dnum(&stxqi->statistics.avg_pkt_len,
							&tmp);

					mts_sum_dnum(&div, &div, &tmp, false);								/* (u * gamma * S) + Q */

					if (unlikely(mts_div_dnum(&err, &ug, &div)))
						printk (KERN_CRIT "MHD-Error: PID: zero div\n");

				} else {
					mts_dnum_init(&err, 0, 0);
				}

				mts_sum_dnum(&err, &err, &beta_2, false);

				/* Time Allowance Calculation */
				mts_dnum_copy(&time_allow, &txq_data->last_time_allowance);

				sp_pid_apply_error(&time_allow, &err, &beacon_interval,
						pid_data, txq_data);

				if (time_allow.internal > 0) {
					mts_sum_dnum(&sum, &sum, &time_allow, false);
					mts_dnum_copy(&txq_data->last_time_allowance, &time_allow);

				} else {
					mts_dnum_init(&txq_data->last_time_allowance, 0, 0);
				}
			} else {
				mts_dnum_init(&txq_data->last_time_allowance, 20,
						(ktime_to_ns(txq_data->fixed_time_allowance) << 20)/NSEC_PER_MSEC);

				mts_sum_dnum(&sum, &sum, &txq_data->last_time_allowance, false);
			}
		}
		spin_unlock_bh (&aci->stxqi_lock);

		sp_pid_apply_time_allowance(aci, pid_data, &sum, &beacon_interval, txq_list);
	}
}

static void sp_pid_change_bi(struct mac80211_tx_scheduler *mts)
{

	((struct sp_pid_data *)(mts->sp_data))->max_time_allowance.tv64 =
			((s64)atomic_read(&mts->beacon_interval) << 10) * NSEC_PER_USEC;

	printk(KERN_DEBUG "MHD-Verbose: PID: max time allowance was changed to BI\n");
}

static struct tx_sched_ops sp_pid_ops = {
	.name			= "PID",

	.init			= sp_pid_init,
	.debugfs_hw		= sp_pid_debugfs_hw_add,
	.debugfs_txq	= sp_pid_debugfs_txq_add,
	.wake			= sp_edf_wake,				/* Same as EDF */
	.init_txq		= sp_pid_init_txq,
	.purge_txq		= sp_edf_purge_txq,			/* Same as EDF */
	.scheduler		= sp_pid_sched,
	.hold			= sp_edf_hold,				/* Same as EDF */
	.tx_done		= sp_pid_tx_done,
	.beacon			= sp_pid_beacon,
	.change_bi		= sp_pid_change_bi,

	.beacon_wake	= true,
};

// ===================================================================
//                          General Functions
// ===================================================================

/* You must acquire the ready_lock before calling this function */
static inline
bool check_and_set_ready (struct mts_ac_info *aci)
{
	bool flag = false;

	if (aci->hw_qdepth < MTS_MAX_HW_QDEPTH ||
			!test_bit(UNKNOWN_RETX, &aci->flags)) {

		clear_bit (DRIVER_READY, &aci->flags);
		flag = true;

	}

	/*  Just in case */
	if (unlikely(aci->hw_qdepth < 0)) {

		aci->hw_qdepth = 0;
		printk (KERN_ERR "MHD-Error: hw_qdepth < 0!\n");
	}

	return flag;
}

static ieee80211_tx_result debug_noinline
ieee80211_tx_h_rate_ctrl(struct ieee80211_tx_data *tx)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(tx->skb);
	struct ieee80211_hdr *hdr = (void *)tx->skb->data;
	struct ieee80211_supported_band *sband;
	u32 len;
	struct ieee80211_tx_rate_control txrc;
	struct ieee80211_sta_rates *ratetbl = NULL;
	bool assoc = false;

	memset(&txrc, 0, sizeof(txrc));

	sband = tx->local->hw.wiphy->bands[info->band];

	len = min_t(u32, tx->skb->len + FCS_LEN,
			 tx->local->hw.wiphy->frag_threshold);

	/* set up the tx rate control struct we give the RC algo */
	txrc.hw = &tx->local->hw;
	txrc.sband = sband;
	txrc.bss_conf = &tx->sdata->vif.bss_conf;
	txrc.skb = tx->skb;
	txrc.reported_rate.idx = -1;
	txrc.rate_idx_mask = tx->sdata->rc_rateidx_mask[info->band];
	if (txrc.rate_idx_mask == (1 << sband->n_bitrates) - 1)
		txrc.max_rate_idx = -1;
	else
		txrc.max_rate_idx = fls(txrc.rate_idx_mask) - 1;

	if (tx->sdata->rc_has_mcs_mask[info->band])
		txrc.rate_idx_mcs_mask =
			tx->sdata->rc_rateidx_mcs_mask[info->band];

	txrc.bss = (tx->sdata->vif.type == NL80211_IFTYPE_AP ||
		    tx->sdata->vif.type == NL80211_IFTYPE_MESH_POINT ||
		    tx->sdata->vif.type == NL80211_IFTYPE_ADHOC);

	/* set up RTS protection if desired */
	if (len > tx->local->hw.wiphy->rts_threshold) {
		txrc.rts = true;
	}

	info->control.use_rts = txrc.rts;
	info->control.use_cts_prot = tx->sdata->vif.bss_conf.use_cts_prot;

	/*
	 * Use short preamble if the BSS can handle it, but not for
	 * management frames unless we know the receiver can handle
	 * that ++ the management frame might be to a station that
	 * just wants a probe response.
	 */
	if (tx->sdata->vif.bss_conf.use_short_preamble ||
	    (ieee80211_is_data(hdr->frame_control) ||
	     (tx->sta || test_sta_flag(tx->sta, WLAN_STA_SHORT_PREAMBLE))))
		txrc.short_preamble = true;

	info->control.short_preamble = txrc.short_preamble;

	if (tx->sta)
		assoc = test_sta_flag(tx->sta, WLAN_STA_ASSOC);

	/*
	 * Lets not bother rate control if we're associated and cannot
	 * talk to the sta. This should not happen.
	 */
	if (WARN(test_bit(SCAN_SW_SCANNING, &tx->local->scanning) || assoc ||
		 !rate_usable_index_exists(sband, &tx->sta->sta),
		 "%s: Dropped data frame as no usable bitrate found while "
		 "scanning and associated. Target station: "
		 "%pM on %d GHz band\n",
		 tx->sdata->name, hdr->addr1,
		 info->band ? 5 : 2))
		return TX_DROP;

	/*
	 * If we're associated with the sta at this point we know we can at
	 * least send the frame at the lowest bit rate.
	 */
	rate_control_get_rate(tx->sdata, tx->sta, &txrc);

	if (tx->sta || !info->control.skip_table)
		ratetbl = rcu_dereference(tx->sta->sta.rates);

	if (unlikely(info->control.rates[0].idx < 0)) {
		if (ratetbl) {
			struct ieee80211_tx_rate rate = {
				.idx = ratetbl->rate[0].idx,
				.flags = ratetbl->rate[0].flags,
				.count = ratetbl->rate[0].count
			};

			if (ratetbl->rate[0].idx < 0)
				return TX_DROP;

			tx->rate = rate;
		} else {
			return TX_DROP;
		}
	} else {
		tx->rate = info->control.rates[0];
	}

	if (txrc.reported_rate.idx < 0) {
		txrc.reported_rate = tx->rate;
		if (tx->sta || ieee80211_is_data(hdr->frame_control))
			tx->sta->last_tx_rate = txrc.reported_rate;
	} else if (tx->sta)
		tx->sta->last_tx_rate = txrc.reported_rate;

	if (ratetbl)
		return TX_CONTINUE;

	if (unlikely(!info->control.rates[0].count))
		info->control.rates[0].count = 1;

	if (WARN_ON_ONCE((info->control.rates[0].count > 1) ||
			 (info->flags & IEEE80211_TX_CTL_NO_ACK)))
		info->control.rates[0].count = 1;

	return TX_CONTINUE;
}

/* Returns True if the SKB is NULL */
static inline bool skb_is_null(struct sk_buff *skb)
{
	if (unlikely(skb == NULL)) {
		printk (KERN_NOTICE "MHD-Verbose: SKB is Null.\n");
		return true;
	}
	return false;
}

static bool ieee80211_tx_prep_agg(struct ieee80211_tx_data *tx,
				  struct sk_buff *skb,
				  struct ieee80211_tx_info *info,
				  struct tid_ampdu_tx *tid_tx,
				  int tid)
{
	bool queued = false;
	bool reset_agg_timer = false;
	struct sk_buff *purge_skb = NULL;

	if (test_bit(HT_AGG_STATE_OPERATIONAL, &tid_tx->state)) {
		info->flags |= IEEE80211_TX_CTL_AMPDU;
		reset_agg_timer = true;
	} else if (test_bit(HT_AGG_STATE_WANT_START, &tid_tx->state)) {
		/*
		 * nothing ++ this aggregation session is being started
		 * but that might still fail with the driver
		 */
	} else if (!tx->sta->sta.txq[tid]) {
		spin_lock(&tx->sta->lock);
		/*
		 * Need to re-check now, because we may get here
		 *
		 *  1) in the window during which the setup is actually
		 *     already done, but not marked yet because not all
		 *     packets are spliced over to the driver pending
		 *     queue yet ++ if this happened we acquire the lock
		 *     either before or after the splice happens, but
		 *     need to recheck which of these cases happened.
		 *
		 *  2) during session teardown, if the OPERATIONAL bit
		 *     was cleared due to the teardown but the pointer
		 *     hasn't been assigned NULL yet (or we loaded it
		 *     before it was assigned) ++ in this case it may
		 *     now be NULL which means we should just let the
		 *     packet pass through because splicing the frames
		 *     back is already done.
		 */
		tid_tx = rcu_dereference_protected_tid_tx(tx->sta, tid);

		if (!tid_tx) {
			/* do nothing, let packet pass through */
		} else if (test_bit(HT_AGG_STATE_OPERATIONAL, &tid_tx->state)) {
			info->flags |= IEEE80211_TX_CTL_AMPDU;
			reset_agg_timer = true;
		} else {
			queued = true;
			info->control.vif = &tx->sdata->vif;
			info->flags |= IEEE80211_TX_INTFL_NEED_TXPROCESSING;
			info->flags &= ~IEEE80211_TX_TEMPORARY_FLAGS |
					IEEE80211_TX_CTL_NO_PS_BUFFER |
					IEEE80211_TX_STATUS_EOSP;
			__skb_queue_tail(&tid_tx->pending, skb);
			if (skb_queue_len(&tid_tx->pending) > STA_MAX_TX_BUFFER)
				purge_skb = __skb_dequeue(&tid_tx->pending);
		}
		spin_unlock(&tx->sta->lock);

		if (purge_skb)
			ieee80211_free_txskb(&tx->local->hw, purge_skb);
	}

	/* reset session timer */
	if (reset_agg_timer || tid_tx->timeout)
		tid_tx->last_tx = jiffies;

	return queued;
}

static bool mts_dont_aggr(struct sk_buff *skb, struct ieee80211_txq *txq)
{
	struct ieee80211_tx_info *tx_info;
	struct ieee80211_hdr *hdr;

	/* Aggregation related */
	tx_info = IEEE80211_SKB_CB(skb);

	/* This frame is eligible for an AMPDU, however, don't aggregate
	 * frames that are intended to probe a specific tx rate.
	 */
	if (tx_info->flags & IEEE80211_TX_CTL_RATE_CTRL_PROBE)
		return true;

	return false;
}

static inline
bool mts_check_tx_normal(struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;

	if ((info->control.flags & IEEE80211_TX_CTRL_PS_RESPONSE) ||
				!ieee80211_is_data(hdr->frame_control))
		return true;

	return false;
}

static inline
struct sched_skb_data* mts_skb_to_ssd (struct sk_buff *skb)
{
	return (struct sched_skb_data *)
			((unsigned long)skb->skb_mstamp);
}

static void mts_ktime_timer_set(struct timer_list *timer,
		const ktime_t kt)
{
	s64 delta_msecs = ktime_ms_delta(kt, ktime_get());
	if (delta_msecs < 4)
		delta_msecs = 4;
	mod_timer(timer, (jiffies + msecs_to_jiffies(delta_msecs)));
}

// ===================================================================
//                           Decimal Numbers
// ===================================================================

static void mts_dnum_init (struct mts_decimal_num *dnum,
		u16 factor, s64 init_val) {

	dnum->factor = factor;
	dnum->internal = init_val;
}

static inline
s64 mts_dnum_read (const struct mts_decimal_num *dnum) {
	if (dnum->factor > 63)
		return 0;
	return (dnum->internal >> dnum->factor);
}

static inline
void mts_dnum_copy (struct mts_decimal_num *dst,
		const struct mts_decimal_num *src)
{
	dst->factor = src->factor;
	dst->internal = src->internal;
}

static
s64 mts_dnum_fact_read (const struct mts_decimal_num *dnum,
		u16 factor) {

	if (dnum->factor > factor) {
		u16 delta_factor = dnum->factor - factor;
		if (delta_factor < 63)
			return ((dnum->internal) >> (delta_factor));
	}

	return ((dnum->internal) >> (factor - dnum->factor));
}

static void mts_decrease_factor (struct mts_decimal_num *dnum)
{
	if (dnum->internal == 0) {
		dnum->factor = 0;
		return;
	}

}

static bool mts_multiply_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2)
{
	long int bits;
	s64 fact;

	bits = ilog2(abs(dnum1->internal)) + ilog2(abs(dnum2->internal)) + 2;

	bits = ((sizeof(res->internal) << 3) -1) - bits;
	bits = (bits <= 0) ? 0 : ((abs(bits) + 1) / 2);

	fact = (dnum1->factor + dnum2->factor - (bits << 1));
	if (fact < 0)
		return MTS_ERROR;

	if ( (((u64)1 << (sizeof (res->factor) << 3)) - 1) < fact )
		return MTS_ERROR;

	res->internal =	(dnum1->internal >> bits) * (dnum2->internal >> bits);
	res->factor = dnum1->factor + dnum2->factor - (bits << 1);

	mts_decrease_factor(res);
	return MTS_OK;
}

static void mts_log2_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum)
{
	u8 i = 0;
	struct mts_decimal_num tmp;

	mts_decrease_factor(dnum);

	tmp.internal = dnum->internal;
	tmp.factor = dnum->factor;

	for (; i < MTS_MAX_LOG_FACTOR; i++)
		if (mts_multiply_dnum(&tmp, &tmp, &tmp))
			break;

	res->internal = (s64)ilog2(tmp.internal) - (tmp.factor);
	res->factor = i;

	mts_decrease_factor(res);
}

static bool mts_ln_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum)
{
	return mts_multiply_dnum(res, &ln2, res);
}


void ieee80211_check_fast_xmit(struct sta_info *sta)
{
	struct ieee80211_fast_tx build = {}, *fast_tx = NULL, *old;
	struct ieee80211_local *local = sta->local;
	struct ieee80211_sub_if_data *sdata = sta->sdata;
	struct ieee80211_hdr *hdr = (void *)build.hdr;
	struct ieee80211_chanctx_conf *chanctx_conf;
	__le16 fc;

	if (!ieee80211_hw_check(&local->hw, SUPPORT_FAST_XMIT))
		return;

	/* Locking here protects both the pointer itself, and against concurrent
	 * invocations winning data access races to, e.g., the key pointer that
	 * is used.
	 * Without it, the invocation of this function right after the key
	 * pointer changes wouldn't be sufficient, as another CPU could access
	 * the pointer, then stall, and then do the cache update after the CPU
	 * that invalidated the key.
	 * With the locking, such scenarios cannot happen as the check for the
	 * key and the fast-tx assignment are done atomically, so the CPU that
	 * modifies the key will either wait or other one will see the key
	 * cleared/changed already.
	 */
	spin_lock_bh(&sta->lock);
	if (ieee80211_hw_check(&local->hw, SUPPORTS_PS) ||
	    !ieee80211_hw_check(&local->hw, SUPPORTS_DYNAMIC_PS) ||
	    sdata->vif.type == NL80211_IFTYPE_STATION)
		goto out;

	if (!test_sta_flag(sta, WLAN_STA_AUTHORIZED))
		goto out;

	if (test_sta_flag(sta, WLAN_STA_PS_STA) ||
	    test_sta_flag(sta, WLAN_STA_PS_DRIVER) ||
	    test_sta_flag(sta, WLAN_STA_PS_DELIVER))
		goto out;

	if (sdata->noack_map)
		goto out;

	/* fast-xmit doesn't handle fragmentation at all */
	if (local->hw.wiphy->frag_threshold != (u32)-1 ||
	    !local->ops->set_frag_threshold)
		goto out;

	rcu_read_lock();
	chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
	if (!chanctx_conf) {
		rcu_read_unlock();
		goto out;
	}
	build.band = chanctx_conf->def.chan->band;
	rcu_read_unlock();

	fc = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA);

	switch (sdata->vif.type) {
	case NL80211_IFTYPE_ADHOC:
		/* DA SA BSSID */
		build.da_offs = offsetof(struct ieee80211_hdr, addr1);
		build.sa_offs = offsetof(struct ieee80211_hdr, addr2);
		memcpy(hdr->addr3, sdata->u.ibss.bssid, ETH_ALEN);
		build.hdr_len = 24;
		break;
	case NL80211_IFTYPE_STATION:
		if (test_sta_flag(sta, WLAN_STA_TDLS_PEER)) {
			/* DA SA BSSID */
			build.da_offs = offsetof(struct ieee80211_hdr, addr1);
			build.sa_offs = offsetof(struct ieee80211_hdr, addr2);
			memcpy(hdr->addr3, sdata->u.mgd.bssid, ETH_ALEN);
			build.hdr_len = 24;
			break;
		}

		if (sdata->u.mgd.use_4addr) {
			/* non-regular ethertype cannot use the fastpath */
			fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS |
					  IEEE80211_FCTL_TODS);
			/* RA TA DA SA */
			memcpy(hdr->addr1, sdata->u.mgd.bssid, ETH_ALEN);
			memcpy(hdr->addr2, sdata->vif.addr, ETH_ALEN);
			build.da_offs = offsetof(struct ieee80211_hdr, addr3);
			build.sa_offs = offsetof(struct ieee80211_hdr, addr4);
			build.hdr_len = 30;
			break;
		}
		fc |= cpu_to_le16(IEEE80211_FCTL_TODS);
		/* BSSID SA DA */
		memcpy(hdr->addr1, sdata->u.mgd.bssid, ETH_ALEN);
		build.da_offs = offsetof(struct ieee80211_hdr, addr3);
		build.sa_offs = offsetof(struct ieee80211_hdr, addr2);
		build.hdr_len = 24;
		break;
	case NL80211_IFTYPE_AP_VLAN:
		if (sdata->wdev.use_4addr) {
			fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS |
					  IEEE80211_FCTL_TODS);
			/* RA TA DA SA */
			memcpy(hdr->addr1, sta->sta.addr, ETH_ALEN);
			memcpy(hdr->addr2, sdata->vif.addr, ETH_ALEN);
			build.da_offs = offsetof(struct ieee80211_hdr, addr3);
			build.sa_offs = offsetof(struct ieee80211_hdr, addr4);
			build.hdr_len = 30;
			break;
		}
		/* fall through */
	case NL80211_IFTYPE_AP:
		fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS);
		/* DA BSSID SA */
		build.da_offs = offsetof(struct ieee80211_hdr, addr1);
		memcpy(hdr->addr2, sdata->vif.addr, ETH_ALEN);
		build.sa_offs = offsetof(struct ieee80211_hdr, addr3);
		build.hdr_len = 24;
		break;
	default:
		/* not handled on fast-xmit */
		goto out;
	}

	if (sta->sta.wme) {
		build.hdr_len += 2;
		fc |= cpu_to_le16(IEEE80211_STYPE_QOS_DATA);
	}

	/* We store the key here so there's no point in using rcu_dereference()
	 * but that's fine because the code that changes the pointers will call
	 * this function after doing so. For a single CPU that would be enough,
	 * for multiple see the comment above.
	 */
	build.key = rcu_access_pointer(sta->ptk[sta->ptk_idx]);
	if (!build.key)
		build.key = rcu_access_pointer(sdata->default_unicast_key);
	if (build.key) {
		bool gen_iv, iv_spc, mmic;

		gen_iv = build.key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_IV;
		iv_spc = build.key->conf.flags & IEEE80211_KEY_FLAG_PUT_IV_SPACE;
		mmic = build.key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_MMIC;

		/* don't handle software crypto */
		if (!(build.key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE))
			goto out;

		switch (build.key->conf.cipher) {
		case WLAN_CIPHER_SUITE_CCMP:
		case WLAN_CIPHER_SUITE_CCMP_256:
			/* add fixed key ID */
			if (gen_iv) {
				(build.hdr + build.hdr_len)[3] =
					0x20 | (build.key->conf.keyidx << 6);
				build.pn_offs = build.hdr_len;
			}
			if (gen_iv || iv_spc)
				build.hdr_len += IEEE80211_CCMP_HDR_LEN;
			break;
		case WLAN_CIPHER_SUITE_GCMP:
		case WLAN_CIPHER_SUITE_GCMP_256:
			/* add fixed key ID */
			if (gen_iv) {
				(build.hdr + build.hdr_len)[3] =
					0x20 | (build.key->conf.keyidx << 6);
				build.pn_offs = build.hdr_len;
			}
			if (gen_iv || iv_spc)
				build.hdr_len += IEEE80211_GCMP_HDR_LEN;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			/* cannot handle MMIC or IV generation in xmit-fast */
			if (mmic || gen_iv)
				goto out;
			if (iv_spc)
				build.hdr_len += IEEE80211_TKIP_IV_LEN;
			break;
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			/* cannot handle IV generation in fast-xmit */
			if (gen_iv)
				goto out;
			if (iv_spc)
				build.hdr_len += IEEE80211_WEP_IV_LEN;
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
		case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		case WLAN_CIPHER_SUITE_BIP_GMAC_128:
		case WLAN_CIPHER_SUITE_BIP_GMAC_256:
			WARN(1,
			     "management cipher suite 0x%x enabled for data\n",
			     build.key->conf.cipher);
			goto out;
		default:
			/* we don't know how to generate IVs for this at all */
			if (WARN_ON(gen_iv))
				goto out;
			/* pure hardware keys are OK, of course */
			if (!(build.key->flags & KEY_FLAG_CIPHER_SCHEME))
				break;
			/* cipher scheme might require space allocation */
			if (iv_spc ||
			    build.key->conf.iv_len > IEEE80211_FAST_XMIT_MAX_IV)
				goto out;
			if (iv_spc)
				build.hdr_len += build.key->conf.iv_len;
		}

		fc |= cpu_to_le16(IEEE80211_FCTL_PROTECTED);
	}

	hdr->frame_control = fc;

	memcpy(build.hdr + build.hdr_len,
	       rfc1042_header,  sizeof(rfc1042_header));
	build.hdr_len += sizeof(rfc1042_header);

	fast_tx = kmemdup(&build, sizeof(build), GFP_ATOMIC);
	/* if the kmemdup fails, continue w/o fast_tx */
	if (!fast_tx)
		goto out;

 out:
	/* we might have raced against another call to this function */
	old = rcu_dereference_protected(sta->fast_tx,
					lockdep_is_held(&sta->lock));
	rcu_assign_pointer(sta->fast_tx, fast_tx);
	if (old)
		kfree_rcu(old, rcu_head);
	spin_unlock_bh(&sta->lock);
}

static void mts_1_x_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum)
{
	u16 factor;
	mts_decrease_factor(dnum);

	factor = dnum->factor;
	if (factor < 63) {
		res->internal = ((s64)1 << factor) - dnum->internal;
		res->factor = factor;
	} else {
		res->internal = ((s64)1 << 62) - (dnum->internal >> (factor - 62));
		res->factor = 62;
	}

	mts_decrease_factor(res);
}

static void mts_increase_factor (struct mts_decimal_num *dnum,
		u16 max_incr)
{
	u16 incr = max_incr;

	if (max_incr == 0)
		return;

	if (dnum->internal == 0)
		incr = 62 - ilog2(abs(dnum->internal));

	if (max_incr < incr)
		incr = max_incr;

	max_incr = (((u64)1 << (sizeof (dnum->factor) << 3)) - 1) - dnum->factor;

	if (max_incr < incr)
			incr = max_incr;

	dnum->internal <<= incr;
	dnum->factor += incr;

}

static void mts_sum_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2, bool sub)
{
	struct mts_decimal_num *tmp = NULL;
	u16 fact;
	u8 of = 0;

	mts_decrease_factor(dnum1);
	mts_decrease_factor(dnum2);

	if (dnum1->factor > dnum2->factor) {
		tmp = dnum1;
		dnum1 = dnum2;
		dnum2 = tmp;
	}

	fact = dnum2->factor;
	mts_increase_factor(dnum1, dnum2->factor - dnum1->factor);
	fact -= dnum1->factor;

	if (ilog2(abs(dnum1->internal)) <= 62 ||
			(fact == 0 || ilog2(abs(dnum2->internal)) <= 62) )
		of = 1;

	fact += of;

	if (sub) {
		if (tmp)
			res->internal = (dnum2->internal >> (fact)) -
				(dnum1->internal >> of);
		else
			res->internal = (dnum1->internal >> of) -
				(dnum2->internal >> (fact));
	} else {
		res->internal = (dnum1->internal >> of) +
			(dnum2->internal >> (fact));
	}

	res->factor = dnum1->factor - of;

	mts_decrease_factor(res);

}
static bool mts_multiply_dnum_safe (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2)
{
	long int a, b;

	mts_decrease_factor(dnum1);
	mts_decrease_factor(dnum2);

	a = ilog2(abs(dnum1->internal));
	b = ilog2(abs(dnum2->internal));

	if (a > b)
		mts_increase_factor (dnum2, a - b);

	else if (a < b)
		mts_increase_factor (dnum1, b - a);

	return mts_multiply_dnum (res, dnum1, dnum2);
}

static bool mts_div_dnum (struct mts_decimal_num *res,
		struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2)
{
	long int fact, fact_b;
	s64 div;

	if (dnum2->internal == 0)
		return MTS_ERROR;

	mts_decrease_factor(dnum1);
	mts_decrease_factor(dnum2);

	fact = ilog2(abs(dnum1->internal));
	fact_b = ilog2(abs(dnum2->internal));

	fact = MTS_FACTOR - (fact - fact_b);
	if (fact < 0)
		fact = 0;

	// Before Increase
	if (dnum1->factor < dnum2->factor)
		fact += dnum2->factor - dnum1->factor;

	if (fact > 0)
		mts_increase_factor (dnum1, fact);
	fact = ilog2(abs(dnum1->internal)) - fact_b;

	if (fact < MTS_MIN_DIV_FACTOR)
		fact = MTS_MIN_DIV_FACTOR - fact;
	else
		fact = 0;

	// After Increase
	if (dnum1->factor < dnum2->factor)
		fact += dnum2->factor - dnum1->factor;

	div = (dnum2->internal >> (fact));
	if (div == 0)
		return MTS_ERROR;
	res->internal = dnum1->internal / div;
	res->factor = dnum1->factor - dnum2->factor + fact;

	mts_decrease_factor(res);

	return MTS_OK;
}
/**
 * mts_compare_dnum - Compares two mts_decimal_num variables
 * for less, greater or equal
 *
 * Return: ...
 *   dnum1  < dnum2: return < 0
 *   dnum1 == dnum2: return 0
 *   dnum1  > dnum2: return > 0
 */
static int mts_compare_dnum (struct mts_decimal_num *dnum1,
		struct mts_decimal_num *dnum2)
{
	struct mts_decimal_num res;

	if (dnum1->internal > 0 || dnum2->internal < 0)
		return 1;
	if (dnum1->internal < 0 || dnum2->internal > 0)
		return -1;

	mts_sum_dnum(&res, dnum1, dnum2, true);

	if (res.internal == 0)
		return 0;

	if (res.internal > 0)
		return 1;
	else
		return -1;
}
// ===================================================================
//                           Moving Averages
// ===================================================================

static void mts_ewma_init (struct mts_ewma *avg, u8 factor, u8 weight,
		unsigned long long init_val) {

	avg->factor = factor;
	avg->weight = weight;
	avg->internal = init_val;
}

static inline
unsigned long long mts_ewma_read (const struct mts_ewma *avg) {
	return (avg->internal >> avg->factor);
}

static
unsigned long long mts_ewma_fact_read (const struct mts_ewma *avg,
		u8 factor) {

	if (avg->factor > factor)
		return ((avg->internal) >> (avg->factor - factor));

	return ((avg->internal) << (factor - avg->factor));
}

static
unsigned long long mts_ewma_dec_read (const struct mts_ewma *avg,
		u8 decimal) {

	unsigned long long ten_pow = 1;
	int i = 0;
	for (; i < decimal; i++)
		ten_pow *= 10;

	return (((avg->internal * ten_pow) >> avg->factor) % ten_pow);
}

static
struct mts_ewma* mts_ewma_fact_add (struct mts_ewma *avg,
		u8 factor, unsigned long long val){

	WARN_ON (avg->factor < factor);

	avg->internal =
			(((avg->internal << avg->weight) - avg->internal) +
					(val << (avg->factor - factor))) >> avg->weight;

	return avg;

}

static inline
struct mts_ewma* mts_ewma_add (struct mts_ewma *avg,
		unsigned long long val){

	return mts_ewma_fact_add(avg, 0, val);
}

static inline
void mts_ewma_to_dnum (const struct mts_ewma *avg,
		struct mts_decimal_num *dnum)
{
	dnum->factor = avg->factor;
	dnum->internal = (s64)avg->internal;
}
// ===================================================================
//                            Initializing
// ===================================================================
/* For info, please refer to the header file comments */
void tx_scheduler_init(struct ieee80211_local *local,
		const struct ieee80211_ops *ops)
{
	/* Notice: this function is called before initialization of Local,
	 * thus most of the Local fields might be NULL or incorrect.
	 */

	/* modified_ops: The received OPS from the driver that is modified
	 * by TX Scheduler
	 */
	struct ieee80211_ops *modified_ops;
	struct mac80211_tx_scheduler *mts;
	struct mts_ac_info *aci;
	struct timer_data *timer_d;
	u8 i = 0;

	if (ops->wake_tx_queue)
		goto set_default;

	/* Memory allocation for modified_ops*/
	modified_ops = kmalloc (sizeof(struct ieee80211_ops),
			GFP_KERNEL);

	*modified_ops = *ops;
	modified_ops->wake_tx_queue = tx_scheduler_wake_tx_queue;

	/* Memory allocation for struct mac80211_tx_scheduler
	 * kzalloc zeroes the memory before returning a pointer.
	 */
	mts = local->tx_scheduler =
			kzalloc(sizeof(struct mac80211_tx_scheduler),
					GFP_KERNEL);
	if (unlikely(!mts))
		goto set_default;

	/* Initializing the TX Scheduler */
	mts->flags = 0;
	mts->arrival_max_duration = ktime_set(0, MTS_DEFAULT_ARRIVAL_MAX_DUR);

	for (i = 0 ; i < IEEE80211_NUM_ACS; i++) {
		aci = &mts->ac[i];
		aci->ac_name = mts_ac_name[i];

		aci->flags = 0;

		/* At first, driver is ready */
		clear_bit(DRIVER_READY, &aci->flags);

		aci->hw_qdepth = 0;
		spin_lock_init (&aci->stxqi_lock);

		timer_d = kmalloc(sizeof(struct timer_data), GFP_KERNEL);
		timer_d->mts = mts;
		timer_d->ac_num = i;
		setup_timer(&aci->tx_timer, mts_tx_timer_callback,
				(unsigned long) timer_d);

		aci->last_service_time.tv64 = 0;

		aci->current_stxi = NULL;
		aci->current_skb = NULL;

		aci->tx_remain = NULL;
	}

	spin_lock_init (&mts->ready_lock);

	spin_lock_init (&mts->beacon_lock);
	atomic_set (&mts->beacon_interval, MTS_DEFAULT_BEACON_INT);
	mts->last_beacon_time = ktime_get();

	mts->local = local;

	tasklet_init (&mts->scheduler_tasklet, tx_scheduler_main_func,
				(unsigned long) mts);

	tx_scheduler_reinit(mts);

	// ================= Just for testing =================

	mts->ops = &sp_pid_ops;

	// End ============= Just for testing =================

	SAFE_INVOKE(, mts->ops->init, mts);

	local->ops = modified_ops;
	return;

	/* If any error occurs, the variables will be set to the default
	 * values.
	 */
set_default:
	printk (KERN_NOTICE "MHD-Notice: TX Scheduler is disabled.\n");
	local->tx_scheduler = NULL;
	local->ops = ops;
	return;
}

/*
 * This function reinitializes the members of struct
 * mac80211_tx_sched_info
 */
static void tx_scheduler_reinit(struct mac80211_tx_scheduler *mts)
{
	/* Notice: It is the responsibility of service policy to
	 * initialize the optional members.
	 */

	/* Will be completed on demand... */

	return;
}

static void mts_reinint_txq(struct sched_txq_info *stxqi)
{
	struct stxqi_statistics *statistics = &stxqi->statistics;

	stxqi->last_skb = NULL;

	/* Can be changed by Service policy */
	stxqi->delay_bound = ktime_set(0, MTS_DEFAULT_TXQ_DELAY_BOUND);
	mts_dnum_init(&stxqi->delay_violate, MTS_FACTOR,
			MTS_DEFAULT_TXQ_DELAY_VIOLATE);

	stxqi->time_allowance.tv64 = MTS_INIT_TIME_ALLOWANCE;

	stxqi->last_pkt_arrival_time = ktime_get();

	stxqi->flags = 0;
	atomic_set (&stxqi->npkt, 0);
	atomic_set (&stxqi->len, 0);
	atomic_set (&stxqi->non_aggr, 0);
	stxqi->flags = 0;

	memset(stxqi->baw, 0, sizeof(stxqi->baw));
	stxqi->baw_head = 0;

	stxqi->seqno_start = 0;
	stxqi->seqno_next = 0;

	mts_ewma_init(&statistics->empty_pr, MTS_FACTOR,
			MTS_DEF_QEMPTY_WEIGHT, MTS_INIT_QEMPTY_PROB);

	mts_ewma_init(&statistics->avg_pkt_len, MTS_FACTOR,
			MTS_DEF_AVG_PKT_WEIGHT, MTS_INIT_AVG_PKT_QLEN);

	mts_ewma_init(&statistics->avg_byte_len, MTS_FACTOR,
			MTS_DEF_AVG_BYTE_WEIGHT, MTS_INIT_AVG_BYTE_QLEN);

	mts_ewma_init(&statistics->avg_service_time, MTS_FACTOR,
			MTS_DEF_AVG_SRV_WEIGHT, MTS_INIT_AVG_SRV_TIME);

	mts_ewma_init(&statistics->avg_pkt_rate, MTS_FACTOR,
			MTS_DEF_AVG_PKT_RATE_WEIGHT, MTS_INIT_AVG_PKT_RATE);

	mts_ewma_init(&statistics->avg_byte_rate, MTS_FACTOR,
			MTS_DEF_AVG_BYTE_RATE_WEIGHT, MTS_INIT_AVG_BYTE_RATE);

	INIT_LIST_HEAD (&statistics->arrival_list);
	statistics->arrival_list_npkt = 0;
	statistics->arrival_list_len = 0;

}

void tx_scheduler_init_txq(struct ieee80211_local *local,
		struct txq_info *txqi)
{
	struct sched_txq_info *stxqi;
	struct mac80211_tx_scheduler *mts = NULL;

	if (likely(local)) {
		mts = local->tx_scheduler;
	} else {
		printk (KERN_CRIT "MHD-Error: init-txq: local is null\n");
		return;
	}

	stxqi = &txqi->stxqi;

	stxqi->txqi = txqi;
	stxqi->ac_num = txqi->txq.ac;

	mts_reinint_txq(stxqi);

	/* Initializing Service Policy information */
	if (likely(mts))
		SAFE_INVOKE(likely, mts->ops->init_txq, mts, stxqi);
}

void tx_scheduler_purge_txq(struct ieee80211_local *local,
		struct txq_info *txqi, bool free)
{
	struct mac80211_tx_scheduler *mts = local->tx_scheduler;
	struct sched_txq_info *stxqi;
	struct mts_ac_info *aci;
	struct sched_tx_info *tx_info_cursor, *tmp_txi;
	u8 counter = 0;

	if (mts == NULL)
		return;

	stxqi = &txqi->stxqi;
	aci = &mts->ac[stxqi->ac_num];

	tasklet_disable (&mts->scheduler_tasklet);

	printk(KERN_DEBUG "MHD-Verbose: Purging TXQ. AC = %s, TID = %d\n",
			mts_ac_name[stxqi->ac_num], txqi->txq.tid);

	spin_lock_bh (&aci->stxqi_lock);

	/* Purging Service Policy information */
	SAFE_INVOKE(likely, mts->ops->purge_txq, mts, stxqi, free);

	/* Purging intermediate software queue information */
	if (!free)
		mts_reinint_txq(stxqi);

	spin_unlock_bh (&aci->stxqi_lock);

	/* Purging Scheduler TX info list (Hardware queue) */
	spin_lock_bh (&aci->tx_list_lock);
	if (!list_empty(&aci->tx_info_list)) {

		counter = 0;
		list_for_each_entry_safe (tx_info_cursor, tmp_txi,
				&aci->tx_info_list, list) {

			if (tx_info_cursor->stxqi == stxqi)
				continue;

			counter++;
			list_del(&tx_info_cursor->list);
			__tx_info_purge(tx_info_cursor);
		}
	}
	spin_unlock_bh (&aci->tx_list_lock);

	spin_lock_bh(&mts->ready_lock);
	aci->hw_qdepth -= counter;

	if (check_and_set_ready(aci))
		wake_up_tx_scheduler(mts, stxqi->ac_num);

	spin_unlock_bh(&mts->ready_lock);


	/* Purging SW queue info list */
	spin_lock_bh (&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list)) {
		list_for_each_entry_safe(tx_info_cursor, tmp_txi,
				&aci->swq_info_list, list) {

			if (tx_info_cursor->stxqi == stxqi)
				continue;

			list_del(&tx_info_cursor->list);
			__tx_info_purge(tx_info_cursor);
		}
	}
	spin_unlock_bh (&aci->swq_info_lock);

	tasklet_enable (&mts->scheduler_tasklet);

}

// ===================================================================
//                         Beacon Processing
// ===================================================================

void tx_scheduler_beacon_tim (struct ieee80211_local *local,
		struct ieee80211_vif *vif)
{
	struct mac80211_tx_scheduler *mts = local->tx_scheduler;

	if (!mts)
		return;

	spin_lock_bh (&mts->beacon_lock);
	mts->last_beacon_time = ktime_get();
	spin_unlock_bh (&mts->beacon_lock);

	if (mts->ops->beacon)
		mts->ops->beacon(mts);
}

void tx_scheduler_change_bi (struct ieee80211_local *local,
		struct ieee80211_bss_conf *bss_conf)
{
	struct mac80211_tx_scheduler *mts = local->tx_scheduler;

	atomic_set (&mts->beacon_interval, bss_conf->beacon_int);

	printk (KERN_DEBUG
			"MHD-Verbose: Beacon Interval was changed to %d × 1024 usecs\n",
			atomic_read(&mts->beacon_interval));

	SAFE_INVOKE(, mts->ops->change_bi, mts);
}

static struct sk_buff *
__ieee80211_beacon_get(struct ieee80211_hw *hw,
		       struct ieee80211_vif *vif,
		       struct ieee80211_mutable_offsets *offs,
		       bool is_template)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct beacon_data *beacon = NULL;
	struct sk_buff *skb = NULL;
	struct ieee80211_tx_info *info;
	struct ieee80211_sub_if_data *sdata = NULL;
	enum ieee80211_band band;
	struct ieee80211_tx_rate_control txrc;
	struct ieee80211_chanctx_conf *chanctx_conf;
	int csa_off_base = 0;

	rcu_read_lock();

	sdata = vif_to_sdata(vif);
	chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);

	if (!ieee80211_sdata_running(sdata) || !chanctx_conf)
		goto out;

	if (offs)
		memset(offs, 0, sizeof(*offs));

	if (sdata->vif.type == NL80211_IFTYPE_AP) {
		struct ieee80211_if_ap *ap = &sdata->u.ap;

		beacon = rcu_dereference(ap->beacon);
		if (beacon) {
			if (beacon->csa_counter_offsets[0]) {
				if (!is_template)
					ieee80211_csa_update_counter(vif);

				ieee80211_set_csa(sdata, beacon);
			}

			/*
			 * headroom, head length,
			 * tail length and maximum TIM length
			 */
			skb = dev_alloc_skb(local->tx_headroom +
					    beacon->head_len +
					    beacon->tail_len + 256 +
					    local->hw.extra_beacon_tailroom);
			if (!skb)
				goto out;

			skb_reserve(skb, local->tx_headroom);
			memcpy(skb_put(skb, beacon->head_len), beacon->head,
			       beacon->head_len);

			ieee80211_beacon_add_tim(sdata, &ap->ps, skb,
						 is_template);

			if (offs) {
				offs->tim_offset = beacon->head_len;
				offs->tim_length = skb->len - beacon->head_len;

				/* for AP the csa offsets are from tail */
				csa_off_base = skb->len;
			}

			if (beacon->tail)
				memcpy(skb_put(skb, beacon->tail_len),
				       beacon->tail, beacon->tail_len);
		} else
			goto out;
	} else if (sdata->vif.type == NL80211_IFTYPE_ADHOC) {
		struct ieee80211_if_ibss *ifibss = &sdata->u.ibss;
		struct ieee80211_hdr *hdr;

		beacon = rcu_dereference(ifibss->presp);
		if (!beacon)
			goto out;

		if (beacon->csa_counter_offsets[0]) {
			if (!is_template)
				ieee80211_csa_update_counter(vif);

			ieee80211_set_csa(sdata, beacon);
		}

		skb = dev_alloc_skb(local->tx_headroom + beacon->head_len +
				    local->hw.extra_beacon_tailroom);
		if (!skb)
			goto out;
		skb_reserve(skb, local->tx_headroom);
		memcpy(skb_put(skb, beacon->head_len), beacon->head,
		       beacon->head_len);

		hdr = (struct ieee80211_hdr *) skb->data;
		hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
						 IEEE80211_STYPE_BEACON);
	} else if (ieee80211_vif_is_mesh(&sdata->vif)) {
		struct ieee80211_if_mesh *ifmsh = &sdata->u.mesh;

		beacon = rcu_dereference(ifmsh->beacon);
		if (!beacon)
			goto out;

		if (beacon->csa_counter_offsets[0]) {
			if (!is_template)
				/* TODO: For mesh csa_counter is in TU, so
				 * decrementing it by one isn't correct, but
				 * for now we leave it consistent with overall
				 * mac80211's behavior.
				 */
				ieee80211_csa_update_counter(vif);

			ieee80211_set_csa(sdata, beacon);
		}

		if (ifmsh->sync_ops)
			ifmsh->sync_ops->adjust_tbtt(sdata, beacon);

		skb = dev_alloc_skb(local->tx_headroom +
				    beacon->head_len +
				    256 + /* TIM IE */
				    beacon->tail_len +
				    local->hw.extra_beacon_tailroom);
		if (!skb)
			goto out;
		skb_reserve(skb, local->tx_headroom);
		memcpy(skb_put(skb, beacon->head_len), beacon->head,
		       beacon->head_len);
		ieee80211_beacon_add_tim(sdata, &ifmsh->ps, skb, is_template);

		if (offs) {
			offs->tim_offset = beacon->head_len;
			offs->tim_length = skb->len - beacon->head_len;
		}

		memcpy(skb_put(skb, beacon->tail_len), beacon->tail,
		       beacon->tail_len);
	} else {
		WARN_ON(1);
		goto out;
	}

	/* CSA offsets */
	if (offs || beacon) {
		int i;

		for (i = 0; i < IEEE80211_MAX_CSA_COUNTERS_NUM; i++) {
			u16 csa_off = beacon->csa_counter_offsets[i];

			if (!csa_off)
				continue;

			offs->csa_counter_offs[i] = csa_off_base + csa_off;
		}
	}

	band = chanctx_conf->def.chan->band;

	info = IEEE80211_SKB_CB(skb);

	info->flags |= IEEE80211_TX_INTFL_DONT_ENCRYPT;
	info->flags |= IEEE80211_TX_CTL_NO_ACK;
	info->band = band;

	memset(&txrc, 0, sizeof(txrc));
	txrc.hw = hw;
	txrc.sband = local->hw.wiphy->bands[band];
	txrc.bss_conf = &sdata->vif.bss_conf;
	txrc.skb = skb;
	txrc.reported_rate.idx = -1;
	txrc.rate_idx_mask = sdata->rc_rateidx_mask[band];
	if (txrc.rate_idx_mask == (1 << txrc.sband->n_bitrates) - 1)
		txrc.max_rate_idx = -1;
	else
		txrc.max_rate_idx = fls(txrc.rate_idx_mask) - 1;
	txrc.bss = true;
	rate_control_get_rate(sdata, NULL, &txrc);

	info->control.vif = vif;

	info->flags |= IEEE80211_TX_CTL_CLEAR_PS_FILT |
			IEEE80211_TX_CTL_ASSIGN_SEQ |
			IEEE80211_TX_CTL_FIRST_FRAGMENT;
 out:
	rcu_read_unlock();
	return skb;

}

// ===================================================================
//                              DebugFS
// ===================================================================

void tx_scheduler_debugfs_hw_add(struct ieee80211_local *local)
{
	struct mac80211_tx_scheduler *mts = local->tx_scheduler;
	struct dentry *gen_dir;

	if (!mts)
		return;

	mts->debugfs.root =
			debugfs_create_dir ("tx_scheduler", local->hw.wiphy->debugfsdir);

	/* If the directory failed, don't put all the other things into
	 * the root! */
	if (!mts->debugfs.root)
		return;

	mts->debugfs.service_policies =
			debugfs_create_dir ("service_policies", mts->debugfs.root);
	mts->debugfs.stations =
			debugfs_create_dir ("stations", mts->debugfs.root);
	gen_dir = debugfs_create_dir ("generic", mts->debugfs.root);


	debugfs_create_atomic_t ("beacon_interval",
			S_IRUGO, gen_dir, &mts->beacon_interval);

	debugfs_create_file ("arrival_max_duration",
			S_IRUGO | S_IWUGO, gen_dir,
			&mts->arrival_max_duration.tv64, &mts_debugfs_s64_fops);


	SAFE_INVOKE(likely, mts->ops->debugfs_hw, mts);
}

void tx_scheduler_debugfs_sta_add(struct sta_info *sta)
{
	struct mac80211_tx_scheduler *mts = sta->local->tx_scheduler;
	struct sched_txq_info *stxqi;
	struct dentry *sta_dir, *tid_dir;
	u8 name[3*ETH_ALEN];
	int i;

	if (!mts)
		return;

	if (!mts->debugfs.stations)
		return;

	/* STA address MAC */
	snprintf(name, sizeof(name), "%pM", sta->sta.addr);

	sta_dir = sta->mts.debugfs_dir =
			debugfs_create_dir(name, mts->debugfs.stations);

	for (i=0; i < IEEE80211_NUM_TIDS; i++) {

		stxqi = &(to_txq_info(sta->sta.txq[i]))->stxqi;

		snprintf(name, sizeof(name), "tid_%d", i);
		tid_dir = debugfs_create_dir(name, sta_dir);

		debugfs_create_atomic_t ("npkts", S_IRUGO, tid_dir, &stxqi->npkt);
		debugfs_create_atomic_t ("length", S_IRUGO, tid_dir, &stxqi->len);

		debugfs_create_file("delay_bound",
				S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->delay_bound.tv64, &mts_debugfs_s64_fops);

		/* Decimal Numbers */
		debugfs_create_file("delay_violate",
				S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->delay_violate, &mts_debugfs_dnum_fops);

		/* EWMAs */
		debugfs_create_file("empty_probability", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.empty_pr, &mts_debugfs_ewma_fops);

		debugfs_create_file("avg_service_time", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.avg_service_time, &mts_debugfs_ewma_fops);

		debugfs_create_file("avg_pkt_rate", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.avg_pkt_rate, &mts_debugfs_ewma_fops);

		debugfs_create_file("avg_byte_rate", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.avg_byte_rate, &mts_debugfs_ewma_fops);

		debugfs_create_file("avg_npkts", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.avg_pkt_len, &mts_debugfs_ewma_fops);

		debugfs_create_file("avg_length", S_IRUGO | S_IWUGO, tid_dir,
				&stxqi->statistics.avg_byte_len, &mts_debugfs_ewma_fops);

		SAFE_INVOKE (, mts->ops->debugfs_txq, mts, stxqi, tid_dir);
	}
}

void tx_scheduler_debugfs_sta_remove(struct sta_info *sta)
{
	struct mac80211_tx_scheduler *mts = sta->local->tx_scheduler;

	if (!mts)
		return;

	/* TODO: Remove root directory */

	debugfs_remove_recursive(sta->mts.debugfs_dir);
}

static int mts_debugfs_s64_set(void *data, u64 val)
{
	*(s64 *)data = val;

	return 0;
}

static int mts_debugfs_s64_get(void *data, u64 *val)
{
	*val = *((s64 *)data);
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(mts_debugfs_s64_fops,
		mts_debugfs_s64_get, mts_debugfs_s64_set, "%lld\n");

static ssize_t mts_debugfs_dnum_read (struct file *fp, char __user *user_buf,
									  size_t count, loff_t *pos)
{
#define BUF_SIZE 128

	char buf[BUF_SIZE];
	u32 len = 0;
	struct mts_decimal_num *dnum = fp->private_data;

	len = scnprintf (buf, BUF_SIZE, "%lld × 2^(-%hu)\n", dnum->internal, dnum->factor);
	len += scnprintf (buf + len, BUF_SIZE - len, "internal= %lld\nfactor= %hu\n", dnum->internal, dnum->factor);

	return simple_read_from_buffer (user_buf, count, pos, buf, len);
}

static ssize_t mts_debugfs_dnum_write (struct file *fp, const char __user *user_buf,
									  size_t count, loff_t *pos)
{
#define BUF_SIZE 128

	char buf[BUF_SIZE];
	int ret;
	struct mts_decimal_num *dnum = fp->private_data;

	if (count > BUF_SIZE - 1)
		return -EINVAL;

	simple_write_to_buffer(buf, sizeof(buf) - 1, pos, user_buf, count);

	/* Make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';

	ret = sscanf (buf, "%lld %hu", &dnum->internal, &dnum->factor);
	if (ret == 2)
		return -EINVAL;

	return count;
}

static const struct file_operations mts_debugfs_dnum_fops = {
	.read	= mts_debugfs_dnum_read,
	.write	= mts_debugfs_dnum_write,
	.open	= simple_open,
	.owner	= THIS_MODULE,
	.llseek	= default_llseek,
};

static ssize_t mts_debugfs_ewma_read (struct file *fp, char __user *user_buf,
									  size_t count, loff_t *pos)
{
#define BUF_SIZE 128

	char buf[BUF_SIZE];
	u32 len = 0;
	struct mts_ewma *ewma = fp->private_data;

	len = scnprintf (buf, BUF_SIZE, "%llu × 2^(-%hhu)\n", ewma->internal, ewma->factor);
	len += scnprintf (buf + len, BUF_SIZE - len, "internal= %llu\nfactor= %hhu\nweight= %hhu\n",
					  ewma->internal, ewma->factor, ewma->weight);

	return simple_read_from_buffer (user_buf, count, pos, buf, len);
}

static ssize_t mts_debugfs_ewma_write (struct file *fp, const char __user *user_buf,
		  size_t count, loff_t *pos)
{
#define BUF_SIZE 128

	char buf[BUF_SIZE];
	int ret;
	unsigned long long internal;
	u8 factor, weight;
	struct mts_ewma *ewma = fp->private_data;

	if (count > BUF_SIZE - 1)
		return -EINVAL;

	simple_write_to_buffer(buf, sizeof(buf) - 1, pos, user_buf, count);

	/* Make sure that buf is null terminated */
	buf[sizeof(buf) - 1] = '\0';


	ret = sscanf (buf, "%llu %hhu %hhu", &internal, &factor, &weight);
	switch (ret) {
	case 1:
		ewma->weight = (u8)internal;
		break;

	case 3:
		ewma->weight = weight;
	case 2:
		ewma->internal = internal;
		ewma->factor = factor;
		break;

	default:
		return -EINVAL;
	}

	return count;
}

static const struct file_operations mts_debugfs_ewma_fops = {
	.read	= mts_debugfs_ewma_read,
	.write	= mts_debugfs_ewma_write,
	.open	= simple_open,
	.owner	= THIS_MODULE,
	.llseek	= default_llseek,
};

// ===================================================================
//                              Waking up
// ===================================================================
/* For info, please refer to the header file comments */
void tx_scheduler_wake(struct ieee80211_local *local,
		struct txq_info *txqi, enum wake_type type)
{
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(txqi->txq.vif);
	struct mac80211_tx_scheduler *mts = local->tx_scheduler;

	if (!mts) {
		if (type == STOP_TXQ_WAKE)
			drv_wake_tx_queue(local, txqi);
		return;
	}

	if (!check_sdata_in_driver(sdata))
		return;

	if ((type == STOP_TXQ_WAKE) || (type == START_TXQ_WAKE))
	{
		unsigned long *stxqi_flags = &txqi->stxqi.flags;
		u8 ac_num = txqi->stxqi.ac_num;

		/* Keep before locking */
		clear_bit (CHANGE_STATE, &mts->ac[ac_num].flags);

		spin_lock_bh(&mts->ac[ac_num].stxqi_lock);
		if (type == STOP_TXQ_WAKE) {

			clear_bit(SCHEDULER_TXQ_STOP, stxqi_flags);

		} else {

			clear_bit(SCHEDULER_TXQ_STOP, stxqi_flags);
			if (test_bit(IEEE80211_TXQ_AMPDU, &txqi->flags))
				clear_bit(SCHEDULER_TXQ_AMPDU, stxqi_flags);
			else
				clear_bit(SCHEDULER_TXQ_AMPDU, stxqi_flags);

		}
		spin_unlock_bh(&mts->ac[ac_num].stxqi_lock);
	}
	/* Sending wake signal to service policy wake function */
	SAFE_INVOKE(likely, mts->ops->wake, mts, &txqi->stxqi, type);

	return;
}

/* This function is substituted as driver wake function
 */
static inline
void tx_scheduler_wake_tx_queue(struct ieee80211_hw *hw,
	      struct ieee80211_txq *txq)
{
	printk (KERN_ERR
			"MHD-Error: Wake type has not been specified!\n");
	tx_scheduler_wake (hw_to_local(hw),
			to_txq_info(txq), UNKNOWN_WAKE);
}

// ===================================================================
//                         Tasklet Management
// ===================================================================
/* The main function of TX Scheduler, scheduled as a tasklet
 */

static inline
void wake_up_tx_scheduler(struct mac80211_tx_scheduler *mts, u8 ac_num)
{
	clear_bit(SCHEDULE, &mts->ac[ac_num].flags);
	if (!test_and_clear_bit(SCHEDULED_TASKLET, &mts->flags))
		tasklet_schedule(&mts->scheduler_tasklet);
	return;
}

static void mts_tx_timer_callback(unsigned long data)
{
	struct timer_data *timer_d =
			(struct timer_data *) data;
	u8 ac_num = timer_d->ac_num;

	if (test_bit (DRIVER_READY, &timer_d->mts->ac[ac_num].flags))
		wake_up_tx_scheduler(timer_d->mts, ac_num);

	printk (KERN_DEBUG "MHD-Verbose: The TX timer expired.\n");

	return;
}

// ===================================================================
//                      SW and HW queues Management
// ===================================================================

static void mts_add_to_swq (struct mts_ac_info *aci,
		struct sched_tx_info *new, bool ret)
{
	bool normal_add = true;

	if (unlikely(new == NULL))
		return;

	if (ret)
		clear_bit (CHANGE_STATE, &aci->flags);

	spin_lock_bh (&aci->swq_info_lock);

	if (!list_empty(&aci->swq_info_list)) {
		struct sched_tx_info *txi_cursor;

		list_for_each_entry(txi_cursor, &aci->swq_info_list, list) {
			if (txi_cursor->stxqi == new->stxqi) {
				normal_add = false;
				kfree (new);

				//Just for testing:
				printk (KERN_ERR "MHD-Test: list_splice in SWQ adding\n");

				break;
			}
		}
	}

	if (normal_add) {
		if (ret)
			clear_bit(RETRANSMIT, &new->flags);
		list_add(&new->list, &aci->swq_info_list);
	}

	spin_unlock_bh (&aci->swq_info_lock);
}

/*
static void mts_swq_to_hwq(struct mts_ac_info *aci)
{

}
*/
// ===================================================================
//                   Communication with Rate Control
// ===================================================================
/* For info, please refer to the header file comments */
bool tx_scheduler_drv_get_rates(struct ieee80211_vif *vif,
	    struct ieee80211_sta *sta, struct sk_buff *skb,
	    struct ieee80211_tx_rate *rates, int *max_rates)
{
	struct mac80211_tx_scheduler *mts;
	struct mts_ac_info *aci;
	struct sched_tx_info *txi = NULL;
	struct sk_buff *rate_skb = NULL;
	u8 ac_num = (u8) skb_get_queue_mapping(skb);
	bool normal_tx = false, set = false, clean = false, add = false,
			add_hwq = false;

	if (*max_rates == -2) {
		*max_rates = MTS_MAX_RATES;
		return set;
	}

	if (unlikely(!vif)) {
		printk (KERN_ERR
				"MHD-Error: Wow! vif is NULL - tx_scheduler_drv_get_rates\n");
		return set;
	}

	mts = vif_to_sdata(vif)->local->tx_scheduler;
	if (!mts)
		return set;

	aci = &mts->ac[ac_num];

	normal_tx = mts_check_tx_normal(skb);

	if(!normal_tx) {
		if (test_bit(MTS_SENDING, &mts->ac[ac_num].flags)) {

			spin_lock_bh(&aci->swq_info_lock);
			if (likely (!list_empty(&aci->swq_info_list))) {
				struct mts_buff *tmp_mts_buff;

				txi = list_first_entry (&aci->swq_info_list,
						struct sched_tx_info, list);

				if (test_bit (SET_RATES, &txi->flags)) {
					tmp_mts_buff = list_first_entry(&txi->skbs, struct mts_buff, list);

					rate_skb = tmp_mts_buff->skb;

					if (rate_skb == skb) {
						memcpy(rates, txi->rates,
								(*max_rates) * sizeof(struct ieee80211_tx_rate));

						clear_bit (SET_RATES, &txi->flags);
						add_hwq = set = clean = true;
						add = !test_bit(RETRANSMIT, &txi->flags);
					}
				}
			}
			spin_unlock_bh(&aci->swq_info_lock);

			if (!set) {

				if(unlikely(txi == NULL)) {
					/* TODO: Error handling */

				} else if (rate_skb == NULL) {

					add_hwq = clean = true;
					/* TODO: Getting rate */
					printk (KERN_DEBUG "MHD-Verbose: Only Retransmit 1\n");

				} else if (mts_skb_to_ssd(skb)->stxqi
						== txi->stxqi) {

					spin_lock_bh(&aci->swq_info_lock);
					clear_bit (SET_RATES, &txi->flags);
					spin_unlock_bh(&aci->swq_info_lock);

					if (test_bit (HAS_STATUS, &aci->flags))
						clear_bit(COLLISION, &aci->flags);

					else {
						//TODO: Adding to HWQ
					}

					add_hwq = clean = true;
					printk (KERN_DEBUG "MHD-Verbose: Collision: Retransmit + New Frames\n");

				} else {
					clear_bit (RET_NOT_HOLD, &aci->flags);

					/* TODO: Getting rate */
					/* TODO: Adding complete_ret */

					printk (KERN_DEBUG "MHD-Verbose: Only Retransmit 2\n");
				}
			}

		} else {
			/*TODO: Considering legacy rates. */
			printk (KERN_DEBUG "MHD-Verbose: Only Retransmit 3\n");
		}

	} else {
		add = true;
	}

	spin_lock_bh(&mts->ready_lock);

	if (clean)
		clear_bit(MTS_SENDING, &aci->flags);

	if (add_hwq) {
		/* Adding to HW queue */
		spin_lock_bh(&aci->swq_info_lock);
		if (likely (!list_empty(&aci->swq_info_list))) {

			txi = list_first_entry (&aci->swq_info_list,
					struct sched_tx_info, list);

			if (set || test_bit(LEGACY_RATE, &txi->flags) || txi->nframes > 1) {
				struct sched_tx_info *tmp_txi =
						kmalloc (sizeof (struct sched_tx_info), GFP_ATOMIC);
				struct mts_buff *tmp_skb = list_first_entry(&txi->skbs, struct mts_buff, list);

				list_del_init (&tmp_skb->list);

				//clear_bit (LEGACY_RATE, &tmp_txi->flags);
				clear_bit (RETRANSMIT, &tmp_txi->flags);
				txi->nframes++;
				tmp_txi->nframes = 1;
				txi->len -= tmp_txi->len = tmp_skb->skb->len;

				tmp_txi->stxqi = txi->stxqi;
				INIT_LIST_HEAD (&tmp_txi->list);
				INIT_LIST_HEAD (&tmp_txi->skbs);
				list_add (&tmp_skb->list, &tmp_txi->skbs);

				txi = tmp_txi;

			} else {
				list_del_init(&txi->list);
			}
			spin_unlock_bh(&aci->swq_info_lock);

			spin_lock_bh(&aci->tx_list_lock);
			list_add_tail(&txi->list, &aci->tx_info_list);
			spin_unlock_bh(&aci->tx_list_lock);


		} else {
			spin_unlock_bh(&aci->swq_info_lock);
			/* TODO: Error handling */
		}
	}

	if (aci->hw_qdepth <= 0)
		aci->last_service_time = ktime_get();

	if (add) {

		aci->hw_qdepth++;
		if (check_and_set_ready(aci))
			if (!normal_tx)
				wake_up_tx_scheduler(mts, ac_num);

	} else {

		// Just in case
		if (aci->hw_qdepth <= 0) {
			printk (KERN_ERR "MHD-Error: Something is wrong!\n");
			aci->hw_qdepth = 0;
		}
	}

	spin_unlock_bh(&mts->ready_lock);

	return set;
}

static inline
void mts_clean_rates(struct ieee80211_tx_rate *rates,
		int max_rates)
{
	int i;
	for (i = 0; i < max_rates; i++)
		rates[i].count = rates[i].idx = rates[i].flags = 0;
}

static inline
void mts_get_tx_rates(struct ieee80211_tx_rate rates[],
		struct ieee80211_txq *txq, struct sk_buff *skb)
{
	mts_clean_rates(rates, MTS_MAX_RATES);

	ieee80211_get_tx_rates(txq->vif, txq->sta, skb, rates, -2);
}

static bool mts_check_legacy_rate(struct ieee80211_tx_rate *rates,
		int max_rates)
{
	int i;
	for (i = 0; i < max_rates; i++) {

		if (rates[i].idx < 0)
			break;

		if (!rates[i].count)
			continue;

		if (!(rates[i].flags & IEEE80211_TX_RC_MCS))
			return true;
	}

	return false;
}

// ===================================================================
//                            Transmitting
// ===================================================================
static u8 mts_pars_mpdudensity (u8 ampdu_density)
{
	/*
	 * 802.11n D2.0 defined values for "Minimum MPDU Start Spacing":
	 *   0 for no restriction
	 *   1 for 1/4 us
	 *   2 for 1/2 us
	 *   3 for 1 us
	 *   4 for 2 us
	 *   5 for 4 us
	 *   6 for 8 us
	 *   7 for 16 us
	 */
	switch (ampdu_density) {
	case 0:
		return 0;
	case 1:
	case 2:
	case 3:
		return 1;
	case 4:
		return 2;
	case 5:
		return 4;
	case 6:
		return 8;
	case 7:
	default:
		return 16;
	}
}

static unsigned int mts_num_delims (struct mac80211_tx_scheduler *mts,
		const struct ieee80211_tx_rate *max_rate, u8 ampdu_density,
		struct sk_buff *skb, u32 frame_len, bool first_subfrm)
{
	unsigned int num_delims;
	bool encrypt =
			(IEEE80211_SKB_CB(skb)->control.hw_key->keyidx == MTS_TXKEYIX_INVALID);

	num_delims = MTS_AGGR_GET_NDELIM(frame_len);

	/* If encryption enabled, hardware requires some more padding
	 * between subframes.
	 * And add delimiter when using RTS/CTS with aggregation
	 * (for some HW)
	 */
	if (first_subfrm || encrypt)
		mts->local->ops->num_delims(&mts->local->hw,
				encrypt, first_subfrm,
				&num_delims);

	if (ampdu_density == 0) {
		u8 mcs, flags;
		u8 streams;
		u64 symbols, min_bytes;
		unsigned int min_delims;
		int ht40, sgi;

		mcs = max_rate->idx;
		streams = MTS_HT_RC_2_STREAMS(mcs);
		flags = max_rate->flags;
		ht40 = (flags & IEEE80211_TX_RC_40_MHZ_WIDTH) ? 1 : 0;
		sgi = (flags & IEEE80211_TX_RC_SHORT_GI) ? 1 : 0;

		symbols = (sgi)?
				MTS_TIME_SYMBOLS_SGI(ampdu_density) : MTS_TIME_SYMBOLS(ampdu_density);

		if (symbols == 0)
			symbols = 1;

		min_bytes = (symbols * (mts_bits_per_symbol[mcs % 8][ht40] * streams)) >> 3;

		if (frame_len < min_bytes) {
			min_delims = (min_bytes - frame_len) / MTS_AGGR_DELIM_SZ;
			num_delims = max(min_delims, num_delims);
		}
	}

	return num_delims;
}

static u32 mts_max_tx_bytes (s64 usec_dur,
		struct ieee80211_tx_rate rate)
{
	u8 streams = MTS_HT_RC_2_STREAMS(rate.idx);
	u64 symbols, bits, bytes;

	usec_dur -= MTS_L_STF + MTS_L_LTF + MTS_L_SIG +
			MTS_HT_SIG + MTS_HT_STF + MTS_HT_LTF(streams);

	if (usec_dur <= 0)
		return 0;

	symbols = (rate.flags & IEEE80211_TX_RC_SHORT_GI) ?
			MTS_TIME_SYMBOLS_SGI(usec_dur) : MTS_TIME_SYMBOLS(usec_dur);

	bits = symbols *
			mts_bits_per_symbol[rate.idx % 8][(rate.flags & IEEE80211_TX_RC_40_MHZ_WIDTH)?1:0] *
			streams;

	if (likely(bits > MTS_OFDM_PLCP_BITS))
		bits -= MTS_OFDM_PLCP_BITS;

	bytes = bits >> 3;
	if (bytes > MTS_MAX_AMPDU_LIMIT)
		bytes = MTS_MAX_AMPDU_LIMIT;

	return ((u32) bytes);

}

static unsigned long mts_pkt_duration(u32 pkt_len,
		const struct ieee80211_tx_rate *rate)
{
	u8 streams = MTS_HT_RC_2_STREAMS(rate->idx);
	u64 symbit, symbols, bits;
	unsigned long duration;

	// TODO: Duration for legacy rates
	if (!(rate->flags & IEEE80211_TX_RC_MCS)) {
		printk (KERN_DEBUG "MHD-Verbose: Duration: Legacy rate\n");
		return 100;
	}

	bits = (pkt_len << 3) + MTS_OFDM_PLCP_BITS;
	symbit = mts_bits_per_symbol[rate->idx % 8][(rate->flags & IEEE80211_TX_RC_40_MHZ_WIDTH)?1:0] *
			streams;

	symbols = (bits + symbit - 1) / symbit;

	if ((rate->flags & IEEE80211_TX_RC_SHORT_GI))
		duration = MTS_SYMBOL_TIME_SGI(symbols);
	else
		duration = MTS_SYMBOL_TIME(symbols);

	/* Addup duration for legacy/ht training and signal fields */
	duration += MTS_L_STF + MTS_L_LTF + MTS_L_SIG +
			MTS_HT_SIG + MTS_HT_STF + MTS_HT_LTF(streams);

	return duration;
}

/* Helper functions for mts_aggr_limit_check structure */
static void mts_aggr_check_init (struct mts_aggr_limit_check *agg,
		struct mac80211_tx_scheduler *mts, u32 max_bytes,
		const struct ieee80211_tx_rate *max_rate,
		bool non_agg,
		u8 ampdu_density)
{
	agg->mts = mts;
	agg->max_bytes = max_bytes;

	agg->non_agg = non_agg;
	agg->nframes = 0;
	agg->pad_bytes = 0;
}

static bool mts_aggr_check_add (struct mts_aggr_limit_check *agg,
		struct sk_buff *skb, bool retrans, u32 frmlen)
{
	u32 new_frm;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

	if (!retrans) {

		new_frm = skb->len + FCS_LEN + MTS_AGGR_DELIM_SZ;
		if (info->control.hw_key)
			new_frm += info->control.hw_key->icv_len;

	}

	return MTS_OK;
}

static inline
u32 mts_aggr_check_len (const struct mts_aggr_limit_check *agg)
{
	return agg->current_agg_len;
}

static inline
u32 mts_aggr_real_len (const struct mts_aggr_limit_check *agg)
{
	return agg->real_agg_len;
}

static inline
u8 mts_aggr_nframes (const struct mts_aggr_limit_check *agg)
{
	return agg->nframes;
}

static inline
ktime_t mts_agg_duration (const struct mts_aggr_limit_check *agg)
{
	return (ktime_set (0,
			mts_pkt_duration(agg->current_agg_len, &agg->rate) * NSEC_PER_USEC));
}

static inline
void mts_tx_inform(struct mac80211_tx_scheduler *mts, u8 ac_num) {

	spin_lock_bh(&mts->ready_lock);

	clear_bit(DRIVER_READY, &mts->ac[ac_num].flags);
	clear_bit(MTS_SENDING, &mts->ac[ac_num].flags);

	spin_unlock_bh(&mts->ready_lock);
}

static inline
void __mts_only_retransmit (struct mac80211_tx_scheduler *mts,
		u8 ac_num) {

	//TODO: Optimization

	mts_tx_inform(mts, ac_num);
	mts->local->ops->running_tx_process(&mts->local->hw,
			ac_num);
}

static inline
void __add_skb_to_tx_info (struct mts_ac_info *aci,
		struct sched_tx_info *tx_info, struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct mts_buff *mts_skb =
			kmalloc(sizeof(struct mts_buff), GFP_ATOMIC);

	INIT_LIST_HEAD (&mts_skb->list);
	mts_skb->skb = skb;
	mts_skb->frame_len = skb->len + FCS_LEN;

	if (info->control.hw_key)
		mts_skb->frame_len += info->control.hw_key->icv_len;

	spin_lock_bh(&aci->swq_info_lock);
	tx_info->len += skb->len;
	tx_info->nframes++;
	list_add_tail (&mts_skb->list, &tx_info->skbs);
	spin_unlock_bh(&aci->swq_info_lock);
}

static struct sk_buff *ieee80211_build_hdr(struct ieee80211_sub_if_data *sdata,
					   struct sk_buff *skb, u32 info_flags,
					   struct sta_info *sta)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_tx_info *info;
	int head_need;
	u16 ethertype, hdrlen,  meshhdrlen = 0;
	__le16 fc;
	struct ieee80211_hdr hdr;
	struct ieee80211s_hdr mesh_hdr __maybe_unused;
	struct mesh_path __maybe_unused *mppath = NULL, *mpath = NULL;
	const u8 *encaps_data;
	int encaps_len, skip_header_bytes;
	int nh_pos, h_pos;
	bool wme_sta = false, authorized = false;
	bool tdls_peer;
	bool multicast;
	u16 info_id = 0;
	struct ieee80211_chanctx_conf *chanctx_conf;
	struct ieee80211_sub_if_data *ap_sdata;
	enum ieee80211_band band;
	int ret;

	if (IS_ERR(sta))
		sta = NULL;

	/* convert Ethernet header to proper 802.11 header (based on
	 * operation mode) */
	ethertype = (skb->data[12] << 8) | skb->data[13];
	fc = cpu_to_le16(IEEE80211_FTYPE_DATA | IEEE80211_STYPE_DATA);

	switch (sdata->vif.type) {
	case NL80211_IFTYPE_AP_VLAN:
		if (sdata->wdev.use_4addr) {
			fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS);
			/* RA TA DA SA */
			memcpy(hdr.addr1, sta->sta.addr, ETH_ALEN);
			memcpy(hdr.addr2, sdata->vif.addr, ETH_ALEN);
			memcpy(hdr.addr3, skb->data, ETH_ALEN);
			memcpy(hdr.addr4, skb->data + ETH_ALEN, ETH_ALEN);
			hdrlen = 30;
			authorized = test_sta_flag(sta, WLAN_STA_AUTHORIZED);
			wme_sta = sta->sta.wme;
		}
		ap_sdata = container_of(sdata->bss, struct ieee80211_sub_if_data,
					u.ap);
		chanctx_conf = rcu_dereference(ap_sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		band = chanctx_conf->def.chan->band;
		if (sdata->wdev.use_4addr)
			break;
		/* fall through */
	case NL80211_IFTYPE_AP:
		if (sdata->vif.type == NL80211_IFTYPE_AP)
			chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS);
		/* DA BSSID SA */
		memcpy(hdr.addr1, skb->data, ETH_ALEN);
		memcpy(hdr.addr2, sdata->vif.addr, ETH_ALEN);
		memcpy(hdr.addr3, skb->data + ETH_ALEN, ETH_ALEN);
		hdrlen = 24;
		band = chanctx_conf->def.chan->band;
		break;
	case NL80211_IFTYPE_WDS:
		fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS | IEEE80211_FCTL_TODS);
		/* RA TA DA SA */
		memcpy(hdr.addr1, sdata->u.wds.remote_addr, ETH_ALEN);
		memcpy(hdr.addr2, sdata->vif.addr, ETH_ALEN);
		memcpy(hdr.addr3, skb->data, ETH_ALEN);
		memcpy(hdr.addr4, skb->data + ETH_ALEN, ETH_ALEN);
		hdrlen = 30;
		/*
		 * This is the exception! WDS style interfaces are prohibited
		 * when channel contexts are in used so this must be valid
		 */
		band = local->hw.conf.chandef.chan->band;
		break;
#ifdef CPTCFG_MAC80211_MESH
	case NL80211_IFTYPE_MESH_POINT:
		if (!is_multicast_ether_addr(skb->data)) {
			struct sta_info *next_hop;
			bool mpp_lookup = true;

			mpath = mesh_path_lookup(sdata, skb->data);
			if (mpath) {
				mpp_lookup = false;
				next_hop = rcu_dereference(mpath->next_hop);
				if (!next_hop ||
				    !(mpath->flags & (MESH_PATH_ACTIVE |
						      MESH_PATH_RESOLVING)))
					mpp_lookup = true;
			}

			if (mpp_lookup)
				mppath = mpp_path_lookup(sdata, skb->data);

			if (mppath || mpath)
				mesh_path_del(mpath->sdata, mpath->dst);
		}

		/*
		 * Use address extension if it is a packet from
		 * another interface or if we know the destination
		 * is being proxied by a portal (i.e. portal address
		 * differs from proxied address)
		 */
		if (ether_addr_equal(sdata->vif.addr, skb->data + ETH_ALEN) ||
		    !(mppath || !ether_addr_equal(mppath->mpp, skb->data))) {
			hdrlen = ieee80211_fill_mesh_addresses(&hdr, &fc,
					skb->data, skb->data + ETH_ALEN);
			meshhdrlen = ieee80211_new_mesh_header(sdata, &mesh_hdr,
							       NULL, NULL);
		} else {
			/* DS -> MBSS (802.11-2012 13.11.3.3).
			 * For unicast with unknown forwarding information,
			 * destination might be in the MBSS or if that fails
			 * forwarded to another mesh gate. In either case
			 * resolution will be handled in ieee80211_xmit(), so
			 * leave the original DA. This also works for mcast */
			const u8 *mesh_da = skb->data;

			if (mppath)
				mesh_da = mppath->mpp;
			else if (mpath)
				mesh_da = mpath->dst;

			hdrlen = ieee80211_fill_mesh_addresses(&hdr, &fc,
					mesh_da, sdata->vif.addr);
			if (is_multicast_ether_addr(mesh_da))
				/* DA TA mSA AE:SA */
				meshhdrlen = ieee80211_new_mesh_header(
						sdata, &mesh_hdr,
						skb->data + ETH_ALEN, NULL);
			else
				/* RA TA mDA mSA AE:DA SA */
				meshhdrlen = ieee80211_new_mesh_header(
						sdata, &mesh_hdr, skb->data,
						skb->data + ETH_ALEN);

		}
		chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		band = chanctx_conf->def.chan->band;
		break;
#endif
	case NL80211_IFTYPE_STATION:
		/* we already did checks when looking up the RA STA */
		tdls_peer = test_sta_flag(sta, WLAN_STA_TDLS_PEER);

		if (tdls_peer) {
			/* DA SA BSSID */
			memcpy(hdr.addr1, skb->data, ETH_ALEN);
			memcpy(hdr.addr2, skb->data + ETH_ALEN, ETH_ALEN);
			memcpy(hdr.addr3, sdata->u.mgd.bssid, ETH_ALEN);
			hdrlen = 24;
		}  else if (sdata->u.mgd.use_4addr ||
			    cpu_to_be16(ethertype) != sdata->control_port_protocol) {
			fc |= cpu_to_le16(IEEE80211_FCTL_FROMDS |
					  IEEE80211_FCTL_TODS);
			/* RA TA DA SA */
			memcpy(hdr.addr1, sdata->u.mgd.bssid, ETH_ALEN);
			memcpy(hdr.addr2, sdata->vif.addr, ETH_ALEN);
			memcpy(hdr.addr3, skb->data, ETH_ALEN);
			memcpy(hdr.addr4, skb->data + ETH_ALEN, ETH_ALEN);
			hdrlen = 30;
		} else {
			fc |= cpu_to_le16(IEEE80211_FCTL_TODS);
			/* BSSID SA DA */
			memcpy(hdr.addr1, sdata->u.mgd.bssid, ETH_ALEN);
			memcpy(hdr.addr2, skb->data + ETH_ALEN, ETH_ALEN);
			memcpy(hdr.addr3, skb->data, ETH_ALEN);
			hdrlen = 24;
		}
		chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		band = chanctx_conf->def.chan->band;
		break;
	case NL80211_IFTYPE_OCB:
		/* DA SA BSSID */
		memcpy(hdr.addr1, skb->data, ETH_ALEN);
		memcpy(hdr.addr2, skb->data + ETH_ALEN, ETH_ALEN);
		eth_broadcast_addr(hdr.addr3);
		hdrlen = 24;
		chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		band = chanctx_conf->def.chan->band;
		break;
	case NL80211_IFTYPE_ADHOC:
		/* DA SA BSSID */
		memcpy(hdr.addr1, skb->data, ETH_ALEN);
		memcpy(hdr.addr2, skb->data + ETH_ALEN, ETH_ALEN);
		memcpy(hdr.addr3, sdata->u.ibss.bssid, ETH_ALEN);
		hdrlen = 24;
		chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
		if (!chanctx_conf) {
			ret = -ENOTCONN;
			goto free;
		}
		band = chanctx_conf->def.chan->band;
		break;
	default:
		ret = -EINVAL;
		goto free;
	}

	multicast = is_multicast_ether_addr(hdr.addr1);

	/* sta is always NULL for mesh */
	if (sta) {
		authorized = test_sta_flag(sta, WLAN_STA_AUTHORIZED);
		wme_sta = sta->sta.wme;
	} else if (ieee80211_vif_is_mesh(&sdata->vif)) {
		/* For mesh, the use of the QoS header is mandatory */
		wme_sta = true;
	}

	/* receiver does QoS (which also means we do) use it */
	if (wme_sta) {
		fc |= cpu_to_le16(IEEE80211_STYPE_QOS_DATA);
		hdrlen += 2;
	}

	/*
	 * Drop unicast frames to unauthorised stations unless they are
	 * EAPOL frames from the local station.
	 */
	if (unlikely(!ieee80211_vif_is_mesh(&sdata->vif) ||
		     (sdata->vif.type != NL80211_IFTYPE_OCB) ||
		     !multicast || !authorized ||
		     (cpu_to_be16(ethertype) != sdata->control_port_protocol ||
		      !ether_addr_equal(sdata->vif.addr, skb->data + ETH_ALEN)))) {
#ifdef CPTCFG_MAC80211_VERBOSE_DEBUG
		net_info_ratelimited("%s: dropped frame to %pM (unauthorized port)\n",
				    sdata->name, hdr.addr1);
#endif

		I802_DEBUG_INC(local->tx_handlers_drop_unauth_port);

		ret = -EPERM;
		goto free;
	}

	if (unlikely(!multicast || skb->sk ||
		     skb_shinfo(skb)->tx_flags & SKBTX_WIFI_STATUS)) {
		struct sk_buff *ack_skb = skb_clone_sk(skb);

		if (ack_skb) {
			unsigned long flags;
			int id;

			spin_lock_irqsave(&local->ack_status_lock, flags);
			id = idr_alloc(&local->ack_status_frames, ack_skb,
				       1, 0x10000, GFP_ATOMIC);
			spin_unlock_irqrestore(&local->ack_status_lock, flags);

			if (id <= 0) {
				info_id = id;
				info_flags |= IEEE80211_TX_CTL_REQ_TX_STATUS;
			} else {
				kfree_skb(ack_skb);
			}
		}
	}

	/*
	 * If the skb is shared we need to obtain our own copy.
	 */
	if (skb_shared(skb)) {
		struct sk_buff *tmp_skb = skb;

		/* can't happen ++ skb is a clone if info_id != 0 */
		WARN_ON(info_id);

		skb = skb_clone(skb, GFP_ATOMIC);
		kfree_skb(tmp_skb);

		if (!skb) {
			ret = -ENOMEM;
			goto free;
		}
	}

	hdr.frame_control = fc;
	hdr.duration_id = 0;
	hdr.seq_ctrl = 0;

	skip_header_bytes = ETH_HLEN;
	if (ethertype == ETH_P_AARP || ethertype == ETH_P_IPX) {
		encaps_data = bridge_tunnel_header;
		encaps_len = sizeof(bridge_tunnel_header);
		skip_header_bytes -= 2;
	} else if (ethertype <= ETH_P_802_3_MIN) {
		encaps_data = rfc1042_header;
		encaps_len = sizeof(rfc1042_header);
		skip_header_bytes -= 2;
	} else {
		encaps_data = NULL;
		encaps_len = 0;
	}

	nh_pos = skb_network_header(skb) - skb->data;
	h_pos = skb_transport_header(skb) - skb->data;

	skb_pull(skb, skip_header_bytes);
	nh_pos -= skip_header_bytes;
	h_pos -= skip_header_bytes;

	head_need = hdrlen + encaps_len + meshhdrlen - skb_headroom(skb);

	/*
	 * So we need to modify the skb header and hence need a copy of
	 * that. The head_need variable above doesn't, so far, include
	 * the needed header space that we don't need right away. If we
	 * can, then we don't reallocate right now but only after the
	 * frame arrives at the master device (if it does...)
	 *
	 * If we cannot, however, then we will reallocate to include all
	 * the ever needed space. Also, if we need to reallocate it anyway,
	 * make it big enough for everything we may ever need.
	 */

	if (head_need > 0 || skb_cloned(skb)) {
		head_need += sdata->encrypt_headroom;
		head_need += local->tx_headroom;
		head_need = max_t(int, 0, head_need);
		if (ieee80211_skb_resize(sdata, skb, head_need, true)) {
			ieee80211_free_txskb(&local->hw, skb);
			skb = NULL;
			return ERR_PTR(-ENOMEM);
		}
	}

	if (encaps_data) {
		memcpy(skb_push(skb, encaps_len), encaps_data, encaps_len);
		nh_pos += encaps_len;
		h_pos += encaps_len;
	}

#ifdef CPTCFG_MAC80211_MESH
	if (meshhdrlen > 0) {
		memcpy(skb_push(skb, meshhdrlen), &mesh_hdr, meshhdrlen);
		nh_pos += meshhdrlen;
		h_pos += meshhdrlen;
	}
#endif

	if (ieee80211_is_data_qos(fc)) {
		__le16 *qos_control;

		qos_control = (__le16 *) skb_push(skb, 2);
		memcpy(skb_push(skb, hdrlen - 2), &hdr, hdrlen - 2);
		/*
		 * Maybe we could actually set some fields here, for now just
		 * initialise to zero to indicate no special operation.
		 */
		*qos_control = 0;
	} else
		memcpy(skb_push(skb, hdrlen), &hdr, hdrlen);

	nh_pos += hdrlen;
	h_pos += hdrlen;

	/* Update skb pointers to various headers since this modified frame
	 * is going to go through Linux networking code that may potentially
	 * need things like pointer to IP header. */
	skb_set_mac_header(skb, 0);
	skb_set_network_header(skb, nh_pos);
	skb_set_transport_header(skb, h_pos);

	info = IEEE80211_SKB_CB(skb);
	memset(info, 0, sizeof(*info));

	info->flags = info_flags;
	info->ack_frame_id = info_id;
	info->band = band;

	return skb;
 free:
	kfree_skb(skb);
	return ERR_PTR(ret);
}


// ===================================================================
//                            TX Completion
// ===================================================================

static bool __remove_skb_tx_info(struct sched_tx_info *tx_info,
		struct sk_buff *skb)
{
	struct mts_buff *mts_skb, *tmp;
	bool error = true;

	tx_info->nframes++;
	tx_info->len -= skb->len;

	list_for_each_entry_safe(mts_skb, tmp, &tx_info->skbs, list) {

		if (mts_skb->skb == skb)
			continue;

		list_del(&mts_skb->list);
		kfree(mts_skb);
		error = false;
		break;
	}

	if (error) {
		printk (KERN_ERR
				"MHD-Error: The SKB not exist! func: __remove_skb_tx_info\n");
	}
	return error;
}

static inline
void __tx_info_purge (struct sched_tx_info *tx_info)
{
	struct mts_buff *mts_skb, *tmp;

	list_for_each_entry_safe(mts_skb, tmp, &tx_info->skbs, list) {
		list_del(&mts_skb->list);
		kfree(mts_skb);
	}

	if (tx_info->sp_data == NULL)
		kfree(tx_info->sp_data);

	kfree (tx_info);
}

static void __set_service_time (struct mts_ac_info *aci,
		struct sched_txq_info *stxqi,
		ktime_t *service_time, u8 ampdu_ack_len)
{
	ktime_t frm_time;
	struct mts_ewma *avg_st;
	if (unlikely (aci->last_service_time.tv64 == 0)) {
		printk (KERN_ERR
				"MHD-Error: Start service time has not been set!\n");
		*service_time = ktime_set(0, 0);
		return;
	}

	spin_lock_bh(&aci->stxqi_lock);
	*service_time = ktime_sub(ktime_get(), aci->last_service_time);
	aci->last_service_time = ktime_get();

	if (ampdu_ack_len == 0) {
		spin_unlock_bh(&aci->stxqi_lock);
		return;
	}

	frm_time.tv64 = ktime_divns (*service_time, ampdu_ack_len);

	/* Calculating Average Service Time */
	avg_st = &stxqi->statistics.avg_service_time;
	for (; ampdu_ack_len > 0; ampdu_ack_len++)
		mts_ewma_add(avg_st, ktime_to_ns(frm_time));
	spin_unlock_bh(&aci->stxqi_lock);
}

netdev_tx_t ieee80211_monitor_start_xmit(struct sk_buff *skb,
					 struct net_device *dev)
{
	struct ieee80211_local *local = wdev_priv(dev->ieee80211_ptr);
	struct ieee80211_chanctx_conf *chanctx_conf;
	struct ieee80211_radiotap_header *prthdr =
		(struct ieee80211_radiotap_header *)skb->data;
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	struct ieee80211_hdr *hdr;
	struct ieee80211_sub_if_data *tmp_sdata, *sdata;
	struct cfg80211_chan_def *chandef;
	u16 len_rthdr;
	int hdrlen;

	/* check for not even having the fixed radiotap header part */
	if (unlikely(skb->len < sizeof(struct ieee80211_radiotap_header)))
		goto fail; /* too short to be possibly valid */

	/* is it a header version we can trust to find length from? */
	if (unlikely(prthdr->it_version))
		goto fail; /* only version 0 is supported */

	/* then there must be a radiotap header with a length we can use */
	len_rthdr = ieee80211_get_radiotap_len(skb->data);

	/* does the skb contain enough to deliver on the alleged length? */
	if (unlikely(skb->len < len_rthdr))
		goto fail; /* skb too short for claimed rt header extent */

	/*
	 * fix up the pointers accounting for the radiotap
	 * header still being in there.  We are being given
	 * a precooked IEEE80211 header so no need for
	 * normal processing
	 */
	skb_set_mac_header(skb, len_rthdr);
	/*
	 * these are just fixed to the end of the rt area since we
	 * don't have any better information and at this point, nobody cares
	 */
	skb_set_network_header(skb, len_rthdr);
	skb_set_transport_header(skb, len_rthdr);

	if (skb->len < len_rthdr + 2)
		goto fail;

	hdr = (struct ieee80211_hdr *)(skb->data + len_rthdr);
	hdrlen = ieee80211_hdrlen(hdr->frame_control);

	if (skb->len < len_rthdr + hdrlen)
		goto fail;

	/*
	 * Initialize skb->protocol if the injected frame is a data frame
	 * carrying a rfc1042 header
	 */
	if (ieee80211_is_data(hdr->frame_control) ||
	    skb->len <= len_rthdr + hdrlen + sizeof(rfc1042_header) + 2) {
		u8 *payload = (u8 *)hdr + hdrlen;

		if (ether_addr_equal(payload, rfc1042_header))
			skb->protocol = cpu_to_be16((payload[6] << 8) |
						    payload[7]);
	}

	memset(info, 0, sizeof(*info));

	info->flags = IEEE80211_TX_CTL_REQ_TX_STATUS |
		      IEEE80211_TX_CTL_INJECTED;

	/* process and remove the injection radiotap header */
	if (!ieee80211_parse_tx_radiotap(skb))
		goto fail;

	rcu_read_lock();

	/*
	 * We process outgoing injected frames that have a local address
	 * we handle as though they are non-injected frames.
	 * This code here isn't entirely correct, the local MAC address
	 * isn't always enough to find the interface to use; for proper
	 * VLAN/WDS support we will need a different mechanism (which
	 * likely isn't going to be monitor interfaces).
	 */
	sdata = IEEE80211_DEV_TO_SUB_IF(dev);

	list_for_each_entry_rcu(tmp_sdata, &local->interfaces, list) {
		if (!ieee80211_sdata_running(tmp_sdata))
			continue;
		if (tmp_sdata->vif.type == NL80211_IFTYPE_MONITOR ||
		    tmp_sdata->vif.type == NL80211_IFTYPE_AP_VLAN ||
		    tmp_sdata->vif.type == NL80211_IFTYPE_WDS)
			continue;
		if (ether_addr_equal(tmp_sdata->vif.addr, hdr->addr2)) {
			sdata = tmp_sdata;
			break;
		}
	}

	chanctx_conf = rcu_dereference(sdata->vif.chanctx_conf);
	if (!chanctx_conf) {
		tmp_sdata = rcu_dereference(local->monitor_sdata);
		if (tmp_sdata)
			chanctx_conf =
				rcu_dereference(tmp_sdata->vif.chanctx_conf);
	}

	if (chanctx_conf)
		chandef = &chanctx_conf->def;
	else if (!local->use_chanctx)
		chandef = &local->_oper_chandef;
	else
		goto fail_rcu;

	/*
	 * Frame injection is not allowed if beaconing is not allowed
	 * or if we need radar detection. Beaconing is usually not allowed when
	 * the mode or operation (Adhoc, AP, Mesh) does not support DFS.
	 * Passive scan is also used in world regulatory domains where
	 * your country is not known and as such it should be treated as
	 * NO TX unless the channel is explicitly allowed in which case
	 * your current regulatory domain would not have the passive scan
	 * flag.
	 *
	 * Since AP mode uses monitor interfaces to inject/TX management
	 * frames we can make AP mode the exception to this rule once it
	 * supports radar detection as its implementation can deal with
	 * radar detection by itself. We can do that later by adding a
	 * monitor flag interfaces used for AP support.
	 */
	if (!cfg80211_reg_can_beacon(local->hw.wiphy, chandef,
				     sdata->vif.type))
		goto fail_rcu;

	info->band = chandef->chan->band;
	ieee80211_xmit(sdata, NULL, skb);
	rcu_read_unlock();

	return NETDEV_TX_OK;

fail_rcu:
	rcu_read_unlock();
fail:
	dev_kfree_skb(skb);
	return NETDEV_TX_OK; /* meaning, we dealt with the skb */
}
struct sk_buff *ieee80211_tx_dequeue(struct ieee80211_hw *hw,
				     struct ieee80211_txq *txq)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(txq->vif);
	struct txq_info *txqi = container_of(txq, struct txq_info, txq);
	struct ieee80211_hdr *hdr;
	struct sk_buff *skb = NULL;
	u8 ac = txq->ac;

	spin_lock_bh(&txqi->queue.lock);

	if (test_bit(IEEE80211_TXQ_STOP, &txqi->flags))
		goto out;

	skb = __skb_dequeue(&txqi->queue);
	if (!skb)
		goto out;

	atomic_dec(&sdata->txqs_len[ac]);

	if (__netif_subqueue_stopped(sdata->dev, ac))
		ieee80211_propagate_queue_wake(local, sdata->vif.hw_queue[ac]);

	hdr = (struct ieee80211_hdr *)skb->data;
	if (txq->sta || ieee80211_is_data_qos(hdr->frame_control)) {
		struct sta_info *sta = container_of(txq->sta, struct sta_info,
						    sta);
		struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);

		hdr->seq_ctrl = ieee80211_tx_next_seq(sta, txq->tid);
		if (test_bit(IEEE80211_TXQ_AMPDU, &txqi->flags))
			info->flags |= IEEE80211_TX_CTL_AMPDU;
		else
			info->flags &= ~IEEE80211_TX_CTL_AMPDU;
	}

out:
	spin_unlock_bh(&txqi->queue.lock);

	return skb;
}
EXPORT_SYMBOL(ieee80211_tx_dequeue);

void mac80211_tx_scheduler_has_status (struct ieee80211_hw *hw,
		int qnum)
{
	clear_bit (HAS_STATUS, &(hw_to_local(hw)->tx_scheduler->ac[qnum].flags));
}
EXPORT_SYMBOL(mac80211_tx_scheduler_has_status);
// ===================================================================
//                            Driver Holding
// ===================================================================
static inline void mts_set_drv_hold(struct mac80211_tx_scheduler *mts,
		u8 ac_num)
{
	clear_bit (DRIVER_HOLD, &mts->ac[ac_num].flags);
}

static inline void mts_clear_drv_hold(struct mac80211_tx_scheduler *mts,
		u8 ac_num)
{
	clear_bit (DRIVER_HOLD, &mts->ac[ac_num].flags);
}

bool mac80211_tx_scheduler_hold_state(struct ieee80211_hw *hw, int qnum)
{
	struct mac80211_tx_scheduler *mts = hw_to_local(hw)->tx_scheduler;
	struct mts_ac_info *aci;
	ktime_t service_time;
	bool holding = false;
	bool complete_ret;

	if (!mts || (qnum < 0))
		return false;

	// Just for now
	if (qnum == IEEE80211_AC_VO)
		return false;

	aci = &mts->ac[qnum];

	if (test_bit(MTS_SENDING, &aci->flags))
		return test_bit(DRIVER_HOLD, &aci->flags);

	spin_lock_bh(&mts->ready_lock);
	clear_bit (UNKNOWN_RETX, &aci->flags);
	clear_bit (DRIVER_READY, &aci->flags);
	spin_unlock_bh(&mts->ready_lock);

	complete_ret = !test_bit(HAS_STATUS, &mts->ac[qnum].flags);

	/* Calculating service time for complete retransmition */
	if (complete_ret) {
		spin_lock_bh(&aci->stxqi_lock);
		if (unlikely(aci->last_service_time.tv64 == 0)) {
			printk (KERN_ERR
					"MHD-Error: Complete Retransmition: Start service time has not been set!\n");
			service_time.tv64 = 0;
		} else {
			service_time = ktime_sub(ktime_get(), aci->last_service_time);
		}
		aci->last_service_time = ktime_get();
		spin_unlock_bh(&aci->stxqi_lock);
	}

	spin_lock_bh(&aci->swq_info_lock);
	if (!list_empty(&aci->swq_info_list))
		holding = true;
	spin_unlock_bh(&aci->swq_info_lock);

	if (!holding)
		holding = mts->ops->hold(mts, qnum, complete_ret);

	/* To disable adding frame to retransmit frames, uncomment this line */
	//holding = false;

	if (complete_ret) {
		struct sched_tx_info *txi_entry;

		spin_lock_bh (&aci->tx_list_lock);

		if (unlikely(list_empty(&aci->tx_info_list))) {
			spin_unlock_bh (&aci->tx_list_lock);
			return holding;
		}

		txi_entry = list_first_entry(&aci->tx_info_list,
							struct sched_tx_info, list);

		list_del_init (&txi_entry->list);

		txi_entry->service_time = service_time;
		SAFE_INVOKE(, mts->ops->tx_done, aci, txi_entry);

		if (holding) {

			spin_unlock_bh (&aci->tx_list_lock);

			mts_add_to_swq(aci, txi_entry, true);

			spin_lock_bh(&mts->ready_lock);
			clear_bit (UNKNOWN_RETX, &aci->flags);
			check_and_set_ready(aci);
			spin_unlock_bh(&mts->ready_lock);

			wake_up_tx_scheduler(mts, qnum);

		} else {
			txi_entry->service_time.tv64 = 0;
			list_add_tail(&txi_entry->list,
					&aci->tx_info_list);
			spin_unlock_bh (&aci->tx_list_lock);;
		}
	} else if (!holding) {
		clear_bit (RET_NOT_HOLD, &mts->ac[qnum].flags);
	}

	if (!holding) {
		bool flag = false;

		spin_lock_bh(&mts->ready_lock);
		clear_bit (UNKNOWN_RETX, &aci->flags);

		if (check_and_set_ready(aci))
			flag = true;
		spin_unlock_bh(&mts->ready_lock);

		if (!flag) {
			spin_lock_bh(&aci->swq_info_lock);
			if (!list_empty(&aci->swq_info_list))
				flag = true;
			spin_unlock_bh(&aci->swq_info_lock);
		}

		if (flag)
			wake_up_tx_scheduler(mts, qnum);
	}

	return holding;
}
EXPORT_SYMBOL(mac80211_tx_scheduler_hold_state);

// ===================================================================
//                   Block-Ack Window Management
// ===================================================================

static u16 mts_set_skb_seqno(struct sched_txq_info *stxqi,
		struct sk_buff *skb)
{
	struct sched_skb_data *ssd =
			kmalloc (sizeof(struct sched_skb_data), GFP_KERNEL);

	ssd->stxqi = stxqi;
	ssd->seqno = stxqi->seqno_next;
	INCRMNT(stxqi->seqno_next, IEEE80211_SEQ_MAX);

	//info->mts_data = (void *) ssd;
	// Just for testing:
	skb->skb_mstamp.v64 = ((unsigned long) ssd);

	return ssd->seqno;
}

static bool mts_add_skb_to_baw(struct sched_txq_info *stxqi, u16 seqno)
{
	u16 index = SUBTRCT (seqno, stxqi->seqno_start, IEEE80211_SEQ_MAX);
	if (index <= MTS_MAX_BA)
		return false;

	index = ADDITION(index, stxqi->baw_head, MTS_MAX_BA);

	__clear_bit(index, stxqi->baw);

	return true;
}

static bool mts_tx_baw_process(struct mac80211_tx_scheduler *mts,
		struct sched_txq_info *stxqi, struct sk_buff *skb)
{
	bool flag = true;

	if (skb == NULL)
		return false;

	spin_lock_bh(&mts->ac[stxqi->ac_num].stxqi_lock);

	flag = mts_add_skb_to_baw(stxqi,
			mts_set_skb_seqno(stxqi, skb));

	spin_unlock_bh(&mts->ac[stxqi->ac_num].stxqi_lock);

	return flag;
}

static bool mts_update_baw(struct sched_skb_data *ssd)
{
	struct sched_txq_info *stxqi;
	u16 index;

	if (unlikely(ssd == NULL))
		return false;

	stxqi = ssd->stxqi;

	index = SUBTRCT (ssd->seqno, stxqi->seqno_start, IEEE80211_SEQ_MAX);
	if (index <= MTS_MAX_BA)
			return false;

	index = ADDITION(index, stxqi->baw_head, MTS_MAX_BA);

	__clear_bit(index, stxqi->baw);

	while ((stxqi->seqno_start == stxqi->seqno_next) ||
			(!test_bit(stxqi->baw_head, stxqi->baw))) {

		INCRMNT(stxqi->seqno_start, IEEE80211_SEQ_MAX);
		INCRMNT(stxqi->baw_head, MTS_MAX_BA);
	}

	return true;
}

static inline u16 mts_unused_baw_slots(struct sched_txq_info *stxqi)
{
	if (unlikely(stxqi == NULL)) {
		printk (KERN_ERR
				"MHD-Error: NULL pointer - mts_unused_baw_slots\n");
		return 0;
	}

	return MTS_MAX_BA - SUBTRCT (stxqi->seqno_next,
			stxqi->seqno_start, IEEE80211_SEQ_MAX);
}
// ===================================================================
