#define LOG_MODULE_NAME "ven_cmd"

#include <net/cfg80211.h>
#include <net/netlink.h>
#include <net/mac80211.h>

#include "wondertap_internal.h"
#include "wonder_log.h"
#include "include/wonder/wonder_ven_cmd.h"
#include "nl80211_ven_cmd.h"

/* @brief Internal helpers to map raw byte sizes to NLA types */
#define _WONDER_TYPE_MAP_1  NLA_U8
#define _WONDER_TYPE_MAP_2  NLA_U16
#define _WONDER_TYPE_MAP_4  NLA_U32
#define _WONDER_TYPE_MAP_8  NLA_U64

#define _WONDER_IND_TYPE_MAP(val) _WONDER_TYPE_MAP_##val
#define _WONDER_GET_TYPE(size) _WONDER_IND_TYPE_MAP(size)

/**
 * @brief Auto-generate policy for integer attributes
 */
#define WONDER_POL_SCALAR(attr) \
	[attr] = { .type = _WONDER_GET_TYPE(attr##_SIZE) }

/**
 * @brief Auto-generate policy for fixed-size binary blobs
 */
#define WONDER_POL_BINARY(attr) \
	[attr] = { .type = NLA_BINARY, .len = attr##_SIZE }

/**
 * @brief Auto-generate policy for fixed-max-length strings
 */
#define WONDER_POL_STRING(attr) \
	[attr] = { .type = NLA_NUL_STRING, .len = attr##_SIZE }

/**
 * @brief Auto-generate policy for nested attributes
 */
#define WONDER_POL_NESTED(attr) \
	[attr] = { .type = NLA_NESTED }

static const struct nla_policy
wonder_set_frequency_policy[WONDER_VEN_ATTR_CHANNEL_ATTR_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_FREQ),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_BANDWIDTH),
};

static const struct nla_policy
wonder_filter_params_policy[WONDER_VEN_ATTR_FILTER_PARAM_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FILTER_BSSID_ENABLED),
	WONDER_POL_BINARY(WONDER_VEN_ATTR_FILTER_BSSID_ADDR),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FILTER_FRAME_ENABLED),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FILTER_FRAME_TYPE),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FILTER_FRAME_SUBTYPE),
};

static const struct nla_policy
wonder_fixed_rate_policy[WONDER_VEN_ATTR_FIXED_TX_RATE_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FIXED_TX_RATE_PREAMBLE),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FIXED_TX_RATE_BW),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FIXED_TX_RATE_GI),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FIXED_TX_RATE_NSS),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_FIXED_TX_RATE_MCS),
};

static const struct nla_policy
wonder_tx_rate_mask_policy[WONDER_VEN_ATTR_TX_RATE_TEST_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_TX_RATE_TEST_PREAMBLE),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_TX_RATE_TEST_BW),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_TX_RATE_TEST_NSS),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_TX_RATE_TEST_MCS),
};

static const struct nla_policy
wonder_set_reg_policy[WONDER_VEN_ATTR_REG_MAX + 1] = {
	WONDER_POL_STRING(WONDER_VEN_ATTR_REG_COUNTRY_CODE),
};

static const struct nla_policy
wonder_channel_schedule_policy[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST_LEN),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_NEXT_IDX),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_DWELL_TIME),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_SWITCH_TIME),
	WONDER_POL_NESTED(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_SCHEDULE_TSF_OFFSET),
};

static const struct nla_policy
wonder_channel_list_entry_policy[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_MAX + 1] = {
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_FREQ),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_BW),
	WONDER_POL_SCALAR(WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_ROLE),
};

static int wonder_vendor_cmd_set_frequency(struct wiphy *wiphy,
					   struct wireless_dev *wdev,
					   const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_CHANNEL_ATTR_MAX + 1];
	struct wondertap_set_freq_params params;

	if (nla_parse(tb, WONDER_VEN_ATTR_CHANNEL_ATTR_MAX, data, data_len,
				 wonder_set_frequency_policy, NULL) < 0) {
		wonder_error("Invalid attributes\n");
		return -EINVAL;
	}

	/* Check that mandatory attributes are present */
	if (!tb[WONDER_VEN_ATTR_CHANNEL_FREQ] || !tb[WONDER_VEN_ATTR_CHANNEL_BANDWIDTH]) {
		wonder_error("Missing mandatory attributes\n");
		return -EINVAL;
	}

	/* Populate the params struct with the received data */
	params.freq = nla_get_u32(tb[WONDER_VEN_ATTR_CHANNEL_FREQ]);
	params.bandwidth = nla_get_u16(tb[WONDER_VEN_ATTR_CHANNEL_BANDWIDTH]);

	wonder_info("SET_FREQUENCY: freq=%u MHz, bandwidth=%u\n",
		params.freq, params.bandwidth);

	return wondertap_set_freq(&wonder->wondertap_data, &params);
}

static int wonder_vendor_cmd_set_filter(struct wiphy *wiphy,
								struct wireless_dev *wdev,
								const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_TOP_FILTER_MAX + 1];
	struct nlattr *params_tb[WONDER_VEN_ATTR_FILTER_PARAM_MAX + 1];
	enum wonder_vendor_filter_type filter_type;
	int ret;

	/* Parse top-level attributes */
	if (nla_parse(tb, WONDER_VEN_ATTR_TOP_FILTER_MAX, data, data_len, NULL, NULL) < 0) {
		wonder_error("Failed to parse top-level attributes\n");
		return -EINVAL;
	}

	if (!tb[WONDER_VEN_ATTR_TOP_FILTER_TYPE] || !tb[WONDER_VEN_ATTR_TOP_FILTER_PARAMS]) {
		wonder_error("Missing mandatory attributes: FILTER_TYPE or FILTER_PARAMS\n");
		return -EINVAL;
	}

	filter_type = nla_get_u32(tb[WONDER_VEN_ATTR_TOP_FILTER_TYPE]);
	wonder_info("Configuring filter type: %u\n", filter_type);

	if (nla_parse(params_tb, WONDER_VEN_ATTR_FILTER_PARAM_MAX,
						nla_data(tb[WONDER_VEN_ATTR_TOP_FILTER_PARAMS]),
						nla_len(tb[WONDER_VEN_ATTR_TOP_FILTER_PARAMS]),
							wonder_filter_params_policy, NULL) < 0) {
		wonder_error("2 Invalid attributes\n");
		return -EINVAL;
	}

	switch (filter_type) {
	case WONDER_VEN_ATTR_FILTER_TYPE_BSSID: {
		struct wondertap_bssid_filter_params params = {};

		if (!params_tb[WONDER_VEN_ATTR_FILTER_BSSID_ENABLED]) {
			wonder_error("BSSID filter missing ENABLED attribute\n");
			return -EINVAL;
		}

		params.enabled = nla_get_u8(
			params_tb[WONDER_VEN_ATTR_FILTER_BSSID_ENABLED]);
		wonder_info("BSSID filter enabled: %s\n", params.enabled ? "true" : "false");
		if (params.enabled) {
			if (!params_tb[WONDER_VEN_ATTR_FILTER_BSSID_ADDR])
				return -EINVAL;
			nla_memcpy(params.bssid,
						params_tb[WONDER_VEN_ATTR_FILTER_BSSID_ADDR],
						ETH_ALEN);
			wonder_info("BSSID filter address: %02X:XX:XX:XX:XX:%02X\n",
				params.bssid[0], params.bssid[5]);
		}
		ret = wondertap_set_bssid_filter(&wonder->wondertap_data, params.bssid);
		break;
	}
	case WONDER_VEN_ATTR_FILTER_TYPE_FRAME: {
		struct wondertap_frame_filter_params params = {};

		if (!params_tb[WONDER_VEN_ATTR_FILTER_FRAME_ENABLED]) {
			wonder_error("Frame filter missing ENABLED attribute\n");
			return -EINVAL;
		}

		params.enabled = nla_get_u8(
			params_tb[WONDER_VEN_ATTR_FILTER_FRAME_ENABLED]);
		wonder_info("Frame filter enabled: %s\n", params.enabled ? "true" : "false");
		if (params.enabled) {
			if (!params_tb[WONDER_VEN_ATTR_FILTER_FRAME_TYPE] ||
				!params_tb[WONDER_VEN_ATTR_FILTER_FRAME_SUBTYPE]) {
				wonder_error(
					"Frame filter is enabled but missing TYPE or SUBTYPE attributes\n");
				return -EINVAL;
			}
			params.frame_type = nla_get_u16(
				params_tb[WONDER_VEN_ATTR_FILTER_FRAME_TYPE]);
			params.frame_subtype = nla_get_u16(
				params_tb[WONDER_VEN_ATTR_FILTER_FRAME_SUBTYPE]);
			wonder_info("Frame filter type=0x%04x, subtype=0x%04x\n",
						params.frame_type, params.frame_subtype);
		}
		ret = wondertap_set_filter(&wonder->wondertap_data,
			WONDERTAP_FILTER_TYPE_FRAME, &params);
		break;
	}
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int wonder_vendor_cmd_set_fixed_tx_rate(struct wiphy *wiphy,
								struct wireless_dev *wdev,
								const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_FIXED_TX_RATE_MAX + 1];
	struct wondertap_fixed_tx_rate_params params = {};

	if (nla_parse(tb, WONDER_VEN_ATTR_FIXED_TX_RATE_MAX, data, data_len,
					wonder_fixed_rate_policy, NULL) < 0) {
		wonder_error("Failed to parse fixed TX rate attributes\n");
		return -EINVAL;
	}

	/* Check that all mandatory attributes are present */
	if (!tb[WONDER_VEN_ATTR_FIXED_TX_RATE_PREAMBLE] || !tb[WONDER_VEN_ATTR_FIXED_TX_RATE_BW] ||
		!tb[WONDER_VEN_ATTR_FIXED_TX_RATE_GI] || !tb[WONDER_VEN_ATTR_FIXED_TX_RATE_NSS] ||
		!tb[WONDER_VEN_ATTR_FIXED_TX_RATE_MCS]) {
		wonder_error("Missing mandatory attributes for fixed TX rate\n");
		return -EINVAL;
	}

	/* Populate the params struct with the received data */
	params.preamble = nla_get_u32(tb[WONDER_VEN_ATTR_FIXED_TX_RATE_PREAMBLE]);
	params.bw = nla_get_u32(tb[WONDER_VEN_ATTR_FIXED_TX_RATE_BW]);
	params.gi = nla_get_u32(tb[WONDER_VEN_ATTR_FIXED_TX_RATE_GI]);
	params.nss = nla_get_u8(tb[WONDER_VEN_ATTR_FIXED_TX_RATE_NSS]);
	params.mcs = nla_get_u8(tb[WONDER_VEN_ATTR_FIXED_TX_RATE_MCS]);

	wonder_info("Set fixed TX rate: preamble=%u, bw=%u, gi=%u, nss=%u, mcs=%u\n",
				params.preamble, params.bw, params.gi, params.nss, params.mcs);

	return wondertap_set_fixed_tx_rate(&wonder->wondertap_data, &params);
}

static int wonder_vendor_cmd_set_tx_rate_test(struct wiphy *wiphy,
								struct wireless_dev *wdev,
								const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_TX_RATE_TEST_MAX + 1];
	struct wondertap_tx_rate_mask_params tx_rate_params = {};

	if (nla_parse(tb, WONDER_VEN_ATTR_TX_RATE_TEST_MAX, data, data_len,
					wonder_tx_rate_mask_policy, NULL) < 0) {
		wonder_error("Failed to parse TX rate mask attributes\n");
		return -EINVAL;
	}

	/* Check that all mandatory attributes are present */
	if (!tb[WONDER_VEN_ATTR_TX_RATE_TEST_PREAMBLE] || !tb[WONDER_VEN_ATTR_TX_RATE_TEST_BW] ||
		!tb[WONDER_VEN_ATTR_TX_RATE_TEST_NSS] || !tb[WONDER_VEN_ATTR_TX_RATE_TEST_MCS]) {
		wonder_error("Missing mandatory attributes for TX rate\n");
		return -EINVAL;
	}

	// Test only
	tx_rate_params.max_preamble = nla_get_u32(tb[WONDER_VEN_ATTR_TX_RATE_TEST_PREAMBLE]);
	tx_rate_params.max_bw = nla_get_u16(tb[WONDER_VEN_ATTR_TX_RATE_TEST_BW]);
	tx_rate_params.max_nss = nla_get_u8(tb[WONDER_VEN_ATTR_TX_RATE_TEST_NSS]);
	tx_rate_params.max_mcs = nla_get_u8(tb[WONDER_VEN_ATTR_TX_RATE_TEST_MCS]);
	wonder_info("Apply TX rate: max_preamble=%u, max_bw=%u, max_nss=%u, max_mcs=%u\n",
		tx_rate_params.max_preamble, tx_rate_params.max_bw, tx_rate_params.max_nss,
		tx_rate_params.max_mcs);
	wondertap_set_tx_rate_mask(&wonder->wondertap_data, &tx_rate_params);

	return 0;
}

static int wonder_vendor_cmd_set_reg(struct wiphy *wiphy,
								struct wireless_dev *wdev,
								const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_REG_MAX + 1];
	const char *country_code;


	if (nla_parse(tb, WONDER_VEN_ATTR_REG_MAX, data, data_len, wonder_set_reg_policy,
			NULL) < 0) {
		wonder_error("Failed to parse attributes\n");
		return -EINVAL;
	}

	/* Check that the mandatory attribute is present */
	if (!tb[WONDER_VEN_ATTR_REG_COUNTRY_CODE]) {
		wonder_error("Missing COUNTRY_CODE attribute\n");
		return -EINVAL;
	}

	/* Retrieve the string from the attribute */
	country_code = nla_data(tb[WONDER_VEN_ATTR_REG_COUNTRY_CODE]);

	wonder_info("Setting regulatory country code to: %s\n", country_code);

	return wondertap_set_reg(&wonder->wondertap_data, country_code);
}

static int wonder_vendor_cmd_get_if_mac_addr(struct wiphy *wiphy,
								struct wireless_dev *wdev,
								const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct sk_buff *skb;
	u8 mac_addr[ETH_ALEN];

	wondertap_get_interface_mac_address(&wonder->wondertap_data, &mac_addr);

	wonder_info("Handling GET_MAC. Found MAC: %02X:XX:XX:XX:XX:%02X\n",
		mac_addr[0], mac_addr[5]);

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, nla_total_size(ETH_ALEN));
	if (!skb) {
		wonder_error("Failed to allocate reply skb\n");
		return -ENOMEM;
	}

	/* Put the MAC address attribute into the skb. */
	if (nla_put(skb, WONDER_VEN_ATTR_IF_ADDR_MAC_ADDR, ETH_ALEN, mac_addr)) {
		pr_err("Failed to put MAC attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

static int wonder_vendor_cmd_get_cap(struct wiphy *wiphy,
	struct wireless_dev *wdev,
	const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct sk_buff *skb;
	const size_t reply_skb_size = sizeof(u32) + sizeof(u8) * 3;
	u8 hw_amsdu = wonder->wondertap_data.cap.bits.amsdu_aggregation;
	u8 hw_ampdu = wonder->wondertap_data.cap.bits.ampdu_aggregation;
	u8 ch_hopping = wonder->wondertap_data.cap.bits.channel_hopping;
	bool is_ibss_mode = wdev->iftype != NL80211_IFTYPE_MONITOR ? true : false;
	u32 mtu_size = is_ibss_mode ? WONDER_IBSS_MODE_MTU_SIZE : INT_MAX;

	pr_info("Handling is_ibss_mode: %d, MTU: %u, HW_AMSDU: %u, HW_AMPDU: %u, CH_HOPPING: %u\n",
		is_ibss_mode, mtu_size, hw_amsdu, hw_ampdu, ch_hopping);

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, nla_total_size(reply_skb_size));
	if (!skb) {
		pr_err("Failed to allocate reply skb\n");
		return -ENOMEM;
	}

	/* Put the MTU length attribute into the skb. */
	if (nla_put(skb, WONDER_VEN_ATTR_CAP_MTU, sizeof(u32), &mtu_size)) {
		pr_err("Failed to put CAP MTU attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	if (nla_put(skb, WONDER_VEN_ATTR_CAP_HW_AMSDU, sizeof(u8), &hw_amsdu)) {
		pr_err("Failed to put CAP HW_AMSDU attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	if (nla_put(skb, WONDER_VEN_ATTR_CAP_HW_AMPDU, sizeof(u8), &hw_ampdu)) {
		pr_err("Failed to put CAP HW_AMPDU attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	if (nla_put(skb, WONDER_VEN_ATTR_CAP_CH_HOPPING, sizeof(u8), &ch_hopping)) {
		pr_err("Failed to put CAP CH_HOPPING attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

static int wonder_vendor_cmd_set_channel_schedule_req(struct wiphy *wiphy,
						      struct wireless_dev *wdev,
						      const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct nlattr *tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_MAX + 1];
	struct channel_schedule_request params = { 0 };
	struct nlattr *nla;
	int rem;
	int i = 0;

	if (nla_parse(tb, WONDER_VEN_ATTR_CHANNEL_SCHEDULE_MAX, data, data_len,
		      NULL, NULL) < 0) {
		wonder_error("Failed to parse channel schedule attributes\n");
		return -EINVAL;
	}

	if (!tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST_LEN] ||
	    !tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_NEXT_IDX] ||
	    !tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_DWELL_TIME] ||
	    !tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST]) {
		wonder_error("Missing mandatory attributes for channel schedule\n");
		return -EINVAL;
	}

	if (tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_TSF_OFFSET]) {
		struct wondertap_capability cap;
		u32 tsf_offset_us;
		u32 mac_tsf;
		int ret;

		wondertap_get_capabilities(&wonder->wondertap_data, &cap);
		if (!cap.bits.channel_hopping) {
			wonder_error("Channel hopping not enabled in capabilities\n");
			return -EOPNOTSUPP;
		}

		ret = wondertap_get_mac_tsf(&wonder->wondertap_data, &mac_tsf);
		if (ret) {
			wonder_error("Failed to get MAC TSF: %d\n", ret);
			return ret;
		}

		tsf_offset_us = nla_get_u32(tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_TSF_OFFSET]);
		params.target_switch_time_tsf =
			mac_tsf + cap.maximum_channel_switch_time_us + tsf_offset_us;
	} else if (tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_SWITCH_TIME]) {
		params.target_switch_time_tsf =
			nla_get_u32(tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_SWITCH_TIME]);
	} else {
		wonder_error(
			"Missing time attribute: need either TSF_OFFSET or SWITCH_TIME\n");
		return -EINVAL;
	}

	params.channel_list_len = nla_get_u8(tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST_LEN]);
	params.next_channel_index = nla_get_u32(tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_NEXT_IDX]);
	params.dwell_time_tu = nla_get_u32(tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_DWELL_TIME]);

	if (params.channel_list_len > 0) {
		params.channel_list = kcalloc(params.channel_list_len,
					      sizeof(*params.channel_list), GFP_KERNEL);
		if (!params.channel_list)
			return -ENOMEM;

		nla_for_each_nested(nla, tb[WONDER_VEN_ATTR_CHANNEL_SCHEDULE_LIST], rem) {
			struct nlattr *entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_MAX + 1];

			if (i >= params.channel_list_len) {
				wonder_error("More entries than specified in channel_list_len\n");
				kfree(params.channel_list);
				return -EINVAL;
			}

			if (nla_parse_nested(entry_tb, WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_MAX, nla,
					     wonder_channel_list_entry_policy, NULL) < 0) {
				wonder_error("Failed to parse channel list entry\n");
				kfree(params.channel_list);
				return -EINVAL;
			}

			if (!entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_BW] ||
			    !entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_ROLE] ||
			    !entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_FREQ]) {
				wonder_error("Missing required attribute in channel list entry\n");
				kfree(params.channel_list);
				return -EINVAL;
			}

			params.channel_list[i].freq =
				nla_get_u32(entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_FREQ]);
			params.channel_list[i].bandwidth =
				nla_get_u32(entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_BW]);
			params.channel_list[i].role =
				nla_get_u32(entry_tb[WONDER_VEN_ATTR_CHANNEL_LIST_ENTRY_ROLE]);
			i++;
		}
	}

	wondertap_channel_schedule_request(&wonder->wondertap_data, &params);

	kfree(params.channel_list);
	return 0;
}

static int wonder_vendor_cmd_get_mac_tsf(struct wiphy *wiphy,
					 struct wireless_dev *wdev,
					 const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct sk_buff *skb;
	u32 mac_tsf;
	int ret;

	ret = wondertap_get_mac_tsf(&wonder->wondertap_data, &mac_tsf);
	if (ret)
		return ret;

	wonder_info("Handling GET_MAC_TSF. Found TSF: %u\n", mac_tsf);

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, nla_total_size(sizeof(u32)));
	if (!skb) {
		wonder_error("Failed to allocate reply skb\n");
		return -ENOMEM;
	}

	if (nla_put_u32(skb, WONDER_VEN_ATTR_MAC_TSF, mac_tsf)) {
		pr_err("Failed to put MAC TSF attribute\n");
		kfree_skb(skb);
		return -EMSGSIZE;
	}

	return cfg80211_vendor_cmd_reply(skb);
}

static int wonder_vendor_cmd_get_channel_status_report(struct wiphy *wiphy,
						       struct wireless_dev *wdev,
						       const void *data, int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct wonder_data *wonder = hw->priv;
	struct wondertap_data *wondertap = &wonder->wondertap_data;
	struct wondertap_channel_status_report *report;
	struct sk_buff *skb;
	struct nlattr *list;
	u32 num_channels;
	size_t size;
	int ret, i;

	mutex_lock(&wondertap->lock);
	num_channels = wondertap->cached_channel_schedule.channel_list_len;
	mutex_unlock(&wondertap->lock);

	if (num_channels == 0) {
		pr_err("Channel hopping list is empty.\n");
		return -EINVAL;
	}

	size = sizeof(*report) + num_channels * sizeof(struct wondertap_channel_status);
	report = kzalloc(size, GFP_KERNEL);
	if (!report)
		return -ENOMEM;

	report->channel_status_len = num_channels;
	ret = wondertap_get_channel_status_report(wondertap, report);
	if (ret) {
		pr_err("Failed to get channel status report: %d\n", ret);
		kfree(report);
		return ret;
	}

	/* Calculate Netlink attribute sizes (header + list + array items) */
	size = nla_total_size(sizeof(u32)) * 3 +
	       nla_total_size(0) +
	       num_channels * (nla_total_size(0) +
			       nla_total_size(sizeof(u32)) * 6 +
			       nla_total_size_64bit(sizeof(u64)) * 2);

	skb = cfg80211_vendor_cmd_alloc_reply_skb(wiphy, size + 100);
	if (!skb) {
		pr_err("Failed to allocate reply skb\n");
		kfree(report);
		return -ENOMEM;
	}

	if (nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_REPORT_REQ_TSF,
			report->current_channel_hopping_request_tsf) ||
	    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_REPORT_CUR_IDX,
			report->current_channel_index) ||
	    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_REPORT_LEN,
			report->channel_status_len))
		goto nla_put_failure;

	list = nla_nest_start(skb, WONDER_VEN_ATTR_CH_STATUS_REPORT_LIST);
	if (!list)
		goto nla_put_failure;

	for (i = 0; i < report->channel_status_len; i++) {
		struct nlattr *entry;
		struct wondertap_channel_status *status = &report->status[i];

		entry = nla_nest_start(skb, i + 1);
		if (!entry)
			goto nla_put_failure;

		if (nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_SWITCH_TSF,
				status->channel_switch_tsf) ||
		    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_FREQ,
				status->freq) ||
		    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_START_TSF,
				status->channel_start_tsf) ||
		    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_END_TSF,
				status->channel_end_tsf) ||
		    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_TX_FRAMES,
				status->tx_frames) ||
		    nla_put_u64_64bit(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_TX_BYTES,
				      status->tx_bytes,
				      WONDER_VEN_ATTR_CH_STATUS_ENTRY_UNSPEC) ||
		    nla_put_u32(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_RX_FRAMES,
				status->rx_frames) ||
		    nla_put_u64_64bit(skb, WONDER_VEN_ATTR_CH_STATUS_ENTRY_RX_BYTES,
				      status->rx_bytes,
				      WONDER_VEN_ATTR_CH_STATUS_ENTRY_UNSPEC))
			goto nla_put_failure;

		nla_nest_end(skb, entry);
	}
	nla_nest_end(skb, list);

	kfree(report);
	return cfg80211_vendor_cmd_reply(skb);

nla_put_failure:
	pr_err("Failed to put channel status report attribute\n");
	kfree_skb(skb);
	kfree(report);
	return -EMSGSIZE;
}

static const struct wiphy_vendor_command wonder_vendor_cmds[] = {
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_FREQUENCY
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_frequency,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_FILTER
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_filter,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_FIXED_TX_RATE
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_fixed_tx_rate,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_TX_RATE_TEST
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_tx_rate_test,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_REGULATORY
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_reg,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_GET_IF_MAC_ADDR
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_get_if_mac_addr,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_GET_CAP
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_get_cap,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_SET_CHANNEL_SCHEDULE_REQ
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_set_channel_schedule_req,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_GET_MAC_TSF
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_get_mac_tsf,
		.policy = VENDOR_CMD_RAW_DATA,
	},
	{
		.info = {
			.vendor_id = WONDER_VENDOR_ID,
			.subcmd = WONDER_VEN_SUBCMD_GET_CHANNEL_STATUS_REPORT
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = wonder_vendor_cmd_get_channel_status_report,
		.policy = VENDOR_CMD_RAW_DATA,
	},
};

const struct wiphy_vendor_command *wonder_get_wiphy_vendor_command(void) {
	return wonder_vendor_cmds;
}

size_t wonder_get_wiphy_vendor_command_array_size(void) {
	return ARRAY_SIZE(wonder_vendor_cmds);
}
