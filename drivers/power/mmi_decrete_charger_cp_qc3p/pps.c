#include "mmi_charger_core.h"

#define PCA_PPS_CMD_RETRY_COUNT	2


static inline int check_typec_attached_snk(struct tcpc_device *tcpc)
{
	if (tcpm_inquire_typec_attach_state(tcpc) != TYPEC_ATTACHED_SNK)
		return -EINVAL;
	return 0;
}

int usbpd_pps_enable_charging(struct mmi_charger_manager *chip, bool en,
				   u32 mV, u32 mA)
{
	int ret, cnt = 0;

	if (check_typec_attached_snk(chip->tcpc) < 0)
		return -EINVAL;
	pr_err("usbpd_pps_enable_charging en = %d, %dmV, %dmA\n", en, mV, mA);

	do {
		if (en)
			ret = tcpm_set_apdo_charging_policy(chip->tcpc,
				DPM_CHARGING_POLICY_PPS, mV, mA, NULL);
		else
			ret = tcpm_reset_pd_charging_policy(chip->tcpc, NULL);
		cnt++;
	} while (ret != TCP_DPM_RET_SUCCESS && cnt < PCA_PPS_CMD_RETRY_COUNT);

	if (ret != TCP_DPM_RET_SUCCESS)
		pr_err("usbpd_pps_enable_charging fail(%d)\n", ret);
	return ret > 0 ? -ret : ret;
}

int usbpd_select_pdo(struct mmi_charger_manager *chip, u32 mV, u32 mA)
{
	int ret, cnt = 0;

	if (check_typec_attached_snk(chip->tcpc) < 0)
		return -EINVAL;
	pr_err("usbpd_select_pdo: %dmV, %dmA\n", mV, mA);

	if (!tcpm_inquire_pd_connected(chip->tcpc)) {
		pr_err("pd not connected\n");
		return -EINVAL;
	}

	do {
		ret = tcpm_dpm_pd_request(chip->tcpc, mV, mA, NULL);
		cnt++;
	} while (ret != TCP_DPM_RET_SUCCESS && cnt < PCA_PPS_CMD_RETRY_COUNT);

	if (ret != TCP_DPM_RET_SUCCESS) {
		pr_err("usbpd_select_pdo: fail(%d)\n", ret);
		return -EINVAL;
	}
	else
		return 0;
}

bool usbpd_get_pps_status_curr_volt(struct mmi_charger_manager *chip)
{
	int ret, apdo_idx = -1;
	struct tcpm_power_cap_val apdo_cap = {0};
	struct tcpm_power_cap_val selected_apdo_cap;
	u8 cap_idx;

	if (check_typec_attached_snk(chip->tcpc) < 0)
		return false;

	if (!chip->is_pps_en_unlock) {
		pr_err("pps en is locked\n");
		return false;
	}

	if (!tcpm_inquire_pd_pe_ready(chip->tcpc)) {
		pr_err("PD PE not ready\n");
		return false;
	}

	cap_idx = 0;
	while (1) {
		ret = tcpm_inquire_pd_source_apdo(chip->tcpc,
						  TCPM_POWER_CAP_APDO_TYPE_PPS,
						  &cap_idx, &apdo_cap);
		if (ret != TCP_DPM_RET_SUCCESS) {
			pr_err("inquire pd apdo fail(%d)\n", ret);
			break;
		}

		pr_err("cap_idx[%d], %d mv ~ %d mv, %d ma, pl: %d\n", cap_idx,
			 apdo_cap.min_mv, apdo_cap.max_mv, apdo_cap.ma,
			 apdo_cap.pwr_limit);

		if (apdo_cap.max_mv < PUMP_CHARGER_PPS_MIN_VOLT / 1000||
			apdo_cap.ma <  chip->typec_middle_current / 1000)
				continue;

		if (apdo_idx == -1 || apdo_cap.ma > selected_apdo_cap.ma) {
			memcpy(&selected_apdo_cap, &apdo_cap,
			       sizeof(struct tcpm_power_cap_val));
			apdo_idx = cap_idx;
			pr_err("select potential cap_idx[%d]\n", cap_idx);
		}
	}

    if (apdo_idx != -1) {
		if (selected_apdo_cap.max_mv* 1000 < pd_volt_max_init) {
			chip->pd_volt_max = selected_apdo_cap.max_mv* 1000;
		} else {
			chip->pd_volt_max = pd_volt_max_init;
		}
		if (selected_apdo_cap.ma* 1000 < pd_curr_max_init) {
			chip->pd_curr_max = selected_apdo_cap.ma * 1000;
		} else {
			chip->pd_curr_max = pd_curr_max_init;
		}
		return true;
    }

    return false;
}

bool usbpd_get_pps_status(struct mmi_charger_manager *chip)
{
	int ret, apdo_idx = -1;
	struct tcpm_power_cap_val apdo_cap = {0};
	struct tcpm_power_cap_val selected_apdo_cap;
	u8 cap_idx;

	if (check_typec_attached_snk(chip->tcpc) < 0)
		return false;

	if (!chip->is_pps_en_unlock) {
		pr_err("pps en is locked\n");
		return false;
	}

	if (!tcpm_inquire_pd_pe_ready(chip->tcpc)) {
		pr_err("PD PE not ready\n");
		return false;
	}

	cap_idx = 0;
	while (1) {
		ret = tcpm_inquire_pd_source_apdo(chip->tcpc,
						  TCPM_POWER_CAP_APDO_TYPE_PPS,
						  &cap_idx, &apdo_cap);
		if (ret != TCP_DPM_RET_SUCCESS) {
			pr_err("inquire pd apdo fail(%d)\n", ret);
			break;
		}

		pr_err("cap_idx[%d], %d mv ~ %d mv, %d ma, pl: %d\n", cap_idx,
			 apdo_cap.min_mv, apdo_cap.max_mv, apdo_cap.ma,
			 apdo_cap.pwr_limit);

		if (apdo_cap.max_mv < PUMP_CHARGER_PPS_MIN_VOLT / 1000||
			apdo_cap.ma <  chip->typec_middle_current / 1000)
				continue;

		if (apdo_idx == -1 || apdo_cap.ma > selected_apdo_cap.ma) {
			memcpy(&selected_apdo_cap, &apdo_cap,
			       sizeof(struct tcpm_power_cap_val));
			apdo_idx = cap_idx;
			pr_err("select potential cap_idx[%d]\n", cap_idx);
		}
	}

    if (apdo_idx != -1) {
		ret = usbpd_pps_enable_charging(chip, true, 5000, 3000);
		if (ret != TCP_DPM_RET_SUCCESS)
			return false;

		if (selected_apdo_cap.max_mv* 1000 <
							chip->pd_volt_max) {
			chip->pd_volt_max =
						selected_apdo_cap.max_mv* 1000;
		}
		if (selected_apdo_cap.ma * 1000 <
							chip->pd_curr_max)
			chip->pd_curr_max =
					selected_apdo_cap.ma * 1000;
		return true;
    }

    return false;
}

static int pca_pps_tcp_notifier_call(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct mmi_charger_manager *chip =
		container_of(nb, struct mmi_charger_manager, tcp_nb);
	struct tcp_notify *noti = data;

	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case PD_CONNECT_NONE:
			pr_err("detached\n");
			chip->is_pps_en_unlock = false;
			chip->hrst_cnt = 0;
			break;
		case PD_CONNECT_HARD_RESET:
			chip->hrst_cnt++;
			pr_err("pd hardreset, cnt = %d\n",
				 chip->hrst_cnt);
			chip->is_pps_en_unlock = false;
			break;
		case PD_CONNECT_PE_READY_SNK_APDO:
			if (chip->hrst_cnt < 5) {
				pr_err("en unlock\n");
				chip->is_pps_en_unlock = true;
			}
			break;
		default:
			break;
		}
	default:
		break;
	}

	if(chip->usb_psy)
		power_supply_changed(chip->usb_psy);

	return NOTIFY_OK;
}


int init_tcpc(struct mmi_charger_manager *chip)
{
	int ret = 0;

	if (!chip->tcpc) {
		chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
		if (!chip->tcpc) {
			pr_err("get tcpc dev fail\n");
			return -ENODEV;
		}
	}

	/* register tcp notifier callback */
	chip->tcp_nb.notifier_call = pca_pps_tcp_notifier_call;
	ret = register_tcp_dev_notifier(chip->tcpc, &chip->tcp_nb,
					TCP_NOTIFY_TYPE_USB);
	if (ret < 0) {
		pr_err("register tcpc notifier fail\n");
		return ret;
	}

	return 0;
}
