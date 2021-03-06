/*
 * This file is part of wl1251
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Contact: Kalle Valo <kalle.valo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#include "wl1251_netlink.h"

#include <linux/mutex.h>
#include <linux/socket.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <net/genetlink.h>
#include <net/wireless.h>
#include <net/mac80211.h>

#include "wl1251.h"
#include "wl1251_spi.h"
#include "wl1251_acx.h"

/* FIXME: this should be changed as soon as user space catches up */
#define WL1251_NL_NAME "wl1251"
#define WL1251_NL_VERSION 1

#define WL1251_MAX_TEST_LENGTH 1024
#define WL1251_MAX_NVS_LENGTH 1024

enum wl1251_nl_commands {
	WL1251_NL_CMD_UNSPEC,
	WL1251_NL_CMD_TEST,
	WL1251_NL_CMD_INTERROGATE,
	WL1251_NL_CMD_CONFIGURE,
	WL1251_NL_CMD_PHY_REG_READ,
	WL1251_NL_CMD_NVS_PUSH,
	WL1251_NL_CMD_REG_WRITE,
	WL1251_NL_CMD_REG_READ,
	WL1251_NL_CMD_SET_PLT_MODE,

	__WL1251_NL_CMD_AFTER_LAST
};
#define WL1251_NL_CMD_MAX (__WL1251_NL_CMD_AFTER_LAST - 1)

enum wl1251_nl_attrs {
	WL1251_NL_ATTR_UNSPEC,
	WL1251_NL_ATTR_IFNAME,
	WL1251_NL_ATTR_CMD_TEST_PARAM,
	WL1251_NL_ATTR_CMD_TEST_ANSWER,
	WL1251_NL_ATTR_CMD_IE,
	WL1251_NL_ATTR_CMD_IE_LEN,
	WL1251_NL_ATTR_CMD_IE_BUFFER,
	WL1251_NL_ATTR_CMD_IE_ANSWER,
	WL1251_NL_ATTR_REG_ADDR,
	WL1251_NL_ATTR_REG_VAL,
	WL1251_NL_ATTR_NVS_BUFFER,
	WL1251_NL_ATTR_NVS_LEN,
	WL1251_NL_ATTR_PLT_MODE,

	__WL1251_NL_ATTR_AFTER_LAST
};
#define WL1251_NL_ATTR_MAX (__WL1251_NL_ATTR_AFTER_LAST - 1)

static struct genl_family wl1251_nl_family = {
	.id = GENL_ID_GENERATE,
	.name = WL1251_NL_NAME,
	.hdrsize = 0,
	.version = WL1251_NL_VERSION,
	.maxattr = WL1251_NL_ATTR_MAX,
};

static struct net_device *ifname_to_netdev(struct net *net,
					   struct genl_info *info)
{
	char *ifname;

	if (!info->attrs[WL1251_NL_ATTR_IFNAME])
		return NULL;

	ifname = nla_data(info->attrs[WL1251_NL_ATTR_IFNAME]);

	wl1251_debug(DEBUG_NETLINK, "Looking for %s", ifname);

	return dev_get_by_name(net, ifname);
}

static struct wl1251 *ifname_to_wl1251(struct net *net, struct genl_info *info)
{
	struct net_device *netdev;
	struct wireless_dev *wdev;
	struct wiphy *wiphy;
	struct ieee80211_hw *hw;

	netdev = ifname_to_netdev(net, info);
	if (netdev == NULL) {
		wl1251_error("Wrong interface");
		return NULL;
	}

	wdev = netdev->ieee80211_ptr;
	if (wdev == NULL) {
		wl1251_error("ieee80211_ptr is NULL");
		return NULL;
	}

	wiphy = wdev->wiphy;
	if (wiphy == NULL) {
		wl1251_error("wiphy is NULL");
		return NULL;
	}

	hw = wiphy_priv(wiphy);
	if (hw == NULL) {
		wl1251_error("hw is NULL");
		return NULL;
	}

	dev_put(netdev);

	return hw->priv;
}

static int wl1251_nl_test_cmd(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	struct wl1251_command *cmd;
	char *buf;
	int buf_len, ret, cmd_len;
	u8 answer;

	if (!info->attrs[WL1251_NL_ATTR_CMD_TEST_PARAM])
		return -EINVAL;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		return -EINVAL;
	}

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	buf = nla_data(info->attrs[WL1251_NL_ATTR_CMD_TEST_PARAM]);
	buf_len = nla_len(info->attrs[WL1251_NL_ATTR_CMD_TEST_PARAM]);
	answer = nla_get_u8(info->attrs[WL1251_NL_ATTR_CMD_TEST_ANSWER]);

	cmd->header.id = CMD_TEST;
	memcpy(cmd->parameters, buf, buf_len);
	cmd_len = sizeof(struct wl1251_cmd_header) + buf_len;

	mutex_lock(&wl->mutex);
	ret = wl1251_cmd_test(wl, cmd, cmd_len, answer);
	mutex_unlock(&wl->mutex);

	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto out;
	}

	if (answer) {
		struct sk_buff *msg;
		void *hdr;

		msg = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
		if (!msg) {
			ret = -ENOMEM;
			goto out;
		}

		hdr = genlmsg_put(msg, info->snd_pid, info->snd_seq,
				  &wl1251_nl_family, 0, WL1251_NL_CMD_TEST);
		if (IS_ERR(hdr)) {
			ret = PTR_ERR(hdr);
			goto nla_put_failure;
		}

		NLA_PUT_STRING(msg, WL1251_NL_ATTR_IFNAME,
			       nla_data(info->attrs[WL1251_NL_ATTR_IFNAME]));
		NLA_PUT(msg, WL1251_NL_ATTR_CMD_TEST_ANSWER,
			sizeof(*cmd), cmd);

		ret = genlmsg_end(msg, hdr);
		if (ret < 0) {
			wl1251_error("%s() failed", __func__);
			goto nla_put_failure;
		}

		wl1251_debug(DEBUG_NETLINK, "TEST cmd sent, answer");
		ret = genlmsg_reply(msg, info);
		goto out;

 nla_put_failure:
		nlmsg_free(msg);
	} else
		wl1251_debug(DEBUG_NETLINK, "TEST cmd sent");

out:
	kfree(cmd);
	return ret;
}

static int wl1251_nl_interrogate(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	struct sk_buff *msg;
	int ret = -ENOBUFS, cmd_ie, cmd_ie_len;
	struct wl1251_command *cmd;
	void *hdr;

	if (!info->attrs[WL1251_NL_ATTR_CMD_IE])
		return -EINVAL;

	if (!info->attrs[WL1251_NL_ATTR_CMD_IE_LEN])
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	msg = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		ret = -EINVAL;
		goto nla_put_failure;
	}

	/* acx id */
	cmd_ie = nla_get_u32(info->attrs[WL1251_NL_ATTR_CMD_IE]);

	/* maximum length of acx, including all headers */
	cmd_ie_len = nla_get_u32(info->attrs[WL1251_NL_ATTR_CMD_IE_LEN]);

	wl1251_debug(DEBUG_NETLINK, "Getting IE 0x%x (len %d)",
		     cmd_ie, cmd_ie_len);

	mutex_lock(&wl->mutex);
	ret = wl1251_cmd_interrogate(wl, cmd_ie, cmd, cmd_ie_len);
	mutex_unlock(&wl->mutex);

	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}

	hdr = genlmsg_put(msg, info->snd_pid, info->snd_seq,
			  &wl1251_nl_family, 0, WL1251_NL_CMD_INTERROGATE);
	if (IS_ERR(hdr)) {
		ret = PTR_ERR(hdr);
		goto nla_put_failure;
	}

	NLA_PUT_STRING(msg, WL1251_NL_ATTR_IFNAME,
		       nla_data(info->attrs[WL1251_NL_ATTR_IFNAME]));
	NLA_PUT(msg, WL1251_NL_ATTR_CMD_IE_ANSWER, cmd_ie_len, cmd);

	ret = genlmsg_end(msg, hdr);
	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}

	kfree(cmd);
	return genlmsg_reply(msg, info);

 nla_put_failure:
	kfree(cmd);
	nlmsg_free(msg);

	return ret;
}

static int wl1251_nl_configure(struct sk_buff *skb, struct genl_info *info)
{
	int ret = 0, cmd_ie_len, acx_len;
	struct acx_header *acx = NULL;
	struct sk_buff *msg;
	struct wl1251 *wl;
	void *cmd_ie;
	u16 *id;

	if (!info->attrs[WL1251_NL_ATTR_CMD_IE_BUFFER])
		return -EINVAL;

	if (!info->attrs[WL1251_NL_ATTR_CMD_IE_LEN])
		return -EINVAL;

	msg = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		ret = -EINVAL;
		goto nla_put_failure;
	}

	/* contains the acx header but not the cmd header */
	cmd_ie = nla_data(info->attrs[WL1251_NL_ATTR_CMD_IE_BUFFER]);

	cmd_ie_len = nla_get_u32(info->attrs[WL1251_NL_ATTR_CMD_IE_LEN]);

	/* acx id is in the first two bytes */
	id = cmd_ie;

	/* need to add acx_header before cmd_ie, so create a new command */
	acx_len = sizeof(struct acx_header) + cmd_ie_len;
	acx = kzalloc(acx_len, GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto nla_put_failure;
	}

	/* copy the acx header and the payload */
	memcpy(&acx->id, cmd_ie, cmd_ie_len);

	mutex_lock(&wl->mutex);
	ret = wl1251_cmd_configure(wl, *id, acx, acx_len);
	mutex_unlock(&wl->mutex);

	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}

	wl1251_debug(DEBUG_NETLINK, "CONFIGURE cmd sent");

 nla_put_failure:
	kfree(acx);
	nlmsg_free(msg);

	return ret;
}

static int wl1251_nl_phy_reg_read(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	struct sk_buff *msg;
	u32 reg_addr, *reg_value = NULL;
	int ret = 0;
	void *hdr;

	if (!info->attrs[WL1251_NL_ATTR_REG_ADDR])
		return -EINVAL;

	msg = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		ret = -EINVAL;
		goto nla_put_failure;
	}

	reg_value = kmalloc(sizeof(*reg_value), GFP_KERNEL);
	if (!reg_value) {
		ret = -ENOMEM;
		goto nla_put_failure;
	}

	reg_addr = nla_get_u32(info->attrs[WL1251_NL_ATTR_REG_ADDR]);

	wl1251_debug(DEBUG_NETLINK, "Reading PHY reg 0x%x", reg_addr);

	mutex_lock(&wl->mutex);
	ret = wl1251_cmd_read_memory(wl, reg_addr, reg_value,
				     sizeof(*reg_value));
	mutex_unlock(&wl->mutex);

	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}


	hdr = genlmsg_put(msg, info->snd_pid, info->snd_seq,
			  &wl1251_nl_family, 0, WL1251_NL_CMD_PHY_REG_READ);
	if (IS_ERR(hdr)) {
		ret = PTR_ERR(hdr);
		goto nla_put_failure;
	}

	NLA_PUT_STRING(msg, WL1251_NL_ATTR_IFNAME,
		       nla_data(info->attrs[WL1251_NL_ATTR_IFNAME]));

	NLA_PUT_U32(msg, WL1251_NL_ATTR_REG_VAL, *reg_value);

	ret = genlmsg_end(msg, hdr);
	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}

	kfree(reg_value);

	return genlmsg_reply(msg, info);

 nla_put_failure:
	nlmsg_free(msg);
	kfree(reg_value);

	return ret;
}

static int wl1251_nl_nvs_push(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	int ret = 0;

	if (!info->attrs[WL1251_NL_ATTR_NVS_BUFFER])
		return -EINVAL;

	if (!info->attrs[WL1251_NL_ATTR_NVS_LEN])
		return -EINVAL;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		return -EINVAL;
	}

	mutex_lock(&wl->mutex);
	wl->nvs_len = nla_get_u32(info->attrs[WL1251_NL_ATTR_NVS_LEN]);
	if (wl->nvs_len % 4) {
		wl1251_error("NVS size is not multiple of 32: %d", wl->nvs_len);
		ret = -EILSEQ;
		goto out;
	}

	/* If we already have an NVS, we should free it */
	kfree(wl->nvs);

	wl->nvs = kzalloc(wl->nvs_len, GFP_KERNEL);
	if (wl->nvs == NULL) {
		wl1251_error("Can't allocate NVS");
		ret = -ENOMEM;
		goto out;
	}

	memcpy(wl->nvs,
	       nla_data(info->attrs[WL1251_NL_ATTR_NVS_BUFFER]),
	       wl->nvs_len);

	wl1251_debug(DEBUG_NETLINK, "got NVS from userspace, %d bytes",
		     wl->nvs_len);

out:
	mutex_unlock(&wl->mutex);

	return ret;
}

static int wl1251_nl_reg_read(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	u32 addr, val;
	int ret = 0;
	struct sk_buff *msg;
	void *hdr;

	if (!info->attrs[WL1251_NL_ATTR_REG_ADDR])
		return -EINVAL;

	msg = nlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		return -EINVAL;
	}

	addr = nla_get_u32(info->attrs[WL1251_NL_ATTR_REG_ADDR]);

	mutex_lock(&wl->mutex);
	val = wl1251_reg_read32(wl, addr);
	mutex_unlock(&wl->mutex);

	hdr = genlmsg_put(msg, info->snd_pid, info->snd_seq,
			  &wl1251_nl_family, 0, WL1251_NL_CMD_PHY_REG_READ);
	if (IS_ERR(hdr)) {
		ret = PTR_ERR(hdr);
		goto nla_put_failure;
	}

	NLA_PUT_STRING(msg, WL1251_NL_ATTR_IFNAME,
		       nla_data(info->attrs[WL1251_NL_ATTR_IFNAME]));

	NLA_PUT_U32(msg, WL1251_NL_ATTR_REG_VAL, val);

	ret = genlmsg_end(msg, hdr);
	if (ret < 0) {
		wl1251_error("%s() failed", __func__);
		goto nla_put_failure;
	}

	return genlmsg_reply(msg, info);

 nla_put_failure:
	nlmsg_free(msg);

	return ret;
}

static int wl1251_nl_reg_write(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	u32 addr, val;

	if (!info->attrs[WL1251_NL_ATTR_REG_ADDR])
		return -EINVAL;

	if (!info->attrs[WL1251_NL_ATTR_REG_VAL])
		return -EINVAL;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		return -EINVAL;
	}

	addr = nla_get_u32(info->attrs[WL1251_NL_ATTR_REG_ADDR]);
	val = nla_get_u32(info->attrs[WL1251_NL_ATTR_REG_VAL]);

	mutex_lock(&wl->mutex);
	wl1251_reg_write32(wl, addr, val);
	mutex_unlock(&wl->mutex);

	return 0;
}

static int wl1251_nl_set_plt_mode(struct sk_buff *skb, struct genl_info *info)
{
	struct wl1251 *wl;
	u32 val;
	int ret;

	if (!info->attrs[WL1251_NL_ATTR_PLT_MODE])
		return -EINVAL;

	wl = ifname_to_wl1251(&init_net, info);
	if (wl == NULL) {
		wl1251_error("wl1251 not found");
		return -EINVAL;
	}

	val = nla_get_u32(info->attrs[WL1251_NL_ATTR_PLT_MODE]);

	switch (val) {
	case 0:
		ret = wl1251_plt_stop(wl);
		break;
	case 1:
		ret = wl1251_plt_start(wl);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static struct nla_policy wl1251_nl_policy[WL1251_NL_ATTR_MAX + 1] = {
	[WL1251_NL_ATTR_IFNAME] =            { .type = NLA_NUL_STRING,
					       .len = IFNAMSIZ-1 },
	[WL1251_NL_ATTR_CMD_TEST_PARAM] =    { .type = NLA_BINARY,
					       .len = WL1251_MAX_TEST_LENGTH },
	[WL1251_NL_ATTR_CMD_TEST_ANSWER] =   { .type = NLA_U8 },
	[WL1251_NL_ATTR_CMD_IE] =            { .type = NLA_U32 },
	[WL1251_NL_ATTR_CMD_IE_LEN] =        { .type = NLA_U32 },
	[WL1251_NL_ATTR_CMD_IE_BUFFER] =     { .type = NLA_BINARY,
					       .len = WL1251_MAX_TEST_LENGTH },
	[WL1251_NL_ATTR_CMD_IE_ANSWER] =     { .type = NLA_BINARY,
					       .len = WL1251_MAX_TEST_LENGTH },
	[WL1251_NL_ATTR_REG_ADDR] =          { .type = NLA_U32 },
	[WL1251_NL_ATTR_REG_VAL] =           { .type = NLA_U32 },
	[WL1251_NL_ATTR_NVS_BUFFER] =        { .type = NLA_BINARY,
					       .len = WL1251_MAX_NVS_LENGTH },
	[WL1251_NL_ATTR_NVS_LEN] =           { .type = NLA_U32 },
	[WL1251_NL_ATTR_PLT_MODE] =          { .type = NLA_U32 },
};

static struct genl_ops wl1251_nl_ops[] = {
	{
		.cmd = WL1251_NL_CMD_TEST,
		.doit = wl1251_nl_test_cmd,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_INTERROGATE,
		.doit = wl1251_nl_interrogate,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_CONFIGURE,
		.doit = wl1251_nl_configure,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_PHY_REG_READ,
		.doit = wl1251_nl_phy_reg_read,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_NVS_PUSH,
		.doit = wl1251_nl_nvs_push,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_REG_WRITE,
		.doit = wl1251_nl_reg_write,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_REG_READ,
		.doit = wl1251_nl_reg_read,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = WL1251_NL_CMD_SET_PLT_MODE,
		.doit = wl1251_nl_set_plt_mode,
		.policy = wl1251_nl_policy,
		.flags = GENL_ADMIN_PERM,
	},
};

int wl1251_nl_register(void)
{
	int err, i;

	err = genl_register_family(&wl1251_nl_family);
	if (err)
		return err;

	for (i = 0; i < ARRAY_SIZE(wl1251_nl_ops); i++) {
		err = genl_register_ops(&wl1251_nl_family, &wl1251_nl_ops[i]);
		if (err)
			goto err_out;
	}
	return 0;
 err_out:
	genl_unregister_family(&wl1251_nl_family);
	return err;
}

void wl1251_nl_unregister(void)
{
	genl_unregister_family(&wl1251_nl_family);
}
