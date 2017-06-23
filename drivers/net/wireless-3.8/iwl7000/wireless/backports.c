/*
 * Copyright(c) 2015 - 2017 Intel Deutschland GmbH
 *
 * Backport functionality introduced in Linux 4.4.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "symbols-rename.h"

#include <linux/export.h>
#include <linux/if_vlan.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <net/ip.h>
#include <asm/unaligned.h>
#include <linux/device.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,7,0)
#include <linux/devcoredump.h>

static void devcd_free_sgtable(void *data)
{
	struct scatterlist *table = data;
	int i;
	struct page *page;
	struct scatterlist *iter;
	struct scatterlist *delete_iter;

	/* free pages */
	iter = table;
	for_each_sg(table, iter, sg_nents(table), i) {
		page = sg_page(iter);
		if (page)
			__free_page(page);
	}

	/* then free all chained tables */
	iter = table;
	delete_iter = table;	/* always points on a head of a table */
	while (!sg_is_last(iter)) {
		iter++;
		if (sg_is_chain(iter)) {
			iter = sg_chain_ptr(iter);
			kfree(delete_iter);
			delete_iter = iter;
		}
	}

	/* free the last table */
	kfree(delete_iter);
}

static ssize_t devcd_read_from_sgtable(char *buffer, loff_t offset,
				       size_t buf_len, void *data,
				       size_t data_len)
{
	struct scatterlist *table = data;

	if (offset > data_len)
		return -EINVAL;

	if (offset + buf_len > data_len)
		buf_len = data_len - offset;
	return sg_pcopy_to_buffer(table, sg_nents(table), buffer, buf_len,
				  offset);
}

void dev_coredumpsg(struct device *dev, struct scatterlist *table,
		    size_t datalen, gfp_t gfp)
{
	/*
	 * those casts are needed due to const issues, but it's fine
	 * since calling convention for const/non-const is identical
	 */
	dev_coredumpm(dev, THIS_MODULE, table, datalen, gfp,
		      (void *)devcd_read_from_sgtable,
		      (void *)devcd_free_sgtable);
}
EXPORT_SYMBOL_GPL(dev_coredumpsg);
#endif /* < 4.7.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,6,0)
int kstrtobool(const char *s, bool *res)
{
	if (!s)
		return -EINVAL;

	switch (s[0]) {
	case 'y':
	case 'Y':
	case '1':
		*res = true;
		return 0;
	case 'n':
	case 'N':
	case '0':
		*res = false;
		return 0;
	case 'o':
	case 'O':
		switch (s[1]) {
		case 'n':
		case 'N':
			*res = true;
			return 0;
		case 'f':
		case 'F':
			*res = false;
			return 0;
		default:
			break;
		}
	default:
		break;
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(kstrtobool);

int kstrtobool_from_user(const char __user *s, size_t count, bool *res)
{
	/* Longest string needed to differentiate, newline, terminator */
	char buf[4];

	count = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, s, count))
		return -EFAULT;
	buf[count] = '\0';
	return kstrtobool(buf, res);
}
EXPORT_SYMBOL_GPL(kstrtobool_from_user);
#endif /* < 4.6.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,4,0)
#ifdef CONFIG_DEBUG_FS
static ssize_t _debugfs_read_file_bool(struct file *file, char __user *user_buf,
				       size_t count, loff_t *ppos)
{
	char buf[3];
	bool *val = file->private_data;

	if (*val)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t _debugfs_write_file_bool(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;
	bool bv;
	bool *val = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	buf[buf_size] = '\0';
	if (strtobool(buf, &bv) == 0)
		*val = bv;

	return count;
}

static const struct file_operations fops_bool = {
	.read =		_debugfs_read_file_bool,
	.write =	_debugfs_write_file_bool,
	.open =		simple_open,
	.llseek =	default_llseek,
};

struct dentry *iwl_debugfs_create_bool(const char *name, umode_t mode,
				       struct dentry *parent, bool *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_bool);
}
EXPORT_SYMBOL_GPL(iwl_debugfs_create_bool);

#endif /* CONFIG_DEBUG_FS */

/* Calculate expected number of TX descriptors */
int tso_count_descs(struct sk_buff *skb)
{
	/* The Marvell Way */
	return skb_shinfo(skb)->gso_segs * 2 + skb_shinfo(skb)->nr_frags;
}
EXPORT_SYMBOL(tso_count_descs);

void tso_build_hdr(struct sk_buff *skb, char *hdr, struct tso_t *tso,
		   int size, bool is_last)
{
	struct tcphdr *tcph;
	int hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
	int mac_hdr_len = skb_network_offset(skb);

	memcpy(hdr, skb->data, hdr_len);
	if (!tso->ipv6) {
		struct iphdr *iph = (void *)(hdr + mac_hdr_len);

		iph->id = htons(tso->ip_id);
		iph->tot_len = htons(size + hdr_len - mac_hdr_len);
		tso->ip_id++;
	} else {
		struct ipv6hdr *iph = (void *)(hdr + mac_hdr_len);

		iph->payload_len = htons(size + tcp_hdrlen(skb));
	}
	tcph = (struct tcphdr *)(hdr + skb_transport_offset(skb));
	put_unaligned_be32(tso->tcp_seq, &tcph->seq);

	if (!is_last) {
		/* Clear all special flags for not last packet */
		tcph->psh = 0;
		tcph->fin = 0;
		tcph->rst = 0;
	}
}
EXPORT_SYMBOL(tso_build_hdr);

void tso_build_data(struct sk_buff *skb, struct tso_t *tso, int size)
{
	tso->tcp_seq += size;
	tso->size -= size;
	tso->data += size;

	if ((tso->size == 0) &&
	    (tso->next_frag_idx < skb_shinfo(skb)->nr_frags)) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[tso->next_frag_idx];

		/* Move to next segment */
		tso->size = frag->size;
		tso->data = page_address(frag->page.p) + frag->page_offset;
		tso->next_frag_idx++;
	}
}
EXPORT_SYMBOL(tso_build_data);

void tso_start(struct sk_buff *skb, struct tso_t *tso)
{
	int hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);

	tso->ip_id = ntohs(ip_hdr(skb)->id);
	tso->tcp_seq = ntohl(tcp_hdr(skb)->seq);
	tso->next_frag_idx = 0;
	tso->ipv6 = vlan_get_protocol(skb) == htons(ETH_P_IPV6);

	/* Build first data */
	tso->size = skb_headlen(skb) - hdr_len;
	tso->data = skb->data + hdr_len;
	if ((tso->size == 0) &&
	    (tso->next_frag_idx < skb_shinfo(skb)->nr_frags)) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[tso->next_frag_idx];

		/* Move to next segment */
		tso->size = frag->size;
		tso->data = page_address(frag->page.p) + frag->page_offset;
		tso->next_frag_idx++;
	}
}
EXPORT_SYMBOL(tso_start);

#endif /* < 4.4.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#define NETDEV_RSS_KEY_LEN 40

void netdev_rss_key_fill(void *buffer, size_t len)
{
	static u8 netdev_rss_key[NETDEV_RSS_KEY_LEN];
	static bool initialized;

	BUG_ON(len > sizeof(netdev_rss_key));
	/*
	 * A crude "do once". It's not protected and potentially racy, but we
	 * will get here only on mvm_up which is protected by the rtnl lock.
	 */
	if (!initialized) {
		get_random_bytes(netdev_rss_key, sizeof(netdev_rss_key));
		initialized = true;
	}
	memcpy(buffer, netdev_rss_key, len);
}
EXPORT_SYMBOL_GPL(netdev_rss_key_fill);
#endif /* < 3.19.0 */


#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
/**
 * memdup_user_nul - duplicate memory region from user space and NUL-terminate
 *
 * @src: source address in user space
 * @len: number of bytes to copy
 *
 * Returns an ERR_PTR() on failure.
 */
void *memdup_user_nul(const void __user *src, size_t len)
{
	char *p;

	/*
	 * Always use GFP_KERNEL, since copy_from_user() can sleep and
	 * cause pagefault, which makes it pointless to use GFP_NOFS
	 * or GFP_ATOMIC.
	 */
	p = kmalloc(len + 1, GFP_KERNEL);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}
	p[len] = '\0';

	return p;
}
EXPORT_SYMBOL(memdup_user_nul);
#endif /* < 4.5.0 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
/**
 * devm_kvasprintf - Allocate resource managed space
 *			for the formatted string.
 * @dev: Device to allocate memory for
 * @gfp: the GFP mask used in the devm_kmalloc() call when
 *       allocating memory
 * @fmt: the formatted string to duplicate
 * @ap: the list of tokens to be placed in the formatted string
 * RETURNS:
 * Pointer to allocated string on success, NULL on failure.
 */
char *devm_kvasprintf(struct device *dev, gfp_t gfp, const char *fmt,
		      va_list ap)
{
	unsigned int len;
	char *p;
	va_list aq;

	va_copy(aq, ap);
	len = vsnprintf(NULL, 0, fmt, aq);
	va_end(aq);

	p = devm_kzalloc(dev, len + 1, gfp);
	if (!p)
		return NULL;

	vsnprintf(p, len + 1, fmt, ap);

	return p;
}
EXPORT_SYMBOL_GPL(devm_kvasprintf);

/**
 * devm_kasprintf - Allocate resource managed space
 *		and copy an existing formatted string into that
 * @dev: Device to allocate memory for
 * @gfp: the GFP mask used in the devm_kmalloc() call when
 *       allocating memory
 * @fmt: the string to duplicate
 * RETURNS:
 * Pointer to allocated string on success, NULL on failure.
 */
char *devm_kasprintf(struct device *dev, gfp_t gfp, const char *fmt, ...)
{
	va_list ap;
	char *p;

	va_start(ap, fmt);
	p = devm_kvasprintf(dev, gfp, fmt, ap);
	va_end(ap);

	return p;
}
EXPORT_SYMBOL_GPL(devm_kasprintf);
#endif /* < 3.17 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
#undef genl_register_family
#undef genl_unregister_family

int __backport_genl_register_family(struct genl_family *family)
{
	int i, ret;

#define __copy(_field) family->family._field = family->_field
	__copy(id);
	__copy(hdrsize);
	__copy(version);
	__copy(maxattr);
	strncpy(family->family.name, family->name, sizeof(family->family.name));
	__copy(netnsok);
	__copy(pre_doit);
	__copy(post_doit);
#if LINUX_VERSION_IS_GEQ(3,10,0)
	__copy(parallel_ops);
#endif
#if LINUX_VERSION_IS_GEQ(3,11,0)
	__copy(module);
#endif
#undef __copy

	ret = genl_register_family(&family->family);
	if (ret < 0)
		return ret;

	family->attrbuf = family->family.attrbuf;
	family->id = family->family.id;

	for (i = 0; i < family->n_ops; i++) {
		ret = genl_register_ops(&family->family, &family->ops[i]);
		if (ret < 0)
			goto error;
	}

	for (i = 0; i < family->n_mcgrps; i++) {
		ret = genl_register_mc_group(&family->family,
					     &family->mcgrps[i]);
		if (ret)
			goto error;
	}

	return 0;

 error:
	backport_genl_unregister_family(family);
	return ret;
}
EXPORT_SYMBOL_GPL(__backport_genl_register_family);

int backport_genl_unregister_family(struct genl_family *family)
{
	int err;
	err = genl_unregister_family(&family->family);
	return err;
}
EXPORT_SYMBOL_GPL(backport_genl_unregister_family);
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
static size_t sg_copy_buffer(struct scatterlist *sgl, unsigned int nents,
			     void *buf, size_t buflen, off_t skip,
			     bool to_buffer)
{
	unsigned int offset = 0;
	struct sg_mapping_iter miter;
	unsigned long flags;
	unsigned int sg_flags = SG_MITER_ATOMIC;

	if (to_buffer)
		sg_flags |= SG_MITER_FROM_SG;
	else
		sg_flags |= SG_MITER_TO_SG;

	sg_miter_start(&miter, sgl, nents, sg_flags);

	/* we'd like to call sg_miter_skip here, but it doesn't exist yet ... */

	local_irq_save(flags);

	while (sg_miter_next(&miter) && offset < buflen) {
		unsigned int len;
		unsigned int inner_offs = 0;

		/* so we do some extra bookkeeping here */
		if (miter.length <= skip) {
			skip -= miter.length;
			continue;
		} else if (skip) {
			inner_offs = skip;
			skip = 0;
		}

		len = min(miter.length - inner_offs, buflen - offset);

		if (to_buffer)
			memcpy(buf + offset, miter.addr + inner_offs, len);
		else
			memcpy(miter.addr + inner_offs, buf + offset, len);

		offset += len;
	}

	sg_miter_stop(&miter);

	local_irq_restore(flags);
	return offset;
}

size_t sg_pcopy_from_buffer(struct scatterlist *sgl, unsigned int nents,
			    const void *buf, size_t buflen, off_t skip)
{
	return sg_copy_buffer(sgl, nents, (void *)buf, buflen, skip, false);
}
EXPORT_SYMBOL_GPL(sg_pcopy_from_buffer);

size_t sg_pcopy_to_buffer(struct scatterlist *sgl, unsigned int nents,
			  void *buf, size_t buflen, off_t skip)
{
	return sg_copy_buffer(sgl, nents, buf, buflen, skip, true);
}
EXPORT_SYMBOL_GPL(sg_pcopy_to_buffer);
#endif /* < 3.11 */
