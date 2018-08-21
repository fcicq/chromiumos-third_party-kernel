/*
 * Linux Security Module for Chromium OS
 *
 * Copyright 2011 Google Inc. All Rights Reserved
 *
 * Authors:
 *      Stephan Uphoff  <ups@google.com>
 *      Kees Cook       <keescook@chromium.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "Chromium OS LSM: " fmt

#include <linux/module.h>
#include <linux/security.h>
#include <linux/sched.h>	/* current and other task related stuff */
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/mount.h>
#include <linux/path.h>

#include "inode_mark.h"
#include "utils.h"

static void report(const char *origin, struct path *path, char *operation)
{
	char *alloced = NULL, *cmdline;
	char *pathname; /* Pointer to either static string or "alloced". */

	if (!path)
		pathname = "<unknown>";
	else {
		/* We will allow 11 spaces for ' (deleted)' to be appended */
		alloced = pathname = kmalloc(PATH_MAX+11, GFP_KERNEL);
		if (!pathname)
			pathname = "<no_memory>";
		else {
			pathname = d_path(path, pathname, PATH_MAX+11);
			if (IS_ERR(pathname))
				pathname = "<too_long>";
			else {
				pathname = printable(pathname);
				kfree(alloced);
				alloced = pathname;
			}
		}
	}

	cmdline = printable_cmdline(current);

	pr_notice("%s %s obj=%s pid=%d cmdline=%s\n", origin,
		  operation, pathname, task_pid_nr(current), cmdline);

	kfree(cmdline);
	kfree(alloced);
}

int chromiumos_security_sb_mount(const char *dev_name, struct path *path,
				 const char *type, unsigned long flags,
				 void *data)
{
#ifdef CONFIG_SECURITY_CHROMIUMOS_NO_SYMLINK_MOUNT
	if (current->total_link_count) {
		report("sb_mount", path, "Mount path with symlinks prohibited");
		pr_notice("sb_mount dev=%s type=%s flags=%#lx\n",
			  dev_name, type, flags);
		return -ELOOP;
	}
#endif

	if ((!(flags & (MS_BIND | MS_MOVE | MS_SHARED | MS_PRIVATE | MS_SLAVE |
			MS_UNBINDABLE)) ||
	     ((flags & MS_REMOUNT) && (flags & MS_BIND))) &&
	    !capable(CAP_SYS_ADMIN)) {
		int required_mnt_flags = MNT_NOEXEC | MNT_NOSUID | MNT_NODEV;

		if (flags & MS_REMOUNT) {
			/*
			 * If this is a remount, we only require that the
			 * requested flags are a superset of the original mount
			 * flags.
			 */
			required_mnt_flags &= path->mnt->mnt_flags;
		}
		/*
		 * The three flags we are interested in disallowing in
		 * unprivileged user namespaces (MS_NOEXEC, MS_NOSUID, MS_NODEV)
		 * cannot be modified when doing a bind-mount. The kernel
		 * attempts to dispatch calls to do_mount() within
		 * fs/namespace.c in the following order:
		 *
		 * * If the MS_REMOUNT flag is present, it calls do_remount().
		 *   When MS_BIND is also present, it only allows to modify the
		 *   per-mount flags, which are copied into
		 *   |required_mnt_flags|.  Otherwise it bails in the absence of
		 *   the CAP_SYS_ADMIN in the init ns.
		 * * If the MS_BIND flag is present, the only other flag checked
		 *   is MS_REC.
		 * * If any of the mount propagation flags are present
		 *   (MS_SHARED, MS_PRIVATE, MS_SLAVE, MS_UNBINDABLE),
		 *   flags_to_propagation_type() filters out any additional
		 *   flags.
		 * * If MS_MOVE flag is present, all other flags are ignored.
		 */
		if ((required_mnt_flags & MNT_NOEXEC) && !(flags & MS_NOEXEC)) {
			report("sb_mount", path,
			       "Mounting a filesystem with 'exec' flag requires CAP_SYS_ADMIN in init ns");
			pr_notice("sb_mount dev=%s type=%s flags=%#lx\n",
				  dev_name, type, flags);
			return -EPERM;
		}
		if ((required_mnt_flags & MNT_NOSUID) && !(flags & MS_NOSUID)) {
			report("sb_mount", path,
			       "Mounting a filesystem with 'suid' flag requires CAP_SYS_ADMIN in init ns");
			pr_notice("sb_mount dev=%s type=%s flags=%#lx\n",
				  dev_name, type, flags);
			return -EPERM;
		}
		if ((required_mnt_flags & MNT_NODEV) && !(flags & MS_NODEV) &&
		    strcmp(type, "devpts")) {
			report("sb_mount", path,
			       "Mounting a filesystem with 'dev' flag requires CAP_SYS_ADMIN in init ns");
			pr_notice("sb_mount dev=%s type=%s flags=%#lx\n",
				  dev_name, type, flags);
			return -EPERM;
		}
	}

	return 0;
}

static int module_locking = 1;
static struct vfsmount *locked_root;
static DEFINE_SPINLOCK(locked_root_spinlock);

#ifdef CONFIG_SYSCTL
static int zero;
static int one = 1;

static struct ctl_path chromiumos_sysctl_path[] = {
	{ .procname = "kernel", },
	{ .procname = "chromiumos", },
	{ }
};

static struct ctl_table chromiumos_sysctl_table[] = {
	{
		.procname       = "module_locking",
		.data           = &module_locking,
		.maxlen         = sizeof(int),
		.mode           = 0644,
		.proc_handler   = proc_dointvec_minmax,
		.extra1         = &zero,
		.extra2         = &one,
	},
	{ }
};

/*
 * This must be called after early kernel init, since then the rootdev
 * is available.
 */
static void check_locking_enforcement(struct vfsmount *mnt)
{
	bool ro;

	/*
	 * If module locking is not enforced via a read-only block
	 * device, allow sysctl to change modes for testing.
	 */
	if (mnt->mnt_sb->s_bdev) {
		ro = bdev_read_only(mnt->mnt_sb->s_bdev);
		pr_info("dev(%u,%u): %s\n",
			MAJOR(mnt->mnt_sb->s_bdev->bd_dev),
			MINOR(mnt->mnt_sb->s_bdev->bd_dev),
			ro ? "read-only" : "writable");
	} else {
		/*
		 * In the weird case where there is no underlying block device
		 * (e.g. tmpfs), assume it is read-only.
		 */
		ro = 1;
		pr_info("dev(?,?): No s_bdev, assuming read-only.\n");
	}

	if (!ro) {
		if (!register_sysctl_paths(chromiumos_sysctl_path,
					   chromiumos_sysctl_table))
			pr_notice("sysctl registration failed!\n");
		else
			pr_info("module locking can be disabled.\n");
	} else
		pr_info("module locking engaged.\n");
}
#else
static void check_locking_enforcement(void) { }
#endif

int chromiumos_security_sb_umount(struct vfsmount *mnt, int flags)
{
	/*
	 * When unmounting the filesystem we were using for module
	 * pinning, we must release our reservation, but make sure
	 * no other modules can be loaded.
	 */
	if (!IS_ERR_OR_NULL(locked_root) && mnt == locked_root) {
		mntput(locked_root);
		locked_root = ERR_PTR(-EIO);
		pr_info("umount pinned fs: refusing further module loads\n");
	}

	return 0;
}

static int check_pinning(const char *origin, struct file *file)
{
	struct vfsmount *module_root;

	if (!file) {
		if (!module_locking) {
			report(origin, NULL, "old-api-locking-ignored");
			return 0;
		}

		report(origin, NULL, "old-api-denied");
		return -EPERM;
	}

	module_root = file->f_path.mnt;

	/* First loaded module defines the root for all others. */
	spin_lock(&locked_root_spinlock);
	/*
	 * locked_root is only NULL at startup. Otherwise, it is either
	 * a valid reference, or an ERR_PTR.
	 */
	if (!locked_root) {
		locked_root = mntget(module_root);
		/*
		 * Unlock now since it's only locked_root we care about.
		 * In the worst case, we will (correctly) report locking
		 * failures before we have announced that locking is
		 * enabled. This would be purely cosmetic.
		 */
		spin_unlock(&locked_root_spinlock);
		check_locking_enforcement(locked_root);
		report(origin, &file->f_path, "locked");
	} else {
		spin_unlock(&locked_root_spinlock);
	}

	if (IS_ERR_OR_NULL(locked_root) || module_root != locked_root) {
		if (unlikely(!module_locking)) {
			report(origin, &file->f_path, "locking-ignored");
			return 0;
		}

		report(origin, &file->f_path, "denied");
		return -EPERM;
	}

	return 0;
}

int chromiumos_security_load_module(struct file *file)
{
	return check_pinning("init_module", file);
}

int chromiumos_security_load_firmware(struct file *file, char *buf, size_t size)
{
	return check_pinning("request_firmware", file);
}

int chromiumos_security_inode_follow_link(struct dentry *dentry,
					  struct nameidata *nd)
{
	static char accessed_path[PATH_MAX];
	enum chromiumos_inode_security_policy policy;

	policy = chromiumos_get_inode_security_policy(
		dentry,
		CHROMIUMOS_SYMLINK_TRAVERSAL);
	/*
	 * Emit a warning in cases of blocked symlink traversal attempts. These
	 * will show up in kernel warning reports collected by the crash
	 * reporter, so we have some insight on spurious failures that need
	 * addressing.
	 */
	WARN(policy == CHROMIUMOS_INODE_POLICY_BLOCK,
	     "Blocked symlink traversal for path %x:%x:%s (see https://goo.gl/8xICW6 for context and rationale)\n",
	     MAJOR(dentry->d_sb->s_dev), MINOR(dentry->d_sb->s_dev),
	     dentry_path(dentry, accessed_path, PATH_MAX));

	return policy == CHROMIUMOS_INODE_POLICY_BLOCK ? -EACCES : 0;
}

int chromiumos_security_file_open(
	struct file *file,
	const struct cred *cred)
{
	static char accessed_path[PATH_MAX];
	enum chromiumos_inode_security_policy policy;
	struct dentry *dentry = file->f_path.dentry;

	/* Returns 0 if file is not a FIFO */
	if (!S_ISFIFO(file->f_inode->i_mode))
		return 0;

	policy = chromiumos_get_inode_security_policy(
		dentry,
		CHROMIUMOS_FIFO_ACCESS);

	/*
	 * Emit a warning in cases of blocked fifo access attempts. These will
	 * show up in kernel warning reports collected by the crash reporter,
	 * so we have some insight on spurious failures that need addressing.
	 */
	WARN(policy == CHROMIUMOS_INODE_POLICY_BLOCK,
	     "Blocked fifo access for path %x:%x:%s\n (see https://goo.gl/8xICW6 for context and rationale)\n",
	     MAJOR(dentry->d_sb->s_dev), MINOR(dentry->d_sb->s_dev),
	     dentry_path(dentry, accessed_path, PATH_MAX));

	return policy == CHROMIUMOS_INODE_POLICY_BLOCK ? -EACCES : 0;
}

static int __init chromiumos_security_init(void)
{
	pr_info("enabled");

	return 0;
}
security_initcall(chromiumos_security_init);

/* Should not be mutable after boot, so not listed in sysfs (perm == 0). */
module_param(module_locking, int, 0);
MODULE_PARM_DESC(module_locking, "Module loading restrictions (default: true)");
