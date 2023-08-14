#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include "../../../sound/core/pcm_local.h"
#include "bcm2835-dma.min.h"
#include "audio_over_ether.h"

MODULE_DESCRIPTION("Audio over Ether Sound Driver for BCM2835");
MODULE_AUTHOR("K.Yoshioka");
MODULE_LICENSE("GPL");

#define PROC_NAME_AOEBUF	"aoe_buf"
#define PROC_NAME_VITAL		"aoe_vital"
#define SRC_SWAP_THRESHOLD	2

/* Externed variables in pcm_native.c */
extern struct snd_pcm_substream *act_substream;

/* Externed variables in bcm2835-dma.c
 * To execute the functions of this module within a cyclic callback,
 * update the function pointer defined in the DMA driver from this driver.
 */
extern void (*bcm2835_cyclic_callback_ptr)(struct bcm2835_chan *c);
extern void (*bcm2835_dma_terminate_ptr)(struct bcm2835_chan *c);

/* tasklet and arg */
struct callback_args {
	struct bcm2835_chan * chan;
	bool scheduled;
};
static struct callback_args dma_callback_arg;
static struct tasklet_struct dma_callback_task;
static struct tasklet_struct dma_terminate_task;

/* netmap */
struct lut_entry {
	void *vaddr;	/* virtual address. */
};
/* mmap */
struct mmap_info {
    char *data;
};

/* EXPORT for netmap_generic.c and netmap_kern.h */
struct ext_slot *ext;
struct lut_entry ext_lut[AOE_NUM_SLOTS];
struct snd_dma_buffer *aoe_buf;
EXPORT_SYMBOL(ext);
EXPORT_SYMBOL(ext_lut);
EXPORT_SYMBOL(aoe_buf);

struct server_stats {
	struct snd_pcm_substream * substream;
	int cb_cur_prediction;
	unsigned long cb_index_error_count;
	unsigned long tasklet_duplicate_error_count;
};
static struct server_stats g = {NULL, 0, 0, 0};

static int cb_head;
static int cb_tail;
static int cb_cur;

/* netmap extend slot */
dma_addr_t idx_to_addr (unsigned long idx)
{
	return aoe_buf->addr + NM_BUFSZ * AOE_BUF_IDX(idx)
		+ sizeof(struct ether_header) + sizeof(struct aoe_header);
}
const void * idx_to_area (unsigned long idx)
{
	return (const char *)aoe_buf->area + NM_BUFSZ * AOE_BUF_IDX(idx)
		+ sizeof(struct ether_header) + sizeof(struct aoe_header);
}

/*
 * Identify the currently executing control block and return its index.
 */
int seek_cb_cur(struct bcm2835_chan *c)
{
	if (!c) {
		pr_info("seek_cb_cur chan is null!\n");
	} else if (!c->desc) {
		pr_info("seek_cb_cur c->desc is null!\n");
	}
	int ret = -1;
	unsigned long flags;
	struct bcm2835_desc *d = c->desc;
	spin_lock_irqsave(&c->vc.lock, flags);

	/* update hw_ptr */
	void __iomem *base  = c->chan_base + BCM2835_DMA_SOURCE_AD;
	dma_addr_t hwaddr = readl(base);

	unsigned int i;
	for (i = 0; i < d->frames; i++) {
		struct bcm2835_dma_cb *control_block = d->cb_list[i].cb;
		size_t this_size = control_block->length;
		dma_addr_t dma = control_block->src;
		if (dma <= hwaddr && hwaddr < dma + this_size) {
			ret = i;
			break;
		}
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
	return ret;
}

/*
 * Update the appl_ptr
 */
int apply_appl_ptr(struct snd_pcm_substream *substream,snd_pcm_uframes_t appl_ptr)
{
	if (appl_ptr >= substream->runtime->boundary)
		appl_ptr -= substream->runtime->boundary;

	snd_pcm_uframes_t old_appl_ptr = substream->runtime->control->appl_ptr;
	if (old_appl_ptr == appl_ptr)
		return 0;

	substream->runtime->control->appl_ptr = appl_ptr;
	if (substream->ops->ack) {
		int ret;
		ret = substream->ops->ack(substream);
		if (ret < 0) {
			pr_info("error! appl_ptr update has failed");
			substream->runtime->control->appl_ptr = old_appl_ptr;
			return ret;
		}
	}

	return 0;
}

/*
 * Set the address of the netmap buffer to the "src" in the control block,
 * and update the "cur" and "tail" of the netmap buffer.
 */
static inline void aoebuf_swap(struct bcm2835_chan * c)
{
	const struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	const struct snd_pcm_mmap_status * __restrict__ status = runtime->status;

	if(likely(ext && c->desc && status->state == SNDRV_PCM_STATE_RUNNING)) {
		const int _tail = ext->tail;
		const int _head = smp_load_acquire(&ext->head);
		int _cur = ext->cur;

		/* Get the precise index of cb_list and correct it. */
		const int cb_cur_backup = cb_cur;
		cb_cur = seek_cb_cur(c);
		if (cb_cur != g.cb_cur_prediction) {
			pr_info("cb index error! cb_cur:%d->%d cb_cur_prediction:%d\n", cb_cur_backup, cb_cur, g.cb_cur_prediction);
			g.cb_index_error_count++;
		}

		const int cb_played = CIRC_CNT(cb_cur, cb_tail, c->desc->frames);
		cb_tail = cb_cur;

		// update ext->tail
		smp_store_release(&ext->tail, CIRC_INDEX(_tail + cb_played, ext->num_slots));

		int ext_playable = CIRC_CNT(_head, _cur, ext->num_slots);
		int cb_space = CIRC_SPACE(cb_head, cb_tail, c->desc->frames);

		if (ext_playable > 0 && cb_space > 0) {
			int i;
			for (i = 0; i < cb_space; i++) {
				if (ext_playable == 0)
					break;

				c->desc->cb_list[cb_head].cb->src = idx_to_addr(ext->slot[_cur].buf_idx);
				cb_head = CIRC_INDEX(cb_head+1, c->desc->frames);
				_cur = CIRC_INDEX(_cur+1, ext->num_slots);

				ext_playable--;
			}
			/* update appl_ptr */
			if (i > 0) {
				int appl = runtime->control->appl_ptr + runtime->period_size * i;
				apply_appl_ptr(g.substream, appl);
			}
			/* update ext->cur */
			smp_store_release(&ext->cur, _cur);
		}
	}
}

void dma_callback_func(unsigned long data)
{
	struct callback_args *arg = (struct callback_args *)data;
	struct bcm2835_chan *c = arg->chan;
	aoebuf_swap(c);
	if (!c->desc) {
		pr_info("callback: c->desc is null!\n");
	}
	if (!ext) {
		pr_info("callback: ext is null!\n");
	}

	dma_callback_arg.scheduled = false;
}

/*
 * The argument (bcm2835_desc) of this callback has already been checked for null. Therefore,
 * it is guaranteed to not be null.
 */
void rtalsa_cyclic_callback(struct bcm2835_chan *c)
{
	g.cb_cur_prediction = CIRC_INDEX(g.cb_cur_prediction + 1, c->desc->frames);

	const int _cb_cur = g.cb_cur_prediction;
	const int _cb_head = cb_head;
	const int cb_cnt = CIRC_CNT(_cb_head, _cb_cur, c->desc->frames);

	if (dma_callback_arg.scheduled) {
		g.tasklet_duplicate_error_count++;
	} else if (cb_cnt <= SRC_SWAP_THRESHOLD) {
		dma_callback_arg.chan = c;
		dma_callback_arg.scheduled = true;
		tasklet_hi_schedule(&dma_callback_task);
	}

	return;
}

void local_pcm_stop(snd_pcm_state_t state)
{
	if (ext->stat == ACTIVE) {
		snd_pcm_stop(g.substream, state);

		/* reset ext->tail = ext->head = ext->cur */
		smp_store_release(&ext->cur, ext->head);
		smp_store_release(&ext->tail, ext->head);
		ext->stat = INACTIVE;
	}
}

/*
 * The callback is invoked at the end of DMA, stopping the PCM.
 */
void dma_terminate_func(unsigned long data)
{
	local_pcm_stop(SNDRV_PCM_STATE_XRUN);
	pr_info("callback... bcm2835_dma_terminate snd_pcm_stop done! (state:%d)\n"
		, g.substream->runtime->status->state);
}
void rtalsa_dma_terminate(struct bcm2835_chan *c)
{
	tasklet_schedule(&dma_terminate_task);
}

/* ioctl aoe_buf */
static long ioctl_ops(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	const struct snd_pcm_mmap_status * __restrict__ status = runtime->status;
	struct dmaengine_pcm_runtime_data *prtd = runtime->private_data;
	struct bcm2835_chan *c = container_of(prtd->dma_chan, struct bcm2835_chan, vc.chan);


	if (!bcm2835_cyclic_callback_ptr)
		pr_info("bcm2835_cyclic_callback_ptr is null!\n");
	if (!bcm2835_dma_terminate_ptr)
		pr_info("bcm2835_dma_terminate_ptr is null!\n");

	switch (cmd) {
	case IOCTL_PCM_START:
		if (runtime->status->state == SNDRV_PCM_STATE_XRUN) {
			/* prepare */
			snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
		}

		if (status->state == SNDRV_PCM_STATE_PREPARED) {
			/* snd_pcm_start */
			int appl = runtime->control->appl_ptr + (snd_pcm_sframes_t)runtime->start_threshold;
			apply_appl_ptr(g.substream, appl);

			int err;
			err = snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_START, NULL);
			if (err < 0)
				pr_info("ioctl... 2 NG! snd_pcm_start error!?\n");
			if(c->desc) {
				pr_info("ioctl   ... IOCTL_PCM_START (state:%d)\n", runtime->status->state);
			} else {
				pr_info("ioctl... 3 NG! dma not started!!\n");
			}
		} else {
			pr_info("ioctl... NG! state is not PREPARED!\n");
		}

		// initialize index
		cb_cur = seek_cb_cur(c);
		g.cb_cur_prediction = cb_cur;
		cb_tail = cb_cur;
		cb_head = CIRC_INDEX(cb_cur + 1, c->desc->frames); // write this point!
		smp_store_release(&ext->cur, ext->tail);
		int init_appl = runtime->hw_ptr_base + runtime->period_size * (cb_cur + 1);
		apply_appl_ptr(g.substream, init_appl);

		// first swap!
		aoebuf_swap(c);
		ext->stat = ACTIVE;

		ret = 0;
		break;
	case IOCTL_PCM_STOP:
		/* tasklet kill */
		tasklet_kill(&dma_callback_task);
		tasklet_kill(&dma_terminate_task);
		/* pcm stop */
		pr_info("ioctl... snd_pcm_stop!\n");
		local_pcm_stop(SNDRV_PCM_STATE_XRUN);
		pr_info("	snd_pcm_stop done! (state:%d)\n", g.substream->runtime->status->state);
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/* mmap aoe_buf, aoe_buf_ext */
static struct snd_dma_buffer * build_dmab(struct file *filp, size_t size)
{
	struct mmap_info *info;
	struct snd_dma_buffer * dmab;
	dmab = kzalloc(sizeof(*dmab), GFP_KERNEL);
	if (! dmab)
		return NULL;

	/* init substream */
	if (act_substream) {
		g.substream = act_substream;
		dmab->dev = g.substream->dma_buffer.dev;
		if (snd_dma_alloc_pages(dmab->dev.type, dmab->dev.dev, size, dmab) < 0) {
			kfree(dmab);
			pr_info("build_dmab: snd_dma_alloc_pages error\n");
			return NULL;
		}
		pr_info("snd_dma_buffer area:%p addr:%llu bytes:%lu\n", dmab->area, dmab->addr, dmab->bytes);
	} else {
		return NULL;
	}

	info = kmalloc(sizeof(struct mmap_info), GFP_KERNEL);
	info->data = dmab->area;
	filp->private_data = info;

	memset(dmab->area, 0, size);
	return dmab;
}
void dmab_release(struct file *filp, struct snd_dma_buffer *dmab)
{
	pr_info("dmab_release start\n");
	struct mmap_info *info;
	info = filp->private_data;
	if (dmab) {
		pr_info("dmab_release snd_dma_free_pages\n");
		snd_dma_free_pages(dmab);
		pr_info("dmab_release kfree(dmab)\n");
		kfree(dmab);
		pr_info("dmab_release kfree(dmab) done\n");
	}

	pr_info("dmab_release kfree(info) \n");
	kfree(info);
	pr_info("dmab_release kfree(info) done\n");
	filp->private_data = NULL;
	return;
}

/* open */
static int open_ops(struct inode *inode, struct file *filp)
{
	aoe_buf = build_dmab(filp, AOE_BUF_SIZE*2);
	if (!aoe_buf) {
		pr_info("open_ops build dmab error!\n");
		return -1;
	}

	/* ext_slot */
	ext = (struct ext_slot *)((void *)aoe_buf->area + AOE_BUF_SIZE + NM_BUFSZ*AOE_NUM_SLOTS);
	ext->stat = INACTIVE;
	int i;
	for (i = 0; i < AOE_NUM_SLOTS; i++) {
		ext->slot[i].buf_idx = AOE_LUT_INDEX + i;
	}
	return 0;
}

/* release */
static int release_ops(struct inode *inode, struct file *filp)
{
	pr_info("release_ops start\n");
	dmab_release(filp, aoe_buf);
	pr_info("release_ops dmab_release done\n");
	aoe_buf = NULL;
	ext = NULL;
	return 0;
}

/* mmap */
static int mmap_ops(struct file *filp, struct vm_area_struct *vma)
{
	pr_info("mmap_ops ext head:%d cur:%d tail:%d\n", ext->head, ext->cur, ext->tail);
	pr_info("mmap_ops dev.dev:%p area:%p addr:%llu bytes:%lu\n", aoe_buf->dev.dev, aoe_buf->area, aoe_buf->addr, aoe_buf->bytes);
	return dma_mmap_coherent(aoe_buf->dev.dev, vma, aoe_buf->area, aoe_buf->addr, aoe_buf->bytes);
}

static const struct proc_ops fops = {
	.proc_ioctl = ioctl_ops,
	.proc_mmap = mmap_ops,
	.proc_open = open_ops,
	.proc_release = release_ops,
};

#define MASTER		0
#define SLAVE		1
static ssize_t aoe_vital_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int mode = SLAVE;
	ssize_t i = 0;
	struct snd_pcm_runtime * runtime = NULL;
	struct bcm2835_chan *c = NULL;
	if (g.substream) {
		runtime = g.substream->runtime;
		struct dmaengine_pcm_runtime_data *prtd = runtime->private_data;
		c = container_of(prtd->dma_chan, struct bcm2835_chan, vc.chan);

		/* MASTER/SLAVE */
		struct snd_soc_pcm_runtime *rtd = g.substream->private_data;
		switch (rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_MASTER_MASK) {
			case SND_SOC_DAIFMT_CBM_CFM: // master
				mode = MASTER;
				break;
			case SND_SOC_DAIFMT_CBS_CFS: // slave
				mode = SLAVE;
				break;
		}
	}
	i += sprintf(buf + i, "\x1B[35m# DMA Control Block\x1B[0m\n");
	int _cb_cur = g.cb_cur_prediction;
	if (c && c->desc) {
		i += sprintf(buf + i, "tail:%d cur:%d->%d head:%d space:%d->%d frames:%d size:%ld\n",
			cb_tail, cb_cur, _cb_cur, cb_head,
			CIRC_SPACE(cb_head, cb_tail, c->desc->frames),
			CIRC_SPACE(cb_head, _cb_cur, c->desc->frames),
			c->desc->frames, c->desc->size);

	} else {
		i += sprintf(buf + i, "tail:%d cur:%d->%d head:%d cb_desc is null!\n",
			cb_tail, cb_cur, _cb_cur, cb_head);
	}
		i += sprintf(buf + i, "cb index error         :%lu\n", g.cb_index_error_count);
		i += sprintf(buf + i, "tasklet duplicate error:%lu\n", g.tasklet_duplicate_error_count);

	i += sprintf(buf + i, "\x1B[35m# Netmap Slot\x1B[0m\n");
	if (ext) {
		i += sprintf(buf + i, "stat:%s\n", ext->stat == ACTIVE ? "ACTIVE":"INACTIVE");
		i += sprintf(buf + i, "tail:%d cur:%d head:%d space:%d num_slots:%d playable:%d\n",
			ext->tail, ext->cur, ext->head,
			CIRC_SPACE(ext->head, ext->tail, ext->num_slots),
			ext->num_slots,
			CIRC_CNT(ext->head, ext->cur, ext->num_slots));
		i += sprintf(buf + i, "unarrived_dreq_packets:%d\n", ext->unarrived_dreq_packets);
	} else {
		i += sprintf(buf + i, "struct ext_slot *ext is null!\n");
	}
	i += sprintf(buf + i, "\x1B[35m# PCM Substream\x1B[0m\n");
	if (g.substream) {
		i += sprintf(buf + i, "status:%d (0:OPEN 1:SETUP 2:PREPARED 3:RUNNING 4:XRUN and others.)\n",
			runtime->status->state);
		i += sprintf(buf + i, "hw_ptr:%lu appl_ptr:%lu period_size:%lu periods:%u buffer_size:%lu\n",
			runtime->status->hw_ptr, runtime->control->appl_ptr,
			runtime->period_size, runtime->periods, runtime->buffer_size);
		i += sprintf(buf + i, "format:%s rate:%u channels:%u\n",
			snd_pcm_format_name(runtime->format), runtime->rate, runtime->channels);
		i += sprintf(buf + i, "start_threshold:%ld stop_threshold:%ld dma_bytes:%lu\n",
			runtime->start_threshold, runtime->stop_threshold, runtime->dma_bytes);
		i += sprintf(buf + i, "mode:%s\n", mode == SLAVE ? "DAC SLAVE":"DAC MASTER");
	} else {
		i += sprintf(buf + i, "substream is null!\n");
	}
	return i;
}
static struct kobj_attribute vital_attribute = __ATTR_RO(aoe_vital);

__attribute__((cold))
static int __init bcm2835_aoe_init(void)
{
	/* proc for mmap */
	proc_create(PROC_NAME_AOEBUF, 0, NULL, &fops);

	/* sysfs for vital */
	sysfs_create_file(kernel_kobj, &vital_attribute.attr);

	/* function pointer */
	bcm2835_cyclic_callback_ptr = &rtalsa_cyclic_callback;
	bcm2835_dma_terminate_ptr = &rtalsa_dma_terminate;

	/* bcm2835-dma callback/terminate task */
	dma_callback_arg.scheduled = false;
	tasklet_init(&dma_callback_task, dma_callback_func, (unsigned long)&dma_callback_arg);
	tasklet_init(&dma_terminate_task, dma_terminate_func, 0);

	return 0;
}

__attribute__((cold))
static void __exit bcm2835_aoe_exit(void)
{
	/* proc */
	remove_proc_entry(PROC_NAME_AOEBUF, NULL);

	/* sysfs */
	sysfs_remove_file(kernel_kobj, &vital_attribute.attr);

	/* function pointer */
	bcm2835_cyclic_callback_ptr = NULL;
	bcm2835_dma_terminate_ptr = NULL;

	/* callback task */
	tasklet_kill(&dma_callback_task);
	tasklet_kill(&dma_terminate_task);
}

module_init(bcm2835_aoe_init);
module_exit(bcm2835_aoe_exit);
