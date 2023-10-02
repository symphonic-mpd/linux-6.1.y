#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/proc_fs.h>
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
#define SWAP_THRESHOLD	8
#define COPY_THRESHOLD	8

/* Externed variables in pcm_native.c */
extern struct snd_pcm_substream *act_substream;

/* Externed variables in pcm_lib.c */
extern void (*pcm_period_elapsed_callback_ptr)(void);

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
EXPORT_SYMBOL(ext);		/* extend slots */
EXPORT_SYMBOL(ext_lut);	/* look up table for extend slots */
EXPORT_SYMBOL(aoe_buf);	/* AoE buffer */

enum OutputType {I2S, USB};
struct server_stats {
	struct snd_pcm_substream * substream;
	enum OutputType type;
//	int cb_cur_prediction;
//	unsigned long cb_index_error_count;
//	unsigned long tasklet_duplicate_error_count;
//	unsigned long cb_index_missing_count;
};
static struct server_stats g = {NULL, I2S};

/* control block buf idx */
static int cb_head;
static int cb_tail;
static int cb_cur;
atomic_t work_queued;

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

void drain_func(struct work_struct *work)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	if (runtime->status->state != SNDRV_PCM_STATE_XRUN) {
		pr_info("ioctl_drain!");
		snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_DRAIN, NULL);
	}
	smp_store_release(&ext->cur, ext->head);
	smp_store_release(&ext->tail, ext->head);
}
static DECLARE_WORK(drain_work, drain_func);

/*
 * Set the address of the netmap buffer to the "src" in the control block,
 * and update the "cur" and "tail" of the netmap buffer.
 */
void aoebuf_swap(void)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	struct dmaengine_pcm_runtime_data *prtd = runtime->private_data;
	struct bcm2835_chan *c = container_of(prtd->dma_chan, struct bcm2835_chan, vc.chan);

//	const struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
//	const struct snd_pcm_mmap_status * __restrict__ status = runtime->status;

//	if(likely(ext && c->desc && status->state == SNDRV_PCM_STATE_RUNNING)) {
	if(likely(ext && c->desc && ext->stat == ACTIVE)) {
//		unsigned long flags;
//		snd_pcm_stream_lock_irqsave(g.substream, flags);
		const int _tail = ext->tail;
		const int _head = smp_load_acquire(&ext->head);
		int _cur = ext->cur;
		const int _cb_cur = cb_cur;
		const int cb_played = CIRC_CNT(_cb_cur, cb_tail, runtime->periods);
		cb_tail = _cb_cur;

		// update ext->tail
		smp_store_release(&ext->tail, CIRC_INDEX(_tail + cb_played, ext->num_slots));

		int ext_playable = CIRC_CNT(_head, _cur, ext->num_slots);
		int cb_space = CIRC_SPACE(cb_head, cb_tail, runtime->periods);

		int i = 0;
		if (ext_playable > 0 && cb_space > 0) {
			for (i = 0; i < cb_space; i++) {
				if (ext_playable == 0)
					break;

				c->desc->cb_list[cb_head].cb->src = idx_to_addr(ext->slot[_cur].buf_idx);
				cb_head = CIRC_INDEX(cb_head+1, runtime->periods);
				_cur = CIRC_INDEX(_cur+1, ext->num_slots);

				ext_playable--;
			}
			/* update appl_ptr */
			if (i > 0) {
				snd_pcm_uframes_t appl = runtime->control->appl_ptr + runtime->period_size * i;
				apply_appl_ptr(g.substream, appl);
			}
			/* update ext->cur */
			smp_store_release(&ext->cur, _cur);
		}
//		snd_pcm_stream_unlock_irqrestore(g.substream, flags);

		if (ext_playable == 0 && cb_space > i) {
//			pr_info("ioctl_drain...!");
//			snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_DRAIN, NULL);
//			smp_store_release(&ext->cur, ext->head);
//			smp_store_release(&ext->tail, ext->head);
			ext->stat = INACTIVE;
			queue_work(system_highpri_wq, &drain_work);
		}
	}
}

void aoebuf_copy(void)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	if(likely(ext && ext->stat == ACTIVE)) {

		const int _tail = ext->tail;
		const int _head = smp_load_acquire(&ext->head);
		int _cur = ext->cur;
		const int _cb_cur = cb_cur;
		const int cb_played = CIRC_CNT(_cb_cur, cb_tail, runtime->periods);
		cb_tail = _cb_cur;

		// update ext->tail
		smp_store_release(&ext->tail, CIRC_INDEX(_tail + cb_played, ext->num_slots));

		int ext_playable = CIRC_CNT(_head, _cur, ext->num_slots);
		int cb_space = CIRC_SPACE(cb_head, cb_tail, runtime->periods);

		int i = 0;
		if (ext_playable > 0 && cb_space > 0) {
			int period_bytes = frames_to_bytes(runtime, runtime->period_size);
			for (i = 0; i < cb_space; i++) {
				if (ext_playable == 0)
					break;

				memcpy(runtime->dma_area + cb_head * period_bytes, idx_to_area(ext->slot[_cur].buf_idx), period_bytes);

				cb_head = CIRC_INDEX(cb_head+1, runtime->periods);
				_cur = CIRC_INDEX(_cur+1, ext->num_slots);

				ext_playable--;
			}
			/* update appl_ptr */
			if (i > 0) {
				snd_pcm_uframes_t appl = runtime->control->appl_ptr + runtime->period_size * i;
				apply_appl_ptr(g.substream, appl);
			}
			/* update ext->cur */
			smp_store_release(&ext->cur, _cur);
		}
		if (ext_playable == 0 && cb_space > i) {
//			pr_info("ioctl_drain!");
			ext->stat = INACTIVE;
//			snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_DRAIN, NULL);
//			smp_store_release(&ext->cur, ext->head);
//			smp_store_release(&ext->tail, ext->head);
			queue_work(system_highpri_wq, &drain_work);
		}
	}
}
/*
static void period_elapsed_func(struct work_struct *work)
{
	if (g.type == I2S) {
		aoebuf_swap();
	} else if (g.type == USB && ext->stat == ACTIVE) {
		aoebuf_copy();
	}
	atomic_dec(&work_queued);
}
static DECLARE_WORK(period_elapsed_work, period_elapsed_func);
*/
void pcm_period_elapsed_callback_func(void)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;

	if (runtime->status->state == SNDRV_PCM_STATE_XRUN) {
		pr_info("xrun!");
		smp_store_release(&ext->cur, ext->head);
		smp_store_release(&ext->tail, ext->head);
		ext->stat = INACTIVE;
		return;
	}

	cb_cur = (runtime->status->hw_ptr - runtime->hw_ptr_base)/runtime->period_size;
	const int cb_cnt = CIRC_CNT(cb_head, cb_cur, runtime->periods);

	if (g.type == I2S) {
		if (cb_cnt <= runtime->periods / SWAP_THRESHOLD)
			aoebuf_swap();
	} else {
		if (cb_cnt <= runtime->periods - COPY_THRESHOLD)
			aoebuf_copy();
/*		if (cb_cnt <= runtime->periods - COPY_THRESHOLD * 2) {
			if (atomic_add_unless(&work_queued, 1, 1))
				queue_work(system_highpri_wq, &period_elapsed_work);
		} else if (cb_cnt <= runtime->periods - COPY_THRESHOLD) {
			if (atomic_read(&work_queued) == 0)
				aoebuf_copy();
		}
*/
	}

	return;
}

void i2s_prepare(void)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;
	const struct snd_pcm_mmap_status * __restrict__ status = runtime->status;
	struct dmaengine_pcm_runtime_data *prtd = runtime->private_data;
	struct bcm2835_chan *c = container_of(prtd->dma_chan, struct bcm2835_chan, vc.chan);

	uint8_t * s;

		if (runtime->status->state == SNDRV_PCM_STATE_XRUN ||
			runtime->status->state == SNDRV_PCM_STATE_SETUP) {
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
				pr_info("ioctl... NG! snd_pcm_start error!?\n");
			if(!c->desc) {
				pr_info("ioctl... NG! dma not started!!\n");
			}
		} else {
			pr_info("ioctl... NG! state is not PREPARED!\n");
		}

		s = (uint8_t *)runtime->dma_area;
		pr_info("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			*(s), *(s+1), *(s+2), *(s+3), *(s+4), *(s+5), *(s+6), *(s+7),
			*(s+8), *(s+9), *(s+10), *(s+11), *(s+12), *(s+13), *(s+14), *(s+15));

		// initialize index
		cb_cur = seek_cb_cur(c);
//		g.cb_cur_prediction = cb_cur;
		cb_tail = cb_cur;
		cb_head = CIRC_INDEX(cb_cur + 1, c->desc->frames); // write this point!
		smp_store_release(&ext->cur, ext->tail);
		int init_appl = runtime->hw_ptr_base + runtime->period_size * (cb_cur + 1);
		apply_appl_ptr(g.substream, init_appl);

		pr_info("ioctl   ... IOCTL_PCM_START (state:%d cb_cur:%d)\n",
			runtime->status->state, cb_cur);

		// first swap!
		ext->stat = ACTIVE;
		aoebuf_swap();
}

void usb_prepare(void)
{
	struct snd_pcm_runtime * __restrict__ runtime = g.substream->runtime;

	if (runtime && runtime->status &&
		(runtime->status->state == SNDRV_PCM_STATE_XRUN ||
		 runtime->status->state == SNDRV_PCM_STATE_SETUP)) {
		/* prepare */
		snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_PREPARE, NULL);
	}

//	cb_cur = cb_tail = cb_head = g.cb_cur_prediction = 0;
	cb_cur = cb_tail = cb_head = 0;
	smp_store_release(&ext->cur, ext->tail);
	ext->stat = ACTIVE;
	aoebuf_copy();

	if (runtime->status->state == SNDRV_PCM_STATE_PREPARED) {
		int err;
		err = snd_pcm_kernel_ioctl(g.substream, SNDRV_PCM_IOCTL_START, NULL);
		if (err < 0)
			pr_info("ioctl... NG! snd_pcm_start error!?\n");
	} else {
		pr_info("ioctl... NG! state is not PREPARED!\n");
	}

	pr_info("ioctl   ... IOCTL_PCM_START (state:%d cb_cur:%d)\n",
			runtime->status->state, cb_cur);
}

/* ioctl aoe_buf */
static long ioctl_ops(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	int value;
	switch (cmd) {
	case IOCTL_PCM_START:
		if (act_substream)
			g.substream = act_substream;

		copy_from_user(&value, (int __user *)arg, sizeof(int));
		g.type = (enum OutputType)value;
		atomic_set(&work_queued, 0);

		pcm_period_elapsed_callback_ptr = &pcm_period_elapsed_callback_func;

		if (g.type == I2S) {
			i2s_prepare();
		} else if (g.type == USB) {
			usb_prepare();
		} else {
			pr_info("ioctl_pcm_start type error! (g.type:%d)\n", g.type);
		}
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
	if (!aoe_buf) {
		aoe_buf = build_dmab(filp, AOE_BUF_SIZE);
		if (!aoe_buf) {
			pr_info("open_ops build dmab error!\n");
			return -1;
		}

	}
	/* clear */
	memset(aoe_buf->area, 0, AOE_BUF_SIZE);

	/* ext_slot */
	ext = (struct ext_slot *)((void *)aoe_buf->area + UNIT_BUF_SIZE*4);
	ext->stat = INACTIVE;
	int i;
	for (i = 0; i < AOE_NUM_SLOTS; i++) {
		ext->slot[i].buf_idx = AOE_LUT_INDEX + i;
	}
	return 0;
}

/* release */
/*
static int release_ops(struct inode *inode, struct file *filp)
{
	pr_info("release_ops start\n");
	dmab_release(filp, aoe_buf);
	pr_info("release_ops dmab_release done\n");
	aoe_buf = NULL;
	ext = NULL;
	return 0;
}
*/
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
//	.proc_release = release_ops,
};

static ssize_t aoe_vital_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	char mode[11];
	if (g.type == USB)
		memcpy(mode, "USB", sizeof(mode));
	ssize_t i = 0;
	struct snd_pcm_runtime * runtime = NULL;
	struct bcm2835_chan *c = NULL;
	if (g.substream) {
		runtime = g.substream->runtime;
		if (g.type == I2S && runtime) {
			struct dmaengine_pcm_runtime_data *prtd = runtime->private_data;
			c = container_of(prtd->dma_chan, struct bcm2835_chan, vc.chan);

			/* MASTER/SLAVE */
			struct snd_soc_pcm_runtime *rtd = g.substream->private_data;
			switch (rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_MASTER_MASK) {
				case SND_SOC_DAIFMT_CBM_CFM: // master
					strcpy(mode, "DAC MASTER");
					break;
				case SND_SOC_DAIFMT_CBS_CFS: // slave
					strcpy(mode, "DAC SLAVE");
					break;
			}
		}
	}
	i += sprintf(buf + i, "\x1B[35m# DMA Control Block\x1B[0m\n");
//	int _cb_cur = g.cb_cur_prediction;
	int _cb_cur = cb_cur;
	int _cb_tail = cb_tail;
	int _cb_head = cb_head;
	if (g.type == I2S) {
		if (c && c->desc) {
			i += sprintf(buf + i, "tail:%d cur:%d head:%d space:%d->%d frames:%d size:%ld\n",
				_cb_tail, _cb_cur, _cb_head,
				CIRC_SPACE(_cb_head, cb_tail, c->desc->frames),
				CIRC_SPACE(_cb_head, _cb_cur, c->desc->frames),
				c->desc->frames, c->desc->size);
		} else {
			i += sprintf(buf + i, "tail:%d cur:%d head:%d cb_desc is null!\n",
				_cb_tail, _cb_cur, _cb_head);
		}
	} else if (g.type == USB) {
			i += sprintf(buf + i, "tail:%d cur:%d head:%d (USB Playback Alsa Buffer)\n",
				_cb_tail, _cb_cur, _cb_head);
	}
//	i += sprintf(buf + i, "cb index error         :%lu\n", g.cb_index_error_count);
//	i += sprintf(buf + i, "cb index missing error :%lu\n", g.cb_index_missing_count);
//	i += sprintf(buf + i, "tasklet duplicate error:%lu\n", g.tasklet_duplicate_error_count);

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
	if (g.substream && runtime) {
		i += sprintf(buf + i, "status:%d (0:OPEN 1:SETUP 2:PREPARED 3:RUNNING 4:XRUN and others.)\n",
			runtime->status->state);
		i += sprintf(buf + i, "hw_ptr:%lu appl_ptr:%lu period_size:%lu periods:%u buffer_size:%lu\n",
			runtime->status->hw_ptr, runtime->control->appl_ptr,
			runtime->period_size, runtime->periods, runtime->buffer_size);
		i += sprintf(buf + i, "format:%s rate:%u channels:%u\n",
			snd_pcm_format_name(runtime->format), runtime->rate, runtime->channels);
		i += sprintf(buf + i, "start_threshold:%ld stop_threshold:%ld dma_bytes:%lu\n",
			runtime->start_threshold, runtime->stop_threshold, runtime->dma_bytes);
		i += sprintf(buf + i, "mode:%s\n", mode);
	} else {
		i += sprintf(buf + i, "substream is null!\n");
	}
	return i;
}
static struct kobj_attribute vital_attribute = __ATTR_RO(aoe_vital);

__attribute__((cold))
static int __init bcm2835_aoe_init(void)
{
	g.type = I2S;
	atomic_set(&work_queued, 0);

	/* proc for mmap */
	proc_create(PROC_NAME_AOEBUF, 0, NULL, &fops);

	/* sysfs for vital */
	sysfs_create_file(kernel_kobj, &vital_attribute.attr);

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
	pcm_period_elapsed_callback_ptr = NULL;

//	cancel_work_sync(&period_elapsed_work);

	if (aoe_buf) {
		pr_info("dmab_release snd_dma_free_pages\n");
		snd_dma_free_pages(aoe_buf);
		pr_info("dmab_release kfree(dmab)\n");
		kfree(aoe_buf);
		pr_info("dmab_release kfree(dmab) done\n");
	}
}

module_init(bcm2835_aoe_init);
module_exit(bcm2835_aoe_exit);
