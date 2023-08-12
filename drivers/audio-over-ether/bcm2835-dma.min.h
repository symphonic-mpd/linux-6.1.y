#define BCM2835_DMA_SOURCE_AD	0x0c

/* drivers/dma/virt-dma.h */
struct virt_dma_desc {
	struct dma_async_tx_descriptor tx;
	struct dmaengine_result tx_result; //6.1~
	struct list_head node;

};
struct virt_dma_chan { // 6.1 changed
	struct dma_chan	chan;
	struct tasklet_struct task;
	void (*desc_free)(struct virt_dma_desc *);
	spinlock_t lock;
	struct list_head desc_allocated;
	struct list_head desc_submitted;
	struct list_head desc_issued;
	struct list_head desc_completed;
	struct list_head desc_terminated;
	struct virt_dma_desc *cyclic;
};

/* sound/core/pcm_dmaengine.c */
struct dmaengine_pcm_runtime_data {
	struct dma_chan *dma_chan;
	dma_cookie_t cookie;

	unsigned int pos;
};

/* drivers/dma/bcm2835-dma.c */
struct bcm2835_chan {
	struct virt_dma_chan vc;

// Variable that existed in 4.19 but was removed in 6.1.
//	struct list_head node;

	struct dma_slave_config	cfg;
	unsigned int dreq;

	int ch;
	struct bcm2835_desc *desc;
	struct dma_pool *cb_pool;

	void __iomem *chan_base;
	int irq_number;
	unsigned int irq_flags;

	bool is_lite_channel;
	bool is_40bit_channel;
};
struct bcm2835_dma_cb {
	uint32_t info;
	uint32_t src;
	uint32_t dst;
	uint32_t length;
	uint32_t stride;
	uint32_t next;
	uint32_t pad[2];
};
struct bcm2835_cb_entry {
	struct bcm2835_dma_cb *cb;
	dma_addr_t paddr;
};
struct bcm2835_desc {
	struct bcm2835_chan *c;
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;
	unsigned int frames;
	size_t size;
	bool cyclic;
	struct bcm2835_cb_entry cb_list[];
};
