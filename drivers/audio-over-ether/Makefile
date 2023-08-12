ccflags-y := -Wno-declaration-after-statement -Werror -Wall -Wno-unused-function -Wno-unused-value
ccflags-y += -I$(srctree)/../netmap/apps/audio-over-ether
#ccflags-y += -frename-registers

# debug
#ccflags-y += -g -save-temps=obj

obj-$(CONFIG_SMPD_AUDIO_OVER_ETH) := bcm2835-aoe.o

KDIR ?= $(CURDIR)/../../build
default:
	$(MAKE) -C $(KDIR) M=drivers/audio-over-ether modules

clean:
	$(MAKE) -C $(KDIR) M=drivers/audio-over-ether clean

