# SPDX-License-Identifier: GPL-2.0
obj-m :=  ejcm3_mixer.o
ejcm3_mixer-objs  := ejcm3_mixer.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean