# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the V4L2 Proxy-Mixer driver.
#
# ── In-tree build ───────────────────────────────────────────────────────────
#   Place this directory under  drivers/media/usb/proxy-mixer/
#   Add to drivers/media/usb/Makefile:
#       obj-$(CONFIG_VIDEO_PROXY_MIXER) += proxy-mixer/
#   Add to drivers/media/usb/Kconfig:
#       source "drivers/media/usb/proxy-mixer/Kconfig"
#
# ── Out-of-tree (standalone) build ──────────────────────────────────────────
#   make -C /lib/modules/$(uname -r)/build M=$(pwd) modules
#   make -C /lib/modules/$(uname -r)/build M=$(pwd) modules_install
#   make -C /lib/modules/$(uname -r)/build M=$(pwd) clean
# ────────────────────────────────────────────────────────────────────────────

# --- Object list ------------------------------------------------------------
proxy_mixer-y := ejcm3_mixer.o

obj-$(CONFIG_VIDEO_PROXY_MIXER) += proxy_mixer.o

# --- Out-of-tree fallback ---------------------------------------------------
# When invoked directly (not via Kbuild), CONFIG_VIDEO_PROXY_MIXER is unset.
# Default to module build so `make` alone does the right thing.
ifneq ($(KERNELRELEASE),)
  # In-kernel Kbuild pass – nothing extra needed.
else
  # Standalone / out-of-tree pass.
  KDIR  ?= /lib/modules/$(shell uname -r)/build
  PWD   := $(shell pwd)

  # Treat the module as always-enabled for the out-of-tree build.
  KBUILD_EXTRA_SYMBOLS :=
  EXTRA_CFLAGS := -DCONFIG_VIDEO_PROXY_MIXER=m

.PHONY: all modules modules_install clean help

all: modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_VIDEO_PROXY_MIXER=m modules

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_VIDEO_PROXY_MIXER=m modules_install
	depmod -a

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

help:
	@echo "Targets:"
	@echo "  all / modules        - build the module (default)"
	@echo "  modules_install      - install and run depmod"
	@echo "  clean                - remove build artefacts"
	@echo ""
	@echo "Variables:"
	@echo "  KDIR=<path>          - kernel build directory"
	@echo "                         (default: /lib/modules/\$$(uname -r)/build)"
	@echo ""
	@echo "Example (cross-compile):"
	@echo "  make KDIR=/path/to/kernel ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-"

endif
