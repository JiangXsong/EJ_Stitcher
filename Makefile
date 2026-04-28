# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for the V4L2 Proxy-Mixer driver (proxy_mixer.ko)
#
# ── Quick reference ─────────────────────────────────────────────────────────
#   make                 Build the module
#   make install         Install + depmod (enables modprobe)
#   make uninstall       Remove installed module + depmod
#   make load            modprobe the module
#   make unload          rmmod the module
#   make reload          rmmod + modprobe
#   make clang-format    Format all .c / .h files with kernel style
#   make clean           Remove build artefacts
# ────────────────────────────────────────────────────────────────────────────

MODULE_NAME := proxy_mixer

# --- Object list ------------------------------------------------------------
$(MODULE_NAME)-y := ejcm3_mixer.o

obj-$(CONFIG_VIDEO_PROXY_MIXER) += $(MODULE_NAME).o

# --- Out-of-tree build -------------------------------------------------------
ifneq ($(KERNELRELEASE),)
  # Kbuild pass — nothing extra needed.
else

KDIR  ?= /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)

EXTRA_CFLAGS := -DCONFIG_VIDEO_PROXY_MIXER=m

.PHONY: all modules modules_install install uninstall load unload reload clean help

all: modules

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_VIDEO_PROXY_MIXER=m modules

# --- Install / modprobe support ----------------------------------------------

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) CONFIG_VIDEO_PROXY_MIXER=m modules_install
	depmod -a

install: modules modules_install
	@echo "$(MODULE_NAME).ko installed — you can now use: modprobe $(MODULE_NAME)"

uninstall:
	@mod_path=$$(modinfo -n $(MODULE_NAME) 2>/dev/null); \
	if [ -n "$$mod_path" ] && [ -f "$$mod_path" ]; then \
		echo "Removing $$mod_path"; \
		rm -f "$$mod_path"; \
		depmod -a; \
		echo "$(MODULE_NAME) uninstalled"; \
	else \
		echo "$(MODULE_NAME) is not installed"; \
	fi

load:
	@if lsmod | grep -qw $(MODULE_NAME); then \
		echo "$(MODULE_NAME) is already loaded"; \
	else \
		modprobe $(MODULE_NAME); \
		echo "$(MODULE_NAME) loaded"; \
	fi

unload:
	@if lsmod | grep -qw $(MODULE_NAME); then \
		rmmod $(MODULE_NAME); \
		echo "$(MODULE_NAME) unloaded"; \
	else \
		echo "$(MODULE_NAME) is not loaded"; \
	fi

reload: unload load

# --- clang-format (kernel coding style) --------------------------------------

.clang-format:
	curl -sS "https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git/plain/.clang-format" -o $@

# Collect whatever source files exist; won't fail on empty globs.
FORMAT_SRCS := $(wildcard *.c *.h utils/*.c utils/*.h)

.PHONY: clang-format clang-format-check
clang-format: .clang-format
	@if [ -z "$(FORMAT_SRCS)" ]; then \
		echo "No .c / .h files found to format"; \
	else \
		clang-format -i $(FORMAT_SRCS); \
		echo "Formatted: $(FORMAT_SRCS)"; \
	fi

clang-format-check: .clang-format
	@clang-format --dry-run --Werror $(FORMAT_SRCS) && \
		echo "clang-format: all files OK" || \
		(echo "clang-format: style violations found (run 'make clang-format' to fix)"; exit 1)

# --- Cleanup ------------------------------------------------------------------

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f .clang-format

# --- Help ---------------------------------------------------------------------

help:
	@echo "Targets:"
	@echo "  all / modules        - build the module (default)"
	@echo "  install              - build + install + depmod (enables modprobe)"
	@echo "  uninstall            - remove installed .ko + depmod"
	@echo "  load                 - modprobe $(MODULE_NAME)"
	@echo "  unload               - rmmod $(MODULE_NAME)"
	@echo "  reload               - rmmod + modprobe"
	@echo "  clang-format         - format sources with kernel .clang-format"
	@echo "  clang-format-check   - dry-run check (CI friendly)"
	@echo "  clean                - remove build artefacts + .clang-format"
	@echo ""
	@echo "Variables:"
	@echo "  KDIR=<path>          - kernel build directory"
	@echo "                         (default: /lib/modules/$$(uname -r)/build)"
	@echo ""
	@echo "Example (cross-compile):"
	@echo "  make KDIR=/path/to/kernel ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-"

endif
