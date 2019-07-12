###############################################################
#
# Installers for tools
#
# NOTE: These are not tied to the default goals
#       and must be invoked manually
#
###############################################################

##############################
#
# Check that environmental variables are sane
#
##############################

# Set up ARM (STM32) SDK
ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-none-eabi-6-2017-q2-update
#ARM_SDK_DIR ?= $(TOOLS_DIR)/gcc-arm-none-eabi-8-2018-q4-major
# Checked below, Should match the output of $(shell arm-none-eabi-gcc -dumpversion)
GCC_REQUILED_RED_VERSION ?= 6.3.1
#GCC_REQUILED_RED_VERSION ?= 8.2.1

.PHONY: arm_sdk_version

arm_sdk_version:
	$(V1) $(ARM_SDK_PREFIX)gcc --version

## arm_sdk_install   : Install Arm SDK
.PHONY: arm_sdk_install

ARM_SDK_URL_BASE  := https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update
#ARM_SDK_URL_BASE  := https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-8-2018-q4-major

# source: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
ifdef LINUX
  ARM_SDK_URL  := $(ARM_SDK_URL_BASE)-linux.tar.bz2
endif

ifdef MACOSX
  ARM_SDK_URL  := $(ARM_SDK_URL_BASE)-mac.tar.bz2
endif

ifdef WINDOWS
  ARM_SDK_URL  := $(ARM_SDK_URL_BASE)-win32.zip
endif

ARM_SDK_FILE := $(notdir $(ARM_SDK_URL))

SDK_INSTALL_MARKER := $(ARM_SDK_DIR)/bin/arm-none-eabi-gcc-$(GCC_REQUILED_RED_VERSION)

# order-only prereq on directory existance:
arm_sdk_install: | $(TOOLS_DIR)

arm_sdk_install: arm_sdk_download $(SDK_INSTALL_MARKER)

$(SDK_INSTALL_MARKER):
ifneq ($(OSFAMILY), windows)
        # binary only release so just extract it
	$(V1) tar -C $(TOOLS_DIR) -xjf "$(DL_DIR)/$(ARM_SDK_FILE)"
else
	$(V1) unzip -q -d $(ARM_SDK_DIR) "$(DL_DIR)/$(ARM_SDK_FILE)"
endif

.PHONY: arm_sdk_download
arm_sdk_download: | $(DL_DIR)
arm_sdk_download: $(DL_DIR)/$(ARM_SDK_FILE)
$(DL_DIR)/$(ARM_SDK_FILE):
        # download the source only if it's newer than what we already have
	$(V1) curl -L -k -o "$(DL_DIR)/$(ARM_SDK_FILE)" -z "$(DL_DIR)/$(ARM_SDK_FILE)" "$(ARM_SDK_URL)"


## arm_sdk_clean     : Uninstall Arm SDK
.PHONY: arm_sdk_clean
arm_sdk_clean:
	$(V1) [ ! -d "$(ARM_SDK_DIR)" ] || $(RM) -r $(ARM_SDK_DIR)
	$(V1) [ ! -d "$(DL_DIR)" ] || $(RM) -r $(DL_DIR)

.PHONY: openocd_win_install

openocd_win_install: | $(DL_DIR) $(TOOLS_DIR)
openocd_win_install: OPENOCD_URL  := git://git.code.sf.net/p/openocd/code
openocd_win_install: OPENOCD_REV  := cf1418e9a85013bbf8dbcc2d2e9985695993d9f4
openocd_win_install: OPENOCD_OPTIONS :=

ifeq ($(OPENOCD_FTDI), yes)
openocd_win_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --enable-ft2232_ftd2xx --with-ftd2xx-win32-zipdir=$(FTD2XX_DIR)
endif

openocd_win_install: openocd_win_clean libusb_win_install ftd2xx_install
        # download the source
	$(V0) @echo " DOWNLOAD     $(OPENOCD_URL) @ $(OPENOCD_REV)"
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_BUILD_DIR)"
	$(V1) git clone --no-checkout $(OPENOCD_URL) "$(DL_DIR)/openocd-build"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git checkout -q $(OPENOCD_REV) ; \
	)

        # apply patches
	$(V0) @echo " PATCH        $(OPENOCD_BUILD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git apply < $(ROOT_DIR)/flight/Project/OpenOCD/0003-freertos-cm4f-fpu-support.patch ; \
	  git apply < $(ROOT_DIR)/flight/Project/OpenOCD/0004-st-icdi-disable.patch ; \
	)

        # build and install
	$(V0) @echo " BUILD        $(OPENOCD_WIN_DIR)"
	$(V1) mkdir -p "$(OPENOCD_WIN_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  ./bootstrap ; \
	  ./configure --enable-maintainer-mode --prefix="$(OPENOCD_WIN_DIR)" \
		--build=i686-pc-linux-gnu --host=i586-mingw32msvc \
		CPPFLAGS=-I$(LIBUSB_WIN_DIR)/include \
		LDFLAGS=-L$(LIBUSB_WIN_DIR)/lib/gcc \
		$(OPENOCD_OPTIONS) \
		--disable-werror \
		--enable-stlink ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)

        # delete the extracted source when we're done
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"

.PHONY: openocd_win_clean
openocd_win_clean:
	$(V0) @echo " CLEAN        $(OPENOCD_WIN_DIR)"
	$(V1) [ ! -d "$(OPENOCD_WIN_DIR)" ] || $(RM) -r "$(OPENOCD_WIN_DIR)"

# Set up openocd tools
OPENOCD_DIR       := $(TOOLS_DIR)/openocd
OPENOCD_WIN_DIR   := $(TOOLS_DIR)/openocd_win
OPENOCD_BUILD_DIR := $(DL_DIR)/openocd-build

.PHONY: openocd_install

openocd_install: | $(DL_DIR) $(TOOLS_DIR)
openocd_install: OPENOCD_URL     := git://git.code.sf.net/p/openocd/code
openocd_install: OPENOCD_TAG     := v0.9.0
openocd_install: OPENOCD_OPTIONS := --enable-maintainer-mode --prefix="$(OPENOCD_DIR)" --enable-buspirate --enable-stlink

ifeq ($(OPENOCD_FTDI), yes)
openocd_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --enable-ftdi
endif

ifeq ($(UNAME), Darwin)
openocd_install: OPENOCD_OPTIONS := $(OPENOCD_OPTIONS) --disable-option-checking
endif

openocd_install: openocd_clean
        # download the source
	$(V0) @echo " DOWNLOAD     $(OPENOCD_URL) @ $(OPENOCD_TAG)"
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_BUILD_DIR)"
	$(V1) git clone --no-checkout $(OPENOCD_URL) "$(OPENOCD_BUILD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  git checkout -q tags/$(OPENOCD_TAG) ; \
	)

        # build and install
	$(V0) @echo " BUILD        $(OPENOCD_DIR)"
	$(V1) mkdir -p "$(OPENOCD_DIR)"
	$(V1) ( \
	  cd $(OPENOCD_BUILD_DIR) ; \
	  ./bootstrap ; \
	  ./configure  $(OPENOCD_OPTIONS) ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)

        # delete the extracted source when we're done
	$(V1) [ ! -d "$(OPENOCD_BUILD_DIR)" ] || $(RM) -rf "$(OPENOCD_BUILD_DIR)"

.PHONY: openocd_clean
openocd_clean:
	$(V0) @echo " CLEAN        $(OPENOCD_DIR)"
	$(V1) [ ! -d "$(OPENOCD_DIR)" ] || $(RM) -r "$(OPENOCD_DIR)"

STM32FLASH_DIR := $(TOOLS_DIR)/stm32flash

.PHONY: stm32flash_install
stm32flash_install: STM32FLASH_URL := http://stm32flash.googlecode.com/svn/trunk
stm32flash_install: STM32FLASH_REV := 61
stm32flash_install: stm32flash_clean
        # download the source
	$(V0) @echo " DOWNLOAD     $(STM32FLASH_URL) @ r$(STM32FLASH_REV)"
	$(V1) svn export -q -r "$(STM32FLASH_REV)" "$(STM32FLASH_URL)" "$(STM32FLASH_DIR)"

        # build
	$(V0) @echo " BUILD        $(STM32FLASH_DIR)"
	$(V1) $(MAKE) --silent -C $(STM32FLASH_DIR) all

.PHONY: stm32flash_clean
stm32flash_clean:
	$(V0) @echo " CLEAN        $(STM32FLASH_DIR)"
	$(V1) [ ! -d "$(STM32FLASH_DIR)" ] || $(RM) -r "$(STM32FLASH_DIR)"

DFUUTIL_DIR := $(TOOLS_DIR)/dfu-util

.PHONY: dfuutil_install
dfuutil_install: DFUUTIL_URL  := http://dfu-util.sourceforge.net/releases/dfu-util-0.8.tar.gz
dfuutil_install: DFUUTIL_FILE := $(notdir $(DFUUTIL_URL))
dfuutil_install: | $(DL_DIR) $(TOOLS_DIR)
dfuutil_install: dfuutil_clean
        # download the source
	$(V0) @echo " DOWNLOAD     $(DFUUTIL_URL)"
	$(V1) curl -L -k -o "$(DL_DIR)/$(DFUUTIL_FILE)" "$(DFUUTIL_URL)"

        # extract the source
	$(V0) @echo " EXTRACT      $(DFUUTIL_FILE)"
	$(V1) [ ! -d "$(DL_DIR)/dfuutil-build" ] || $(RM) -r "$(DL_DIR)/dfuutil-build"
	$(V1) mkdir -p "$(DL_DIR)/dfuutil-build"
	$(V1) tar -C $(DL_DIR)/dfuutil-build -xf "$(DL_DIR)/$(DFUUTIL_FILE)"

        # build
	$(V0) @echo " BUILD        $(DFUUTIL_DIR)"
	$(V1) mkdir -p "$(DFUUTIL_DIR)"
	$(V1) ( \
	  cd $(DL_DIR)/dfuutil-build/dfu-util-0.8 ; \
	  ./configure --prefix="$(DFUUTIL_DIR)" ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)

.PHONY: dfuutil_clean
dfuutil_clean:
	$(V0) @echo " CLEAN        $(DFUUTIL_DIR)"
	$(V1) [ ! -d "$(DFUUTIL_DIR)" ] || $(RM) -r "$(DFUUTIL_DIR)"

# Set up uncrustify tools
UNCRUSTIFY_DIR := $(TOOLS_DIR)/uncrustify-0.61
UNCRUSTIFY_BUILD_DIR := $(DL_DIR)/uncrustify

.PHONY: uncrustify_install
uncrustify_install: | $(DL_DIR) $(TOOLS_DIR)
uncrustify_install: UNCRUSTIFY_URL := http://downloads.sourceforge.net/project/uncrustify/uncrustify/uncrustify-0.61/uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_FILE := uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_OPTIONS := prefix=$(UNCRUSTIFY_DIR)
uncrustify_install: uncrustify_clean
ifneq ($(OSFAMILY), windows)
	$(V0) @echo " DOWNLOAD     $(UNCRUSTIFY_URL)"
	$(V1) curl -L -k -o "$(DL_DIR)/$(UNCRUSTIFY_FILE)" "$(UNCRUSTIFY_URL)"
endif
        # extract the src
	$(V0) @echo " EXTRACT      $(UNCRUSTIFY_FILE)"
	$(V1) tar -C $(TOOLS_DIR) -xf "$(DL_DIR)/$(UNCRUSTIFY_FILE)"

	$(V0) @echo " BUILD        $(UNCRUSTIFY_DIR)"
	$(V1) ( \
	  cd $(UNCRUSTIFY_DIR) ; \
	  ./configure --prefix="$(UNCRUSTIFY_DIR)" ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)
	      # delete the extracted source when we're done
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

.PHONY: uncrustify_clean
uncrustify_clean:
	$(V0) @echo " CLEAN        $(UNCRUSTIFY_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_DIR)"
	$(V0) @echo " CLEAN        $(UNCRUSTIFY_BUILD_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

# ZIP download URL
zip_install: ZIP_URL  := http://pkgs.fedoraproject.org/repo/pkgs/zip/zip30.tar.gz/7b74551e63f8ee6aab6fbc86676c0d37/zip30.tar.gz

zip_install: ZIP_FILE := $(notdir $(ZIP_URL))

ZIP_DIR = $(TOOLS_DIR)/zip30

# order-only prereq on directory existance:
zip_install : | $(DL_DIR) $(TOOLS_DIR)
zip_install: zip_clean
	$(V1) curl -L -k -o "$(DL_DIR)/$(ZIP_FILE)" "$(ZIP_URL)"
	$(V1) tar --force-local -C $(TOOLS_DIR) -xzf "$(DL_DIR)/$(ZIP_FILE)"
ifneq ($(OSFAMILY), windows)
	$(V1) cd "$(ZIP_DIR)" && $(MAKE) -f unix/Makefile generic_gcc
else
	$(V1) cd "$(ZIP_DIR)" && $(MAKE) -f win32/makefile.gcc
endif

.PHONY: zip_clean
zip_clean:
	$(V1) [ ! -d "$(ZIP_DIR)" ] || $(RM) -rf $(ZIP_DIR)

##############################
#
# Set up paths to tools
#
##############################

ifeq ($(shell [ -d "$(ARM_SDK_DIR)" ] && echo "exists"), exists)
  ARM_SDK_PREFIX := $(ARM_SDK_DIR)/bin/arm-none-eabi-
else ifeq (,$(findstring _install,$(MAKECMDGOALS)))
  GCC_VERSION = $(shell arm-none-eabi-gcc -dumpversion)
  ifeq ($(GCC_VERSION),)
    $(error **ERROR** arm-none-eabi-gcc not in the PATH. Run 'make arm_sdk_install' to install automatically in the tools folder of this repo)
  else ifneq ($(GCC_VERSION), $(GCC_REQUILED_RED_VERSION))
    $(error **ERROR** your arm-none-eabi-gcc is '$(GCC_VERSION)', but '$(GCC_REQUILED_RED_VERSION)' is expected. Override with 'GCC_REQUILED_RED_VERSION' in make/local.mk or run 'make arm_sdk_install' to install the right version automatically in the tools folder of this repo)
  endif

  # ARM tookchain is in the path, and the version is what's required.
  ARM_SDK_PREFIX ?= arm-none-eabi-
endif

ifeq ($(shell [ -d "$(ZIP_DIR)" ] && echo "exists"), exists)
  export ZIPBIN := $(ZIP_DIR)/zip
else
  export ZIPBIN := zip
endif

ifeq ($(shell [ -d "$(OPENOCD_DIR)" ] && echo "exists"), exists)
  OPENOCD := $(OPENOCD_DIR)/bin/openocd
else
  # not installed, hope it's in the path...
  OPENOCD ?= openocd
endif

ifeq ($(shell [ -d "$(UNCRUSTIFY_DIR)" ] && echo "exists"), exists)
  UNCRUSTIFY := $(UNCRUSTIFY_DIR)/bin/uncrustify
else
  # not installed, hope it's in the path...
  UNCRUSTIFY ?= uncrustify
endif

# Google Breakpad
DUMP_SYMBOLS_TOOL := $(TOOLS_DIR)/breakpad/$(OSFAMILY)-$(ARCHFAMILY)/dump_syms
BREAKPAD_URL := http://dronin.tracer.nz/tools/breakpad.zip
BREAKPAD_DL_FILE := $(DL_DIR)/$(notdir $(BREAKPAD_URL))
BREAKPAD_DIR := $(TOOLS_DIR)/breakpad

.PHONY: breakpad_install
breakpad_install: | $(DL_DIR) $(TOOLS_DIR)
breakpad_install: breakpad_clean
	$(V0) @echo " DOWNLOAD     $(BREAKPAD_URL)"
	$(V1) $(V1) curl -L -k -z "$(BREAKPAD_DL_FILE)" -o "$(BREAKPAD_DL_FILE)" "$(BREAKPAD_URL)"
	$(V0) @echo " EXTRACT      $(notdir $(BREAKPAD_DL_FILE))"
	$(V1) mkdir -p "$(BREAKPAD_DIR)"
	$(V1) unzip -q -d $(BREAKPAD_DIR) "$(BREAKPAD_DL_FILE)"
ifeq ($(OSFAMILY), windows)
	$(V1) ln -s "$(TOOLS_DIR)/breakpad/$(OSFAMILY)-i686" "$(TOOLS_DIR)/breakpad/$(OSFAMILY)-x86_64"
endif

.PHONY: breakpad_clean
breakpad_clean:
	$(V0) @echo " CLEAN        $(BREAKPAD_DIR)"
	$(V1) [ ! -d "$(BREAKPAD_DIR)" ] || $(RM) -rf $(BREAKPAD_DIR)
	$(V0) @echo " CLEAN        $(BREAKPAD_DL_FILE)"
	$(V1) $(RM) -f $(BREAKPAD_DL_FILE)
