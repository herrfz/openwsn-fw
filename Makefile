MODULE:=$(shell basename $(CURDIR))
export OPENWSN_ROOT=$(CURDIR)

DIRS += $(OPENWSN_ROOT)/projects/common/03oos_openwsn \
		$(OPENWSN_ROOT)/openstack \
		$(OPENWSN_ROOT)/openapps \
		$(OPENWSN_ROOT)/kernel/openos

INCLUDES += -I$(OPENWSN_ROOT)/kernel \
			-I$(OPENWSN_ROOT)/inc \
			-I$(OPENWSN_ROOT)/bsp/boards/ \
			-I$(OPENWSN_ROOT)/bsp/boards/riot-adaption \
			-I$(OPENWSN_ROOT)/openstack \
			-I$(OPENWSN_ROOT)/openapps

include $(RIOTBASE)/Makefile.base