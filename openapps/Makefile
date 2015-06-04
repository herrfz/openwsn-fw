#MODULE = openwsn

DIRS += $(CURDIR)/c6t \
        $(CURDIR)/uecho \
        $(CURDIR)/uinject \
        $(CURDIR)/techo \
        $(CURDIR)/cinfo \
        $(CURDIR)/cleds \
        $(CURDIR)/cstorm \
        $(CURDIR)/cwellknown \
        $(CURDIR)/rrt

INCLUDES += -I$(CURDIR)/c6t \
			-I$(CURDIR)/techo \
			-I$(CURDIR)/uecho \
			-I$(CURDIR)/uinject \
			-I$(CURDIR)/cinfo \
			-I$(CURDIR)/cleds \
			-I$(CURDIR)/cstorm \
			-I$(CURDIR)/cwellknown \
			-I$(CURDIR)/rrt\
			-I$(OPENWSN_ROOT)/openstack/04-TRAN \
			-I$(OPENWSN_ROOT)/drivers/common \
			-I$(OPENWSN_ROOT)/openstack/02a-MAClow \
			-I$(OPENWSN_ROOT)/openstack/02b-MAChigh \
			-I$(OPENWSN_ROOT)/openstack/03a-IPHC \
			-I$(OPENWSN_ROOT)/openstack/03b-IPv6 \
			-I$(OPENWSN_ROOT)/openstack/cross-layers

include $(RIOTBASE)/Makefile.base
