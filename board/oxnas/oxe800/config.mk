#
# image should be loaded at 0x01000000
#

TEXT_BASE = 0x48d00000

TEST_BRD ?= 0
MEM_SIZE ?= 32
PROBE_MEM_SIZE ?= 0

PLATFORM_CPPFLAGS +=	-DPROBE_MEM_SIZE=$(PROBE_MEM_SIZE) \
			-DMEM_SIZE=$(MEM_SIZE) \
			-DIS_TEST_BRD=$(TEST_BRD)
