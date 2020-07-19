#
# drivers/nfc/Makefile
#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2014 KYOCERA Corporation
# (C) 2015 KYOCERA Corporation
# (C) 2016 KYOCERA Corporation
#

#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533.o
obj-$(CONFIG_NFC_WILINK)	+= nfcwilink.o
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_QNCI)		+= nfc-nci.o
obj-$(CONFIG_NFC_NQ)		+= nq-nci.o
obj-$(CONFIG_CXD224X_NFC)	+= cxd224x-i2c.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
