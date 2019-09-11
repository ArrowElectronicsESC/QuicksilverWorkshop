#
# Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
# All Rights Reserved.
#
# This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
# the contents of this file may not be disclosed to third parties, copied
# or duplicated in any form, in whole or in part, without the prior
# written permission of Broadcom Corporation.
#

NAME := QuicksilverAzureIoT

WIFI_CONFIG_DCT_H  := wifi_config_dct.h

PROJ_DIR := .

$(NAME)_SOURCES	:= $(PROJ_DIR)/azure_https_client.c \
                   $(PROJ_DIR)/ccan_json.c \
                   $(PROJ_DIR)/Drivers/Sensors/LIS2DH12/lis2dh12.c \
                   $(PROJ_DIR)/Drivers/Sensors/HTS221/hts221.c \
                   $(PROJ_DIR)/Drivers/LED/APA102/apa102.c \


$(NAME)_COMPONENTS := protocols/HTTP_client \
                      libraries/utilities/JSON_parser

GLOBAL_INCLUDES := . \
                    $(SDK_ROOT) \
                    $(SDK_ROOT)/include \
                    $(SDK_ROOT)/sys \


# < Please add client certificate and private key here if you want to enable client authentication >
#CERTIFICATE := $(SOURCE_ROOT)resources/certificates/brcm_demo_server_cert.cer
#PRIVATE_KEY := $(SOURCE_ROOT)resources/certificates/brcm_demo_server_cert_key.key