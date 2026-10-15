# OpenWrt SDK package of the Fleet-DT channel monitor. Copy this directory to
# package/fdt-chmon/ of the SDK and set FDT_SRC to the root of the fleet-dt repository.
include $(TOPDIR)/rules.mk

PKG_NAME:=fdt-chmon
PKG_VERSION:=2.0
PKG_RELEASE:=1
PKG_LICENSE:=MIT

FDT_SRC?=$(TOPDIR)/../fleet-dt

include $(INCLUDE_DIR)/package.mk

define Package/fdt-chmon
  SECTION:=net
  CATEGORY:=Network
  TITLE:=Fleet-DT channel monitor (y(t) of the GS radio)
  DEPENDS:=+libmosquitto-nossl +cJSON
endef

define Package/fdt-chmon/description
  Reads the survey and station counters of nl80211 at each step boundary and publishes
  y(t) of Fleet-DT on fleet/throughput of the GS MQTT broker.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) $(FDT_SRC)/common/fdt_common.c $(FDT_SRC)/common/fdt_common.h $(PKG_BUILD_DIR)/
	$(CP) $(FDT_SRC)/radio/channel_monitor/fdt_chmon.c $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(TARGET_CC) $(TARGET_CPPFLAGS) $(TARGET_CFLAGS) -std=gnu11 -I$(PKG_BUILD_DIR) \
		-o $(PKG_BUILD_DIR)/fdt_chmon $(PKG_BUILD_DIR)/fdt_chmon.c $(PKG_BUILD_DIR)/fdt_common.c \
		$(TARGET_LDFLAGS) -lmosquitto -lcjson -lm
endef

define Package/fdt-chmon/install
	$(INSTALL_DIR) $(1)/usr/bin $(1)/etc/init.d $(1)/etc/config $(1)/etc/fleetdt
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/fdt_chmon $(1)/usr/bin/
	$(INSTALL_BIN) $(FDT_SRC)/radio/openwrt/gs/etc/init.d/fdt-chmon $(1)/etc/init.d/
	$(INSTALL_CONF) $(FDT_SRC)/radio/openwrt/gs/etc/config/fdt-chmon $(1)/etc/config/
	$(INSTALL_DATA) $(FDT_SRC)/dtp.json $(1)/etc/fleetdt/
endef

define Package/fdt-chmon/conffiles
/etc/config/fdt-chmon
/etc/fleetdt/dtp.json
endef

$(eval $(call BuildPackage,fdt-chmon))
