#
# modules.boot
#
# List of modules the bootloader (multiboot) shall load.
#

# Watchdogs
omap_wdt
twl4030_wdt

# Real Time Clock
#rtc-core
#rtc-twl4030

# Switch core
switch_class

# Accl sensor
lis302dl

# Light sensor
tsl2563

# LEDs
led-class
leds-lp5523
leds-twl4030-vibra

# Battery
power_supply

# Nokia AV
nokia-av

# Touchscreen
tsc2005 prescale=1

# Phonet
phonet
pn_pep
ssi_mcsaab_imp

# Misc binary format
binfmt_misc

# RF power control
rfkill
rfkill-input
wl127x-rfkill

# Wifi
mac80211
crc7

# Bluetooth
bluetooth
rfcomm
hci_h4p
sco
hidp

# OMAP DSP
#bridgedriver
#dspbridge

# OMAP iommu
omap3-iommu
iommu
iommu2
iovmm

# Camera subsystem
#smia-sensor
#omap34xxcam-mod
#board-rx51-camera

# power vr
pvrsrvkm
omaplfb

# fm transmitter
#fmtx-si4713

# autofocus
ad5820

# torch
adp1653

# ASEC containers
twofish_common
twofish
dm-mod
dm-crypt
dm-loop

# Filesystems
fat
vfat
msdos

# Android gadget (adb,storage)
g_android

