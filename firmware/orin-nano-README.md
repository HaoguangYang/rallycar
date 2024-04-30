## For NVIDIA Orin Nano DevKit

Enter factory recovery mode on the Orin DevKit: use a jumper to connect `FC REC` pin to `GND`. Connect power to the board, a green LED should turn on indicating that it is up. Connect a USB-C cable to a Linux PC. On the PC use `lsusb` to list USB devices. You should see an entry with `NVIDIA` in its name, indicating that the board is in recovery mode. **Unplug the jumper**.

Command to initiate the raw image creation + flashing, the example below is for flashing onto a mounted NVMe SSD:
```sh
cd JetPack_x.y.../Linux_for_Tegra && sudo ./nvsdkmanager_flash.sh --storage nvme0n1p1
```
Alternatively, if you plug in an SD card, you can run:
```sh
cd JetPack_x.y.../Linux_for_Tegra && sudo ./nvsdkmanager_flash.sh --storage mmcblk0p1
```
The commands above flash the official image. You may need to change a parameter to re-route the flash program to work with your custom image.

- Possible errors:
1. timed out at flashing:
    ```log
    mount.nfs: Connection timed out
    ```
    **Mitigation:** Check your firewall, and make sure `nfs` ports are allowed. Alternatively, disable firewall during flashing, e.g.:
    ```sh
    sudo ufw disable
    ```
    and re-enable it after the flash.

## Example log
```log
user@ubuntu:~/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra$ sudo ./nvsdkmanager_flash.sh --storage nvme0n1p1
user entered nvme0n1p1
*** Checking ONLINE mode ... OK.
*** Checking target board connection ... 1 connections found.
*** Reading ECID ... FUSELEVEL=fuselevel_production hwchipid=0x23 bootauth=NS
*** Reading EEPROM ... "/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py" --chip 0x23 --applet "/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin" --cfg readinfo_t234_min_prod.xml --dev_params tegra234-br-bct-diag-boot.dts --device_config tegra234-mb1-bct-device-p3701-0000.dts --misc_config tegra234-mb1-bct-misc-p3701-0000.dts --bins "mb2_applet applet_t234.bin" --skipuid --cmd "dump eeprom cvm cvm.bin; reboot recovery"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0252 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.0255 ] File rcm_state open failed
[   0.0260 ] ERROR: failed to read rcm_state
[   0.0260 ]
[   0.0269 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0271 ] Assuming zero filled SBK key
[   0.0265 ] Pre-processing config: tegra234-mb1-bct-device-p3701-0000.dts
[   0.0341 ] Pre-processing config: tegra234-mb1-bct-misc-p3701-0000.dts
[   0.0439 ] Parsing partition layout
[   0.0443 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0453 ] Kernel DTB used: None
[   0.0453 ] WARNING: dce base dtb is not provided

[   0.0453 ] Parsing partition layout
[   0.0457 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0464 ] Creating list of images to be signed
[   0.0468 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --list images_list.xml zerosbk
[   0.0472 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0485 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0490 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0546 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0553 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0607 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0610 ] adding BCH for mb2_t234_aligned.bin
[   0.0643 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0782 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0786 ] adding BCH for mb2_t234_aligned.bin
[   0.0954 ] Filling MB1 storage info
[   0.0954 ] Parsing dev params for multi chains
[   0.1028 ] Generating br-bct
[   0.1032 ] Updating dev and MSS params in BR BCT
[   0.1032 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1040 ] Updating bl info
[   0.1044 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin
[   0.1048 ] WARNING: boot chain is not completed. set to 0
[   0.1056 ] Generating signatures
[   0.1064 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1066 ] Assuming zero filled SBK key
[   0.1172 ] Warning: pub_key.key is not found
[   0.1166 ] Parsing dev params for multi chains
[   0.1166 ] Generating br-bct
[   0.1170 ] Updating dev and MSS params in BR BCT
[   0.1170 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1177 ] Updating bl info
[   0.1181 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1184 ] WARNING: boot chain is not completed. set to 0
[   0.1201 ] Generating SHA2 Hash
[   0.1217 ] Sha saved in br_bct_BR.sha
[   0.1211 ] Get Signed section of bct
[   0.1214 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   0.1220 ] Signing BCT
[   0.1228 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1229 ] Assuming zero filled SBK key
[   0.1252 ] Sha saved in br_bct_BR.sha
[   0.1254 ] Warning: pub_key.key is not found
[   0.1246 ] Updating BCT with signature
[   0.1250 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   0.1253 ] Offset :4608 Len :3584
[   0.1258 ] Generating SHA2 Hash
[   0.1266 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   0.1267 ] Assuming zero filled SBK key
[   0.1267 ] Assuming zero filled SBK key
[   0.1290 ] Sha saved in br_bct_BR.sha
[   0.1285 ] Updating BCT with SHA2 Hash
[   0.1288 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   0.1293 ] Offset :4608 Len :3584
[   0.1296 ] Offset :68 Len :8124
[   0.1298 ] Generating coldboot mb1-bct
[   0.1302 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3701-0000_cpp.dtb --device tegra234-mb1-bct-device-p3701-0000_cpp.dtb
[   0.1305 ] MB1-BCT version: 0.13

[   0.1324 ] Parsing config file :tegra234-mb1-bct-device-p3701-0000_cpp.dtb
[   0.1327 ] Added Platform Config 9 data with size :- 100
[   0.1327 ]
[   0.1327 ] Updating mb1-bct with firmware information
[   0.1332 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1347 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   0.1356 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   0.1358 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   0.1376 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1377 ] Assuming zero filled SBK key
[   0.1390 ] Warning: pub_key.key is not found
[   0.1387 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1397 ] Generating recovery mb1-bct
[   0.1401 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3701-0000_cpp.dtb --device tegra234-mb1-bct-device-p3701-0000_cpp.dtb
[   0.1405 ] MB1-BCT version: 0.13

[   0.1424 ] Parsing config file :tegra234-mb1-bct-device-p3701-0000_cpp.dtb
[   0.1425 ] Added Platform Config 9 data with size :- 100
[   0.1425 ]
[   0.1426 ] Updating mb1-bct with firmware information
[   0.1431 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1451 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   0.1456 ] Generating SHA2 Hash for mb1bct
[   0.1474 ] Sha saved in mb1_bct_MB1_aligned.sha
[   0.1471 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   0.1475 ] adding BCH for mb1_bct_MB1_aligned.bct
[   0.1491 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1493 ] Assuming zero filled SBK key
[   0.1506 ] Warning: pub_key.key is not found
[   0.1503 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1514 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1514 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1514 ] Copying signatures
[   0.1518 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1598 ] mb1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1600 ] psc_bl1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1600 ] Boot Rom communication
[   0.1604 ] tegrarcm_v2 --new_session --chip 0x23 0 --uid --download bct_br br_bct_BR.bct --download mb1 mb1_t234_prod_aligned_sigheader.bin.encrypt --download psc_bl1 psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
[   0.1608 ] BR_CID: 0x80012344705DF1975C000000110300C0
[   0.2081 ] Sending bct_br
[   0.2518 ] Sending mb1
[   0.2530 ] Sending psc_bl1
[   0.2629 ] Sending bct_mb1
[   0.2688 ] Boot Rom communication completed
[   0.2700 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   0.2713 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --appendsigheader applet_t234_aligned.bin zerosbk
[   0.2717 ] adding BCH for applet_t234_aligned.bin
[   0.2898 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.2900 ] Assuming zero filled SBK key
[   0.2925 ] Warning: pub_key.key is not found
[   0.2925 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   0.2946 ] Sending mb2_applet...

[   0.2952 ] tegrarcm_v2 --chip 0x23 0 --pollbl --download applet applet_t234_sigheader.bin.encrypt
[   0.2955 ] BL: version 1.4.0.1-t234-54845784-08e631ca last_boot_error: 0
[   0.4931 ] Sending applet
[   0.5925 ] completed
[   0.5939 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.5950 ] MB2 Applet version 01.00.0000
[   0.7793 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.7796 ] MB2 Applet version 01.00.0000
[   0.8278 ] Retrieving board information
[   0.8286 ] tegrarcm_v2 --chip 0x23 0 --oem platformdetails chip chip_info.bin
[   0.8296 ] MB2 Applet version 01.00.0000
[   0.8796 ] Saved platform info in chip_info.bin
[   0.8868 ] Chip minor revision: 1
[   0.8874 ] Bootrom revision: 0x7
[   0.8875 ] Ram code: 0x2
[   0.8875 ] Chip sku: 0xd5
[   0.8875 ] Chip Sample: prod
[   0.8875 ]
[   0.8881 ] Retrieving EEPROM data
[   0.8882 ] tegrarcm_v2 --oem platformdetails eeprom cvm /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin --chip 0x23 0
[   0.8893 ] MB2 Applet version 01.00.0000
[   0.9378 ] Saved platform info in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   0.9718 ] Rebooting to recovery mode
[   0.9729 ] tegrarcm_v2 --chip 0x23 0 --ismb2
[   1.0235 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.0244 ] MB2 Applet version 01.00.0000
[   1.0722 ] Booting to recovery mode
[   1.0735 ] tegrarcm_v2 --chip 0x23 0 --reboot recovery
[   1.0745 ] MB2 Applet version 01.00.0000
--- Reading board information succeeded.
--- Parsing chip_info.bin information succeeded.
Chip SKU(00:00:00:D5) ramcode(00:00:00:02)
Parsing module EEPROM:
--- Parsing board ID (3767) succeeded.
--- Parsing board version (300) succeeded.
--- Parsing board SKU (0005) succeeded.
--- Parsing board REV (K.2) succeeded.
jetson-orin-nano-devkit found.
Parsing boardid successful
Target board is jetson-orin-nano-devkit
External storage specified nvme0n1p1
Flashing Jeton Orin Nano
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/l4t_initrd_flash_internal.sh --no-flash --external-device nvme0n1p1 -c tools/kernel_flash/flash_l4t_t234_nvme.xml --showlogs --network usb0 -p --no-systemimg -c bootloader/generic/cfg/flash_t234_qspi.xml jetson-orin-nano-devkit internal
************************************
*                                  *
*  Step 1: Generate flash packages *
*                                  *
************************************
Create folder to store images to flash
Generate image for internal storage devices
Generate images to be flashed
ADDITIONAL_DTB_OVERLAY="BootOrderNvme.dtbo"  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/flash.sh --no-flash --sign  --no-systemimg -c bootloader/generic/cfg/flash_t234_qspi.xml jetson-orin-nano-devkit internal

###############################################################################
# L4T BSP Information:
# R36 , REVISION: 2.0
# User release: 0.0
###############################################################################
ECID is 0x80012344705DF1975C000000110300C0
copying emc_fuse_dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-diag-boot.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
./tegraflash.py --chip "0x23" --applet "/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin" --skipuid --cfg readinfo_t234_min_prod.xml --dev_params tegra234-br-bct-diag-boot.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --bins "mb2_applet applet_t234.bin" --cmd "dump eeprom cvm cvm.bin; dump try_custinfo custinfo_out.bin; reboot recovery"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0246 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.0250 ] File rcm_state open failed
[   0.0254 ] ERROR: failed to read rcm_state
[   0.0254 ]
[   0.0261 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0265 ] Assuming zero filled SBK key
[   0.0258 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.0328 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.0436 ] Parsing partition layout
[   0.0440 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0451 ] Kernel DTB used: None
[   0.0451 ] WARNING: dce base dtb is not provided

[   0.0451 ] Parsing partition layout
[   0.0456 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0463 ] Creating list of images to be signed
[   0.0467 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --list images_list.xml zerosbk
[   0.0470 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0482 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0487 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0544 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0551 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0605 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0609 ] adding BCH for mb2_t234_aligned.bin
[   0.0640 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0780 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0784 ] adding BCH for mb2_t234_aligned.bin
[   0.0950 ] Filling MB1 storage info
[   0.0950 ] Parsing dev params for multi chains
[   0.1022 ] Generating br-bct
[   0.1027 ] Updating dev and MSS params in BR BCT
[   0.1027 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1034 ] Updating bl info
[   0.1038 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin
[   0.1042 ] WARNING: boot chain is not completed. set to 0
[   0.1051 ] Generating signatures
[   0.1058 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1060 ] Assuming zero filled SBK key
[   0.1171 ] Warning: pub_key.key is not found
[   0.1165 ] Parsing dev params for multi chains
[   0.1165 ] Generating br-bct
[   0.1169 ] Updating dev and MSS params in BR BCT
[   0.1170 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1177 ] Updating bl info
[   0.1181 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1185 ] WARNING: boot chain is not completed. set to 0
[   0.1199 ] Generating SHA2 Hash
[   0.1217 ] Sha saved in br_bct_BR.sha
[   0.1210 ] Get Signed section of bct
[   0.1215 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   0.1221 ] Signing BCT
[   0.1229 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1231 ] Assuming zero filled SBK key
[   0.1254 ] Sha saved in br_bct_BR.sha
[   0.1256 ] Warning: pub_key.key is not found
[   0.1248 ] Updating BCT with signature
[   0.1252 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   0.1257 ] Offset :4608 Len :3584
[   0.1261 ] Generating SHA2 Hash
[   0.1269 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   0.1271 ] Assuming zero filled SBK key
[   0.1271 ] Assuming zero filled SBK key
[   0.1295 ] Sha saved in br_bct_BR.sha
[   0.1290 ] Updating BCT with SHA2 Hash
[   0.1293 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   0.1297 ] Offset :4608 Len :3584
[   0.1300 ] Offset :68 Len :8124
[   0.1302 ] Generating coldboot mb1-bct
[   0.1306 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1310 ] MB1-BCT version: 0.13

[   0.1328 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1330 ] Added Platform Config 9 data with size :- 100
[   0.1330 ]
[   0.1331 ] Updating mb1-bct with firmware information
[   0.1334 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1352 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   0.1361 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   0.1365 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   0.1381 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1382 ] Assuming zero filled SBK key
[   0.1396 ] Warning: pub_key.key is not found
[   0.1393 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1404 ] Generating recovery mb1-bct
[   0.1408 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1411 ] MB1-BCT version: 0.13

[   0.1430 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1433 ] Added Platform Config 9 data with size :- 100
[   0.1433 ]
[   0.1433 ] Updating mb1-bct with firmware information
[   0.1437 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1455 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   0.1460 ] Generating SHA2 Hash for mb1bct
[   0.1479 ] Sha saved in mb1_bct_MB1_aligned.sha
[   0.1476 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   0.1480 ] adding BCH for mb1_bct_MB1_aligned.bct
[   0.1495 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1496 ] Assuming zero filled SBK key
[   0.1510 ] Warning: pub_key.key is not found
[   0.1508 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1519 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1519 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1519 ] Copying signatures
[   0.1523 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1605 ] mb1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1607 ] psc_bl1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1607 ] Boot Rom communication
[   0.1611 ] tegrarcm_v2 --new_session --chip 0x23 0 --uid --download bct_br br_bct_BR.bct --download mb1 mb1_t234_prod_aligned_sigheader.bin.encrypt --download psc_bl1 psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
[   0.1615 ] BR_CID: 0x80012344705DF1975C000000110300C0
[   0.2070 ] Sending bct_br
[   0.2512 ] Sending mb1
[   0.2524 ] Sending psc_bl1
[   0.2633 ] Sending bct_mb1
[   0.2692 ] Boot Rom communication completed
[   0.2705 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   0.2720 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --appendsigheader applet_t234_aligned.bin zerosbk
[   0.2726 ] adding BCH for applet_t234_aligned.bin
[   0.2865 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.2867 ] Assuming zero filled SBK key
[   0.2893 ] Warning: pub_key.key is not found
[   0.2892 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   0.2910 ] Sending mb2_applet...

[   0.2915 ] tegrarcm_v2 --chip 0x23 0 --pollbl --download applet applet_t234_sigheader.bin.encrypt
[   0.2919 ] BL: version 1.4.0.1-t234-54845784-08e631ca last_boot_error: 0
[   0.4943 ] Sending applet
[   0.5941 ] completed
[   0.5954 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.5966 ] MB2 Applet version 01.00.0000
[   0.7793 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.7797 ] MB2 Applet version 01.00.0000
[   0.8285 ] Retrieving board information
[   0.8299 ] tegrarcm_v2 --chip 0x23 0 --oem platformdetails chip chip_info.bin
[   0.8310 ] MB2 Applet version 01.00.0000
[   0.8795 ] Saved platform info in chip_info.bin
[   0.8859 ] Chip minor revision: 1
[   0.8863 ] Bootrom revision: 0x7
[   0.8863 ] Ram code: 0x2
[   0.8863 ] Chip sku: 0xd5
[   0.8863 ] Chip Sample: prod
[   0.8863 ]
[   0.8868 ] Retrieving EEPROM data
[   0.8869 ] tegrarcm_v2 --oem platformdetails eeprom cvm /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin --chip 0x23 0
[   0.8881 ] MB2 Applet version 01.00.0000
[   0.9363 ] Saved platform info in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   0.9714 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.9725 ] MB2 Applet version 01.00.0000
[   1.0226 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.0235 ] MB2 Applet version 01.00.0000
[   1.0731 ] Dumping customer Info
[   1.0744 ] tegrarcm_v2 --chip 0x23 0 --oem dump bct tmp.bct
[   1.0753 ] MB2 Applet version 01.00.0000
[   1.1219 ] Saved bct in tmp.bct
[   1.1427 ] tegrabct_v2 --brbct tmp.bct --chip 0x23 0 --custinfo /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin
[   1.1431 ] Customer[   1.1436 ]  data saved in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin successfully
[   1.1437 ] Rebooting to recovery mode
[   1.1442 ] tegrarcm_v2 --chip 0x23 0 --ismb2
[   1.1914 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.1919 ] MB2 Applet version 01.00.0000
[   1.2381 ] Booting to recovery mode
[   1.2388 ] tegrarcm_v2 --chip 0x23 0 --reboot recovery
[   1.2393 ] MB2 Applet version 01.00.0000
Board ID(3767) version(300) sku(0005) revision(K.2)
Chip SKU(00:00:00:D5) ramcode(00:00:00:02) fuselevel(fuselevel_production) board_FAB(300)
emc_opt_disable_fuse:(0)
Copy /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec
Using UUID b7f91cd6-b44e-4695-ad8b-20fc32da1078 for mounting root APP partition.
copying bctfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-sdram-l4t.dts)... done.
copying minratchet_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-ratchet-p3767-0000.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
copying pinmux_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi)... done.
copying gpioint_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-gpioint-p3767-0000.dts)... done.
copying pmic_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pmic-p3767-0000-a02.dts)... done.
copying pmc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi)... done.
copying deviceprod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-cprod-p3767-0000.dts)... done.
copying prod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-prod-p3767-0000.dts)... done.
copying scr_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-scr-p3767-0000.dts)... done.
copying wb0sdram(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-wb0sdram-l4t.dts)... done.
copying bootrom_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-reset-p3767-0000.dts)... done.
Existing uphylane_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegra234-mb1-bct-uphylane-si.dtsi) reused.
copying dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-p3767-0000-l4t.dts)... done.
copying dev_params_b(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct_b-p3767-0000-l4t.dts)... done.
copying mb2bct_cfg(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-misc-p3767-0000.dts)... done.
Existing pscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/pscfw_t234_prod.bin) reused.
Existing pscbl1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_bl1_t234_prod.bin) reused.
Existing mtsmcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mce_flash_o10_cr_prod.bin) reused.
Existing tscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tsec_t234.bin) reused.
Existing mb2applet(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/applet_t234.bin) reused.
Existing bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
copying initrd(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs/boot/initrd)... done.
bl is uefi
Making Boot image... done.
Not signing of boot.img
Making recovery ramdisk for recovery image...
Re-generating recovery ramdisk for recovery image...
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/ramdisk_tmp /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
52057 blocks

gzip: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/Image: not in gzip format
_BASE_KERNEL_VERSION=5.15.122-tegra
74756 blocks
Making Recovery image...
copying recdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec)... done.
20+0 records in
20+0 records out
20 bytes copied, 0.0002493 s, 80.2 kB/s
Existing sosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing tegraboot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing cpu_bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing mb2blfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing xusbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/xusb_t234_prod.bin) reused.
Existing pvafile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvpva_020.fw) reused.
Existing dcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/display-t234-dce.bin) reused.
Existing nvdecfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvdec_t234_prod.fw) reused.
Existing psc_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_rf_t234_prod.bin) reused.
Existing mb2_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2rf_t234.bin) reused.
Existing mb1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing bpffile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/bpmp_t234-TE950M-A1_prod.bin) reused.
copying bpfdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/tegra234-bpmp-3767-0003-3509-a02.dtb)... done.
Existing camerafw(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/camera-rtcpu-t234-rce.img) reused.
Existing apefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/adsp-fw.bin) reused.
Existing spefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/spe_t234.bin) reused.
Existing wb0boot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/sc7_t234_prod.bin) reused.
Existing tosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tos-optee_t234.img) reused.
Existing eksfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/eks_t234.img) reused.
copying dtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
Copying nv_boot_control.conf to rootfs
Skip generating system.img
Not signing of kernel-dtb
Existing tbcfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/uefi_jetson.bin) reused.
131072+0 records in
131072+0 records out
67108864 bytes (67 MB, 64 MiB) copied, 0.590498 s, 114 MB/s
        Sync'ing esp.img ... done.
copying tbcdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
copying cfgfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/cfg/flash_t234_qspi.xml) to flash.xml... done.
Existing flashapp(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py) reused.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/L4TConfiguration.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-carveouts.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra-optee.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0000-dynamic.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/BootOrderNvme.dtbo)... done.
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration_updated.dts: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration.dtbo: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
./tegraflash.py  --bl uefi_jetson_with_dtb.bin  --odmdata gbe-uphy-config-8,hsstp-lane-map-3,hsio-uphy-config-0  --overlay_dtb L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo,BootOrderNvme.dtbo  --bldtb tegra234-p3768-0000+p3767-0005-nv.dtb --applet mb1_t234_prod.bin --cmd "sign"  --cfg flash.xml --chip "0x23" --concat_cpubl_bldtb --cpubl uefi_jetson.bin --minratchet_config tegra234-mb1-bct-ratchet-p3767-0000.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --pinmux_config tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi --gpioint_config tegra234-mb1-bct-gpioint-p3767-0000.dts --pmic_config tegra234-mb1-bct-pmic-p3767-0000-a02.dts --pmc_config tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi --deviceprod_config tegra234-mb1-bct-cprod-p3767-0000.dts --prod_config tegra234-mb1-bct-prod-p3767-0000.dts --scr_config tegra234-mb2-bct-scr-p3767-0000.dts --wb0sdram_config tegra234-p3767-0001-wb0sdram-l4t.dts --br_cmd_config tegra234-mb1-bct-reset-p3767-0000.dts --uphy tegra234-mb1-bct-uphylane-si.dtsi --dev_params tegra234-br-bct-p3767-0000-l4t.dts,tegra234-br-bct_b-p3767-0000-l4t.dts --mb2bct_cfg tegra234-mb2-bct-misc-p3767-0000.dts  --bins "psc_fw pscfw_t234_prod.bin; mts_mce mce_flash_o10_cr_prod.bin; tsec_fw tsec_t234.bin; mb2_applet applet_t234.bin; mb2_bootloader mb2_t234.bin; xusb_fw xusb_t234_prod.bin; pva_fw nvpva_020.fw; dce_fw display-t234-dce.bin; nvdec nvdec_t234_prod.fw; bpmp_fw bpmp_t234-TE950M-A1_prod.bin; bpmp_fw_dtb tegra234-bpmp-3767-0003-3509-a02.dtb; rce_fw camera-rtcpu-t234-rce.img; ape_fw adsp-fw.bin; spe_fw spe_t234.bin; tos tos-optee_t234.img; eks eks_t234.img"  --sdram_config tegra234-p3767-0001-sdram-l4t.dts  --cust_info custinfo_out.bin  --bct_backup  --boot_chain A
saving flash command in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/flashcmd.txt
saving Windows flash command to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/flash_win.bat
*** Sign and generate flashing ready partition images... ***
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0329 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0331 ] Assuming zero filled SBK key
[   0.0373 ] Parsing partition layout
[   0.0379 ] tegraparser_v2 --pt flash.xml.tmp
[   0.0435 ] Change tegra234-bpmp-3767-0003-3509-a02.dtb to tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb
[   0.0435 ] Change tegra234-bpmp-3767-0003-3509-a02.dtb to tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb
[   0.0971 ] /usr/bin/python3 dtbcheck.py -c t234 -o tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb tegra234-bpmp-3767-0003-3509-a02_with_odm_tmp.dtb
[   0.3741 ] Concatenating L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo,BootOrderNvme.dtbo to tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   0.3744 ] Concatenating bl dtb to cpubl binary
[   0.3773 ] MB2 binary: mb2_t234.bin
[   0.3774 ] Pre-processing mb2bct config: tegra234-mb2-bct-misc-p3767-0000.dts
[   0.3893 ] Pre-processing mb2bct config: tegra234-mb2-bct-scr-p3767-0000.dts
[   0.8753 ] Generating coldboot mb2-bct
[   0.8753 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_cold_boot_bct.cfg --mb2bctcfg tegra234-mb2-bct-misc-p3767-0000_cpp.dtb --scr tegra234-mb2-bct-scr-p3767-0000_cpp.dtb
[   0.8758 ] ERROR: value 0x31 is out of range
[   0.8771 ] ERROR: value 0x31 is out of range
[   0.8773 ] ERROR: value 0x31 is out of range
[   0.8775 ] ERROR: value 0x31 is out of range
[   0.8776 ] WARNING: unknown property 'tfc_version'
[   0.8778 ] WARNING: unknown property 'addr_header_version'
[   0.8903 ] Updating mb2-bct with storage information
[   0.8908 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_cold_boot_bct_MB2.bct --updatestorageinfo flash.xml.bin
[   0.8932 ] Concatenating mb2-bct to mb2 binary
[   0.8932 ] mb2_bin_file = mb2_t234.bin
[   0.8932 ] mb2_bct_file = mb2_cold_boot_bct_MB2.bct
[   0.8967 ] DCE binary: display-t234-dce.bin
[   0.8967 ] Kernel DTB used: tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   0.8967 ] Concatenating kernel-dtb to dce-fw binary
[   0.8968 ] dce_bin = display-t234-dce.bin
[   0.8968 ] kernel_dtb = tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   0.8968 ] dce_with_dtb = display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.bin
[   0.9067 ] Update display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.bin to dce_fw partitions
[   0.9089 ] Parsing partition layout
[   0.9094 ] tegraparser_v2 --pt flash.xml.tmp
[   0.9104 ] Creating list of images to be signed
[   0.9110 ] Generating ratchet blob
[   0.9110 ] Pre-processing config: tegra234-mb1-bct-reset-p3767-0000.dts
[   0.9183 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.9255 ] Pre-processing config: tegra234-mb1-bct-cprod-p3767-0000.dts
[   0.9331 ] Pre-processing config: tegra234-mb1-bct-gpioint-p3767-0000.dts
[   0.9414 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.9523 ] Pre-processing config: tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi
[   0.9658 ] Pre-processing config: tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi
[   0.9730 ] Pre-processing config: tegra234-mb1-bct-pmic-p3767-0000-a02.dts
[   0.9805 ] Pre-processing config: tegra234-mb1-bct-prod-p3767-0000.dts
[   0.9876 ] Pre-processing config: tegra234-p3767-0001-sdram-l4t.dts
[   2.2355 ] Pre-processing config: tegra234-mb1-bct-uphylane-si.dtsi
[   2.2424 ] Pre-processing config: tegra234-p3767-0001-wb0sdram-l4t.dts
[   3.4878 ] Pre-processing config: tegra234-mb1-bct-ratchet-p3767-0000.dts
[   3.4946 ] Generating coldboot mb1-bct
[   3.4950 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   3.4954 ] MB1-BCT version: 0.13

[   3.4974 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   3.4979 ] Added Platform Config 0 data with size :- 2416

[   3.5005 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   3.5009 ] WARNING: unknown node 'g2'
[   3.5010 ] WARNING: unknown node 'g2'
[   3.5011 ] WARNING: unknown node 'g9'
[   3.5012 ] WARNING: unknown node 'g9'
[   3.5013 ] Added Platform Config 2 data with size :- 24
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   3.5014 ] Added Platform Config 4 data with size :- 288
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   3.5014 ] Added Platform Config 3 data with size :- 52
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   3.5014 ] WARNING: unknown property 'major'
[   3.5014 ] WARNING: unknown property 'minor'
[   3.5014 ] Added Platform Config 5 data with size :- 512
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   3.5014 ] WARNING: unknown property 'major'
[   3.5014 ] WARNING: unknown property 'minor'
[   3.5014 ] Added Platform Config 7 data with size :- 380
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   3.5014 ] Added Platform Config 8 data with size :- 24
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   3.5014 ] Added Platform Config 9 data with size :- 100
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   3.5014 ] ModuleCount 0 NumProdNames 0
[   3.5014 ] Added Platform Config 6 data with size :- 16
[   3.5014 ]
[   3.5014 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   3.5014 ] ERROR: /ratchet/atf is not supported
[   3.5014 ]
[   3.5014 ] Updating mb1-bct with firmware information
[   3.5018 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   3.5046 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   3.5055 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   3.5061 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   3.5080 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   3.5082 ] Assuming zero filled SBK key
[   3.5097 ] Warning: pub_key.key is not found
[   3.5094 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   3.5106 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --ratchet_blob ratchet_blob.bin --list images_list.xml zerosbk
[   3.5111 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   3.5121 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   3.5126 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   3.5178 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   3.5180 ] Header already present for tsec_t234_aligned.bin
[   3.5205 ] Header already present for nvdec_t234_prod_aligned.fw
[   3.5244 ] adding BCH for mb2_t234_with_mb2_cold_boot_bct_MB2_aligned.bin
[   3.5310 ] adding BCH for xusb_t234_prod_aligned.bin
[   3.5457 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   3.5534 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   3.5713 ] Header already present for pscfw_t234_prod_aligned.bin
[   3.5783 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   3.5839 ] Header already present for sc7_t234_prod.bin
[   3.5873 ] Header already present for psc_rf_t234_prod_aligned.bin
[   3.5910 ] adding BCH for mb2rf_t234_aligned.bin
[   3.5936 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   3.6062 ] adding BCH for tos-optee_t234_aligned.img
[   3.7325 ] adding BCH for eks_t234_aligned.img
[   3.7794 ] INFO: compressing display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin
[   3.9955 ] INFO: complete compression, display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin, ratio = 6%
[   4.0204 ] adding BCH for display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_aligned.bin
[   4.0414 ] adding BCH for spe_t234_aligned.bin
[   4.0553 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   4.0653 ] adding BCH for adsp-fw_aligned.bin
[   4.0828 ] INFO: compressing nvpva_020_aligned.fw
[   4.1434 ] INFO: complete compression, nvpva_020_aligned.fw, ratio = 2%
[   4.1473 ] adding BCH for nvpva_020_aligned_blob_w_bin_aligned.fw
[   4.1498 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   4.1518 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   4.1522 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   4.1582 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   4.1586 ] Header already present for tsec_t234_aligned.bin
[   4.1614 ] Header already present for nvdec_t234_prod_aligned.fw
[   4.1656 ] adding BCH for mb2_t234_with_mb2_cold_boot_bct_MB2_aligned.bin
[   4.1735 ] adding BCH for xusb_t234_prod_aligned.bin
[   4.1888 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   4.1974 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   4.2153 ] Header already present for pscfw_t234_prod_aligned.bin
[   4.2228 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   4.2288 ] Header already present for sc7_t234_prod.bin
[   4.2324 ] Header already present for psc_rf_t234_prod_aligned.bin
[   4.2365 ] adding BCH for mb2rf_t234_aligned.bin
[   4.2392 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   4.2533 ] adding BCH for tos-optee_t234_aligned.img
[   4.3717 ] adding BCH for eks_t234_aligned.img
[   4.4150 ] INFO: compressing display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin
[   4.6500 ] INFO: complete compression, display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin, ratio = 6%
[   4.6756 ] adding BCH for display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_aligned.bin
[   4.6965 ] adding BCH for spe_t234_aligned.bin
[   4.7099 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   4.7199 ] adding BCH for adsp-fw_aligned.bin
[   4.7363 ] INFO: compressing nvpva_020_aligned.fw
[   4.7898 ] INFO: complete compression, nvpva_020_aligned.fw, ratio = 2%
[   4.7937 ] adding BCH for nvpva_020_aligned_blob_w_bin_aligned.fw
[   4.7978 ] Filling MB1 storage info
[   4.7978 ] Parsing dev params for multi chains
[   4.8051 ] Generating br-bct
[   4.8055 ] Updating dev and MSS params in BR BCT
[   4.8055 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   4.9893 ] Updating bl info
[   4.9899 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   4.9985 ] Generating br-bct
[   4.9989 ] Updating dev and MSS params in BR BCT
[   4.9989 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.1692 ] Updating bl info
[   5.1696 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   5.1721 ] Generating BCT backup image
[   5.1722 ] dd if=/dev/zero of=bct_backup.img bs=1 count=32768
[   5.1727 ] 32768+0 records in
[   5.2469 ] 32768+0 records out
[   5.2469 ] 32768 bytes (33 kB, 32 KiB) copied, 0.0735488 s, 446 kB/s
[   5.2469 ]
[   5.2470 ] Concatenating BCT for chain A to bct_backup.img

[   5.2470 ] dd if=br_bct_BR.bct of=bct_backup.img bs=1 seek=0 conv=notrunc
[   5.2477 ] 8192+0 records in
[   5.2614 ] 8192+0 records out
[   5.2614 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0130839 s, 626 kB/s
[   5.2614 ]
[   5.2614 ] Concatenating BCT for chain B to bct_backup.img

[   5.2614 ] dd if=br_bct_b_BR.bct of=bct_backup.img bs=1 seek=16384 conv=notrunc
[   5.2622 ] 8192+0 records in
[   5.2766 ] 8192+0 records out
[   5.2767 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0139127 s, 589 kB/s
[   5.2767 ]
[   5.2767 ] Generating signatures
[   5.2776 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.2778 ] Assuming zero filled SBK key
[   5.3880 ] Warning: pub_key.key is not found
[   5.3898 ] Parsing dev params for multi chains
[   5.3898 ] Generating br-bct
[   5.3902 ] Updating dev and MSS params in BR BCT
[   5.3903 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.5605 ] Updating customer data section
[   5.5610 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   5.5635 ] Updating bl info
[   5.5641 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   5.5672 ] Generating SHA2 Hash
[   5.5691 ] Sha saved in br_bct_BR.sha
[   5.5685 ] Get Signed section of bct
[   5.5691 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   5.5698 ] Signing BCT
[   5.5706 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.5708 ] Assuming zero filled SBK key
[   5.5733 ] Sha saved in br_bct_BR.sha
[   5.5735 ] Warning: pub_key.key is not found
[   5.5727 ] Updating BCT with signature
[   5.5732 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   5.5736 ] Offset :4608 Len :3584
[   5.5751 ] Generating SHA2 Hash
[   5.5760 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   5.5761 ] Assuming zero filled SBK key
[   5.5761 ] Assuming zero filled SBK key
[   5.5785 ] Sha saved in br_bct_BR.sha
[   5.5779 ] Updating BCT with SHA2 Hash
[   5.5783 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   5.5786 ] Offset :4608 Len :3584
[   5.5791 ] Offset :68 Len :8124
[   5.5802 ] Generating br-bct
[   5.5806 ] Updating dev and MSS params in BR BCT
[   5.5806 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.7634 ] Updating customer data section
[   5.7640 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   5.7662 ] Updating bl info
[   5.7667 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   5.7687 ] Generating SHA2 Hash
[   5.7708 ] Sha saved in br_bct_BR.sha
[   5.7701 ] Get Signed section of bct
[   5.7705 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   5.7714 ] Signing BCT
[   5.7722 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.7724 ] Assuming zero filled SBK key
[   5.7755 ] Sha saved in br_bct_BR.sha
[   5.7758 ] Warning: pub_key.key is not found
[   5.7750 ] Updating BCT with signature
[   5.7755 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   5.7759 ] Offset :4608 Len :3584
[   5.7766 ] Generating SHA2 Hash
[   5.7774 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   5.7775 ] Assuming zero filled SBK key
[   5.7775 ] Assuming zero filled SBK key
[   5.7806 ] Sha saved in br_bct_BR.sha
[   5.7801 ] Updating BCT with SHA2 Hash
[   5.7806 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   5.7810 ] Offset :4608 Len :3584
[   5.7814 ] Offset :68 Len :8124
[   5.7818 ] Generating BCT backup image
[   5.7818 ] dd if=/dev/zero of=bct_backup.img bs=1 count=32768
[   5.7825 ] 32768+0 records in
[   5.8573 ] 32768+0 records out
[   5.8573 ] 32768 bytes (33 kB, 32 KiB) copied, 0.0740852 s, 442 kB/s
[   5.8573 ]
[   5.8573 ] Concatenating BCT for chain A to bct_backup.img

[   5.8573 ] dd if=br_bct_BR.bct of=bct_backup.img bs=1 seek=0 conv=notrunc
[   5.8581 ] 8192+0 records in
[   5.8719 ] 8192+0 records out
[   5.8719 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0132444 s, 619 kB/s
[   5.8719 ]
[   5.8720 ] Concatenating BCT for chain B to bct_backup.img

[   5.8720 ] dd if=br_bct_b_BR.bct of=bct_backup.img bs=1 seek=16384 conv=notrunc
[   5.8725 ] 8192+0 records in
[   5.8866 ] 8192+0 records out
[   5.8866 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0134572 s, 609 kB/s
[   5.8866 ]
[   5.8867 ] Generating coldboot mb1-bct
[   5.8872 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   5.8876 ] MB1-BCT version: 0.13

[   5.8895 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   5.8899 ] Added Platform Config 0 data with size :- 2416

[   5.8924 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   5.8926 ] WARNING: unknown node 'g2'
[   5.8928 ] WARNING: unknown node 'g2'
[   5.8929 ] WARNING: unknown node 'g9'
[   5.8929 ] WARNING: unknown node 'g9'
[   5.8930 ] Added Platform Config 2 data with size :- 24
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   5.8931 ] Added Platform Config 4 data with size :- 288
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   5.8931 ] Added Platform Config 3 data with size :- 52
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   5.8931 ] WARNING: unknown property 'major'
[   5.8931 ] WARNING: unknown property 'minor'
[   5.8931 ] Added Platform Config 5 data with size :- 512
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   5.8931 ] WARNING: unknown property 'major'
[   5.8931 ] WARNING: unknown property 'minor'
[   5.8931 ] Added Platform Config 7 data with size :- 380
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   5.8931 ] Added Platform Config 8 data with size :- 24
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   5.8931 ] Added Platform Config 9 data with size :- 100
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   5.8931 ] ModuleCount 0 NumProdNames 0
[   5.8931 ] Added Platform Config 6 data with size :- 16
[   5.8931 ]
[   5.8931 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   5.8931 ] ERROR: /ratchet/atf is not supported
[   5.8931 ]
[   5.8932 ] Updating mb1-bct with firmware information
[   5.8935 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   5.8947 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   5.8955 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   5.8958 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   5.8975 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.8977 ] Assuming zero filled SBK key
[   5.8993 ] Warning: pub_key.key is not found
[   5.8990 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   5.9000 ] Generating recovery mb1-bct
[   5.9004 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   5.9008 ] MB1-BCT version: 0.13

[   5.9026 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   5.9029 ] Added Platform Config 0 data with size :- 2416

[   5.9053 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   5.9057 ] WARNING: unknown node 'g2'
[   5.9058 ] WARNING: unknown node 'g2'
[   5.9059 ] WARNING: unknown node 'g9'
[   5.9060 ] WARNING: unknown node 'g9'
[   5.9061 ] Added Platform Config 2 data with size :- 24
[   5.9062 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   5.9063 ] Added Platform Config 4 data with size :- 288
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   5.9063 ] Added Platform Config 3 data with size :- 52
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   5.9063 ] WARNING: unknown property 'major'
[   5.9063 ] WARNING: unknown property 'minor'
[   5.9063 ] Added Platform Config 5 data with size :- 512
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   5.9063 ] WARNING: unknown property 'major'
[   5.9063 ] WARNING: unknown property 'minor'
[   5.9063 ] Added Platform Config 7 data with size :- 380
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   5.9063 ] Added Platform Config 8 data with size :- 24
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   5.9063 ] Added Platform Config 9 data with size :- 100
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   5.9063 ] ModuleCount 0 NumProdNames 0
[   5.9063 ] Added Platform Config 6 data with size :- 16
[   5.9063 ]
[   5.9063 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   5.9063 ] ERROR: /ratchet/atf is not supported
[   5.9063 ]
[   5.9063 ] Updating mb1-bct with firmware information
[   5.9066 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo flash.xml.bin
[   5.9078 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   5.9083 ] Generating SHA2 Hash for mb1bct
[   5.9103 ] Sha saved in mb1_bct_MB1_aligned.sha
[   5.9099 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   5.9103 ] adding BCH for mb1_bct_MB1_aligned.bct
[   5.9121 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.9123 ] Assuming zero filled SBK key
[   5.9137 ] Warning: pub_key.key is not found
[   5.9134 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   5.9145 ] Generating coldboot mem-bct
[   5.9149 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   5.9152 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   6.0853 ] Packing sdram param for instance[0]
[   6.0855 ] Packing sdram param for instance[1]
[   6.0857 ] Packing sdram param for instance[2]
[   6.0859 ] Packing sdram param for instance[3]
[   6.0861 ] Packing sdram param for instance[4]
[   6.0862 ] Packing sdram param for instance[5]
[   6.0864 ] Packing sdram param for instance[6]
[   6.0866 ] Packing sdram param for instance[7]
[   6.0868 ] Packing sdram param for instance[8]
[   6.0869 ] Packing sdram param for instance[9]
[   6.0871 ] Packing sdram param for instance[10]
[   6.0873 ] Packing sdram param for instance[11]
[   6.0875 ] Packing sdram param for instance[12]
[   6.0876 ] Packing sdram param for instance[13]
[   6.0878 ] Packing sdram param for instance[14]
[   6.0879 ] Packing sdram param for instance[15]
[   6.2702 ] Getting sector size from pt
[   6.2707 ] tegraparser_v2 --getsectorsize flash.xml.bin sector_info.bin
[   6.2714 ] BlockSize read from layout is 0x200

[   6.2717 ] tegrahost_v2 --chip 0x23 0 --blocksize 512 --magicid MEMB --addsigheader_multi tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   6.2722 ] Binary 0 length is 58752
[   6.2724 ] Binary 0 align length is 58880
[   6.2736 ] Binary 1 length is 58752
[   6.2737 ] Binary 1 align length is 58880
[   6.2748 ] Binary 2 length is 58752
[   6.2749 ] Binary 2 align length is 58880
[   6.2759 ] Binary 3 length is 58752
[   6.2760 ] Binary 3 align length is 58880
[   6.2771 ] Buffer length is 235520
[   6.2771 ] adding BCH for tegra234-p3767-0001-sdram-l4t_cpp_1.bct
[   6.2773 ] new length is 243712
[   6.2784 ] tegrahost_v2 --chip 0x23 0 --align mem_coldboot_aligned.bct
[   6.2793 ] tegrahost_v2 --chip 0x23 0 --magicid MEMB --ratchet_blob ratchet_blob.bin --appendsigheader mem_coldboot_aligned.bct zerosbk
[   6.2796 ] Header already present for mem_coldboot_aligned.bct
[   6.2815 ] tegrasign_v3.py --key None --list mem_coldboot_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.2817 ] Assuming zero filled SBK key
[   6.2836 ] Warning: pub_key.key is not found
[   6.2833 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_coldboot_aligned_sigheader.bct.encrypt mem_coldboot_aligned_sigheader.bct.hash zerosbk
[   6.2849 ] Generating recovery mem-bct
[   6.2853 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   6.2856 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   6.4551 ] Packing sdram param for instance[0]
[   6.4553 ] Packing sdram param for instance[1]
[   6.4554 ] Packing sdram param for instance[2]
[   6.4555 ] Packing sdram param for instance[3]
[   6.4557 ] Packing sdram param for instance[4]
[   6.4558 ] Packing sdram param for instance[5]
[   6.4560 ] Packing sdram param for instance[6]
[   6.4561 ] Packing sdram param for instance[7]
[   6.4562 ] Packing sdram param for instance[8]
[   6.4564 ] Packing sdram param for instance[9]
[   6.4565 ] Packing sdram param for instance[10]
[   6.4567 ] Packing sdram param for instance[11]
[   6.4568 ] Packing sdram param for instance[12]
[   6.4570 ] Packing sdram param for instance[13]
[   6.4571 ] Packing sdram param for instance[14]
[   6.4572 ] Packing sdram param for instance[15]
[   6.6417 ] Reading ramcode from backup chip_info.bin file
[   6.6418 ] RAMCODE Read from Device: 2

[   6.6418 ] Using ramcode 0
[   6.6418 ] Disabled BPMP dtb trim, using default dtb
[   6.6418 ]
[   6.6426 ] tegrahost_v2 --chip 0x23 0 --align mem_rcm_aligned.bct
[   6.6437 ] tegrahost_v2 --chip 0x23 0 --magicid MEM0 --ratchet_blob ratchet_blob.bin --appendsigheader mem_rcm_aligned.bct zerosbk
[   6.6441 ] adding BCH for mem_rcm_aligned.bct
[   6.6479 ] tegrasign_v3.py --key None --list mem_rcm_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.6480 ] Assuming zero filled SBK key
[   6.6495 ] Warning: pub_key.key is not found
[   6.6493 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_rcm_aligned_sigheader.bct.encrypt mem_rcm_aligned_sigheader.bct.hash zerosbk
[   6.6505 ] Copying signatures
[   6.6510 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --updatesig images_list_signed.xml
[   6.7485 ] tegraparser_v2 --generategpt --pt flash.xml.bin
[   6.7491 ] gpt_secondary_3_0.bin:
[   6.7494 ] partition_id       partition_name                        StartingLba       EndingLba
[   6.7497 ]            1       BCT                                            0            2047
[   6.7500 ]            2       A_mb1                                       2048            3071
[   6.7504 ]            3       A_psc_bl1                                   3072            3583
[   6.7507 ]            4       A_MB1_BCT                                   3584            3839
[   6.7511 ]            5       A_MEM_BCT                                   3840            4351
[   6.7514 ]            6       A_tsec-fw                                   4352            6399
[   6.7518 ]            7       A_nvdec                                     6400            8447
[   6.7521 ]            8       A_mb2                                       8448            9471
[   6.7526 ]            9       A_xusb-fw                                   9472            9983
[   6.7526 ]           10       A_bpmp-fw                                   9984           13055
[   6.7526 ]           11       A_bpmp-fw-dtb                              13056           21247
[   6.7526 ]           12       A_psc-fw                                   21248           22783
[   6.7526 ]           13       A_mts-mce                                  22784           23807
[   6.7526 ]           14       A_sc7                                      23808           24191
[   6.7526 ]           15       A_pscrf                                    24192           24575
[   6.7526 ]           16       A_mb2rf                                    24576           24831
[   6.7526 ]           17       A_cpu-bootloader                           24832           31999
[   6.7526 ]           18       A_secure-os                                32000           40191
[   6.7526 ]           19       A_smm-fw                                   40192           44287
[   6.7526 ]           20       A_eks                                      44288           44799
[   6.7526 ]           21       A_dce-fw                                   44800           55039
[   6.7526 ]           22       A_spe-fw                                   55040           56191
[   6.7526 ]           23       A_rce-fw                                   56192           58239
[   6.7526 ]           24       A_adsp-fw                                  58240           62335
[   6.7526 ]           25       A_pva-fw                                   62336           62847
[   6.7526 ]           26       A_reserved_on_boot                         62848           65023
[   6.7526 ]           27       B_mb1                                      65024           66047
[   6.7526 ]           28       B_psc_bl1                                  66048           66559
[   6.7526 ]           29       B_MB1_BCT                                  66560           66815
[   6.7526 ]           30       B_MEM_BCT                                  66816           67327
[   6.7526 ]           31       B_tsec-fw                                  67328           69375
[   6.7526 ]           32       B_nvdec                                    69376           71423
[   6.7526 ]           33       B_mb2                                      71424           72447
[   6.7526 ]           34       B_xusb-fw                                  72448           72959
[   6.7526 ]           35       B_bpmp-fw                                  72960           76031
[   6.7526 ]           36       B_bpmp-fw-dtb                              76032           84223
[   6.7526 ]           37       B_psc-fw                                   84224           85759
[   6.7526 ]           38       B_mts-mce                                  85760           86783
[   6.7526 ]           39       B_sc7                                      86784           87167
[   6.7526 ]           40       B_pscrf                                    87168           87551
[   6.7526 ]           41       B_mb2rf                                    87552           87807
[   6.7526 ]           42       B_cpu-bootloader                           87808           94975
[   6.7526 ]           43       B_secure-os                                94976          103167
[   6.7526 ]           44       B_smm-fw                                  103168          107263
[   6.7526 ]           45       B_eks                                     107264          107775
[   6.7526 ]           46       B_dce-fw                                  107776          118015
[   6.7526 ]           47       B_spe-fw                                  118016          119167
[   6.7526 ]           48       B_rce-fw                                  119168          121215
[   6.7526 ]           49       B_adsp-fw                                 121216          125311
[   6.7526 ]           50       B_pva-fw                                  125312          125823
[   6.7526 ]           51       B_reserved_on_boot                        125824          127999
[   6.7526 ]           52       uefi_variables                            128000          128511
[   6.7526 ]           53       uefi_ftw                                  128512          129535
[   6.7526 ]           54       reserved                                  129536          129919
[   6.7526 ]           55       worm                                      129920          130303
[   6.7526 ]           56       BCT-boot-chain_backup                     130304          130431
[   6.7526 ]           57       reserved_partition                        130432          130559
[   6.7526 ]           58       secondary_gpt_backup                      130560          130687
[   6.7526 ]           59       B_VER                                     130688          130815
[   6.7526 ]           60       A_VER                                     130816          130943
[   6.7526 ]
[   6.7543 ] Get magic id
[   6.7548 ] tegraparser_v2 --get_magic psc_fw
[   6.7551 ] PFWP
[   6.7552 ] partition type psc_fw, magic id = PFWP
[   6.7561 ] tegrahost_v2 --chip 0x23 0 --align pscfw_t234_prod_aligned.bin
[   6.7569 ] tegrahost_v2 --chip 0x23 0 --magicid PFWP --ratchet_blob ratchet_blob.bin --appendsigheader pscfw_t234_prod_aligned.bin zerosbk
[   6.7573 ] Header already present for pscfw_t234_prod_aligned.bin
[   6.7645 ] tegrasign_v3.py --key None --list pscfw_t234_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.7647 ] Assuming zero filled SBK key
[   6.7665 ] Warning: pub_key.key is not found
[   6.7663 ] tegrahost_v2 --chip 0x23 0 --updatesigheader pscfw_t234_prod_aligned_sigheader.bin.encrypt pscfw_t234_prod_aligned_sigheader.bin.hash zerosbk
[   6.7681 ] Get magic id
[   6.7686 ] tegraparser_v2 --get_magic mts_mce
[   6.7690 ] MTSM
[   6.7691 ] partition type mts_mce, magic id = MTSM
[   6.7699 ] tegrahost_v2 --chip 0x23 0 --align mce_flash_o10_cr_prod_aligned.bin
[   6.7708 ] tegrahost_v2 --chip 0x23 0 --magicid MTSM --ratchet_blob ratchet_blob.bin --appendsigheader mce_flash_o10_cr_prod_aligned.bin zerosbk
[   6.7711 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   6.7760 ] tegrasign_v3.py --key None --list mce_flash_o10_cr_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.7761 ] Assuming zero filled SBK key
[   6.7779 ] Warning: pub_key.key is not found
[   6.7776 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mce_flash_o10_cr_prod_aligned_sigheader.bin.encrypt mce_flash_o10_cr_prod_aligned_sigheader.bin.hash zerosbk
[   6.7791 ] Get magic id
[   6.7796 ] tegraparser_v2 --get_magic tsec_fw
[   6.7799 ] TSEC
[   6.7800 ] partition type tsec_fw, magic id = TSEC
[   6.7808 ] tegrahost_v2 --chip 0x23 0 --align tsec_t234_aligned.bin
[   6.7818 ] tegrahost_v2 --chip 0x23 0 --magicid TSEC --ratchet_blob ratchet_blob.bin --appendsigheader tsec_t234_aligned.bin zerosbk
[   6.7822 ] Header already present for tsec_t234_aligned.bin
[   6.7868 ] tegrasign_v3.py --key None --list tsec_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.7870 ] Assuming zero filled SBK key
[   6.7886 ] Warning: pub_key.key is not found
[   6.7884 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tsec_t234_aligned_sigheader.bin.encrypt tsec_t234_aligned_sigheader.bin.hash zerosbk
[   6.7898 ] Get magic id
[   6.7903 ] tegraparser_v2 --get_magic mb2_applet
[   6.7906 ] MB2A
[   6.7907 ] partition type mb2_applet, magic id = MB2A
[   6.7915 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   6.7925 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --ratchet_blob ratchet_blob.bin --appendsigheader applet_t234_aligned.bin zerosbk
[   6.7929 ] adding BCH for applet_t234_aligned.bin
[   6.8043 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8044 ] Assuming zero filled SBK key
[   6.8062 ] Warning: pub_key.key is not found
[   6.8060 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   6.8082 ] Generating recovery mb2-bct
[   6.8082 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_bct.cfg --recov --mb2bctcfg tegra234-mb2-bct-misc-p3767-0000_cpp.dtb --scr tegra234-mb2-bct-scr-p3767-0000_cpp.dtb
[   6.8085 ] ERROR: value 0x31 is out of range
[   6.8096 ] ERROR: value 0x31 is out of range
[   6.8098 ] ERROR: value 0x31 is out of range
[   6.8100 ] ERROR: value 0x31 is out of range
[   6.8101 ] WARNING: unknown property 'tfc_version'
[   6.8103 ] WARNING: unknown property 'addr_header_version'
[   6.8225 ] Updating mb2-bct with storage information for RCM
[   6.8230 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_bct_MB2.bct --updatestorageinfo flash.xml.bin
[   6.8240 ] Concatenating mb2-bct to mb2 binary
[   6.8240 ] mb2_bin_file = mb2_t234.bin
[   6.8240 ] mb2_bct_file = mb2_bct_MB2.bct
[   6.8245 ] Get magic id
[   6.8249 ] tegraparser_v2 --get_magic mb2_bootloader
[   6.8253 ] MB2B
[   6.8255 ] partition type mb2_bootloader, magic id = MB2B
[   6.8264 ] tegrahost_v2 --chip 0x23 0 --align mb2_t234_with_mb2_bct_MB2_aligned.bin
[   6.8274 ] tegrahost_v2 --chip 0x23 0 --magicid MB2B --ratchet_blob ratchet_blob.bin --appendsigheader mb2_t234_with_mb2_bct_MB2_aligned.bin zerosbk
[   6.8278 ] adding BCH for mb2_t234_with_mb2_bct_MB2_aligned.bin
[   6.8440 ] tegrasign_v3.py --key None --list mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8442 ] Assuming zero filled SBK key
[   6.8464 ] Warning: pub_key.key is not found
[   6.8462 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin.encrypt mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin.hash zerosbk
[   6.8482 ] Get magic id
[   6.8487 ] tegraparser_v2 --get_magic xusb_fw
[   6.8491 ] XUSB
[   6.8493 ] partition type xusb_fw, magic id = XUSB
[   6.8503 ] tegrahost_v2 --chip 0x23 0 --align xusb_t234_prod_aligned.bin
[   6.8512 ] tegrahost_v2 --chip 0x23 0 --magicid XUSB --ratchet_blob ratchet_blob.bin --appendsigheader xusb_t234_prod_aligned.bin zerosbk
[   6.8516 ] adding BCH for xusb_t234_prod_aligned.bin
[   6.8587 ] tegrasign_v3.py --key None --list xusb_t234_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8589 ] Assuming zero filled SBK key
[   6.8605 ] Warning: pub_key.key is not found
[   6.8603 ] tegrahost_v2 --chip 0x23 0 --updatesigheader xusb_t234_prod_aligned_sigheader.bin.encrypt xusb_t234_prod_aligned_sigheader.bin.hash zerosbk
[   6.8617 ] Get magic id
[   6.8621 ] tegraparser_v2 --get_magic pva_fw
[   6.8625 ] PVAF
[   6.8626 ] partition type pva_fw, magic id = PVAF
[   6.8654 ] tegrahost_v2 --chip 0x23 0 --align nvpva_020_aligned.fw
[   6.8662 ] tegrahost_v2 --chip 0x23 0 --magicid PVAF --ratchet_blob ratchet_blob.bin --appendsigheader nvpva_020_aligned.fw zerosbk
[   6.8666 ] adding BCH for nvpva_020_aligned.fw
[   6.9467 ] tegrasign_v3.py --key None --list nvpva_020_aligned_sigheader.fw_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.9468 ] Assuming zero filled SBK key
[   6.9526 ] Warning: pub_key.key is not found
[   6.9524 ] tegrahost_v2 --chip 0x23 0 --updatesigheader nvpva_020_aligned_sigheader.fw.encrypt nvpva_020_aligned_sigheader.fw.hash zerosbk
[   6.9594 ] Kernel DTB used: tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   6.9594 ] Concatenating kernel-dtb to dce-fw binary
[   6.9594 ] dce_bin = display-t234-dce.bin
[   6.9594 ] kernel_dtb = tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   6.9594 ] dce_with_dtb = display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.bin
[   6.9733 ] Get magic id
[   6.9739 ] tegraparser_v2 --get_magic dce_fw
[   6.9742 ] DCEF
[   6.9744 ] partition type dce_fw, magic id = DCEF
[   7.0396 ] tegrahost_v2 --chip 0x23 0 --align display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin
[   7.0411 ] tegrahost_v2 --chip 0x23 0 --magicid DCEF --ratchet_blob ratchet_blob.bin --appendsigheader display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned.bin zerosbk compress
[   7.0416 ] INFO: compressing display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned.bin
[   7.2690 ] INFO: complete compression, display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned.bin, ratio = 6%
[   7.2945 ] adding BCH for display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned_blob_w_bin.bin
[   7.3312 ] tegrasign_v3.py --key None --list display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned_blob_w_bin_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.3313 ] Assuming zero filled SBK key
[   7.3342 ] Warning: pub_key.key is not found
[   7.3341 ] tegrahost_v2 --chip 0x23 0 --updatesigheader display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned_blob_w_bin_sigheader.bin.encrypt display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_aligned_blob_w_bin_sigheader.bin.hash zerosbk
[   7.3385 ] Get magic id
[   7.3391 ] tegraparser_v2 --get_magic nvdec
[   7.3395 ] NDEC
[   7.3397 ] partition type nvdec, magic id = NDEC
[   7.3409 ] tegrahost_v2 --chip 0x23 0 --align nvdec_t234_prod_aligned.fw
[   7.3421 ] tegrahost_v2 --chip 0x23 0 --magicid NDEC --ratchet_blob ratchet_blob.bin --appendsigheader nvdec_t234_prod_aligned.fw zerosbk
[   7.3426 ] Header already present for nvdec_t234_prod_aligned.fw
[   7.3501 ] tegrasign_v3.py --key None --list nvdec_t234_prod_aligned_sigheader.fw_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.3503 ] Assuming zero filled SBK key
[   7.3523 ] Warning: pub_key.key is not found
[   7.3523 ] tegrahost_v2 --chip 0x23 0 --updatesigheader nvdec_t234_prod_aligned_sigheader.fw.encrypt nvdec_t234_prod_aligned_sigheader.fw.hash zerosbk
[   7.3543 ] Get magic id
[   7.3549 ] tegraparser_v2 --get_magic bpmp_fw
[   7.3554 ] BPMF
[   7.3556 ] partition type bpmp_fw, magic id = BPMF
[   7.3574 ] tegrahost_v2 --chip 0x23 0 --align bpmp_t234-TE950M-A1_prod_aligned.bin
[   7.3583 ] tegrahost_v2 --chip 0x23 0 --magicid BPMF --ratchet_blob ratchet_blob.bin --appendsigheader bpmp_t234-TE950M-A1_prod_aligned.bin zerosbk
[   7.3587 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   7.3808 ] tegrasign_v3.py --key None --list bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.3810 ] Assuming zero filled SBK key
[   7.3841 ] Warning: pub_key.key is not found
[   7.3840 ] tegrahost_v2 --chip 0x23 0 --updatesigheader bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin.encrypt bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin.hash zerosbk
[   7.3882 ] Using bpmp-dtb concatenated with odmdata
[   7.3882 ] Get magic id
[   7.3887 ] tegraparser_v2 --get_magic bpmp_fw_dtb
[   7.3892 ] BPMD
[   7.3893 ] partition type bpmp_fw_dtb, magic id = BPMD
[   7.3901 ] tegrahost_v2 --chip 0x23 0 --align tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   7.3911 ] tegrahost_v2 --chip 0x23 0 --magicid BPMD --ratchet_blob ratchet_blob.bin --appendsigheader tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb zerosbk
[   7.3915 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   7.4005 ] tegrasign_v3.py --key None --list tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.4007 ] Assuming zero filled SBK key
[   7.4032 ] Warning: pub_key.key is not found
[   7.4030 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb.encrypt tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb.hash zerosbk
[   7.4047 ] Get magic id
[   7.4052 ] tegraparser_v2 --get_magic rce_fw
[   7.4055 ] RCEF
[   7.4056 ] partition type rce_fw, magic id = RCEF
[   7.4067 ] tegrahost_v2 --chip 0x23 0 --align camera-rtcpu-t234-rce_aligned.img
[   7.4076 ] tegrahost_v2 --chip 0x23 0 --magicid RCEF --ratchet_blob ratchet_blob.bin --appendsigheader camera-rtcpu-t234-rce_aligned.img zerosbk
[   7.4079 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   7.4257 ] tegrasign_v3.py --key None --list camera-rtcpu-t234-rce_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.4259 ] Assuming zero filled SBK key
[   7.4280 ] Warning: pub_key.key is not found
[   7.4277 ] tegrahost_v2 --chip 0x23 0 --updatesigheader camera-rtcpu-t234-rce_aligned_sigheader.img.encrypt camera-rtcpu-t234-rce_aligned_sigheader.img.hash zerosbk
[   7.4300 ] Get magic id
[   7.4305 ] tegraparser_v2 --get_magic ape_fw
[   7.4309 ] APEF
[   7.4310 ] partition type ape_fw, magic id = APEF
[   7.4320 ] tegrahost_v2 --chip 0x23 0 --align adsp-fw_aligned.bin
[   7.4330 ] tegrahost_v2 --chip 0x23 0 --magicid APEF --ratchet_blob ratchet_blob.bin --appendsigheader adsp-fw_aligned.bin zerosbk
[   7.4334 ] adding BCH for adsp-fw_aligned.bin
[   7.4493 ] tegrasign_v3.py --key None --list adsp-fw_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.4494 ] Assuming zero filled SBK key
[   7.4513 ] Warning: pub_key.key is not found
[   7.4510 ] tegrahost_v2 --chip 0x23 0 --updatesigheader adsp-fw_aligned_sigheader.bin.encrypt adsp-fw_aligned_sigheader.bin.hash zerosbk
[   7.4531 ] Get magic id
[   7.4535 ] tegraparser_v2 --get_magic spe_fw
[   7.4539 ] SPEF
[   7.4540 ] partition type spe_fw, magic id = SPEF
[   7.4549 ] tegrahost_v2 --chip 0x23 0 --align spe_t234_aligned.bin
[   7.4557 ] tegrahost_v2 --chip 0x23 0 --magicid SPEF --ratchet_blob ratchet_blob.bin --appendsigheader spe_t234_aligned.bin zerosbk
[   7.4560 ] adding BCH for spe_t234_aligned.bin
[   7.4667 ] tegrasign_v3.py --key None --list spe_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.4668 ] Assuming zero filled SBK key
[   7.4686 ] Warning: pub_key.key is not found
[   7.4683 ] tegrahost_v2 --chip 0x23 0 --updatesigheader spe_t234_aligned_sigheader.bin.encrypt spe_t234_aligned_sigheader.bin.hash zerosbk
[   7.4700 ] Get magic id
[   7.4704 ] tegraparser_v2 --get_magic tos
[   7.4708 ] TOSB
[   7.4710 ] partition type tos, magic id = TOSB
[   7.4728 ] tegrahost_v2 --chip 0x23 0 --align tos-optee_t234_aligned.img
[   7.4738 ] tegrahost_v2 --chip 0x23 0 --magicid TOSB --ratchet_blob ratchet_blob.bin --appendsigheader tos-optee_t234_aligned.img zerosbk
[   7.4742 ] adding BCH for tos-optee_t234_aligned.img
[   7.5191 ] tegrasign_v3.py --key None --list tos-optee_t234_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5192 ] Assuming zero filled SBK key
[   7.5223 ] Warning: pub_key.key is not found
[   7.5220 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tos-optee_t234_aligned_sigheader.img.encrypt tos-optee_t234_aligned_sigheader.img.hash zerosbk
[   7.5262 ] Get magic id
[   7.5267 ] tegraparser_v2 --get_magic eks
[   7.5271 ] EKSB
[   7.5273 ] partition type eks, magic id = EKSB
[   7.5279 ] tegrahost_v2 --chip 0x23 0 --align eks_t234_aligned.img
[   7.5289 ] tegrahost_v2 --chip 0x23 0 --magicid EKSB --ratchet_blob ratchet_blob.bin --appendsigheader eks_t234_aligned.img zerosbk
[   7.5292 ] adding BCH for eks_t234_aligned.img
[   7.5310 ] tegrasign_v3.py --key None --list eks_t234_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5311 ] Assuming zero filled SBK key
[   7.5326 ] Warning: pub_key.key is not found
[   7.5323 ] tegrahost_v2 --chip 0x23 0 --updatesigheader eks_t234_aligned_sigheader.img.encrypt eks_t234_aligned_sigheader.img.hash zerosbk
[   7.5375 ] tegrahost_v2 --chip 0x23 0 --align uefi_jetson_with_dtb_aligned.bin
[   7.5385 ] tegrahost_v2 --chip 0x23 0 --magicid CPBL --ratchet_blob ratchet_blob.bin --appendsigheader uefi_jetson_with_dtb_aligned.bin zerosbk
[   7.5389 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   7.6562 ] tegrasign_v3.py --key None --list uefi_jetson_with_dtb_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.6563 ] Assuming zero filled SBK key
[   7.6642 ] Warning: pub_key.key is not found
[   7.6640 ] tegrahost_v2 --chip 0x23 0 --updatesigheader uefi_jetson_with_dtb_aligned_sigheader.bin.encrypt uefi_jetson_with_dtb_aligned_sigheader.bin.hash zerosbk
[   7.6736 ] Copying enc\/signed file in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7222 ] Copying br bct for multi chains
[   7.7224 ] Signed BCT for boot chain A is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_BR.bct

[   7.7225 ] Signed BCT for boot chain B is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_b_BR.bct

[   7.7225 ] Copying BCT backup image bct_backup.img to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bct_backup.img
[   7.7263 ] Copying pscfw_t234_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7267 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt
[   7.7267 ] Copying mce_flash_o10_cr_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7271 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt
[   7.7271 ] Copying tsec_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7274 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt
[   7.7274 ] Copying applet_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7277 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/applet_t234_sigheader.bin.encrypt
[   7.7277 ] Copying mb2_t234_with_mb2_bct_MB2_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7281 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_bct_MB2_sigheader.bin.encrypt
[   7.7282 ] Copying xusb_t234_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7285 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt
[   7.7285 ] Copying nvpva_020_sigheader.fw.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7303 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_sigheader.fw.encrypt
[   7.7303 ] Copying display-t234-dce_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7311 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_sigheader.bin.encrypt
[   7.7311 ] Copying display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_blob_w_bin_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7318 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_blob_w_bin_sigheader.bin.encrypt
[   7.7318 ] Copying nvdec_t234_prod_sigheader.fw.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7322 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt
[   7.7322 ] Copying bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7336 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
[   7.7336 ] Copying tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7339 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
[   7.7339 ] Copying camera-rtcpu-t234-rce_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7345 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt
[   7.7345 ] Copying adsp-fw_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7351 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt
[   7.7351 ] Copying spe_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7355 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt
[   7.7356 ] Copying tos-optee_t234_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7372 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt
[   7.7372 ] Copying eks_t234_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7373 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt
[   7.7373 ] Copying uefi_jetson_with_dtb_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.7416 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt
[   7.7435 ] tegraparser_v2 --pt flash.xml.bin --generateflashindex /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.xml.tmp flash.idx
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.idx' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/flash.idx'
Flash index file is /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.idx
Number of lines is 61
max_index=60
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_BR.bct  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/br_bct_BR.bct
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_BR.bct' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/br_bct_BR.bct'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_t234_prod_aligned_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_t234_prod_aligned_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mem_coldboot_sigheader.bct.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mem_coldboot_sigheader.bct.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mem_coldboot_sigheader.bct.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mem_coldboot_sigheader.bct.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tsec_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tsec_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvdec_t234_prod_sigheader.fw.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvdec_t234_prod_sigheader.fw.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/xusb_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/xusb_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/pscfw_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/pscfw_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/sc7_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/sc7_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/sc7_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/sc7_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_rf_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_rf_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_rf_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_rf_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2rf_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2rf_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2rf_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2rf_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tos-optee_t234_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tos-optee_t234_sigheader.img.encrypt'
Warning: skip writing A_smm-fw partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/eks_t234_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/eks_t234_sigheader.img.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/spe_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/spe_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/adsp-fw_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/adsp-fw_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt'
Warning: skip writing A_reserved_on_boot partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_t234_prod_aligned_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_t234_prod_aligned_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mem_coldboot_sigheader.bct.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mem_coldboot_sigheader.bct.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mem_coldboot_sigheader.bct.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mem_coldboot_sigheader.bct.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tsec_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tsec_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvdec_t234_prod_sigheader.fw.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvdec_t234_prod_sigheader.fw.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/xusb_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/xusb_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/pscfw_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/pscfw_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/sc7_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/sc7_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/sc7_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/sc7_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_rf_t234_prod_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_rf_t234_prod_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/psc_rf_t234_prod_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/psc_rf_t234_prod_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2rf_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2rf_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2rf_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/mb2rf_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tos-optee_t234_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/tos-optee_t234_sigheader.img.encrypt'
Warning: skip writing B_smm-fw partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/eks_t234_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/eks_t234_sigheader.img.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/spe_t234_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/spe_t234_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/adsp-fw_sigheader.bin.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/adsp-fw_sigheader.bin.encrypt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt'
Warning: skip writing B_reserved_on_boot partition as no image is specified
Warning: skip writing uefi_variables partition as no image is specified
Warning: skip writing uefi_ftw partition as no image is specified
Warning: skip writing reserved partition as no image is specified
Warning: skip writing worm partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bct_backup.img  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bct_backup.img
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bct_backup.img' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/bct_backup.img'
Warning: skip writing reserved_partition partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_backup_secondary_3_0.bin  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/gpt_backup_secondary_3_0.bin
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_backup_secondary_3_0.bin' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/gpt_backup_secondary_3_0.bin'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/qspi_bootblob_ver.txt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/qspi_bootblob_ver.txt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/qspi_bootblob_ver.txt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/qspi_bootblob_ver.txt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/qspi_bootblob_ver.txt  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/qspi_bootblob_ver.txt
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/qspi_bootblob_ver.txt' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/qspi_bootblob_ver.txt'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_secondary_3_0.bin  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/gpt_secondary_3_0.bin
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_secondary_3_0.bin' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/internal/gpt_secondary_3_0.bin'
Generate image for external storage devices
Generate images to be flashed
BOOTDEV=internal ADDITIONAL_DTB_OVERLAY="BootOrderNvme.dtbo"  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/flash.sh --no-flash --sign --external-device -c "tools/kernel_flash/flash_l4t_t234_nvme.xml"  jetson-orin-nano-devkit internal

###############################################################################
# L4T BSP Information:
# R36 , REVISION: 2.0
# User release: 0.0
###############################################################################
ECID is 0x80012344705DF1975C000000110300C0
copying emc_fuse_dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-diag-boot.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
./tegraflash.py --chip "0x23" --applet "/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin" --skipuid --cfg readinfo_t234_min_prod.xml --dev_params tegra234-br-bct-diag-boot.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --bins "mb2_applet applet_t234.bin" --cmd "dump eeprom cvm cvm.bin; dump try_custinfo custinfo_out.bin; reboot recovery"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0281 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.0286 ] File rcm_state open failed
[   0.0289 ] ERROR: failed to read rcm_state
[   0.0289 ]
[   0.0302 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0305 ] Assuming zero filled SBK key
[   0.0294 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.0366 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.0473 ] Parsing partition layout
[   0.0477 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0488 ] Kernel DTB used: None
[   0.0488 ] WARNING: dce base dtb is not provided

[   0.0488 ] Parsing partition layout
[   0.0493 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0500 ] Creating list of images to be signed
[   0.0504 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --list images_list.xml zerosbk
[   0.0507 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0519 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0524 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0581 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0588 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0643 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0646 ] adding BCH for mb2_t234_aligned.bin
[   0.0679 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0819 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0823 ] adding BCH for mb2_t234_aligned.bin
[   0.0990 ] Filling MB1 storage info
[   0.0990 ] Parsing dev params for multi chains
[   0.1064 ] Generating br-bct
[   0.1068 ] Updating dev and MSS params in BR BCT
[   0.1068 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1077 ] Updating bl info
[   0.1081 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin
[   0.1085 ] WARNING: boot chain is not completed. set to 0
[   0.1097 ] Generating signatures
[   0.1109 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1111 ] Assuming zero filled SBK key
[   0.1224 ] Warning: pub_key.key is not found
[   0.1214 ] Parsing dev params for multi chains
[   0.1214 ] Generating br-bct
[   0.1218 ] Updating dev and MSS params in BR BCT
[   0.1219 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1226 ] Updating bl info
[   0.1230 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1234 ] WARNING: boot chain is not completed. set to 0
[   0.1247 ] Generating SHA2 Hash
[   0.1270 ] Sha saved in br_bct_BR.sha
[   0.1259 ] Get Signed section of bct
[   0.1263 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   0.1269 ] Signing BCT
[   0.1282 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1283 ] Assuming zero filled SBK key
[   0.1307 ] Sha saved in br_bct_BR.sha
[   0.1309 ] Warning: pub_key.key is not found
[   0.1296 ] Updating BCT with signature
[   0.1301 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   0.1304 ] Offset :4608 Len :3584
[   0.1309 ] Generating SHA2 Hash
[   0.1322 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   0.1323 ] Assuming zero filled SBK key
[   0.1323 ] Assuming zero filled SBK key
[   0.1347 ] Sha saved in br_bct_BR.sha
[   0.1337 ] Updating BCT with SHA2 Hash
[   0.1341 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   0.1345 ] Offset :4608 Len :3584
[   0.1349 ] Offset :68 Len :8124
[   0.1351 ] Generating coldboot mb1-bct
[   0.1355 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1358 ] MB1-BCT version: 0.13

[   0.1377 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1380 ] Added Platform Config 9 data with size :- 100
[   0.1380 ]
[   0.1380 ] Updating mb1-bct with firmware information
[   0.1385 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1400 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   0.1410 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   0.1414 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   0.1436 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1437 ] Assuming zero filled SBK key
[   0.1452 ] Warning: pub_key.key is not found
[   0.1444 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1454 ] Generating recovery mb1-bct
[   0.1458 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1462 ] MB1-BCT version: 0.13

[   0.1481 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1483 ] Added Platform Config 9 data with size :- 100
[   0.1483 ]
[   0.1483 ] Updating mb1-bct with firmware information
[   0.1487 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1506 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   0.1511 ] Generating SHA2 Hash for mb1bct
[   0.1535 ] Sha saved in mb1_bct_MB1_aligned.sha
[   0.1527 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   0.1531 ] adding BCH for mb1_bct_MB1_aligned.bct
[   0.1553 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1554 ] Assuming zero filled SBK key
[   0.1570 ] Warning: pub_key.key is not found
[   0.1563 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1572 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1572 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.1572 ] Copying signatures
[   0.1577 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1659 ] mb1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1660 ] psc_bl1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.1660 ] Boot Rom communication
[   0.1665 ] tegrarcm_v2 --new_session --chip 0x23 0 --uid --download bct_br br_bct_BR.bct --download mb1 mb1_t234_prod_aligned_sigheader.bin.encrypt --download psc_bl1 psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
[   0.1669 ] BR_CID: 0x80012344705DF1975C000000110300C0
[   0.2120 ] Sending bct_br
[   0.2564 ] Sending mb1
[   0.2576 ] Sending psc_bl1
[   0.2705 ] Sending bct_mb1
[   0.2769 ] Boot Rom communication completed
[   0.2794 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   0.2821 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --appendsigheader applet_t234_aligned.bin zerosbk
[   0.2832 ] adding BCH for applet_t234_aligned.bin
[   0.3177 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.3180 ] Assuming zero filled SBK key
[   0.3215 ] Warning: pub_key.key is not found
[   0.3211 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   0.3240 ] Sending mb2_applet...

[   0.3247 ] tegrarcm_v2 --chip 0x23 0 --pollbl --download applet applet_t234_sigheader.bin.encrypt
[   0.3253 ] BL: version 1.4.0.1-t234-54845784-08e631ca last_boot_error: 0
[   0.5018 ] Sending applet
[   0.6020 ] completed
[   0.6034 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.6044 ] MB2 Applet version 01.00.0000
[   0.7881 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.7885 ] MB2 Applet version 01.00.0000
[   0.8355 ] Retrieving board information
[   0.8360 ] tegrarcm_v2 --chip 0x23 0 --oem platformdetails chip chip_info.bin
[   0.8364 ] MB2 Applet version 01.00.0000
[   0.8840 ] Saved platform info in chip_info.bin
[   0.8895 ] Chip minor revision: 1
[   0.8896 ] Bootrom revision: 0x7
[   0.8897 ] Ram code: 0x2
[   0.8898 ] Chip sku: 0xd5
[   0.8899 ] Chip Sample: prod
[   0.8905 ] Retrieving EEPROM data
[   0.8906 ] tegrarcm_v2 --oem platformdetails eeprom cvm /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin --chip 0x23 0
[   0.8910 ] MB2 Applet version 01.00.0000
[   0.9381 ] Saved platform info in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   0.9740 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.9753 ] MB2 Applet version 01.00.0000
[   1.0270 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.0281 ] MB2 Applet version 01.00.0000
[   1.0783 ] Dumping customer Info
[   1.0796 ] tegrarcm_v2 --chip 0x23 0 --oem dump bct tmp.bct
[   1.0807 ] MB2 Applet version 01.00.0000
[   1.1300 ] Saved bct in tmp.bct
[   1.1534 ] tegrabct_v2 --brbct tmp.bct --chip 0x23 0 --custinfo /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin
[   1.1545 ] Cu[   1.1554 ] stomer data saved in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin successfully
[   1.1555 ] Rebooting to recovery mode
[   1.1568 ] tegrarcm_v2 --chip 0x23 0 --ismb2
[   1.2070 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.2081 ] MB2 Applet version 01.00.0000
[   1.2550 ] Booting to recovery mode
[   1.2556 ] tegrarcm_v2 --chip 0x23 0 --reboot recovery
[   1.2561 ] MB2 Applet version 01.00.0000
Board ID(3767) version(300) sku(0005) revision(K.2)
Chip SKU(00:00:00:D5) ramcode(00:00:00:02) fuselevel(fuselevel_production) board_FAB(300)
emc_opt_disable_fuse:(0)
Copy /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec
Using UUID 08556824-5fd1-4132-9609-802682a2526c for mounting root APP_ext partition.
copying bctfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-sdram-l4t.dts)... done.
copying minratchet_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-ratchet-p3767-0000.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
copying pinmux_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi)... done.
copying gpioint_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-gpioint-p3767-0000.dts)... done.
copying pmic_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pmic-p3767-0000-a02.dts)... done.
copying pmc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi)... done.
copying deviceprod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-cprod-p3767-0000.dts)... done.
copying prod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-prod-p3767-0000.dts)... done.
copying scr_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-scr-p3767-0000.dts)... done.
copying wb0sdram(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-wb0sdram-l4t.dts)... done.
copying bootrom_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-reset-p3767-0000.dts)... done.
Existing uphylane_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegra234-mb1-bct-uphylane-si.dtsi) reused.
copying dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-p3767-0000-l4t.dts)... done.
copying dev_params_b(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct_b-p3767-0000-l4t.dts)... done.
copying mb2bct_cfg(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-misc-p3767-0000.dts)... done.
Existing pscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/pscfw_t234_prod.bin) reused.
Existing pscbl1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_bl1_t234_prod.bin) reused.
Existing mtsmcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mce_flash_o10_cr_prod.bin) reused.
Existing tscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tsec_t234.bin) reused.
Existing mb2applet(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/applet_t234.bin) reused.
Existing bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
copying initrd(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs/boot/initrd)... done.
bl is uefi
Making Boot image... done.
Not signing of boot.img
Making recovery ramdisk for recovery image...
Re-generating recovery ramdisk for recovery image...
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/ramdisk_tmp /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
52057 blocks

gzip: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/Image: not in gzip format
_BASE_KERNEL_VERSION=5.15.122-tegra
74756 blocks
Making Recovery image...
copying recdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec)... done.
20+0 records in
20+0 records out
20 bytes copied, 0.000237955 s, 84.0 kB/s
Existing sosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing tegraboot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing cpu_bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing mb2blfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing xusbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/xusb_t234_prod.bin) reused.
Existing pvafile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvpva_020.fw) reused.
Existing dcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/display-t234-dce.bin) reused.
Existing nvdecfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvdec_t234_prod.fw) reused.
Existing psc_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_rf_t234_prod.bin) reused.
Existing mb2_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2rf_t234.bin) reused.
Existing mb1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing bpffile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/bpmp_t234-TE950M-A1_prod.bin) reused.
copying bpfdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/tegra234-bpmp-3767-0003-3509-a02.dtb)... done.
Existing camerafw(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/camera-rtcpu-t234-rce.img) reused.
Existing apefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/adsp-fw.bin) reused.
Existing spefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/spe_t234.bin) reused.
Existing wb0boot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/sc7_t234_prod.bin) reused.
Existing tosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tos-optee_t234.img) reused.
Existing eksfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/eks_t234.img) reused.
copying dtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
Copying nv_boot_control.conf to rootfs
        populating kernel to rootfs... done.
        populating initrd to rootfs... done.
        populating kernel_tegra234-p3768-0000+p3767-0005-nv.dtb to rootfs... done.
Making system.img...
        Setting "FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb" successfully in the extlinux.conf...done.
        populating rootfs from /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs ...        populating /boot/extlinux/extlinux.conf ... done.
        Sync'ing system.img ... done.
        Converting RAW image to Sparse image... done.
system.img built successfully.
Not signing of kernel-dtb
Existing tbcfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/uefi_jetson.bin) reused.
131072+0 records in
131072+0 records out
67108864 bytes (67 MB, 64 MiB) copied, 0.81453 s, 82.4 MB/s
        Sync'ing esp.img ... done.
copying tbcdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
copying cfgfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/flash_l4t_t234_nvme.xml) to flash.xml... done.
Existing flashapp(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py) reused.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/L4TConfiguration.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-carveouts.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra-optee.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0000-dynamic.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/BootOrderNvme.dtbo)... done.
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration_updated.dts: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration.dtbo: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
./tegraflash.py  --bl uefi_jetson_with_dtb.bin  --odmdata gbe-uphy-config-8,hsstp-lane-map-3,hsio-uphy-config-0  --overlay_dtb L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo,BootOrderNvme.dtbo  --bldtb tegra234-p3768-0000+p3767-0005-nv.dtb --applet mb1_t234_prod.bin --cmd "sign"  --cfg flash.xml --chip "0x23" --concat_cpubl_bldtb --cpubl uefi_jetson.bin --minratchet_config tegra234-mb1-bct-ratchet-p3767-0000.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --pinmux_config tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi --gpioint_config tegra234-mb1-bct-gpioint-p3767-0000.dts --pmic_config tegra234-mb1-bct-pmic-p3767-0000-a02.dts --pmc_config tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi --deviceprod_config tegra234-mb1-bct-cprod-p3767-0000.dts --prod_config tegra234-mb1-bct-prod-p3767-0000.dts --scr_config tegra234-mb2-bct-scr-p3767-0000.dts --wb0sdram_config tegra234-p3767-0001-wb0sdram-l4t.dts --br_cmd_config tegra234-mb1-bct-reset-p3767-0000.dts --uphy tegra234-mb1-bct-uphylane-si.dtsi --dev_params tegra234-br-bct-p3767-0000-l4t.dts,tegra234-br-bct_b-p3767-0000-l4t.dts --mb2bct_cfg tegra234-mb2-bct-misc-p3767-0000.dts  --bins "psc_fw pscfw_t234_prod.bin; mts_mce mce_flash_o10_cr_prod.bin; tsec_fw tsec_t234.bin; mb2_applet applet_t234.bin; mb2_bootloader mb2_t234.bin; xusb_fw xusb_t234_prod.bin; pva_fw nvpva_020.fw; dce_fw display-t234-dce.bin; nvdec nvdec_t234_prod.fw; bpmp_fw bpmp_t234-TE950M-A1_prod.bin; bpmp_fw_dtb tegra234-bpmp-3767-0003-3509-a02.dtb; rce_fw camera-rtcpu-t234-rce.img; ape_fw adsp-fw.bin; spe_fw spe_t234.bin; tos tos-optee_t234.img; eks eks_t234.img"  --sdram_config tegra234-p3767-0001-sdram-l4t.dts  --cust_info custinfo_out.bin --external_device  --boot_chain A
saving flash command in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/flashcmd.txt
saving Windows flash command to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/flash_win.bat
*** Sign and generate flashing ready partition images... ***
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0359 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0361 ] Assuming zero filled SBK key
[   0.0371 ] Parsing partition layout
[   0.0383 ] tegraparser_v2 --pt flash.xml.tmp
[   0.0928 ] /usr/bin/python3 dtbcheck.py -c t234 -o tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb tegra234-bpmp-3767-0003-3509-a02_with_odm_tmp.dtb
[   0.3861 ] Concatenating L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo,BootOrderNvme.dtbo to tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   0.3864 ] Concatenating bl dtb to cpubl binary
[   0.3894 ] Kernel DTB used: kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
[   0.3895 ] Parsing partition layout
[   0.3900 ] tegraparser_v2 --pt flash.xml.tmp
[   0.3909 ] Creating list of images to be signed
[   0.3919 ] Generating ratchet blob
[   0.3919 ] Pre-processing config: tegra234-mb1-bct-reset-p3767-0000.dts
[   0.4239 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.4324 ] Pre-processing config: tegra234-mb1-bct-cprod-p3767-0000.dts
[   0.4412 ] Pre-processing config: tegra234-mb1-bct-gpioint-p3767-0000.dts
[   0.4542 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.4837 ] Pre-processing config: tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi
[   0.5044 ] Pre-processing config: tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi
[   0.5133 ] Pre-processing config: tegra234-mb1-bct-pmic-p3767-0000-a02.dts
[   0.5235 ] Pre-processing config: tegra234-mb1-bct-prod-p3767-0000.dts
[   0.5319 ] Pre-processing config: tegra234-mb2-bct-scr-p3767-0000.dts
[   1.0046 ] Pre-processing config: tegra234-p3767-0001-sdram-l4t.dts
[   2.2669 ] Pre-processing config: tegra234-mb1-bct-uphylane-si.dtsi
[   2.2758 ] Pre-processing config: tegra234-p3767-0001-wb0sdram-l4t.dts
[   3.5330 ] Pre-processing config: tegra234-mb1-bct-ratchet-p3767-0000.dts
[   3.6021 ] Generating coldboot mb1-bct
[   3.6079 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   3.6091 ] MB1-BCT version: 0.13

[   3.6160 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   3.6169 ] Added Platform Config 0 data with size :- 2416

[   3.6254 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   3.6264 ] WARNING: unknown node 'g2'
[   3.6268 ] WARNING: unknown node 'g2'
[   3.6272 ] WARNING: unknown node 'g9'
[   3.6276 ] WARNING: unknown node 'g9'
[   3.6279 ] Added Platform Config 2 data with size :- 24
[   3.6287 ]
[   3.6287 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   3.6288 ] Added Platform Config 4 data with size :- 288
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   3.6288 ] Added Platform Config 3 data with size :- 52
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   3.6288 ] WARNING: unknown property 'major'
[   3.6288 ] WARNING: unknown property 'minor'
[   3.6288 ] Added Platform Config 5 data with size :- 512
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   3.6288 ] WARNING: unknown property 'major'
[   3.6288 ] WARNING: unknown property 'minor'
[   3.6288 ] Added Platform Config 7 data with size :- 380
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   3.6288 ] Added Platform Config 8 data with size :- 24
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   3.6288 ] Added Platform Config 9 data with size :- 100
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   3.6288 ] ModuleCount 0 NumProdNames 0
[   3.6288 ] Added Platform Config 6 data with size :- 16
[   3.6288 ]
[   3.6288 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   3.6288 ] ERROR: /ratchet/atf is not supported
[   3.6288 ]
[   3.6289 ] Updating mb1-bct with firmware information
[   3.6301 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   3.6337 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   3.6362 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   3.6371 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   3.6426 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   3.6430 ] Assuming zero filled SBK key
[   3.6488 ] Warning: pub_key.key is not found
[   3.6482 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   3.6514 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --ratchet_blob ratchet_blob.bin --list images_list.xml zerosbk
[   3.6534 ] Filling MB1 storage info
[   3.6534 ] Parsing dev params for multi chains
[   3.6816 ] Generating br-bct
[   3.6827 ] Updating dev and MSS params in BR BCT
[   3.6828 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   3.8944 ] Updating bl info
[   3.8949 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   3.9081 ] Generating br-bct
[   3.9085 ] Updating dev and MSS params in BR BCT
[   3.9085 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   4.0780 ] Updating bl info
[   4.0784 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   4.0811 ] Generating signatures
[   4.0834 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.0836 ] Assuming zero filled SBK key
[   4.0838 ] Warning: pub_key.key is not found
[   4.0816 ] Parsing dev params for multi chains
[   4.0816 ] Generating br-bct
[   4.0821 ] Updating dev and MSS params in BR BCT
[   4.0822 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   4.2655 ] Updating customer data section
[   4.2660 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   4.2686 ] Updating bl info
[   4.2689 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   4.2699 ] Generating SHA2 Hash
[   4.2733 ] Sha saved in br_bct_BR.sha
[   4.2713 ] Get Signed section of bct
[   4.2717 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   4.2724 ] Signing BCT
[   4.2746 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.2748 ] Assuming zero filled SBK key
[   4.2773 ] Sha saved in br_bct_BR.sha
[   4.2775 ] Warning: pub_key.key is not found
[   4.2753 ] Updating BCT with signature
[   4.2757 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   4.2760 ] Offset :4608 Len :3584
[   4.2789 ] Generating SHA2 Hash
[   4.2812 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   4.2814 ] Assuming zero filled SBK key
[   4.2814 ] Assuming zero filled SBK key
[   4.2842 ] Sha saved in br_bct_BR.sha
[   4.2822 ] Updating BCT with SHA2 Hash
[   4.2827 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   4.2830 ] Offset :4608 Len :3584
[   4.2834 ] Offset :68 Len :8124
[   4.2893 ] Generating br-bct
[   4.2898 ] Updating dev and MSS params in BR BCT
[   4.2899 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   4.4601 ] Updating customer data section
[   4.4606 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   4.4630 ] Updating bl info
[   4.4636 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   4.4734 ] Generating SHA2 Hash
[   4.4769 ] Sha saved in br_bct_BR.sha
[   4.4748 ] Get Signed section of bct
[   4.4751 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   4.4759 ] Signing BCT
[   4.4781 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.4783 ] Assuming zero filled SBK key
[   4.4839 ] Sha saved in br_bct_BR.sha
[   4.4842 ] Warning: pub_key.key is not found
[   4.4820 ] Updating BCT with signature
[   4.4825 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   4.4829 ] Offset :4608 Len :3584
[   4.4836 ] Generating SHA2 Hash
[   4.4858 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   4.4859 ] Assuming zero filled SBK key
[   4.4859 ] Assuming zero filled SBK key
[   4.4885 ] Sha saved in br_bct_BR.sha
[   4.4865 ] Updating BCT with SHA2 Hash
[   4.4869 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   4.4872 ] Offset :4608 Len :3584
[   4.4876 ] Offset :68 Len :8124
[   4.4940 ] Generating coldboot mb1-bct
[   4.4946 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   4.4950 ] MB1-BCT version: 0.13

[   4.4968 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   4.4971 ] Added Platform Config 0 data with size :- 2416

[   4.4995 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   4.4999 ] WARNING: unknown node 'g2'
[   4.5000 ] WARNING: unknown node 'g2'
[   4.5001 ] WARNING: unknown node 'g9'
[   4.5003 ] WARNING: unknown node 'g9'
[   4.5003 ] Added Platform Config 2 data with size :- 24
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   4.5003 ] Added Platform Config 4 data with size :- 288
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   4.5003 ] Added Platform Config 3 data with size :- 52
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   4.5003 ] WARNING: unknown property 'major'
[   4.5003 ] WARNING: unknown property 'minor'
[   4.5003 ] Added Platform Config 5 data with size :- 512
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   4.5003 ] WARNING: unknown property 'major'
[   4.5003 ] WARNING: unknown property 'minor'
[   4.5003 ] Added Platform Config 7 data with size :- 380
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   4.5003 ] Added Platform Config 8 data with size :- 24
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   4.5003 ] Added Platform Config 9 data with size :- 100
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   4.5003 ] ModuleCount 0 NumProdNames 0
[   4.5003 ] Added Platform Config 6 data with size :- 16
[   4.5003 ]
[   4.5003 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   4.5003 ] ERROR: /ratchet/atf is not supported
[   4.5003 ]
[   4.5004 ] Updating mb1-bct with firmware information
[   4.5008 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   4.5029 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   4.5038 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   4.5042 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   4.5075 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.5076 ] Assuming zero filled SBK key
[   4.5091 ] Warning: pub_key.key is not found
[   4.5074 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   4.5130 ] Generating recovery mb1-bct
[   4.5134 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   4.5137 ] MB1-BCT version: 0.13

[   4.5155 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   4.5159 ] Added Platform Config 0 data with size :- 2416

[   4.5183 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   4.5185 ] WARNING: unknown node 'g2'
[   4.5186 ] WARNING: unknown node 'g2'
[   4.5187 ] WARNING: unknown node 'g9'
[   4.5188 ] WARNING: unknown node 'g9'
[   4.5189 ] Added Platform Config 2 data with size :- 24
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   4.5190 ] Added Platform Config 4 data with size :- 288
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   4.5190 ] Added Platform Config 3 data with size :- 52
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   4.5190 ] WARNING: unknown property 'major'
[   4.5190 ] WARNING: unknown property 'minor'
[   4.5190 ] Added Platform Config 5 data with size :- 512
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   4.5190 ] WARNING: unknown property 'major'
[   4.5190 ] WARNING: unknown property 'minor'
[   4.5190 ] Added Platform Config 7 data with size :- 380
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   4.5190 ] Added Platform Config 8 data with size :- 24
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   4.5190 ] Added Platform Config 9 data with size :- 100
[   4.5190 ]
[   4.5190 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   4.5190 ] ModuleCount 0 NumProdNames 0
[   4.5190 ] Added Platform Config 6 data with size :- 16
[   4.5191 ]
[   4.5191 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   4.5191 ] ERROR: /ratchet/atf is not supported
[   4.5191 ]
[   4.5191 ] Updating mb1-bct with firmware information
[   4.5194 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo flash.xml.bin
[   4.5221 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   4.5227 ] Generating SHA2 Hash for mb1bct
[   4.5261 ] Sha saved in mb1_bct_MB1_aligned.sha
[   4.5244 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   4.5248 ] adding BCH for mb1_bct_MB1_aligned.bct
[   4.5280 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.5281 ] Assuming zero filled SBK key
[   4.5297 ] Warning: pub_key.key is not found
[   4.5280 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   4.5290 ] Generating coldboot mem-bct
[   4.5294 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   4.5298 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   4.7128 ] Packing sdram param for instance[0]
[   4.7129 ] Packing sdram param for instance[1]
[   4.7130 ] Packing sdram param for instance[2]
[   4.7132 ] Packing sdram param for instance[3]
[   4.7133 ] Packing sdram param for instance[4]
[   4.7135 ] Packing sdram param for instance[5]
[   4.7136 ] Packing sdram param for instance[6]
[   4.7137 ] Packing sdram param for instance[7]
[   4.7139 ] Packing sdram param for instance[8]
[   4.7140 ] Packing sdram param for instance[9]
[   4.7141 ] Packing sdram param for instance[10]
[   4.7143 ] Packing sdram param for instance[11]
[   4.7144 ] Packing sdram param for instance[12]
[   4.7146 ] Packing sdram param for instance[13]
[   4.7148 ] Packing sdram param for instance[14]
[   4.7150 ] Packing sdram param for instance[15]
[   4.8840 ] Getting sector size from pt
[   4.8845 ] tegraparser_v2 --getsectorsize flash.xml.bin sector_info.bin
[   4.8852 ] BlockSize read from layout is 0x200

[   4.8856 ] tegrahost_v2 --chip 0x23 0 --blocksize 512 --magicid MEMB --addsigheader_multi tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   4.8860 ] Binary 0 length is 58752
[   4.8863 ] Binary 0 align length is 58880
[   4.8873 ] Binary 1 length is 58752
[   4.8875 ] Binary 1 align length is 58880
[   4.8884 ] Binary 2 length is 58752
[   4.8885 ] Binary 2 align length is 58880
[   4.8895 ] Binary 3 length is 58752
[   4.8896 ] Binary 3 align length is 58880
[   4.8907 ] Buffer length is 235520
[   4.8908 ] adding BCH for tegra234-p3767-0001-sdram-l4t_cpp_1.bct
[   4.8910 ] new length is 243712
[   4.8921 ] tegrahost_v2 --chip 0x23 0 --align mem_coldboot_aligned.bct
[   4.8931 ] tegrahost_v2 --chip 0x23 0 --magicid MEMB --ratchet_blob ratchet_blob.bin --appendsigheader mem_coldboot_aligned.bct zerosbk
[   4.8935 ] Header already present for mem_coldboot_aligned.bct
[   4.8969 ] tegrasign_v3.py --key None --list mem_coldboot_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   4.8970 ] Assuming zero filled SBK key
[   4.8987 ] Warning: pub_key.key is not found
[   4.8970 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_coldboot_aligned_sigheader.bct.encrypt mem_coldboot_aligned_sigheader.bct.hash zerosbk
[   4.9005 ] Generating recovery mem-bct
[   4.9009 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   4.9012 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   5.0852 ] Packing sdram param for instance[0]
[   5.0854 ] Packing sdram param for instance[1]
[   5.0855 ] Packing sdram param for instance[2]
[   5.0857 ] Packing sdram param for instance[3]
[   5.0858 ] Packing sdram param for instance[4]
[   5.0860 ] Packing sdram param for instance[5]
[   5.0861 ] Packing sdram param for instance[6]
[   5.0862 ] Packing sdram param for instance[7]
[   5.0863 ] Packing sdram param for instance[8]
[   5.0865 ] Packing sdram param for instance[9]
[   5.0866 ] Packing sdram param for instance[10]
[   5.0868 ] Packing sdram param for instance[11]
[   5.0869 ] Packing sdram param for instance[12]
[   5.0870 ] Packing sdram param for instance[13]
[   5.0872 ] Packing sdram param for instance[14]
[   5.0874 ] Packing sdram param for instance[15]
[   5.2586 ] Reading ramcode from backup chip_info.bin file
[   5.2589 ] RAMCODE Read from Device: 2

[   5.2590 ] Using ramcode 0
[   5.2590 ] Disabled BPMP dtb trim, using default dtb
[   5.2590 ]
[   5.2599 ] tegrahost_v2 --chip 0x23 0 --align mem_rcm_aligned.bct
[   5.2610 ] tegrahost_v2 --chip 0x23 0 --magicid MEM0 --ratchet_blob ratchet_blob.bin --appendsigheader mem_rcm_aligned.bct zerosbk
[   5.2614 ] adding BCH for mem_rcm_aligned.bct
[   5.2665 ] tegrasign_v3.py --key None --list mem_rcm_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.2666 ] Assuming zero filled SBK key
[   5.2682 ] Warning: pub_key.key is not found
[   5.2665 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_rcm_aligned_sigheader.bct.encrypt mem_rcm_aligned_sigheader.bct.hash zerosbk
[   5.2677 ] Copying signatures
[   5.2683 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --updatesig images_list_signed.xml
[   5.2693 ] tegraparser_v2 --generategpt --pt flash.xml.bin
[   5.2697 ] gpt_primary_9_0.bin:
[   5.2701 ] partition_id       partition_name                        StartingLba       EndingLba
[   5.2703 ]            1       APP                                      3050048        119537623
[   5.2705 ]            2       A_kernel                                      40          262183
[   5.2707 ]            3       A_kernel-dtb                              262184          263719
[   5.2709 ]            4       A_reserved_on_user                        263720          328487
[   5.2712 ]            5       B_kernel                                  328488          590631
[   5.2715 ]            6       B_kernel-dtb                              590632          592167
[   5.2718 ]            7       B_reserved_on_user                        592168          656935
[   5.2718 ]            8       recovery                                  656936          820775
[   5.2718 ]            9       recovery-dtb                              820776          821799
[   5.2718 ]           10       esp                                       821800          952871
[   5.2718 ]           11       recovery_alt                              952872         1116711
[   5.2718 ]           12       recovery-dtb_alt                         1116712         1117735
[   5.2718 ]           13       esp_alt                                  1117736         1248807
[   5.2718 ]           14       UDA                                      1248832         2068031
[   5.2718 ]           15       reserved                                 2068032         3050047
[   5.2718 ] gpt_secondary_9_0.bin:
[   5.2718 ] partition_id       partition_name                        StartingLba       EndingLba
[   5.2718 ]            1       APP                                      3050048        119537623
[   5.2718 ]            2       A_kernel                                      40          262183
[   5.2718 ]            3       A_kernel-dtb                              262184          263719
[   5.2718 ]            4       A_reserved_on_user                        263720          328487
[   5.2718 ]            5       B_kernel                                  328488          590631
[   5.2718 ]            6       B_kernel-dtb                              590632          592167
[   5.2718 ]            7       B_reserved_on_user                        592168          656935
[   5.2718 ]            8       recovery                                  656936          820775
[   5.2718 ]            9       recovery-dtb                              820776          821799
[   5.2718 ]           10       esp                                       821800          952871
[   5.2718 ]           11       recovery_alt                              952872         1116711
[   5.2718 ]           12       recovery-dtb_alt                         1116712         1117735
[   5.2718 ]           13       esp_alt                                  1117736         1248807
[   5.2718 ]           14       UDA                                      1248832         2068031
[   5.2718 ]           15       reserved                                 2068032         3050047
[   5.2718 ]
[   5.3017 ] tegrahost_v2 --chip 0x23 0 --align uefi_jetson_with_dtb_aligned.bin
[   5.3046 ] tegrahost_v2 --chip 0x23 0 --magicid CPBL --ratchet_blob ratchet_blob.bin --appendsigheader uefi_jetson_with_dtb_aligned.bin zerosbk
[   5.3057 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   5.4659 ] tegrasign_v3.py --key None --list uefi_jetson_with_dtb_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.4661 ] Assuming zero filled SBK key
[   5.4756 ] Warning: pub_key.key is not found
[   5.4739 ] tegrahost_v2 --chip 0x23 0 --updatesigheader uefi_jetson_with_dtb_aligned_sigheader.bin.encrypt uefi_jetson_with_dtb_aligned_sigheader.bin.hash zerosbk
[   5.4829 ] Copying enc\/signed file in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   5.4830 ] Copying br bct for multi chains
[   5.4831 ] Signed BCT for boot chain A is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_BR.bct

[   5.4833 ] Signed BCT for boot chain B is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_b_BR.bct

[   5.4846 ] Copying uefi_jetson_with_dtb_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   5.4870 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt
[   5.4891 ] tegraparser_v2 --pt flash.xml.bin --generateflashindex /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.xml.tmp flash.idx
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.idx' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/flash.idx'
Flash index file is /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.idx
Number of lines is 18
max_index=17
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mbr_9_0.bin  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/mbr_9_0.bin
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mbr_9_0.bin' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/mbr_9_0.bin'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_primary_9_0.bin  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/gpt_primary_9_0.bin
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_primary_9_0.bin' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/gpt_primary_9_0.bin'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/boot.img  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/boot.img
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/boot.img' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/boot.img'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb'
Warning: skip writing A_reserved_on_user partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/boot.img  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/boot.img
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/boot.img' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/boot.img'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb'
Warning: skip writing B_reserved_on_user partition as no image is specified
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/recovery.img  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/recovery.img
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/recovery.img' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/recovery.img'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegra234-p3768-0000+p3767-0005-nv.dtb.rec  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/tegra234-p3768-0000+p3767-0005-nv.dtb.rec
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegra234-p3768-0000+p3767-0005-nv.dtb.rec' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/tegra234-p3768-0000+p3767-0005-nv.dtb.rec'
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/esp.img  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/esp.img
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/esp.img' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/esp.img'
Warning: skip writing recovery_alt partition as no image is specified
Warning: skip writing recovery-dtb_alt partition as no image is specified
Warning: skip writing esp_alt partition as no image is specified
Warning: skip writing UDA partition as no image is specified
Warning: skip writing reserved partition as no image is specified
Copying APP image into  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/system.img.raw
tar: Write checkpoint 10000
tar: Write checkpoint 20000
tar: Write checkpoint 30000
tar: Write checkpoint 40000
tar: Write checkpoint 50000
tar: Write checkpoint 60000
tar: Write checkpoint 70000
tar: Write checkpoint 80000
tar: Write checkpoint 90000
tar: Write checkpoint 100000
tar: Write checkpoint 110000
tar: Write checkpoint 120000
tar: Write checkpoint 130000
tar: Write checkpoint 140000
tar: Write checkpoint 150000
tar: Write checkpoint 160000
tar: Write checkpoint 170000
tar: Write checkpoint 180000
tar: Write checkpoint 190000
tar: Write checkpoint 200000
tar: Write checkpoint 210000
tar: Write checkpoint 220000
tar: Write checkpoint 230000
tar: Write checkpoint 240000
tar: Write checkpoint 250000
tar: Write checkpoint 260000
tar: Write checkpoint 270000
tar: Write checkpoint 280000
tar: Write checkpoint 290000
tar: Write checkpoint 300000
tar: Write checkpoint 310000
tar: Write checkpoint 320000
tar: Write checkpoint 330000
tar: Write checkpoint 340000
tar: Write checkpoint 350000
tar: Write checkpoint 360000
tar: Write checkpoint 370000
tar: Write checkpoint 380000
tar: Write checkpoint 390000
tar: Write checkpoint 400000
tar: Write checkpoint 410000
tar: Write checkpoint 420000
tar: Write checkpoint 430000
tar: Write checkpoint 440000
tar: Write checkpoint 450000
tar: Write checkpoint 460000
tar: Write checkpoint 470000
tar: Write checkpoint 480000
tar: Write checkpoint 490000
tar: Write checkpoint 500000
tar: Write checkpoint 510000
tar: Write checkpoint 520000
tar: Write checkpoint 530000
tar: Write checkpoint 540000
tar: Write checkpoint 550000
tar: Write checkpoint 560000
tar: Write checkpoint 570000
tar: Write checkpoint 580000
tar: Write checkpoint 590000
tar: Write checkpoint 600000
tar: Write checkpoint 610000
tar: Write checkpoint 620000
tar: Write checkpoint 630000
tar: Write checkpoint 640000
tar: Write checkpoint 650000
Copying /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_secondary_9_0.bin  /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/gpt_secondary_9_0.bin
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/gpt_secondary_9_0.bin' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/external/gpt_secondary_9_0.bin'
Copy flash script to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images
'/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/l4t_flash_from_kernel.sh' -> '/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/images/l4t_flash_from_kernel.sh'
Success
******************************************
*                                        *
*  Step 2: Generate rcm boot commandline *
*                                        *
******************************************
ROOTFS_AB= ROOTFS_ENC= /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/flash.sh  --no-flash --rcm-boot jetson-orin-nano-devkit mmcblk0p1
###############################################################################
# L4T BSP Information:
# R36 , REVISION: 2.0
# User release: 0.0
###############################################################################
ECID is 0x80012344705DF1975C000000110300C0
copying emc_fuse_dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-diag-boot.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
./tegraflash.py --chip "0x23" --applet "/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin" --skipuid --cfg readinfo_t234_min_prod.xml --dev_params tegra234-br-bct-diag-boot.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --bins "mb2_applet applet_t234.bin" --cmd "dump eeprom cvm cvm.bin; dump try_custinfo custinfo_out.bin; reboot recovery"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0339 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.0343 ] File rcm_state open failed
[   0.0347 ] ERROR: failed to read rcm_state
[   0.0347 ]
[   0.0367 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0369 ] Assuming zero filled SBK key
[   0.0353 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.0608 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.0740 ] Parsing partition layout
[   0.0748 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0761 ] Kernel DTB used: None
[   0.0761 ] WARNING: dce base dtb is not provided

[   0.0761 ] Parsing partition layout
[   0.0766 ] tegraparser_v2 --pt readinfo_t234_min_prod.xml.tmp
[   0.0773 ] Creating list of images to be signed
[   0.0780 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --list images_list.xml zerosbk
[   0.0784 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0810 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0815 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   0.0874 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   0.0881 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.0978 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.0982 ] adding BCH for mb2_t234_aligned.bin
[   0.1041 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   0.1191 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   0.1195 ] adding BCH for mb2_t234_aligned.bin
[   0.1377 ] Filling MB1 storage info
[   0.1378 ] Parsing dev params for multi chains
[   0.1484 ] Generating br-bct
[   0.1495 ] Updating dev and MSS params in BR BCT
[   0.1495 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1508 ] Updating bl info
[   0.1514 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin
[   0.1519 ] WARNING: boot chain is not completed. set to 0
[   0.1540 ] Generating signatures
[   0.1559 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1561 ] Assuming zero filled SBK key
[   0.1714 ] Warning: pub_key.key is not found
[   0.1697 ] Parsing dev params for multi chains
[   0.1697 ] Generating br-bct
[   0.1702 ] Updating dev and MSS params in BR BCT
[   0.1704 ] tegrabct_v2 --dev_param tegra234-br-bct-diag-boot_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   0.1712 ] Updating bl info
[   0.1718 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.1722 ] WARNING: boot chain is not completed. set to 0
[   0.1751 ] Generating SHA2 Hash
[   0.1781 ] Sha saved in br_bct_BR.sha
[   0.1763 ] Get Signed section of bct
[   0.1768 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   0.1777 ] Signing BCT
[   0.1797 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1799 ] Assuming zero filled SBK key
[   0.1824 ] Sha saved in br_bct_BR.sha
[   0.1827 ] Warning: pub_key.key is not found
[   0.1808 ] Updating BCT with signature
[   0.1813 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   0.1818 ] Offset :4608 Len :3584
[   0.1822 ] Generating SHA2 Hash
[   0.1842 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   0.1844 ] Assuming zero filled SBK key
[   0.1844 ] Assuming zero filled SBK key
[   0.1867 ] Sha saved in br_bct_BR.sha
[   0.1850 ] Updating BCT with SHA2 Hash
[   0.1854 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   0.1858 ] Offset :4608 Len :3584
[   0.1862 ] Offset :68 Len :8124
[   0.1864 ] Generating coldboot mb1-bct
[   0.1869 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1872 ] MB1-BCT version: 0.13

[   0.1892 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1893 ] Added Platform Config 9 data with size :- 100
[   0.1893 ]
[   0.1894 ] Updating mb1-bct with firmware information
[   0.1897 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.1915 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   0.1926 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   0.1929 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   0.1957 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.1958 ] Assuming zero filled SBK key
[   0.1972 ] Warning: pub_key.key is not found
[   0.1958 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.1969 ] Generating recovery mb1-bct
[   0.1973 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1977 ] MB1-BCT version: 0.13

[   0.1995 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   0.1997 ] Added Platform Config 9 data with size :- 100
[   0.1997 ]
[   0.1997 ] Updating mb1-bct with firmware information
[   0.2002 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo readinfo_t234_min_prod.xml.bin
[   0.2024 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   0.2030 ] Generating SHA2 Hash for mb1bct
[   0.2061 ] Sha saved in mb1_bct_MB1_aligned.sha
[   0.2046 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   0.2050 ] adding BCH for mb1_bct_MB1_aligned.bct
[   0.2078 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.2079 ] Assuming zero filled SBK key
[   0.2094 ] Warning: pub_key.key is not found
[   0.2080 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   0.2090 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.2090 ] Error: Skip generating mem_bct because sdram_config is not defined
[   0.2091 ] Copying signatures
[   0.2095 ] tegrahost_v2 --chip 0x23 0 --partitionlayout readinfo_t234_min_prod.xml.bin --updatesig images_list_signed.xml
[   0.2179 ] mb1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.2181 ] psc_bl1_t234_prod_aligned_sigheader.bin.encrypt filename is from images_list
[   0.2181 ] Boot Rom communication
[   0.2186 ] tegrarcm_v2 --new_session --chip 0x23 0 --uid --download bct_br br_bct_BR.bct --download mb1 mb1_t234_prod_aligned_sigheader.bin.encrypt --download psc_bl1 psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
[   0.2189 ] BR_CID: 0x80012344705DF1975C000000110300C0
[   0.2646 ] Sending bct_br
[   0.3095 ] Sending mb1
[   0.3106 ] Sending psc_bl1
[   0.3227 ] Sending bct_mb1
[   0.3290 ] Boot Rom communication completed
[   0.3335 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   0.3364 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --appendsigheader applet_t234_aligned.bin zerosbk
[   0.3374 ] adding BCH for applet_t234_aligned.bin
[   0.3730 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   0.3734 ] Assuming zero filled SBK key
[   0.3771 ] Warning: pub_key.key is not found
[   0.3761 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   0.3790 ] Sending mb2_applet...

[   0.3797 ] tegrarcm_v2 --chip 0x23 0 --pollbl --download applet applet_t234_sigheader.bin.encrypt
[   0.3803 ] BL: version 1.4.0.1-t234-54845784-08e631ca last_boot_error: 0
[   0.5534 ] Sending applet
[   0.6543 ] completed
[   0.6557 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.6569 ] MB2 Applet version 01.00.0000
[   0.8432 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   0.8443 ] MB2 Applet version 01.00.0000
[   0.8940 ] Retrieving board information
[   0.8951 ] tegrarcm_v2 --chip 0x23 0 --oem platformdetails chip chip_info.bin
[   0.8956 ] MB2 Applet version 01.00.0000
[   0.9431 ] Saved platform info in chip_info.bin
[   0.9486 ] Chip minor revision: 1
[   0.9487 ] Bootrom revision: 0x7
[   0.9488 ] Ram code: 0x2
[   0.9489 ] Chip sku: 0xd5
[   0.9490 ] Chip Sample: prod
[   0.9495 ] Retrieving EEPROM data
[   0.9496 ] tegrarcm_v2 --oem platformdetails eeprom cvm /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin --chip 0x23 0
[   0.9500 ] MB2 Applet version 01.00.0000
[   0.9961 ] Saved platform info in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   1.0299 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.0303 ] MB2 Applet version 01.00.0000
[   1.0811 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.0823 ] MB2 Applet version 01.00.0000
[   1.1322 ] Dumping customer Info
[   1.1337 ] tegrarcm_v2 --chip 0x23 0 --oem dump bct tmp.bct
[   1.1349 ] MB2 Applet version 01.00.0000
[   1.1823 ] Saved bct in tmp.bct
[   1.2044 ] tegrabct_v2 --brbct tmp.bct --chip 0x23 0 --custinfo /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin
[   1.2054 ] Cu[   1.2064 ] stomer data saved in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/custinfo_out.bin successfully
[   1.2065 ] Rebooting to recovery mode
[   1.2078 ] tegrarcm_v2 --chip 0x23 0 --ismb2
[   1.2569 ] tegrarcm_v2 --chip 0x23 0 --ismb2applet
[   1.2581 ] MB2 Applet version 01.00.0000
[   1.3056 ] Booting to recovery mode
[   1.3069 ] tegrarcm_v2 --chip 0x23 0 --reboot recovery
[   1.3081 ] MB2 Applet version 01.00.0000
Board ID(3767) version(300) sku(0005) revision(K.2)
Chip SKU(00:00:00:D5) ramcode(00:00:00:02) fuselevel(fuselevel_production) board_FAB(300)
emc_opt_disable_fuse:(0)
Copy /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec
copying bctfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-sdram-l4t.dts)... done.
copying minratchet_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-ratchet-p3767-0000.dts)... done.
copying device_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-device-p3767-0000.dts)... done.
copying misc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-misc-p3767-0000.dts)... done.
copying pinmux_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi)... done.
copying gpioint_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-gpioint-p3767-0000.dts)... done.
copying pmic_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-pmic-p3767-0000-a02.dts)... done.
copying pmc_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi)... done.
copying deviceprod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-cprod-p3767-0000.dts)... done.
copying prod_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-prod-p3767-0000.dts)... done.
copying scr_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-scr-p3767-0000.dts)... done.
copying wb0sdram(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-p3767-0001-wb0sdram-l4t.dts)... done.
copying bootrom_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb1-bct-reset-p3767-0000.dts)... done.
Existing uphylane_config(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegra234-mb1-bct-uphylane-si.dtsi) reused.
copying dev_params(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct-p3767-0000-l4t.dts)... done.
copying dev_params_b(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-br-bct_b-p3767-0000-l4t.dts)... done.
copying mb2bct_cfg(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/BCT/tegra234-mb2-bct-misc-p3767-0000.dts)... done.
Existing pscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/pscfw_t234_prod.bin) reused.
Existing pscbl1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_bl1_t234_prod.bin) reused.
Existing mtsmcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mce_flash_o10_cr_prod.bin) reused.
Existing tscfwfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tsec_t234.bin) reused.
Existing mb2applet(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/applet_t234.bin) reused.
Existing bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
copying initrd(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/rootfs/boot/initrd)... done.
bl is uefi
Making Boot image... done.
Not signing of boot.img
Making recovery ramdisk for recovery image...
Re-generating recovery ramdisk for recovery image...
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/ramdisk_tmp /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
52057 blocks

gzip: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/Image: not in gzip format
_BASE_KERNEL_VERSION=5.15.122-tegra
74756 blocks
Making Recovery image...
copying recdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb.rec)... done.
20+0 records in
20+0 records out
20 bytes copied, 0.000242137 s, 82.6 kB/s
Existing sosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing tegraboot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing cpu_bootloader(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing mb2blfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2_t234.bin) reused.
Existing xusbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/xusb_t234_prod.bin) reused.
Existing pvafile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvpva_020.fw) reused.
Existing dcefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/display-t234-dce.bin) reused.
Existing nvdecfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/nvdec_t234_prod.fw) reused.
Existing psc_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/psc_rf_t234_prod.bin) reused.
Existing mb2_rf(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb2rf_t234.bin) reused.
Existing mb1file(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/mb1_t234_prod.bin) reused.
Existing bpffile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/bpmp_t234-TE950M-A1_prod.bin) reused.
copying bpfdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/tegra234-bpmp-3767-0003-3509-a02.dtb)... done.
Existing camerafw(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/camera-rtcpu-t234-rce.img) reused.
Existing apefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/adsp-fw.bin) reused.
Existing spefile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/spe_t234.bin) reused.
Existing wb0boot(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/sc7_t234_prod.bin) reused.
Existing tosfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tos-optee_t234.img) reused.
Existing eksfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/eks_t234.img) reused.
copying dtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
Copying nv_boot_control.conf to rootfs
Skip generating system.img
Not signing of kernel-dtb
Existing tbcfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/uefi_jetson.bin) reused.
131072+0 records in
131072+0 records out
67108864 bytes (67 MB, 64 MiB) copied, 1.45891 s, 46.0 MB/s
        Sync'ing esp.img ... done.
copying tbcdtbfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0005-nv.dtb)... done.
copying cfgfile(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/generic/cfg/flash_t234_qspi_sd.xml) to flash.xml... done.
Existing flashapp(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py) reused.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/L4TConfiguration.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-carveouts.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra-optee.dtbo)... done.
copying overlay_dtb(/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/kernel/dtb/tegra234-p3768-0000+p3767-0000-dynamic.dtbo)... done.
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration_updated.dts: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration.dtbo: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration_updated.dts: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/L4TConfiguration.dtbo: Warning (unit_address_vs_reg): Node /fragment@0 has a unit name, but no reg property
./tegraflash.py  --bl uefi_jetson_with_dtb.bin  --concat_cpubl_bldtb --cpubl uefi_jetson.bin --sdram_config tegra234-p3767-0001-sdram-l4t.dts  --cust_info custinfo_out.bin  --odmdata gbe-uphy-config-8,hsstp-lane-map-3,hsio-uphy-config-0  --overlay_dtb L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo, --bldtb tegra234-p3768-0000+p3767-0005-nv.dtb --applet mb1_t234_prod.bin --cmd "sign"  --cfg flash.xml --chip 0x23 --minratchet_config tegra234-mb1-bct-ratchet-p3767-0000.dts --device_config tegra234-mb1-bct-device-p3767-0000.dts --misc_config tegra234-mb1-bct-misc-p3767-0000.dts --pinmux_config tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi --gpioint_config tegra234-mb1-bct-gpioint-p3767-0000.dts --pmic_config tegra234-mb1-bct-pmic-p3767-0000-a02.dts --pmc_config tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi --deviceprod_config tegra234-mb1-bct-cprod-p3767-0000.dts --prod_config tegra234-mb1-bct-prod-p3767-0000.dts --scr_config tegra234-mb2-bct-scr-p3767-0000.dts --wb0sdram_config tegra234-p3767-0001-wb0sdram-l4t.dts --br_cmd_config tegra234-mb1-bct-reset-p3767-0000.dts --uphy tegra234-mb1-bct-uphylane-si.dtsi --dev_params tegra234-br-bct-p3767-0000-l4t.dts,tegra234-br-bct_b-p3767-0000-l4t.dts --mb2bct_cfg tegra234-mb2-bct-misc-p3767-0000.dts  --bins "psc_fw pscfw_t234_prod.bin; mts_mce mce_flash_o10_cr_prod.bin; tsec_fw tsec_t234.bin; mb2_applet applet_t234.bin; mb2_bootloader mb2_t234.bin; xusb_fw xusb_t234_prod.bin; pva_fw nvpva_020.fw; dce_fw display-t234-dce.bin; nvdec nvdec_t234_prod.fw; bpmp_fw bpmp_t234-TE950M-A1_prod.bin; bpmp_fw_dtb tegra234-bpmp-3767-0003-3509-a02.dtb; rce_fw camera-rtcpu-t234-rce.img; ape_fw adsp-fw.bin; spe_fw spe_t234.bin; tos tos-optee_t234.img; eks eks_t234.img; kernel boot.img; kernel_dtb tegra234-p3768-0000+p3767-0005-nv.dtb"  --bct_backup
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0322 ] tegrasign_v3.py --key None --getmode mode.txt
[   0.0324 ] Assuming zero filled SBK key
[   0.0318 ] Parsing partition layout
[   0.0324 ] tegraparser_v2 --pt flash.xml.tmp
[   0.0388 ] Change tegra234-bpmp-3767-0003-3509-a02.dtb to tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb
[   0.0389 ] Change tegra234-bpmp-3767-0003-3509-a02.dtb to tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb
[   0.0929 ] /usr/bin/python3 dtbcheck.py -c t234 -o tegra234-bpmp-3767-0003-3509-a02_with_odm.dtb tegra234-bpmp-3767-0003-3509-a02_with_odm_tmp.dtb
[   0.3753 ] Concatenating L4TConfiguration.dtbo,tegra234-carveouts.dtbo,tegra-optee.dtbo,tegra234-p3768-0000+p3767-0000-dynamic.dtbo to tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb.updated
[   0.3757 ] Concatenating bl dtb to cpubl binary
[   0.3793 ] MB2 binary: mb2_t234.bin
[   0.3794 ] Pre-processing mb2bct config: tegra234-mb2-bct-misc-p3767-0000.dts
[   0.3938 ] Pre-processing mb2bct config: tegra234-mb2-bct-scr-p3767-0000.dts
[   0.8505 ] Generating coldboot mb2-bct
[   0.8505 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_cold_boot_bct.cfg --mb2bctcfg tegra234-mb2-bct-misc-p3767-0000_cpp.dtb --scr tegra234-mb2-bct-scr-p3767-0000_cpp.dtb
[   0.8510 ] ERROR: value 0x31 is out of range
[   0.8524 ] ERROR: value 0x31 is out of range
[   0.8527 ] ERROR: value 0x31 is out of range
[   0.8529 ] ERROR: value 0x31 is out of range
[   0.8531 ] WARNING: unknown property 'tfc_version'
[   0.8534 ] WARNING: unknown property 'addr_header_version'
[   0.8652 ] Updating mb2-bct with storage information
[   0.8658 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_cold_boot_bct_MB2.bct --updatestorageinfo flash.xml.bin
[   0.8686 ] Concatenating mb2-bct to mb2 binary
[   0.8686 ] mb2_bin_file = mb2_t234.bin
[   0.8686 ] mb2_bct_file = mb2_cold_boot_bct_MB2.bct
[   0.8731 ] DCE binary: display-t234-dce.bin
[   0.8732 ] Kernel DTB used: kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
[   0.8733 ] Concatenating kernel-dtb to dce-fw binary
[   0.8733 ] dce_bin = display-t234-dce.bin
[   0.8733 ] kernel_dtb = kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
[   0.8734 ] dce_with_dtb = display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv.bin
[   0.8829 ] Update display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv.bin to dce_fw partitions
[   0.8856 ] Parsing partition layout
[   0.8863 ] tegraparser_v2 --pt flash.xml.tmp
[   0.8876 ] Creating list of images to be signed
[   0.8880 ] Generating ratchet blob
[   0.8880 ] Pre-processing config: tegra234-mb1-bct-reset-p3767-0000.dts
[   0.8953 ] Pre-processing config: tegra234-mb1-bct-device-p3767-0000.dts
[   0.9022 ] Pre-processing config: tegra234-mb1-bct-cprod-p3767-0000.dts
[   0.9094 ] Pre-processing config: tegra234-mb1-bct-gpioint-p3767-0000.dts
[   0.9171 ] Pre-processing config: tegra234-mb1-bct-misc-p3767-0000.dts
[   0.9277 ] Pre-processing config: tegra234-mb1-bct-pinmux-p3767-dp-a03.dtsi
[   0.9419 ] Pre-processing config: tegra234-mb1-bct-padvoltage-p3767-dp-a03.dtsi
[   0.9489 ] Pre-processing config: tegra234-mb1-bct-pmic-p3767-0000-a02.dts
[   0.9564 ] Pre-processing config: tegra234-mb1-bct-prod-p3767-0000.dts
[   0.9637 ] Pre-processing config: tegra234-p3767-0001-sdram-l4t.dts
[   2.2159 ] Pre-processing config: tegra234-mb1-bct-uphylane-si.dtsi
[   2.2229 ] Pre-processing config: tegra234-p3767-0001-wb0sdram-l4t.dts
[   3.4813 ] Pre-processing config: tegra234-mb1-bct-ratchet-p3767-0000.dts
[   3.4885 ] Generating coldboot mb1-bct
[   3.4890 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   3.4894 ] MB1-BCT version: 0.13

[   3.4914 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   3.4919 ] Added Platform Config 0 data with size :- 2416

[   3.4943 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   3.4947 ] WARNING: unknown node 'g2'
[   3.4948 ] WARNING: unknown node 'g2'
[   3.4951 ] WARNING: unknown node 'g9'
[   3.4952 ] WARNING: unknown node 'g9'
[   3.4954 ] Added Platform Config 2 data with size :- 24
[   3.4955 ]
[   3.4955 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   3.4955 ] Added Platform Config 4 data with size :- 288
[   3.4955 ]
[   3.4955 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   3.4955 ] Added Platform Config 3 data with size :- 52
[   3.4955 ]
[   3.4955 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   3.4955 ] WARNING: unknown property 'major'
[   3.4955 ] WARNING: unknown property 'minor'
[   3.4955 ] Added Platform Config 5 data with size :- 512
[   3.4955 ]
[   3.4955 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   3.4955 ] WARNING: unknown property 'major'
[   3.4955 ] WARNING: unknown property 'minor'
[   3.4956 ] Added Platform Config 7 data with size :- 380
[   3.4956 ]
[   3.4956 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   3.4956 ] Added Platform Config 8 data with size :- 24
[   3.4956 ]
[   3.4956 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   3.4956 ] Added Platform Config 9 data with size :- 100
[   3.4956 ]
[   3.4956 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   3.4956 ] ModuleCount 0 NumProdNames 0
[   3.4956 ] Added Platform Config 6 data with size :- 16
[   3.4956 ]
[   3.4956 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   3.4956 ] ERROR: /ratchet/atf is not supported
[   3.4956 ]
[   3.4956 ] Updating mb1-bct with firmware information
[   3.4960 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   3.4979 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   3.4990 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   3.4994 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   3.5014 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   3.5016 ] Assuming zero filled SBK key
[   3.5030 ] Warning: pub_key.key is not found
[   3.5026 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   3.5038 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --ratchet_blob ratchet_blob.bin --list images_list.xml zerosbk
[   3.5043 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   3.5054 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   3.5058 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   3.5111 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   3.5114 ] Header already present for tsec_t234_aligned.bin
[   3.5138 ] Header already present for nvdec_t234_prod_aligned.fw
[   3.5177 ] adding BCH for mb2_t234_with_mb2_cold_boot_bct_MB2_aligned.bin
[   3.5241 ] adding BCH for xusb_t234_prod_aligned.bin
[   3.5385 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   3.5464 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   3.5643 ] Header already present for pscfw_t234_prod_aligned.bin
[   3.5714 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   3.5772 ] Header already present for sc7_t234_prod.bin
[   3.5807 ] Header already present for psc_rf_t234_prod_aligned.bin
[   3.5843 ] adding BCH for mb2rf_t234_aligned.bin
[   3.5867 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   3.5997 ] adding BCH for tos-optee_t234_aligned.img
[   3.7301 ] adding BCH for eks_t234_aligned.img
[   3.7727 ] INFO: compressing display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin
[   3.9881 ] INFO: complete compression, display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin, ratio = 6%
[   4.0146 ] adding BCH for display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_blob_w_bin_aligned.bin
[   4.0390 ] adding BCH for spe_t234_aligned.bin
[   4.0541 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   4.0646 ] adding BCH for adsp-fw_aligned.bin
[   4.0828 ] INFO: compressing nvpva_020_aligned.fw
[   4.1379 ] INFO: complete compression, nvpva_020_aligned.fw, ratio = 2%
[   4.1418 ] adding BCH for nvpva_020_aligned_blob_w_bin_aligned.fw
[   4.1445 ] MB1: Nvheader already present is mb1_t234_prod_aligned.bin
[   4.1465 ] Header already present for mb1_t234_prod_aligned_sigheader.bin
[   4.1470 ] MB1: Nvheader already present is psc_bl1_t234_prod_aligned.bin
[   4.1523 ] Header already present for psc_bl1_t234_prod_aligned_sigheader.bin
[   4.1526 ] Header already present for tsec_t234_aligned.bin
[   4.1556 ] Header already present for nvdec_t234_prod_aligned.fw
[   4.1598 ] adding BCH for mb2_t234_with_mb2_cold_boot_bct_MB2_aligned.bin
[   4.1663 ] adding BCH for xusb_t234_prod_aligned.bin
[   4.1814 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   4.1934 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   4.2112 ] Header already present for pscfw_t234_prod_aligned.bin
[   4.2188 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   4.2248 ] Header already present for sc7_t234_prod.bin
[   4.2284 ] Header already present for psc_rf_t234_prod_aligned.bin
[   4.2323 ] adding BCH for mb2rf_t234_aligned.bin
[   4.2349 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   4.2484 ] adding BCH for tos-optee_t234_aligned.img
[   4.3744 ] adding BCH for eks_t234_aligned.img
[   4.4198 ] INFO: compressing display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin
[   4.6791 ] INFO: complete compression, display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin, ratio = 6%
[   4.7047 ] adding BCH for display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_blob_w_bin_aligned.bin
[   4.7325 ] adding BCH for spe_t234_aligned.bin
[   4.7468 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   4.7570 ] adding BCH for adsp-fw_aligned.bin
[   4.7738 ] INFO: compressing nvpva_020_aligned.fw
[   4.8280 ] INFO: complete compression, nvpva_020_aligned.fw, ratio = 2%
[   4.8321 ] adding BCH for nvpva_020_aligned_blob_w_bin_aligned.fw
[   4.8363 ] Filling MB1 storage info
[   4.8363 ] Parsing dev params for multi chains
[   4.8448 ] Generating br-bct
[   4.8453 ] Updating dev and MSS params in BR BCT
[   4.8454 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.0292 ] Updating bl info
[   5.0299 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   5.0399 ] Generating br-bct
[   5.0404 ] Updating dev and MSS params in BR BCT
[   5.0404 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.2097 ] Updating bl info
[   5.2102 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin
[   5.2117 ] Generating BCT backup image
[   5.2118 ] dd if=/dev/zero of=bct_backup.img bs=1 count=32768
[   5.2125 ] 32768+0 records in
[   5.2990 ] 32768+0 records out
[   5.2990 ] 32768 bytes (33 kB, 32 KiB) copied, 0.0858488 s, 382 kB/s
[   5.2990 ]
[   5.2991 ] Concatenating BCT for chain A to bct_backup.img

[   5.2991 ] dd if=br_bct_BR.bct of=bct_backup.img bs=1 seek=0 conv=notrunc
[   5.2997 ] 8192+0 records in
[   5.3152 ] 8192+0 records out
[   5.3152 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0147546 s, 555 kB/s
[   5.3152 ]
[   5.3152 ] Concatenating BCT for chain B to bct_backup.img

[   5.3152 ] dd if=br_bct_b_BR.bct of=bct_backup.img bs=1 seek=16384 conv=notrunc
[   5.3159 ] 8192+0 records in
[   5.3312 ] 8192+0 records out
[   5.3312 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0144632 s, 566 kB/s
[   5.3312 ]
[   5.3312 ] Generating signatures
[   5.3322 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.3324 ] Assuming zero filled SBK key
[   5.4323 ] Warning: pub_key.key is not found
[   5.4347 ] Parsing dev params for multi chains
[   5.4347 ] Generating br-bct
[   5.4352 ] Updating dev and MSS params in BR BCT
[   5.4353 ] tegrabct_v2 --dev_param tegra234-br-bct-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.6087 ] Updating customer data section
[   5.6094 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   5.6110 ] Updating bl info
[   5.6116 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   5.6150 ] Generating SHA2 Hash
[   5.6172 ] Sha saved in br_bct_BR.sha
[   5.6165 ] Get Signed section of bct
[   5.6171 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   5.6179 ] Signing BCT
[   5.6188 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.6190 ] Assuming zero filled SBK key
[   5.6215 ] Sha saved in br_bct_BR.sha
[   5.6218 ] Warning: pub_key.key is not found
[   5.6209 ] Updating BCT with signature
[   5.6214 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   5.6220 ] Offset :4608 Len :3584
[   5.6227 ] Generating SHA2 Hash
[   5.6236 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   5.6237 ] Assuming zero filled SBK key
[   5.6237 ] Assuming zero filled SBK key
[   5.6262 ] Sha saved in br_bct_BR.sha
[   5.6257 ] Updating BCT with SHA2 Hash
[   5.6261 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   5.6266 ] Offset :4608 Len :3584
[   5.6271 ] Offset :68 Len :8124
[   5.6274 ] Generating br-bct
[   5.6278 ] Updating dev and MSS params in BR BCT
[   5.6279 ] tegrabct_v2 --dev_param tegra234-br-bct_b-p3767-0000-l4t_cpp.dtb --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --brbct br_bct.cfg --chip 0x23 0
[   5.8087 ] Updating customer data section
[   5.8094 ] tegrabct_v2 --chip 0x23 0 --brbct br_bct_BR.bct --update_custinfo custinfo_out.bin
[   5.8110 ] Updating bl info
[   5.8115 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updateblinfo flash.xml.bin --updatesig images_list_signed.xml
[   5.8166 ] Generating SHA2 Hash
[   5.8187 ] Sha saved in br_bct_BR.sha
[   5.8179 ] Get Signed section of bct
[   5.8184 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --listbct bct_list.xml
[   5.8194 ] Signing BCT
[   5.8203 ] tegrasign_v3.py --key None --list bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.8206 ] Assuming zero filled SBK key
[   5.8232 ] Sha saved in br_bct_BR.sha
[   5.8236 ] Warning: pub_key.key is not found
[   5.8227 ] Updating BCT with signature
[   5.8232 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesig bct_list_signed.xml
[   5.8237 ] Offset :4608 Len :3584
[   5.8242 ] Generating SHA2 Hash
[   5.8252 ] tegrasign_v3.py --key None --list bct_list.xml --sha sha512
[   5.8253 ] Assuming zero filled SBK key
[   5.8253 ] Assuming zero filled SBK key
[   5.8278 ] Sha saved in br_bct_BR.sha
[   5.8272 ] Updating BCT with SHA2 Hash
[   5.8276 ] tegrabct_v2 --brbct br_bct_BR.bct --chip 0x23 0 --updatesha bct_list_signed.xml
[   5.8281 ] Offset :4608 Len :3584
[   5.8285 ] Offset :68 Len :8124
[   5.8289 ] Generating BCT backup image
[   5.8289 ] dd if=/dev/zero of=bct_backup.img bs=1 count=32768
[   5.8294 ] 32768+0 records in
[   5.9038 ] 32768+0 records out
[   5.9038 ] 32768 bytes (33 kB, 32 KiB) copied, 0.0736687 s, 445 kB/s
[   5.9038 ]
[   5.9038 ] Concatenating BCT for chain A to bct_backup.img

[   5.9039 ] dd if=br_bct_BR.bct of=bct_backup.img bs=1 seek=0 conv=notrunc
[   5.9045 ] 8192+0 records in
[   5.9183 ] 8192+0 records out
[   5.9183 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0131212 s, 624 kB/s
[   5.9183 ]
[   5.9183 ] Concatenating BCT for chain B to bct_backup.img

[   5.9183 ] dd if=br_bct_b_BR.bct of=bct_backup.img bs=1 seek=16384 conv=notrunc
[   5.9191 ] 8192+0 records in
[   5.9328 ] 8192+0 records out
[   5.9328 ] 8192 bytes (8.2 kB, 8.0 KiB) copied, 0.0131465 s, 623 kB/s
[   5.9329 ]
[   5.9329 ] Generating coldboot mb1-bct
[   5.9334 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   5.9338 ] MB1-BCT version: 0.13

[   5.9358 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   5.9363 ] Added Platform Config 0 data with size :- 2416

[   5.9388 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   5.9393 ] WARNING: unknown node 'g2'
[   5.9394 ] WARNING: unknown node 'g2'
[   5.9394 ] WARNING: unknown node 'g9'
[   5.9394 ] WARNING: unknown node 'g9'
[   5.9394 ] Added Platform Config 2 data with size :- 24
[   5.9394 ]
[   5.9394 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   5.9394 ] Added Platform Config 4 data with size :- 288
[   5.9394 ]
[   5.9394 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   5.9394 ] Added Platform Config 3 data with size :- 52
[   5.9394 ]
[   5.9394 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   5.9395 ] WARNING: unknown property 'major'
[   5.9395 ] WARNING: unknown property 'minor'
[   5.9395 ] Added Platform Config 5 data with size :- 512
[   5.9395 ]
[   5.9395 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   5.9395 ] WARNING: unknown property 'major'
[   5.9395 ] WARNING: unknown property 'minor'
[   5.9395 ] Added Platform Config 7 data with size :- 380
[   5.9395 ]
[   5.9395 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   5.9395 ] Added Platform Config 8 data with size :- 24
[   5.9395 ]
[   5.9395 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   5.9396 ] Added Platform Config 9 data with size :- 100
[   5.9396 ]
[   5.9396 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   5.9396 ] ModuleCount 0 NumProdNames 0
[   5.9396 ] Added Platform Config 6 data with size :- 16
[   5.9396 ]
[   5.9396 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   5.9396 ] ERROR: /ratchet/atf is not supported
[   5.9396 ]
[   5.9396 ] Updating mb1-bct with firmware information
[   5.9401 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_cold_boot_bct_MB1.bct --updatefwinfo flash.xml.bin
[   5.9452 ] tegrahost_v2 --chip 0x23 0 --align mb1_cold_boot_bct_MB1_aligned.bct
[   5.9463 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_cold_boot_bct_MB1_aligned.bct zerosbk
[   5.9468 ] adding BCH for mb1_cold_boot_bct_MB1_aligned.bct
[   5.9492 ] tegrasign_v3.py --key None --list mb1_cold_boot_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.9494 ] Assuming zero filled SBK key
[   5.9511 ] Warning: pub_key.key is not found
[   5.9508 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_cold_boot_bct_MB1_aligned_sigheader.bct.encrypt mb1_cold_boot_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   5.9535 ] Generating recovery mb1-bct
[   5.9541 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct.cfg --misc tegra234-mb1-bct-misc-p3767-0000_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --pinmux tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb --pmc tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb --pmic tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb --brcommand tegra234-mb1-bct-reset-p3767-0000_cpp.dtb --prod tegra234-mb1-bct-prod-p3767-0000_cpp.dtb --gpioint tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb --uphy tegra234-mb1-bct-uphylane-si_cpp.dtb --device tegra234-mb1-bct-device-p3767-0000_cpp.dtb --deviceprod tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb --minratchet tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb --ratchet_blob ratchet_blob.bin
[   5.9546 ] MB1-BCT version: 0.13

[   5.9566 ] Parsing config file :tegra234-mb1-bct-pinmux-p3767-dp-a03_cpp.dtb
[   5.9570 ] Added Platform Config 0 data with size :- 2416

[   5.9593 ] Parsing config file :tegra234-mb1-bct-padvoltage-p3767-dp-a03_cpp.dtb
[   5.9598 ] WARNING: unknown node 'g2'
[   5.9599 ] WARNING: unknown node 'g2'
[   5.9599 ] WARNING: unknown node 'g9'
[   5.9599 ] WARNING: unknown node 'g9'
[   5.9599 ] Added Platform Config 2 data with size :- 24
[   5.9599 ]
[   5.9600 ] Parsing config file :tegra234-mb1-bct-pmic-p3767-0000-a02_cpp.dtb
[   5.9600 ] Added Platform Config 4 data with size :- 288
[   5.9600 ]
[   5.9600 ] Parsing config file :tegra234-mb1-bct-reset-p3767-0000_cpp.dtb
[   5.9600 ] Added Platform Config 3 data with size :- 52
[   5.9600 ]
[   5.9600 ] Parsing config file :tegra234-mb1-bct-prod-p3767-0000_cpp.dtb
[   5.9600 ] WARNING: unknown property 'major'
[   5.9600 ] WARNING: unknown property 'minor'
[   5.9600 ] Added Platform Config 5 data with size :- 512
[   5.9600 ]
[   5.9600 ] Parsing config file :tegra234-mb1-bct-gpioint-p3767-0000_cpp.dtb
[   5.9600 ] WARNING: unknown property 'major'
[   5.9600 ] WARNING: unknown property 'minor'
[   5.9600 ] Added Platform Config 7 data with size :- 380
[   5.9600 ]
[   5.9601 ] Parsing config file :tegra234-mb1-bct-uphylane-si_cpp.dtb
[   5.9601 ] Added Platform Config 8 data with size :- 24
[   5.9601 ]
[   5.9601 ] Parsing config file :tegra234-mb1-bct-device-p3767-0000_cpp.dtb
[   5.9601 ] Added Platform Config 9 data with size :- 100
[   5.9601 ]
[   5.9601 ] Parsing config file :tegra234-mb1-bct-cprod-p3767-0000_cpp.dtb
[   5.9601 ] ModuleCount 0 NumProdNames 0
[   5.9601 ] Added Platform Config 6 data with size :- 16
[   5.9601 ]
[   5.9601 ] Parsing config file :tegra234-mb1-bct-ratchet-p3767-0000_cpp.dtb
[   5.9601 ] ERROR: /ratchet/atf is not supported
[   5.9601 ]
[   5.9601 ] Updating mb1-bct with firmware information
[   5.9605 ] tegrabct_v2 --chip 0x23 0 --mb1bct mb1_bct_MB1.bct --recov --updatefwinfo flash.xml.bin
[   5.9616 ] tegrahost_v2 --chip 0x23 0 --align mb1_bct_MB1_aligned.bct
[   5.9620 ] Generating SHA2 Hash for mb1bct
[   5.9640 ] Sha saved in mb1_bct_MB1_aligned.sha
[   5.9636 ] tegrahost_v2 --chip 0x23 0 --magicid MBCT --ratchet_blob ratchet_blob.bin --appendsigheader mb1_bct_MB1_aligned.bct zerosbk
[   5.9639 ] adding BCH for mb1_bct_MB1_aligned.bct
[   5.9659 ] tegrasign_v3.py --key None --list mb1_bct_MB1_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   5.9660 ] Assuming zero filled SBK key
[   5.9674 ] Warning: pub_key.key is not found
[   5.9670 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb1_bct_MB1_aligned_sigheader.bct.encrypt mb1_bct_MB1_aligned_sigheader.bct.hash zerosbk
[   5.9680 ] Generating coldboot mem-bct
[   5.9685 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   5.9688 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   6.1557 ] Packing sdram param for instance[0]
[   6.1559 ] Packing sdram param for instance[1]
[   6.1560 ] Packing sdram param for instance[2]
[   6.1562 ] Packing sdram param for instance[3]
[   6.1563 ] Packing sdram param for instance[4]
[   6.1565 ] Packing sdram param for instance[5]
[   6.1566 ] Packing sdram param for instance[6]
[   6.1567 ] Packing sdram param for instance[7]
[   6.1569 ] Packing sdram param for instance[8]
[   6.1570 ] Packing sdram param for instance[9]
[   6.1572 ] Packing sdram param for instance[10]
[   6.1575 ] Packing sdram param for instance[11]
[   6.1577 ] Packing sdram param for instance[12]
[   6.1580 ] Packing sdram param for instance[13]
[   6.1582 ] Packing sdram param for instance[14]
[   6.1585 ] Packing sdram param for instance[15]
[   6.3260 ] Getting sector size from pt
[   6.3267 ] tegraparser_v2 --getsectorsize flash.xml.bin sector_info.bin
[   6.3275 ] BlockSize read from layout is 0x200

[   6.3281 ] tegrahost_v2 --chip 0x23 0 --blocksize 512 --magicid MEMB --addsigheader_multi tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   6.3285 ] Binary 0 length is 58752
[   6.3290 ] Binary 0 align length is 58880
[   6.3301 ] Binary 1 length is 58752
[   6.3302 ] Binary 1 align length is 58880
[   6.3313 ] Binary 2 length is 58752
[   6.3315 ] Binary 2 align length is 58880
[   6.3326 ] Binary 3 length is 58752
[   6.3327 ] Binary 3 align length is 58880
[   6.3337 ] Buffer length is 235520
[   6.3339 ] adding BCH for tegra234-p3767-0001-sdram-l4t_cpp_1.bct
[   6.3343 ] new length is 243712
[   6.3343 ]
[   6.3350 ] tegrahost_v2 --chip 0x23 0 --align mem_coldboot_aligned.bct
[   6.3359 ] tegrahost_v2 --chip 0x23 0 --magicid MEMB --ratchet_blob ratchet_blob.bin --appendsigheader mem_coldboot_aligned.bct zerosbk
[   6.3363 ] Header already present for mem_coldboot_aligned.bct
[   6.3383 ] tegrasign_v3.py --key None --list mem_coldboot_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.3385 ] Assuming zero filled SBK key
[   6.3404 ] Warning: pub_key.key is not found
[   6.3401 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_coldboot_aligned_sigheader.bct.encrypt mem_coldboot_aligned_sigheader.bct.hash zerosbk
[   6.3419 ] Generating recovery mem-bct
[   6.3424 ] tegrabct_v2 --chip 0x23 0 --sdram tegra234-p3767-0001-sdram-l4t_cpp.dtb --wb0sdram tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb --membct tegra234-p3767-0001-sdram-l4t_cpp_1.bct tegra234-p3767-0001-sdram-l4t_cpp_2.bct tegra234-p3767-0001-sdram-l4t_cpp_3.bct tegra234-p3767-0001-sdram-l4t_cpp_4.bct
[   6.3429 ]  packing sdram params with Wb0 file tegra234-p3767-0001-wb0sdram-l4t_cpp.dtb
[   6.5251 ] Packing sdram param for instance[0]
[   6.5254 ] Packing sdram param for instance[1]
[   6.5256 ] Packing sdram param for instance[2]
[   6.5257 ] Packing sdram param for instance[3]
[   6.5259 ] Packing sdram param for instance[4]
[   6.5261 ] Packing sdram param for instance[5]
[   6.5262 ] Packing sdram param for instance[6]
[   6.5263 ] Packing sdram param for instance[7]
[   6.5265 ] Packing sdram param for instance[8]
[   6.5266 ] Packing sdram param for instance[9]
[   6.5267 ] Packing sdram param for instance[10]
[   6.5268 ] Packing sdram param for instance[11]
[   6.5269 ] Packing sdram param for instance[12]
[   6.5272 ] Packing sdram param for instance[13]
[   6.5274 ] Packing sdram param for instance[14]
[   6.5277 ] Packing sdram param for instance[15]
[   6.6966 ] Reading ramcode from backup chip_info.bin file
[   6.6967 ] RAMCODE Read from Device: 2

[   6.6967 ] Using ramcode 0
[   6.6968 ] Disabled BPMP dtb trim, using default dtb
[   6.6968 ]
[   6.6977 ] tegrahost_v2 --chip 0x23 0 --align mem_rcm_aligned.bct
[   6.6987 ] tegrahost_v2 --chip 0x23 0 --magicid MEM0 --ratchet_blob ratchet_blob.bin --appendsigheader mem_rcm_aligned.bct zerosbk
[   6.6991 ] adding BCH for mem_rcm_aligned.bct
[   6.7028 ] tegrasign_v3.py --key None --list mem_rcm_aligned_sigheader.bct_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.7031 ] Assuming zero filled SBK key
[   6.7047 ] Warning: pub_key.key is not found
[   6.7043 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mem_rcm_aligned_sigheader.bct.encrypt mem_rcm_aligned_sigheader.bct.hash zerosbk
[   6.7056 ] Copying signatures
[   6.7062 ] tegrahost_v2 --chip 0x23 0 --partitionlayout flash.xml.bin --updatesig images_list_signed.xml
[   6.7724 ] tegraparser_v2 --generategpt --pt flash.xml.bin
[   6.7729 ] gpt_secondary_3_0.bin:
[   6.7735 ] partition_id       partition_name                        StartingLba       EndingLba
[   6.7740 ]            1       BCT                                            0            2047
[   6.7743 ]            2       A_mb1                                       2048            3071
[   6.7746 ]            3       A_psc_bl1                                   3072            3583
[   6.7750 ]            4       A_MB1_BCT                                   3584            3839
[   6.7753 ]            5       A_MEM_BCT                                   3840            4351
[   6.7756 ]            6       A_tsec-fw                                   4352            6399
[   6.7758 ]            7       A_nvdec                                     6400            8447
[   6.7761 ]            8       A_mb2                                       8448            9471
[   6.7764 ]            9       A_xusb-fw                                   9472            9983
[   6.7766 ]           10       A_bpmp-fw                                   9984           13055
[   6.7769 ]           11       A_bpmp-fw-dtb                              13056           21247
[   6.7771 ]           12       A_psc-fw                                   21248           22783
[   6.7774 ]           13       A_mts-mce                                  22784           23807
[   6.7776 ]           14       A_sc7                                      23808           24191
[   6.7779 ]           15       A_pscrf                                    24192           24575
[   6.7781 ]           16       A_mb2rf                                    24576           24831
[   6.7784 ]           17       A_cpu-bootloader                           24832           31999
[   6.7787 ]           18       A_secure-os                                32000           40191
[   6.7790 ]           19       A_smm-fw                                   40192           44287
[   6.7793 ]           20       A_eks                                      44288           44799
[   6.7796 ]           21       A_dce-fw                                   44800           55039
[   6.7798 ]           22       A_spe-fw                                   55040           56191
[   6.7801 ]           23       A_rce-fw                                   56192           58239
[   6.7803 ]           24       A_adsp-fw                                  58240           62335
[   6.7806 ]           25       A_pva-fw                                   62336           62847
[   6.7809 ]           26       A_reserved_on_boot                         62848           65023
[   6.7811 ]           27       B_mb1                                      65024           66047
[   6.7814 ]           28       B_psc_bl1                                  66048           66559
[   6.7816 ]           29       B_MB1_BCT                                  66560           66815
[   6.7820 ]           30       B_MEM_BCT                                  66816           67327
[   6.7822 ]           31       B_tsec-fw                                  67328           69375
[   6.7825 ]           32       B_nvdec                                    69376           71423
[   6.7828 ]           33       B_mb2                                      71424           72447
[   6.7830 ]           34       B_xusb-fw                                  72448           72959
[   6.7833 ]           35       B_bpmp-fw                                  72960           76031
[   6.7836 ]           36       B_bpmp-fw-dtb                              76032           84223
[   6.7838 ]           37       B_psc-fw                                   84224           85759
[   6.7841 ]           38       B_mts-mce                                  85760           86783
[   6.7844 ]           39       B_sc7                                      86784           87167
[   6.7846 ]           40       B_pscrf                                    87168           87551
[   6.7849 ]           41       B_mb2rf                                    87552           87807
[   6.7852 ]           42       B_cpu-bootloader                           87808           94975
[   6.7854 ]           43       B_secure-os                                94976          103167
[   6.7857 ]           44       B_smm-fw                                  103168          107263
[   6.7860 ]           45       B_eks                                     107264          107775
[   6.7862 ]           46       B_dce-fw                                  107776          118015
[   6.7865 ]           47       B_spe-fw                                  118016          119167
[   6.7868 ]           48       B_rce-fw                                  119168          121215
[   6.7871 ]           49       B_adsp-fw                                 121216          125311
[   6.7873 ]           50       B_pva-fw                                  125312          125823
[   6.7876 ]           51       B_reserved_on_boot                        125824          127999
[   6.7879 ]           52       uefi_variables                            128000          128511
[   6.7881 ]           53       uefi_ftw                                  128512          129535
[   6.7885 ]           54       reserved                                  129536          129919
[   6.7888 ]           55       worm                                      129920          130303
[   6.7891 ]           56       BCT-boot-chain_backup                     130304          130431
[   6.7894 ]           57       reserved_partition                        130432          130559
[   6.7896 ]           58       secondary_gpt_backup                      130560          130687
[   6.7899 ]           59       B_VER                                     130688          130815
[   6.7903 ]           60       A_VER                                     130816          130943
[   6.7905 ] gpt_primary_6_0.bin:
[   6.8008 ] partition_id       partition_name                        StartingLba       EndingLba
[   6.8013 ]            1       APP                                      3050048        119537623
[   6.8017 ]            2       A_kernel                                      40          262183
[   6.8021 ]            3       A_kernel-dtb                              262184          263719
[   6.8025 ]            4       A_reserved_on_user                        263720          328487
[   6.8029 ]            5       B_kernel                                  328488          590631
[   6.8031 ]            6       B_kernel-dtb                              590632          592167
[   6.8031 ]            7       B_reserved_on_user                        592168          656935
[   6.8032 ]            8       recovery                                  656936          820775
[   6.8032 ]            9       recovery-dtb                              820776          821799
[   6.8032 ]           10       esp                                       821800          952871
[   6.8032 ]           11       recovery_alt                              952872         1116711
[   6.8032 ]           12       recovery-dtb_alt                         1116712         1117735
[   6.8032 ]           13       esp_alt                                  1117736         1248807
[   6.8032 ]           14       UDA                                      1248832         2068031
[   6.8032 ]           15       reserved                                 2068032         3050047
[   6.8032 ] gpt_secondary_6_0.bin:
[   6.8032 ] partition_id       partition_name                        StartingLba       EndingLba
[   6.8032 ]            1       APP                                      3050048        119537623
[   6.8032 ]            2       A_kernel                                      40          262183
[   6.8032 ]            3       A_kernel-dtb                              262184          263719
[   6.8032 ]            4       A_reserved_on_user                        263720          328487
[   6.8032 ]            5       B_kernel                                  328488          590631
[   6.8032 ]            6       B_kernel-dtb                              590632          592167
[   6.8032 ]            7       B_reserved_on_user                        592168          656935
[   6.8032 ]            8       recovery                                  656936          820775
[   6.8032 ]            9       recovery-dtb                              820776          821799
[   6.8032 ]           10       esp                                       821800          952871
[   6.8033 ]           11       recovery_alt                              952872         1116711
[   6.8033 ]           12       recovery-dtb_alt                         1116712         1117735
[   6.8033 ]           13       esp_alt                                  1117736         1248807
[   6.8033 ]           14       UDA                                      1248832         2068031
[   6.8033 ]           15       reserved                                 2068032         3050047
[   6.8033 ]
[   6.8049 ] Get magic id
[   6.8054 ] tegraparser_v2 --get_magic psc_fw
[   6.8058 ] PFWP
[   6.8059 ] partition type psc_fw, magic id = PFWP
[   6.8068 ] tegrahost_v2 --chip 0x23 0 --align pscfw_t234_prod_aligned.bin
[   6.8076 ] tegrahost_v2 --chip 0x23 0 --magicid PFWP --ratchet_blob ratchet_blob.bin --appendsigheader pscfw_t234_prod_aligned.bin zerosbk
[   6.8081 ] Header already present for pscfw_t234_prod_aligned.bin
[   6.8151 ] tegrasign_v3.py --key None --list pscfw_t234_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8152 ] Assuming zero filled SBK key
[   6.8171 ] Warning: pub_key.key is not found
[   6.8167 ] tegrahost_v2 --chip 0x23 0 --updatesigheader pscfw_t234_prod_aligned_sigheader.bin.encrypt pscfw_t234_prod_aligned_sigheader.bin.hash zerosbk
[   6.8184 ] Get magic id
[   6.8188 ] tegraparser_v2 --get_magic mts_mce
[   6.8192 ] MTSM
[   6.8194 ] partition type mts_mce, magic id = MTSM
[   6.8202 ] tegrahost_v2 --chip 0x23 0 --align mce_flash_o10_cr_prod_aligned.bin
[   6.8212 ] tegrahost_v2 --chip 0x23 0 --magicid MTSM --ratchet_blob ratchet_blob.bin --appendsigheader mce_flash_o10_cr_prod_aligned.bin zerosbk
[   6.8216 ] Header already present for mce_flash_o10_cr_prod_aligned.bin
[   6.8265 ] tegrasign_v3.py --key None --list mce_flash_o10_cr_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8266 ] Assuming zero filled SBK key
[   6.8282 ] Warning: pub_key.key is not found
[   6.8278 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mce_flash_o10_cr_prod_aligned_sigheader.bin.encrypt mce_flash_o10_cr_prod_aligned_sigheader.bin.hash zerosbk
[   6.8294 ] Get magic id
[   6.8298 ] tegraparser_v2 --get_magic tsec_fw
[   6.8302 ] TSEC
[   6.8303 ] partition type tsec_fw, magic id = TSEC
[   6.8310 ] tegrahost_v2 --chip 0x23 0 --align tsec_t234_aligned.bin
[   6.8318 ] tegrahost_v2 --chip 0x23 0 --magicid TSEC --ratchet_blob ratchet_blob.bin --appendsigheader tsec_t234_aligned.bin zerosbk
[   6.8322 ] Header already present for tsec_t234_aligned.bin
[   6.8368 ] tegrasign_v3.py --key None --list tsec_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8370 ] Assuming zero filled SBK key
[   6.8387 ] Warning: pub_key.key is not found
[   6.8383 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tsec_t234_aligned_sigheader.bin.encrypt tsec_t234_aligned_sigheader.bin.hash zerosbk
[   6.8397 ] Get magic id
[   6.8401 ] tegraparser_v2 --get_magic mb2_applet
[   6.8405 ] MB2A
[   6.8406 ] partition type mb2_applet, magic id = MB2A
[   6.8413 ] tegrahost_v2 --chip 0x23 0 --align applet_t234_aligned.bin
[   6.8421 ] tegrahost_v2 --chip 0x23 0 --magicid MB2A --ratchet_blob ratchet_blob.bin --appendsigheader applet_t234_aligned.bin zerosbk
[   6.8425 ] adding BCH for applet_t234_aligned.bin
[   6.8540 ] tegrasign_v3.py --key None --list applet_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.8542 ] Assuming zero filled SBK key
[   6.8564 ] Warning: pub_key.key is not found
[   6.8564 ] tegrahost_v2 --chip 0x23 0 --updatesigheader applet_t234_aligned_sigheader.bin.encrypt applet_t234_aligned_sigheader.bin.hash zerosbk
[   6.8588 ] Generating recovery mb2-bct
[   6.8589 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_bct.cfg --recov --mb2bctcfg tegra234-mb2-bct-misc-p3767-0000_cpp.dtb --scr tegra234-mb2-bct-scr-p3767-0000_cpp.dtb
[   6.8593 ] ERROR: value 0x31 is out of range
[   6.8607 ] ERROR: value 0x31 is out of range
[   6.8609 ] ERROR: value 0x31 is out of range
[   6.8611 ] ERROR: value 0x31 is out of range
[   6.8613 ] WARNING: unknown property 'tfc_version'
[   6.8614 ] WARNING: unknown property 'addr_header_version'
[   6.8771 ] Updating mb2-bct with storage information for RCM
[   6.8778 ] tegrabct_v2 --chip 0x23 0 --mb2bct mb2_bct_MB2.bct --updatestorageinfo flash.xml.bin
[   6.8790 ] Concatenating mb2-bct to mb2 binary
[   6.8791 ] mb2_bin_file = mb2_t234.bin
[   6.8791 ] mb2_bct_file = mb2_bct_MB2.bct
[   6.8799 ] Get magic id
[   6.8805 ] tegraparser_v2 --get_magic mb2_bootloader
[   6.8810 ] MB2B
[   6.8811 ] partition type mb2_bootloader, magic id = MB2B
[   6.8824 ] tegrahost_v2 --chip 0x23 0 --align mb2_t234_with_mb2_bct_MB2_aligned.bin
[   6.8835 ] tegrahost_v2 --chip 0x23 0 --magicid MB2B --ratchet_blob ratchet_blob.bin --appendsigheader mb2_t234_with_mb2_bct_MB2_aligned.bin zerosbk
[   6.8840 ] adding BCH for mb2_t234_with_mb2_bct_MB2_aligned.bin
[   6.9019 ] tegrasign_v3.py --key None --list mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.9023 ] Assuming zero filled SBK key
[   6.9046 ] Warning: pub_key.key is not found
[   6.9045 ] tegrahost_v2 --chip 0x23 0 --updatesigheader mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin.encrypt mb2_t234_with_mb2_bct_MB2_aligned_sigheader.bin.hash zerosbk
[   6.9074 ] Get magic id
[   6.9079 ] tegraparser_v2 --get_magic xusb_fw
[   6.9084 ] XUSB
[   6.9087 ] partition type xusb_fw, magic id = XUSB
[   6.9097 ] tegrahost_v2 --chip 0x23 0 --align xusb_t234_prod_aligned.bin
[   6.9106 ] tegrahost_v2 --chip 0x23 0 --magicid XUSB --ratchet_blob ratchet_blob.bin --appendsigheader xusb_t234_prod_aligned.bin zerosbk
[   6.9111 ] adding BCH for xusb_t234_prod_aligned.bin
[   6.9188 ] tegrasign_v3.py --key None --list xusb_t234_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   6.9190 ] Assuming zero filled SBK key
[   6.9208 ] Warning: pub_key.key is not found
[   6.9204 ] tegrahost_v2 --chip 0x23 0 --updatesigheader xusb_t234_prod_aligned_sigheader.bin.encrypt xusb_t234_prod_aligned_sigheader.bin.hash zerosbk
[   6.9220 ] Get magic id
[   6.9225 ] tegraparser_v2 --get_magic pva_fw
[   6.9229 ] PVAF
[   6.9231 ] partition type pva_fw, magic id = PVAF
[   6.9372 ] tegrahost_v2 --chip 0x23 0 --align nvpva_020_aligned.fw
[   6.9383 ] tegrahost_v2 --chip 0x23 0 --magicid PVAF --ratchet_blob ratchet_blob.bin --appendsigheader nvpva_020_aligned.fw zerosbk
[   6.9387 ] adding BCH for nvpva_020_aligned.fw
[   7.0130 ] tegrasign_v3.py --key None --list nvpva_020_aligned_sigheader.fw_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.0132 ] Assuming zero filled SBK key
[   7.0184 ] Warning: pub_key.key is not found
[   7.0181 ] tegrahost_v2 --chip 0x23 0 --updatesigheader nvpva_020_aligned_sigheader.fw.encrypt nvpva_020_aligned_sigheader.fw.hash zerosbk
[   7.0425 ] Kernel DTB used: kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
[   7.0426 ] Concatenating kernel-dtb to dce-fw binary
[   7.0427 ] dce_bin = display-t234-dce.bin
[   7.0427 ] kernel_dtb = kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
[   7.0428 ] dce_with_dtb = display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv.bin
[   7.1101 ] Get magic id
[   7.1109 ] tegraparser_v2 --get_magic dce_fw
[   7.1114 ] DCEF
[   7.1116 ] partition type dce_fw, magic id = DCEF
[   7.1684 ] tegrahost_v2 --chip 0x23 0 --align display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin
[   7.1714 ] tegrahost_v2 --chip 0x23 0 --magicid DCEF --ratchet_blob ratchet_blob.bin --appendsigheader display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned.bin zerosbk compress
[   7.1725 ] INFO: compressing display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned.bin
[   7.4316 ] INFO: complete compression, display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned.bin, ratio = 6%
[   7.4563 ] adding BCH for display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned_blob_w_bin.bin
[   7.4906 ] tegrasign_v3.py --key None --list display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned_blob_w_bin_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.4908 ] Assuming zero filled SBK key
[   7.4933 ] Warning: pub_key.key is not found
[   7.4930 ] tegrahost_v2 --chip 0x23 0 --updatesigheader display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned_blob_w_bin_sigheader.bin.encrypt display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_aligned_aligned_blob_w_bin_sigheader.bin.hash zerosbk
[   7.4967 ] Get magic id
[   7.4974 ] tegraparser_v2 --get_magic nvdec
[   7.4979 ] NDEC
[   7.4981 ] partition type nvdec, magic id = NDEC
[   7.4992 ] tegrahost_v2 --chip 0x23 0 --align nvdec_t234_prod_aligned.fw
[   7.5003 ] tegrahost_v2 --chip 0x23 0 --magicid NDEC --ratchet_blob ratchet_blob.bin --appendsigheader nvdec_t234_prod_aligned.fw zerosbk
[   7.5008 ] Header already present for nvdec_t234_prod_aligned.fw
[   7.5077 ] tegrasign_v3.py --key None --list nvdec_t234_prod_aligned_sigheader.fw_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5081 ] Assuming zero filled SBK key
[   7.5099 ] Warning: pub_key.key is not found
[   7.5096 ] tegrahost_v2 --chip 0x23 0 --updatesigheader nvdec_t234_prod_aligned_sigheader.fw.encrypt nvdec_t234_prod_aligned_sigheader.fw.hash zerosbk
[   7.5114 ] Get magic id
[   7.5119 ] tegraparser_v2 --get_magic bpmp_fw
[   7.5125 ] BPMF
[   7.5126 ] partition type bpmp_fw, magic id = BPMF
[   7.5142 ] tegrahost_v2 --chip 0x23 0 --align bpmp_t234-TE950M-A1_prod_aligned.bin
[   7.5153 ] tegrahost_v2 --chip 0x23 0 --magicid BPMF --ratchet_blob ratchet_blob.bin --appendsigheader bpmp_t234-TE950M-A1_prod_aligned.bin zerosbk
[   7.5158 ] Header already present for bpmp_t234-TE950M-A1_prod_aligned.bin
[   7.5363 ] tegrasign_v3.py --key None --list bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5364 ] Assuming zero filled SBK key
[   7.5391 ] Warning: pub_key.key is not found
[   7.5388 ] tegrahost_v2 --chip 0x23 0 --updatesigheader bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin.encrypt bpmp_t234-TE950M-A1_prod_aligned_sigheader.bin.hash zerosbk
[   7.5423 ] Using bpmp-dtb concatenated with odmdata
[   7.5424 ] Get magic id
[   7.5428 ] tegraparser_v2 --get_magic bpmp_fw_dtb
[   7.5432 ] BPMD
[   7.5434 ] partition type bpmp_fw_dtb, magic id = BPMD
[   7.5442 ] tegrahost_v2 --chip 0x23 0 --align tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   7.5451 ] tegrahost_v2 --chip 0x23 0 --magicid BPMD --ratchet_blob ratchet_blob.bin --appendsigheader tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb zerosbk
[   7.5454 ] adding BCH for tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned.dtb
[   7.5540 ] tegrasign_v3.py --key None --list tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5542 ] Assuming zero filled SBK key
[   7.5558 ] Warning: pub_key.key is not found
[   7.5555 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb.encrypt tegra234-bpmp-3767-0003-3509-a02_with_odm_aligned_sigheader.dtb.hash zerosbk
[   7.5570 ] Get magic id
[   7.5575 ] tegraparser_v2 --get_magic rce_fw
[   7.5579 ] RCEF
[   7.5580 ] partition type rce_fw, magic id = RCEF
[   7.5590 ] tegrahost_v2 --chip 0x23 0 --align camera-rtcpu-t234-rce_aligned.img
[   7.5597 ] tegrahost_v2 --chip 0x23 0 --magicid RCEF --ratchet_blob ratchet_blob.bin --appendsigheader camera-rtcpu-t234-rce_aligned.img zerosbk
[   7.5601 ] adding BCH for camera-rtcpu-t234-rce_aligned.img
[   7.5775 ] tegrasign_v3.py --key None --list camera-rtcpu-t234-rce_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.5776 ] Assuming zero filled SBK key
[   7.5796 ] Warning: pub_key.key is not found
[   7.5792 ] tegrahost_v2 --chip 0x23 0 --updatesigheader camera-rtcpu-t234-rce_aligned_sigheader.img.encrypt camera-rtcpu-t234-rce_aligned_sigheader.img.hash zerosbk
[   7.5812 ] Get magic id
[   7.5816 ] tegraparser_v2 --get_magic ape_fw
[   7.5820 ] APEF
[   7.5821 ] partition type ape_fw, magic id = APEF
[   7.5831 ] tegrahost_v2 --chip 0x23 0 --align adsp-fw_aligned.bin
[   7.5840 ] tegrahost_v2 --chip 0x23 0 --magicid APEF --ratchet_blob ratchet_blob.bin --appendsigheader adsp-fw_aligned.bin zerosbk
[   7.5843 ] adding BCH for adsp-fw_aligned.bin
[   7.6003 ] tegrasign_v3.py --key None --list adsp-fw_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.6005 ] Assuming zero filled SBK key
[   7.6024 ] Warning: pub_key.key is not found
[   7.6020 ] tegrahost_v2 --chip 0x23 0 --updatesigheader adsp-fw_aligned_sigheader.bin.encrypt adsp-fw_aligned_sigheader.bin.hash zerosbk
[   7.6040 ] Get magic id
[   7.6045 ] tegraparser_v2 --get_magic spe_fw
[   7.6048 ] SPEF
[   7.6050 ] partition type spe_fw, magic id = SPEF
[   7.6059 ] tegrahost_v2 --chip 0x23 0 --align spe_t234_aligned.bin
[   7.6069 ] tegrahost_v2 --chip 0x23 0 --magicid SPEF --ratchet_blob ratchet_blob.bin --appendsigheader spe_t234_aligned.bin zerosbk
[   7.6073 ] adding BCH for spe_t234_aligned.bin
[   7.6181 ] tegrasign_v3.py --key None --list spe_t234_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.6183 ] Assuming zero filled SBK key
[   7.6203 ] Warning: pub_key.key is not found
[   7.6200 ] tegrahost_v2 --chip 0x23 0 --updatesigheader spe_t234_aligned_sigheader.bin.encrypt spe_t234_aligned_sigheader.bin.hash zerosbk
[   7.6218 ] Get magic id
[   7.6223 ] tegraparser_v2 --get_magic tos
[   7.6227 ] TOSB
[   7.6228 ] partition type tos, magic id = TOSB
[   7.6247 ] tegrahost_v2 --chip 0x23 0 --align tos-optee_t234_aligned.img
[   7.6257 ] tegrahost_v2 --chip 0x23 0 --magicid TOSB --ratchet_blob ratchet_blob.bin --appendsigheader tos-optee_t234_aligned.img zerosbk
[   7.6261 ] adding BCH for tos-optee_t234_aligned.img
[   7.6751 ] tegrasign_v3.py --key None --list tos-optee_t234_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.6753 ] Assuming zero filled SBK key
[   7.6793 ] Warning: pub_key.key is not found
[   7.6790 ] tegrahost_v2 --chip 0x23 0 --updatesigheader tos-optee_t234_aligned_sigheader.img.encrypt tos-optee_t234_aligned_sigheader.img.hash zerosbk
[   7.6844 ] Get magic id
[   7.6851 ] tegraparser_v2 --get_magic eks
[   7.6856 ] EKSB
[   7.6858 ] partition type eks, magic id = EKSB
[   7.6866 ] tegrahost_v2 --chip 0x23 0 --align eks_t234_aligned.img
[   7.6876 ] tegrahost_v2 --chip 0x23 0 --magicid EKSB --ratchet_blob ratchet_blob.bin --appendsigheader eks_t234_aligned.img zerosbk
[   7.6880 ] adding BCH for eks_t234_aligned.img
[   7.6897 ] tegrasign_v3.py --key None --list eks_t234_aligned_sigheader.img_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.6899 ] Assuming zero filled SBK key
[   7.6918 ] Warning: pub_key.key is not found
[   7.6915 ] tegrahost_v2 --chip 0x23 0 --updatesigheader eks_t234_aligned_sigheader.img.encrypt eks_t234_aligned_sigheader.img.hash zerosbk
[   7.6975 ] tegrahost_v2 --chip 0x23 0 --align uefi_jetson_with_dtb_aligned.bin
[   7.6987 ] tegrahost_v2 --chip 0x23 0 --magicid CPBL --ratchet_blob ratchet_blob.bin --appendsigheader uefi_jetson_with_dtb_aligned.bin zerosbk
[   7.6992 ] adding BCH for uefi_jetson_with_dtb_aligned.bin
[   7.8224 ] tegrasign_v3.py --key None --list uefi_jetson_with_dtb_aligned_sigheader.bin_list.xml --pubkeyhash pub_key.key --sha sha512
[   7.8226 ] Assuming zero filled SBK key
[   7.8302 ] Warning: pub_key.key is not found
[   7.8302 ] tegrahost_v2 --chip 0x23 0 --updatesigheader uefi_jetson_with_dtb_aligned_sigheader.bin.encrypt uefi_jetson_with_dtb_aligned_sigheader.bin.hash zerosbk
[   7.8398 ] Copying enc\/signed file in /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8828 ] Copying br bct for multi chains
[   7.8830 ] Signed BCT for boot chain A is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_BR.bct

[   7.8831 ] Signed BCT for boot chain B is copied to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/br_bct_b_BR.bct

[   7.8831 ] Copying BCT backup image bct_backup.img to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bct_backup.img
[   7.8871 ] Copying pscfw_t234_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8876 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/pscfw_t234_prod_sigheader.bin.encrypt
[   7.8876 ] Copying mce_flash_o10_cr_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8879 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mce_flash_o10_cr_prod_sigheader.bin.encrypt
[   7.8880 ] Copying tsec_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8883 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tsec_t234_sigheader.bin.encrypt
[   7.8883 ] Copying applet_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8886 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/applet_t234_sigheader.bin.encrypt
[   7.8886 ] Copying mb2_t234_with_mb2_bct_MB2_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8891 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/mb2_t234_with_mb2_bct_MB2_sigheader.bin.encrypt
[   7.8891 ] Copying xusb_t234_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8895 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/xusb_t234_prod_sigheader.bin.encrypt
[   7.8895 ] Copying nvpva_020_sigheader.fw.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8912 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvpva_020_sigheader.fw.encrypt
[   7.8912 ] Copying display-t234-dce_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8920 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_sigheader.bin.encrypt
[   7.8920 ] Copying display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_blob_w_bin_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8927 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/display-t234-dce_with_kernel_tegra234-p3768-0000+p3767-0005-nv_blob_w_bin_sigheader.bin.encrypt
[   7.8927 ] Copying nvdec_t234_prod_sigheader.fw.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8932 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/nvdec_t234_prod_sigheader.fw.encrypt
[   7.8932 ] Copying bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8944 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
[   7.8944 ] Copying tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8947 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
[   7.8947 ] Copying camera-rtcpu-t234-rce_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8953 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/camera-rtcpu-t234-rce_sigheader.img.encrypt
[   7.8953 ] Copying adsp-fw_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8959 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/adsp-fw_sigheader.bin.encrypt
[   7.8959 ] Copying spe_t234_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8963 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/spe_t234_sigheader.bin.encrypt
[   7.8963 ] Copying tos-optee_t234_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8978 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/tos-optee_t234_sigheader.img.encrypt
[   7.8978 ] Copying eks_t234_sigheader.img.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.8979 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/eks_t234_sigheader.img.encrypt
[   7.8979 ] Copying uefi_jetson_with_dtb_sigheader.bin.encrypt to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed
[   7.9016 ] Signed file: /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/uefi_jetson_with_dtb_sigheader.bin.encrypt
[   7.9037 ] tegraparser_v2 --pt flash.xml.bin --generateflashindex /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader/signed/flash.xml.tmp flash.idx
Using bpmp-dtb concatenated with odmdata in blob for t23x
./tegraflash.py --bl uefi_jetson_with_dtb_sigheader.bin.encrypt --bct br_bct_BR.bct --securedev  --bldtb tegra234-p3768-0000+p3767-0005-nv.dtb --applet rcm_2_encrypt.rcm --applet_softfuse rcm_1_encrypt.rcm --cmd "rcmboot"  --cfg secureflash.xml --chip 0x23 --mb1_bct mb1_bct_MB1_sigheader.bct.encrypt --mem_bct mem_rcm_sigheader.bct.encrypt --mb1_cold_boot_bct mb1_cold_boot_bct_MB1_sigheader.bct.encrypt --mb1_bin mb1_t234_prod_aligned_sigheader.bin.encrypt --psc_bl1_bin psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --mem_bct_cold_boot mem_coldboot_sigheader.bct.encrypt  --bins "psc_fw pscfw_t234_prod_sigheader.bin.encrypt; mts_mce mce_flash_o10_cr_prod_sigheader.bin.encrypt; tsec_fw tsec_t234_sigheader.bin.encrypt; mb2_applet applet_t234_sigheader.bin.encrypt; mb2_bootloader mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt; xusb_fw xusb_t234_prod_sigheader.bin.encrypt; pva_fw nvpva_020_sigheader.fw.encrypt; dce_fw display-t234-dce_sigheader.bin.encrypt; nvdec nvdec_t234_prod_sigheader.fw.encrypt; bpmp_fw bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt; bpmp_fw_dtb tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt; rce_fw camera-rtcpu-t234-rce_sigheader.img.encrypt; ape_fw adsp-fw_sigheader.bin.encrypt; spe_fw spe_t234_sigheader.bin.encrypt; tos tos-optee_t234_sigheader.img.encrypt; eks eks_t234_sigheader.img.encrypt; kernel boot.img; kernel_dtb tegra234-p3768-0000+p3767-0005-nv.dtb"    --bct_backup
saving flash command in flashcmd.txt

*** no-flash flag enabled. Exiting now... ***

User can run above saved command in factory environment without
providing pkc and sbk keys to flash a device

Example:

    $ cd bootloader
    $ sudo bash ./flashcmd.txt

Save initrd flashing command parameters to /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/initrdflashparam.txt
/tmp/tmp.BIz6KPBxAi /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
writing boot image config in bootimg.cfg
extracting kernel in zImage
extracting ramdisk in initrd.img
/tmp/tmp.BIz6KPBxAi/initrd /tmp/tmp.BIz6KPBxAi /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
74756 blocks
128307 blocks
/tmp/tmp.BIz6KPBxAi /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
flashimg0=boot0.img
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
Success
Cleaning up...
Finish generating flash package.
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/tools/kernel_flash/l4t_initrd_flash_internal.sh --network usb0 --usb-instance 1-5 --device-instance 0 --flash-only --external-device nvme0n1p1 -c "tools/kernel_flash/flash_l4t_t234_nvme.xml" --network usb0 jetson-orin-nano-devkit internal
**********************************************
*                                            *
*  Step 1: Build the flashing environment    *
*                                            *
**********************************************
Create flash environment 0
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/bootloader /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
Finish creating flash environment 0.
****************************************************
*                                                  *
*  Step 2: Boot the device with flash initrd image *
*                                                  *
****************************************************
/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra/temp_initrdflash/bootloader0 /home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
./tegraflash.py --bl uefi_jetson_with_dtb_sigheader.bin.encrypt --bct br_bct_BR.bct --securedev  --bldtb tegra234-p3768-0000+p3767-0005-nv.dtb --applet rcm_2_encrypt.rcm --applet_softfuse rcm_1_encrypt.rcm --cmd "rcmboot"  --cfg secureflash.xml --chip 0x23 --mb1_bct mb1_bct_MB1_sigheader.bct.encrypt --mem_bct mem_rcm_sigheader.bct.encrypt --mb1_cold_boot_bct mb1_cold_boot_bct_MB1_sigheader.bct.encrypt --mb1_bin mb1_t234_prod_aligned_sigheader.bin.encrypt --psc_bl1_bin psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --mem_bct_cold_boot mem_coldboot_sigheader.bct.encrypt  --bins "psc_fw pscfw_t234_prod_sigheader.bin.encrypt; mts_mce mce_flash_o10_cr_prod_sigheader.bin.encrypt; tsec_fw tsec_t234_sigheader.bin.encrypt; mb2_applet applet_t234_sigheader.bin.encrypt; mb2_bootloader mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt; xusb_fw xusb_t234_prod_sigheader.bin.encrypt; pva_fw nvpva_020_sigheader.fw.encrypt; dce_fw display-t234-dce_sigheader.bin.encrypt; nvdec nvdec_t234_prod_sigheader.fw.encrypt; bpmp_fw bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt; bpmp_fw_dtb tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt; rce_fw camera-rtcpu-t234-rce_sigheader.img.encrypt; ape_fw adsp-fw_sigheader.bin.encrypt; spe_fw spe_t234_sigheader.bin.encrypt; tos tos-optee_t234_sigheader.img.encrypt; eks eks_t234_sigheader.img.encrypt; kernel boot0.img; kernel_dtb tegra234-p3768-0000+p3767-0005-nv.dtb"    --bct_backup  --instance 1-5
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands


 Entering RCM boot

[   0.0440 ] mb1_t234_prod_aligned_sigheader.bin.encrypt filename is from --mb1_bin
[   0.0440 ] psc_bl1_t234_prod_aligned_sigheader.bin.encrypt filename is from --psc_bl1_bin
[   0.0440 ] rcm boot with presigned binaries
[   0.0446 ] tegrarcm_v2 --instance 1-5 --new_session --chip 0x23 0 --uid --download bct_br br_bct_BR.bct --download mb1 mb1_t234_prod_aligned_sigheader.bin.encrypt --download psc_bl1 psc_bl1_t234_prod_aligned_sigheader.bin.encrypt --download bct_mb1 mb1_bct_MB1_sigheader.bct.encrypt
[   0.0450 ] BR_CID: 0x80012344705DF1975C000000110300C0
[   0.0560 ] Sending bct_br
[   0.0660 ] Sending mb1
[   0.0666 ] Sending psc_bl1
[   0.0769 ] Sending bct_mb1
[   0.0827 ] Generating blob for T23x
[   0.0842 ] tegrahost_v2 --chip 0x23 0 --generateblob blob.xml blob.bin
[   0.0845 ] The number of images in blob is 19
[   0.0850 ] blobsize is 83881483
[   0.0851 ] Added binary blob_uefi_jetson_with_dtb_sigheader.bin.encrypt of size 3481664
[   0.1299 ] Added binary blob_pscfw_t234_prod_sigheader.bin.encrypt of size 310768
[   0.1310 ] Added binary blob_mce_flash_o10_cr_prod_sigheader.bin.encrypt of size 187120
[   0.1313 ] Added binary blob_tsec_t234_sigheader.bin.encrypt of size 176128
[   0.1317 ] Added binary blob_applet_t234_sigheader.bin.encrypt of size 279616
[   0.1320 ] Not supported type: mb2_applet
[   0.1321 ] Added binary blob_mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt of size 440576
[   0.1326 ] Added binary blob_xusb_t234_prod_sigheader.bin.encrypt of size 164864
[   0.1330 ] Added binary blob_nvpva_020_sigheader.fw.encrypt of size 2164640
[   0.1334 ] Added binary blob_display-t234-dce_sigheader.bin.encrypt of size 779040
[   0.1344 ] Added binary blob_nvdec_t234_prod_sigheader.fw.encrypt of size 294912
[   0.1347 ] Added binary blob_bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt of size 1040512
[   0.1354 ] Added binary blob_tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt of size 204672
[   0.1359 ] Added binary blob_camera-rtcpu-t234-rce_sigheader.img.encrypt of size 458096
[   0.1363 ] Added binary blob_adsp-fw_sigheader.bin.encrypt of size 415200
[   0.1366 ] Added binary blob_spe_t234_sigheader.bin.encrypt of size 270336
[   0.1370 ] Added binary blob_tos-optee_t234_sigheader.img.encrypt of size 1317904
[   0.1374 ] Added binary blob_eks_t234_sigheader.img.encrypt of size 9232
[   0.1377 ] Added binary blob_boot0.img of size 71643136
[   0.1863 ] Added binary blob_tegra234-p3768-0000+p3767-0005-nv.dtb of size 241963
[   0.2686 ] tegrarcm_v2 --instance 1-5 --chip 0x23 0 --pollbl --download bct_mem mem_rcm_sigheader.bct.encrypt --download blob blob.bin
[   0.2690 ] BL: version 1.4.0.1-t234-54845784-08e631ca last_boot_error: 0
[   0.3446 ] Sending bct_mem
[   0.3742 ] Sending blob
[   4.0518 ] RCM-boot started

/home/user/Dev/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_6.0_DP_Linux_DP_JETSON_ORIN_NANO_TARGETS/Linux_for_Tegra
***************************************
*                                     *
*  Step 3: Start the flashing process *
*                                     *
***************************************
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for target to boot-up...
Waiting for device to expose ssh ......RTNETLINK answers: File exists
RTNETLINK answers: File exists
Waiting for device to expose ssh ...Run command: flash on fc00:1:1:0::2
SSH ready
blockdev: cannot open /dev/mmcblk3boot0: No such file or directory
[ 0]: l4t_flash_from_kernel: Starting to create gpt for emmc
Active index file is /mnt/internal/flash.idx
Number of lines is 61
max_index=60
[ 1]: l4t_flash_from_kernel: Successfully create gpt for emmc
[ 1]: l4t_flash_from_kernel: Starting to create gpt for external device
Active index file is /mnt/external/flash.idx
Number of lines is 18
max_index=17
writing item=1, 9:0:primary_gpt, 512, 19968, gpt_primary_9_0.bin, 16896, fixed-<reserved>-0, c80e5c55376a4462aede17374c36956c05198dd3
Writing primary_gpt partition with gpt_primary_9_0.bin
Offset is not aligned to K Bytes, no optimization is applied
dd if=/mnt/external/gpt_primary_9_0.bin of=/dev/nvme0n1 bs=1 skip=0  seek=512 count=16896
16896+0 records in
16896+0 records out
16896 bytes (17 kB, 16 KiB) copied, 0.0309544 s, 546 kB/s
Writing primary_gpt partition done
Error: The backup GPT table is corrupt, but the primary appears OK, so that will be used.
Warning: Not all of the space available to /dev/nvme0n1 appears to be used, you can fix the GPT to use all of the space (an extra 880677552 blocks) or continue with the current setting?
Writing secondary_gpt partition with gpt_secondary_9_0.bin
Offset is not aligned to K Bytes, no optimization is applied
dd if=/mnt/external/gpt_secondary_9_0.bin of=/dev/nvme0n1 bs=1 skip=0  seek=61203267072 count=16896
16896+0 records in
16896+0 records out
16896 bytes (17 kB, 16 KiB) copied, 0.0297579 s, 568 kB/s
Writing secondary_gpt partition done
Fix/Ignore? Fix
Warning: Not all of the space available to /dev/nvme0n1 appears to be used, you can fix the GPT to use all of the space (an extra 880677552 blocks) or continue with the current setting?
Model: TEAM TM8FP6512G (nvme)
Disk /dev/nvme0n1: 512GB
Sector size (logical/physical): 512B/512B
Partition Table: gpt
Disk Flags:

Number  Start   End     Size    File system  Name                Flags
 2      20.5kB  134MB   134MB                A_kernel            msftdata
 3      134MB   135MB   786kB                A_kernel-dtb        msftdata
 4      135MB   168MB   33.2MB               A_reserved_on_user  msftdata
 5      168MB   302MB   134MB                B_kernel            msftdata
 6      302MB   303MB   786kB                B_kernel-dtb        msftdata
 7      303MB   336MB   33.2MB               B_reserved_on_user  msftdata
 8      336MB   420MB   83.9MB               recovery            msftdata
 9      420MB   421MB   524kB                recovery-dtb        msftdata
10      421MB   488MB   67.1MB               esp                 boot, esp
11      488MB   572MB   83.9MB               recovery_alt        msftdata
12      572MB   572MB   524kB                recovery-dtb_alt    msftdata
13      572MB   639MB   67.1MB               esp_alt             msftdata
14      639MB   1059MB  419MB                UDA                 msftdata
15      1059MB  1562MB  503MB                reserved            msftdata
 1      1562MB  61.2GB  59.6GB               APP                 msftdata

[ 2]: l4t_flash_from_kernel: Expanding last partition to fill the storage device
[ 2]: l4t_flash_from_kernel: Successfully create gpt for external device
Flash index file is /mnt/internal/flash.idx
Number of lines is 61
max_index=60
[ 2]: l4t_flash_from_kernel: Starting to flash to qspi
QSPI storage size: 67108864 bytes.
Erased 67108864 bytes from address 0x00000000 in flash
Flash index file is /mnt/internal/flash.idx
Number of lines is 61
max_index=60
Writing br_bct_BR.bct (parittion: BCT) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/br_bct_BR.bct
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:0
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00000000 in flash
[ 173]: l4t_flash_from_kernel: QSPI erase block size is 16384
[ 173]: l4t_flash_from_kernel: Writing 64 copies of /mnt/internal/br_bct_BR.bct
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:16384
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00004000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:32768
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00008000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:49152
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0000c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:65536
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00010000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:81920
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00014000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:98304
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00018000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:114688
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0001c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:131072
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00020000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:147456
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00024000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:163840
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00028000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:180224
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0002c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:196608
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00030000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:212992
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00034000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:229376
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00038000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:245760
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0003c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:262144
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00040000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:278528
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00044000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:294912
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00048000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:311296
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0004c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:327680
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00050000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:344064
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00054000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:360448
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00058000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:376832
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0005c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:393216
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00060000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:409600
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00064000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:425984
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00068000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:442368
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0006c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:458752
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00070000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:475136
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00074000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:491520
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00078000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:507904
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0007c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:524288
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00080000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:540672
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00084000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:557056
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00088000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:573440
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0008c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:589824
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00090000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:606208
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00094000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:622592
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x00098000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:638976
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x0009c000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:655360
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000a0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:671744
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000a4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:688128
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000a8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:704512
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000ac000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:720896
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000b0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:737280
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000b4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:753664
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000b8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:770048
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000bc000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:786432
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000c0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:802816
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000c4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:819200
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000c8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:835584
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000cc000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:851968
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000d0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:868352
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000d4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:884736
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000d8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:901120
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000dc000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:917504
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000e0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:933888
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000e4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:950272
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000e8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:966656
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000ec000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:983040
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000f0000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:999424
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000f4000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:1015808
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000f8000 in flash
Writing /mnt/internal/br_bct_BR.bct (8192 bytes) into  /dev/mtd0:1032192
Copied 8192 bytes from /mnt/internal/br_bct_BR.bct to address 0x000fc000 in flash
Writing mb1_t234_prod_aligned_sigheader.bin.encrypt (parittion: A_mb1) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt
Writing /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt (281328 bytes) into  /dev/mtd0:1048576
Copied 281328 bytes from /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt to address 0x00100000 in flash
Writing psc_bl1_t234_prod_aligned_sigheader.bin.encrypt (parittion: A_psc_bl1) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt
Writing /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt (123008 bytes) into  /dev/mtd0:1572864
Copied 123008 bytes from /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt to address 0x00180000 in flash
Writing mb1_cold_boot_bct_MB1_sigheader.bct.encrypt (parittion: A_MB1_BCT) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt
Writing /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt (17536 bytes) into  /dev/mtd0:1835008
Copied 17536 bytes from /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt to address 0x001c0000 in flash
Writing mem_coldboot_sigheader.bct.encrypt (parittion: A_MEM_BCT) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mem_coldboot_sigheader.bct.encrypt
Writing /mnt/internal/mem_coldboot_sigheader.bct.encrypt (243712 bytes) into  /dev/mtd0:1966080
Copied 243712 bytes from /mnt/internal/mem_coldboot_sigheader.bct.encrypt to address 0x001e0000 in flash
Writing tsec_t234_sigheader.bin.encrypt (parittion: A_tsec-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tsec_t234_sigheader.bin.encrypt
Writing /mnt/internal/tsec_t234_sigheader.bin.encrypt (176128 bytes) into  /dev/mtd0:2228224
Copied 176128 bytes from /mnt/internal/tsec_t234_sigheader.bin.encrypt to address 0x00220000 in flash
Writing nvdec_t234_prod_sigheader.fw.encrypt (parittion: A_nvdec) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt
Writing /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt (294912 bytes) into  /dev/mtd0:3276800
Copied 294912 bytes from /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt to address 0x00320000 in flash
Writing mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt (parittion: A_mb2) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt
Writing /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt (440576 bytes) into  /dev/mtd0:4325376
Copied 440576 bytes from /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt to address 0x00420000 in flash
Writing xusb_t234_prod_sigheader.bin.encrypt (parittion: A_xusb-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt (164864 bytes) into  /dev/mtd0:4849664
Copied 164864 bytes from /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt to address 0x004a0000 in flash
Writing bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt (parittion: A_bpmp-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
Writing /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt (1040512 bytes) into  /dev/mtd0:5111808
Copied 1040512 bytes from /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt to address 0x004e0000 in flash
Writing tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt (parittion: A_bpmp-fw-dtb) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
Writing /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt (204672 bytes) into  /dev/mtd0:6684672
Copied 204672 bytes from /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt to address 0x00660000 in flash
Writing pscfw_t234_prod_sigheader.bin.encrypt (parittion: A_psc-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt (310768 bytes) into  /dev/mtd0:10878976
Copied 310768 bytes from /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt to address 0x00a60000 in flash
Writing mce_flash_o10_cr_prod_sigheader.bin.encrypt (parittion: A_mts-mce) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt
Writing /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt (187120 bytes) into  /dev/mtd0:11665408
Copied 187120 bytes from /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt to address 0x00b20000 in flash
Writing sc7_t234_prod_sigheader.bin.encrypt (parittion: A_sc7) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt (185408 bytes) into  /dev/mtd0:12189696
Copied 185408 bytes from /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt to address 0x00ba0000 in flash
Writing psc_rf_t234_prod_sigheader.bin.encrypt (parittion: A_pscrf) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt (122464 bytes) into  /dev/mtd0:12386304
Copied 122464 bytes from /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt to address 0x00bd0000 in flash
Writing mb2rf_t234_sigheader.bin.encrypt (parittion: A_mb2rf) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb2rf_t234_sigheader.bin.encrypt
Writing /mnt/internal/mb2rf_t234_sigheader.bin.encrypt (122256 bytes) into  /dev/mtd0:12582912
Copied 122256 bytes from /mnt/internal/mb2rf_t234_sigheader.bin.encrypt to address 0x00c00000 in flash
Writing uefi_jetson_with_dtb_sigheader.bin.encrypt (parittion: A_cpu-bootloader) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt
Writing /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt (3485760 bytes) into  /dev/mtd0:12713984
Copied 3485760 bytes from /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt to address 0x00c20000 in flash
Writing tos-optee_t234_sigheader.img.encrypt (parittion: A_secure-os) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tos-optee_t234_sigheader.img.encrypt
Writing /mnt/internal/tos-optee_t234_sigheader.img.encrypt (1317904 bytes) into  /dev/mtd0:16384000
Copied 1317904 bytes from /mnt/internal/tos-optee_t234_sigheader.img.encrypt to address 0x00fa0000 in flash
[ 187]: l4t_flash_from_kernel: Warning: skip writing A_smm-fw partition as no image is specified
Writing eks_t234_sigheader.img.encrypt (parittion: A_eks) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/eks_t234_sigheader.img.encrypt
Writing /mnt/internal/eks_t234_sigheader.img.encrypt (9232 bytes) into  /dev/mtd0:22675456
Copied 9232 bytes from /mnt/internal/eks_t234_sigheader.img.encrypt to address 0x015a0000 in flash
Writing display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt (parittion: A_dce-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt
Writing /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt (781680 bytes) into  /dev/mtd0:22937600
Copied 781680 bytes from /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt to address 0x015e0000 in flash
Writing spe_t234_sigheader.bin.encrypt (parittion: A_spe-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/spe_t234_sigheader.bin.encrypt
Writing /mnt/internal/spe_t234_sigheader.bin.encrypt (270336 bytes) into  /dev/mtd0:28180480
Copied 270336 bytes from /mnt/internal/spe_t234_sigheader.bin.encrypt to address 0x01ae0000 in flash
Writing camera-rtcpu-t234-rce_sigheader.img.encrypt (parittion: A_rce-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt
Writing /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt (458096 bytes) into  /dev/mtd0:28770304
Copied 458096 bytes from /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt to address 0x01b70000 in flash
Writing adsp-fw_sigheader.bin.encrypt (parittion: A_adsp-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/adsp-fw_sigheader.bin.encrypt
Writing /mnt/internal/adsp-fw_sigheader.bin.encrypt (415200 bytes) into  /dev/mtd0:29818880
Copied 415200 bytes from /mnt/internal/adsp-fw_sigheader.bin.encrypt to address 0x01c70000 in flash
Writing nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt (parittion: A_pva-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt
Writing /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt (66512 bytes) into  /dev/mtd0:31916032
Copied 66512 bytes from /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt to address 0x01e70000 in flash
[ 191]: l4t_flash_from_kernel: Warning: skip writing A_reserved_on_boot partition as no image is specified
Writing mb1_t234_prod_aligned_sigheader.bin.encrypt (parittion: B_mb1) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt
Writing /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt (281328 bytes) into  /dev/mtd0:33292288
Copied 281328 bytes from /mnt/internal/mb1_t234_prod_aligned_sigheader.bin.encrypt to address 0x01fc0000 in flash
Writing psc_bl1_t234_prod_aligned_sigheader.bin.encrypt (parittion: B_psc_bl1) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt
Writing /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt (123008 bytes) into  /dev/mtd0:33816576
Copied 123008 bytes from /mnt/internal/psc_bl1_t234_prod_aligned_sigheader.bin.encrypt to address 0x02040000 in flash
Writing mb1_cold_boot_bct_MB1_sigheader.bct.encrypt (parittion: B_MB1_BCT) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt
Writing /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt (17536 bytes) into  /dev/mtd0:34078720
Copied 17536 bytes from /mnt/internal/mb1_cold_boot_bct_MB1_sigheader.bct.encrypt to address 0x02080000 in flash
Writing mem_coldboot_sigheader.bct.encrypt (parittion: B_MEM_BCT) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mem_coldboot_sigheader.bct.encrypt
Writing /mnt/internal/mem_coldboot_sigheader.bct.encrypt (243712 bytes) into  /dev/mtd0:34209792
Copied 243712 bytes from /mnt/internal/mem_coldboot_sigheader.bct.encrypt to address 0x020a0000 in flash
Writing tsec_t234_sigheader.bin.encrypt (parittion: B_tsec-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tsec_t234_sigheader.bin.encrypt
Writing /mnt/internal/tsec_t234_sigheader.bin.encrypt (176128 bytes) into  /dev/mtd0:34471936
Copied 176128 bytes from /mnt/internal/tsec_t234_sigheader.bin.encrypt to address 0x020e0000 in flash
Writing nvdec_t234_prod_sigheader.fw.encrypt (parittion: B_nvdec) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt
Writing /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt (294912 bytes) into  /dev/mtd0:35520512
Copied 294912 bytes from /mnt/internal/nvdec_t234_prod_sigheader.fw.encrypt to address 0x021e0000 in flash
Writing mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt (parittion: B_mb2) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt
Writing /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt (440576 bytes) into  /dev/mtd0:36569088
Copied 440576 bytes from /mnt/internal/mb2_t234_with_mb2_cold_boot_bct_MB2_sigheader.bin.encrypt to address 0x022e0000 in flash
Writing xusb_t234_prod_sigheader.bin.encrypt (parittion: B_xusb-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt (164864 bytes) into  /dev/mtd0:37093376
Copied 164864 bytes from /mnt/internal/xusb_t234_prod_sigheader.bin.encrypt to address 0x02360000 in flash
Writing bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt (parittion: B_bpmp-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt
Writing /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt (1040512 bytes) into  /dev/mtd0:37355520
Copied 1040512 bytes from /mnt/internal/bpmp_t234-TE950M-A1_prod_sigheader.bin.encrypt to address 0x023a0000 in flash
Writing tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt (parittion: B_bpmp-fw-dtb) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt
Writing /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt (204672 bytes) into  /dev/mtd0:38928384
Copied 204672 bytes from /mnt/internal/tegra234-bpmp-3767-0003-3509-a02_with_odm_sigheader.dtb.encrypt to address 0x02520000 in flash
Writing pscfw_t234_prod_sigheader.bin.encrypt (parittion: B_psc-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt (310768 bytes) into  /dev/mtd0:43122688
Copied 310768 bytes from /mnt/internal/pscfw_t234_prod_sigheader.bin.encrypt to address 0x02920000 in flash
Writing mce_flash_o10_cr_prod_sigheader.bin.encrypt (parittion: B_mts-mce) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt
Writing /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt (187120 bytes) into  /dev/mtd0:43909120
Copied 187120 bytes from /mnt/internal/mce_flash_o10_cr_prod_sigheader.bin.encrypt to address 0x029e0000 in flash
Writing sc7_t234_prod_sigheader.bin.encrypt (parittion: B_sc7) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt (185408 bytes) into  /dev/mtd0:44433408
Copied 185408 bytes from /mnt/internal/sc7_t234_prod_sigheader.bin.encrypt to address 0x02a60000 in flash
Writing psc_rf_t234_prod_sigheader.bin.encrypt (parittion: B_pscrf) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt
Writing /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt (122464 bytes) into  /dev/mtd0:44630016
Copied 122464 bytes from /mnt/internal/psc_rf_t234_prod_sigheader.bin.encrypt to address 0x02a90000 in flash
Writing mb2rf_t234_sigheader.bin.encrypt (parittion: B_mb2rf) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/mb2rf_t234_sigheader.bin.encrypt
Writing /mnt/internal/mb2rf_t234_sigheader.bin.encrypt (122256 bytes) into  /dev/mtd0:44826624
Copied 122256 bytes from /mnt/internal/mb2rf_t234_sigheader.bin.encrypt to address 0x02ac0000 in flash
Writing uefi_jetson_with_dtb_sigheader.bin.encrypt (parittion: B_cpu-bootloader) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt
Writing /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt (3485760 bytes) into  /dev/mtd0:44957696
Copied 3485760 bytes from /mnt/internal/uefi_jetson_with_dtb_sigheader.bin.encrypt to address 0x02ae0000 in flash
Writing tos-optee_t234_sigheader.img.encrypt (parittion: B_secure-os) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/tos-optee_t234_sigheader.img.encrypt
Writing /mnt/internal/tos-optee_t234_sigheader.img.encrypt (1317904 bytes) into  /dev/mtd0:48627712
Copied 1317904 bytes from /mnt/internal/tos-optee_t234_sigheader.img.encrypt to address 0x02e60000 in flash
[ 204]: l4t_flash_from_kernel: Warning: skip writing B_smm-fw partition as no image is specified
Writing eks_t234_sigheader.img.encrypt (parittion: B_eks) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/eks_t234_sigheader.img.encrypt
Writing /mnt/internal/eks_t234_sigheader.img.encrypt (9232 bytes) into  /dev/mtd0:54919168
Copied 9232 bytes from /mnt/internal/eks_t234_sigheader.img.encrypt to address 0x03460000 in flash
Writing display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt (parittion: B_dce-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt
Writing /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt (781680 bytes) into  /dev/mtd0:55181312
Copied 781680 bytes from /mnt/internal/display-t234-dce_with_tegra234-p3768-0000+p3767-0005-nv_with_odm_overlay.dtb_aligned_blob_w_bin_sigheader.bin.encrypt to address 0x034a0000 in flash
Writing spe_t234_sigheader.bin.encrypt (parittion: B_spe-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/spe_t234_sigheader.bin.encrypt
Writing /mnt/internal/spe_t234_sigheader.bin.encrypt (270336 bytes) into  /dev/mtd0:60424192
Copied 270336 bytes from /mnt/internal/spe_t234_sigheader.bin.encrypt to address 0x039a0000 in flash
Writing camera-rtcpu-t234-rce_sigheader.img.encrypt (parittion: B_rce-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt
Writing /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt (458096 bytes) into  /dev/mtd0:61014016
Copied 458096 bytes from /mnt/internal/camera-rtcpu-t234-rce_sigheader.img.encrypt to address 0x03a30000 in flash
Writing adsp-fw_sigheader.bin.encrypt (parittion: B_adsp-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/adsp-fw_sigheader.bin.encrypt
Writing /mnt/internal/adsp-fw_sigheader.bin.encrypt (415200 bytes) into  /dev/mtd0:62062592
Copied 415200 bytes from /mnt/internal/adsp-fw_sigheader.bin.encrypt to address 0x03b30000 in flash
Writing nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt (parittion: B_pva-fw) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt
Writing /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt (66512 bytes) into  /dev/mtd0:64159744
Copied 66512 bytes from /mnt/internal/nvpva_020_aligned_blob_w_bin_sigheader.fw.encrypt to address 0x03d30000 in flash
[ 208]: l4t_flash_from_kernel: Warning: skip writing B_reserved_on_boot partition as no image is specified
[ 208]: l4t_flash_from_kernel: Warning: skip writing uefi_variables partition as no image is specified
[ 208]: l4t_flash_from_kernel: Warning: skip writing uefi_ftw partition as no image is specified
[ 208]: l4t_flash_from_kernel: Warning: skip writing reserved partition as no image is specified
[ 208]: l4t_flash_from_kernel: Warning: skip writing worm partition as no image is specified
Writing bct_backup.img (parittion: BCT-boot-chain_backup) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/bct_backup.img
Writing /mnt/internal/bct_backup.img (32768 bytes) into  /dev/mtd0:66715648
Copied 32768 bytes from /mnt/internal/bct_backup.img to address 0x03fa0000 in flash
[ 208]: l4t_flash_from_kernel: Warning: skip writing reserved_partition partition as no image is specified
Writing gpt_backup_secondary_3_0.bin (parittion: secondary_gpt_backup) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/gpt_backup_secondary_3_0.bin
Writing /mnt/internal/gpt_backup_secondary_3_0.bin (16896 bytes) into  /dev/mtd0:66846720
Copied 16896 bytes from /mnt/internal/gpt_backup_secondary_3_0.bin to address 0x03fc0000 in flash
Writing qspi_bootblob_ver.txt (parittion: B_VER) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/qspi_bootblob_ver.txt
Writing /mnt/internal/qspi_bootblob_ver.txt (109 bytes) into  /dev/mtd0:66912256
Copied 109 bytes from /mnt/internal/qspi_bootblob_ver.txt to address 0x03fd0000 in flash
Writing qspi_bootblob_ver.txt (parittion: A_VER) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/qspi_bootblob_ver.txt
Writing /mnt/internal/qspi_bootblob_ver.txt (109 bytes) into  /dev/mtd0:66977792
Copied 109 bytes from /mnt/internal/qspi_bootblob_ver.txt to address 0x03fe0000 in flash
Writing gpt_secondary_3_0.bin (parittion: secondary_gpt) into /dev/mtd0
Sha1 checksum matched for /mnt/internal/gpt_secondary_3_0.bin
Writing /mnt/internal/gpt_secondary_3_0.bin (16896 bytes) into  /dev/mtd0:67091968
Copied 16896 bytes from /mnt/internal/gpt_secondary_3_0.bin to address 0x03ffbe00 in flash
[ 208]: l4t_flash_from_kernel: Successfully flash the qspi
[ 208]: l4t_flash_from_kernel: Starting to flash to emmc
Active index file is /mnt/internal/flash.idx
Number of lines is 61
max_index=60
[ 209]: l4t_flash_from_kernel: Successfully flash the emmc
[ 209]: l4t_flash_from_kernel: Starting to flash to external device
Active index file is /mnt/external/flash.idx
Number of lines is 18
max_index=17
writing item=0, 9:0:master_boot_record, 0, 512, mbr_9_0.bin, 512, fixed-<reserved>-0, 694898d1c345bdb31b377790ed7fc0b0db184bf7
writing item=1, 9:0:primary_gpt, 512, 19968, gpt_primary_9_0.bin, 16896, fixed-<reserved>-0, c80e5c55376a4462aede17374c36956c05198dd3
writing item=2, 9:0:A_kernel, 20480, 134217728, boot.img, 53805056, fixed-<reserved>-2, 5920cad6d6726b5bdbe85de9eb5318b2c3f4230f
Writing A_kernel partition with boot.img
Get size of partition through connection.
53805056 bytes from /mnt/external/boot.img to /dev/nvme0n1: 1KB block=52544 remainder=0
dd if=/mnt/external/boot.img of=/dev/nvme0n1 bs=1K skip=0  seek=20 count=52544
52544+0 records in
52544+0 records out
53805056 bytes (54 MB, 51 MiB) copied, 0.777633 s, 69.2 MB/s
Writing A_kernel partition done
writing item=3, 9:0:A_kernel-dtb, 134238208, 786432, kernel_tegra234-p3768-0000+p3767-0005-nv.dtb, 241963, fixed-<reserved>-3, b59c018c7701b798496dbc9cea0d7f3193f84382
Writing A_kernel-dtb partition with kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
Get size of partition through connection.
241963 bytes from /mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb to /dev/nvme0n1: 1KB block=236 remainder=299
dd if=/mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb of=/dev/nvme0n1 bs=1K skip=0  seek=131092 count=236
236+0 records in
236+0 records out
241664 bytes (242 kB, 236 KiB) copied, 0.00536157 s, 45.1 MB/s
dd if=/mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb of=/dev/nvme0n1 bs=1 skip=241664  seek=134479872 count=299
299+0 records in
299+0 records out
299 bytes copied, 0.00122198 s, 245 kB/s
Writing A_kernel-dtb partition done
writing item=4, 9:0:A_reserved_on_user, 135024640, 33161216, , , fixed-<reserved>-4,
[ 210]: l4t_flash_from_kernel: Warning: skip writing A_reserved_on_user partition as no image is specified
writing item=5, 9:0:B_kernel, 168185856, 134217728, boot.img, 53805056, fixed-<reserved>-5, 5920cad6d6726b5bdbe85de9eb5318b2c3f4230f
Writing B_kernel partition with boot.img
Get size of partition through connection.
53805056 bytes from /mnt/external/boot.img to /dev/nvme0n1: 1KB block=52544 remainder=0
dd if=/mnt/external/boot.img of=/dev/nvme0n1 bs=1K skip=0  seek=164244 count=52544
52544+0 records in
52544+0 records out
53805056 bytes (54 MB, 51 MiB) copied, 0.465824 s, 116 MB/s
Writing B_kernel partition done
writing item=6, 9:0:B_kernel-dtb, 302403584, 786432, kernel_tegra234-p3768-0000+p3767-0005-nv.dtb, 241963, fixed-<reserved>-6, b59c018c7701b798496dbc9cea0d7f3193f84382
Writing B_kernel-dtb partition with kernel_tegra234-p3768-0000+p3767-0005-nv.dtb
Get size of partition through connection.
241963 bytes from /mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb to /dev/nvme0n1: 1KB block=236 remainder=299
dd if=/mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb of=/dev/nvme0n1 bs=1K skip=0  seek=295316 count=236
236+0 records in
236+0 records out
241664 bytes (242 kB, 236 KiB) copied, 0.00251117 s, 96.2 MB/s
dd if=/mnt/external/kernel_tegra234-p3768-0000+p3767-0005-nv.dtb of=/dev/nvme0n1 bs=1 skip=241664  seek=302645248 count=299
299+0 records in
299+0 records out
299 bytes copied, 0.000836576 s, 357 kB/s
Writing B_kernel-dtb partition done
writing item=7, 9:0:B_reserved_on_user, 303190016, 33161216, , , fixed-<reserved>-7,
[ 210]: l4t_flash_from_kernel: Warning: skip writing B_reserved_on_user partition as no image is specified
writing item=8, 9:0:recovery, 336351232, 83886080, recovery.img, 58693632, fixed-<reserved>-8, 4134f850c4a3b4f03d14d2204001cd8f453ca9ed
Writing recovery partition with recovery.img
Get size of partition through connection.
58693632 bytes from /mnt/external/recovery.img to /dev/nvme0n1: 1KB block=57318 remainder=0
dd if=/mnt/external/recovery.img of=/dev/nvme0n1 bs=1K skip=0  seek=328468 count=57318
57318+0 records in
57318+0 records out
58693632 bytes (59 MB, 56 MiB) copied, 0.829574 s, 70.8 MB/s
Writing recovery partition done
writing item=9, 9:0:recovery-dtb, 420237312, 524288, tegra234-p3768-0000+p3767-0005-nv.dtb.rec, 241963, fixed-<reserved>-9, b59c018c7701b798496dbc9cea0d7f3193f84382
Writing recovery-dtb partition with tegra234-p3768-0000+p3767-0005-nv.dtb.rec
Get size of partition through connection.
241963 bytes from /mnt/external/tegra234-p3768-0000+p3767-0005-nv.dtb.rec to /dev/nvme0n1: 1KB block=236 remainder=299
dd if=/mnt/external/tegra234-p3768-0000+p3767-0005-nv.dtb.rec of=/dev/nvme0n1 bs=1K skip=0  seek=410388 count=236
236+0 records in
236+0 records out
241664 bytes (242 kB, 236 KiB) copied, 0.00551782 s, 43.8 MB/s
dd if=/mnt/external/tegra234-p3768-0000+p3767-0005-nv.dtb.rec of=/dev/nvme0n1 bs=1 skip=241664  seek=420478976 count=299
299+0 records in
299+0 records out
299 bytes copied, 0.00106365 s, 281 kB/s
Writing recovery-dtb partition done
writing item=10, 9:0:esp, 420761600, 67108864, esp.img, 67108864, fixed-<reserved>-10, 0a4be1b39d1fef6b36dd09e3bb55d29fcbdc120f
Writing esp partition with esp.img
Get size of partition through connection.
67108864 bytes from /mnt/external/esp.img to /dev/nvme0n1: 1KB block=65536 remainder=0
dd if=/mnt/external/esp.img of=/dev/nvme0n1 bs=1K skip=0  seek=410900 count=65536
65536+0 records in
65536+0 records out
67108864 bytes (67 MB, 64 MiB) copied, 0.848752 s, 79.1 MB/s
Writing esp partition done
writing item=11, 9:0:recovery_alt, 487870464, 83886080, , , fixed-<reserved>-11,
[ 212]: l4t_flash_from_kernel: Warning: skip writing recovery_alt partition as no image is specified
writing item=12, 9:0:recovery-dtb_alt, 571756544, 524288, , , fixed-<reserved>-12,
[ 212]: l4t_flash_from_kernel: Warning: skip writing recovery-dtb_alt partition as no image is specified
writing item=13, 9:0:esp_alt, 572280832, 67108864, , , fixed-<reserved>-13,
[ 212]: l4t_flash_from_kernel: Warning: skip writing esp_alt partition as no image is specified
writing item=14, 9:0:UDA, 639401984, 419430400, , , fixed-<reserved>-14,
[ 212]: l4t_flash_from_kernel: Warning: skip writing UDA partition as no image is specified
writing item=15, 9:0:reserved, 1058832384, 502792192, , , fixed-<reserved>-15,
[ 212]: l4t_flash_from_kernel: Warning: skip writing reserved partition as no image is specified
writing item=16, 9:0:APP, 1561624576, 59641638912, , , expand-<reserved>-1,
Formatting APP partition /dev/nvme0n1p1 ...
mke2fs 1.46.5 (30-Dec-2021)
Discarding device blocks: done
Creating filesystem with 124645641 4k blocks and 31162368 inodes
Filesystem UUID: c1edfcc8-c24b-4e38-86df-e3bb02d358fc
Superblock backups stored on blocks:
        32768, 98304, 163840, 229376, 294912, 819200, 884736, 1605632, 2654208,
        4096000, 7962624, 11239424, 20480000, 23887872, 71663616, 78675968,
        102400000

Allocating group tables: done
Writing inode tables: done
Creating journal (262144 blocks): done
Writing superblocks and filesystem accounting information: done

Formatting APP parition done
Formatting APP partition /dev/nvme0n1p1 ...
tar -xpf /mnt/external/system.img  --checkpoint=10000 --warning=no-timestamp --numeric-owner --xattrs --xattrs-include=*  -C  /tmp/ci-qRyX6aEv39
tar: Read checkpoint 10000
tar: Read checkpoint 20000
tar: Read checkpoint 30000
tar: Read checkpoint 40000
tar: Read checkpoint 50000
tar: Read checkpoint 60000
tar: Read checkpoint 70000
tar: Read checkpoint 80000
tar: Read checkpoint 90000
tar: Read checkpoint 100000
tar: Read checkpoint 110000
tar: Read checkpoint 120000
tar: Read checkpoint 130000
tar: Read checkpoint 140000
tar: Read checkpoint 150000
tar: Read checkpoint 160000
tar: Read checkpoint 170000
tar: Read checkpoint 180000
tar: Read checkpoint 190000
tar: Read checkpoint 200000
tar: Read checkpoint 210000
tar: Read checkpoint 220000
tar: Read checkpoint 230000
tar: Read checkpoint 240000
tar: Read checkpoint 250000
tar: Read checkpoint 260000
tar: Read checkpoint 270000
tar: Read checkpoint 280000
tar: Read checkpoint 290000
tar: Read checkpoint 300000
tar: Read checkpoint 310000
tar: Read checkpoint 320000
tar: Read checkpoint 330000
tar: Read checkpoint 340000
tar: Read checkpoint 350000
tar: Read checkpoint 360000
tar: Read checkpoint 370000
tar: Read checkpoint 380000
tar: Read checkpoint 390000
tar: Read checkpoint 400000
tar: Read checkpoint 410000
tar: Read checkpoint 420000
tar: Read checkpoint 430000
tar: Read checkpoint 440000
tar: Read checkpoint 450000
tar: Read checkpoint 460000
tar: Read checkpoint 470000
tar: Read checkpoint 480000
tar: Read checkpoint 490000
tar: Read checkpoint 500000
tar: Read checkpoint 510000
tar: Read checkpoint 520000
tar: Read checkpoint 530000
tar: Read checkpoint 540000
tar: Read checkpoint 550000
tar: Read checkpoint 560000
tar: Read checkpoint 570000
tar: Read checkpoint 580000
tar: Read checkpoint 590000
tar: Read checkpoint 600000
tar: Read checkpoint 610000
tar: Read checkpoint 620000
tar: Read checkpoint 630000
tar: Read checkpoint 640000
tar: Read checkpoint 650000
writing item=17, 9:0:secondary_gpt, 61203267072, 16896, gpt_secondary_9_0.bin, 16896, fixed-<reserved>-0, 3a530cdc46077d8211c7fb76c559b3bfa7763dd6
[ 306]: l4t_flash_from_kernel: Successfully flash the external device
[ 306]: l4t_flash_from_kernel: Flashing success
[ 306]: l4t_flash_from_kernel: The device size indicated in the partition layout xml is smaller than the actual size. This utility will try to fix the GPT.
Flash is successful
Reboot device
Cleaning up...
Log is saved to Linux_for_Tegra/initrdlog/flash_1-5_0_20240308-021536.log
```
