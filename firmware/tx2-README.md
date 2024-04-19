## For NVIDIA Jetson TX2 with original carrier board

Set recovery mode of the TX2 board by holding down the "RST" button and pressing the "PWR" button. Connect a micro-USB cable with a Linux PC. On the PC run `lsusb`, you should see an item with `NVIDIA` in its name.

Flashing command:
```sh
gunzip -c ./your_custom_image.img.gz > ./nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/system.img
# or:
# cp ./your_custom_img_or_sparse_img.img ./nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/system.img
cd ./nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/
# this command only flashes the APP partition. It assumes the bootloader partitions are properly set up.
sudo ./flash.sh -r -k APP jetson-tx2 mmcblk0p1
```

Reference:
- https://forums.developer.nvidia.com/t/backup-jetson-tx2/144365/4

## Troubleshooting
- Stuck at `tegrarcm_v2 --isapplet`

    You need to do an SDK flash with JetPack 4.6.4, before doing this partition flash.

## Output
```log
###############################################################################
# L4T BSP Information:
# R32 , REVISION: 7.4
###############################################################################
# Target Board Information:
# Name: jetson-tx2, Board Family: t186ref, SoC: Tegra 186,
# OpMode: production, Boot Authentication: NS,
# Disk encryption: disabled ,
###############################################################################
./tegraflash.py --chip 0x18 --applet "/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/mb1_recovery_prod.bin" --skipuid --cmd "dump eeprom boardinfo cvm.bin"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0046 ] Generating RCM messages
[   0.0066 ] tegrarcm_v2 --listrcm rcm_list.xml --chip 0x18 0 --download rcm /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/mb1_recovery_prod.bin 0 0
[   0.0073 ] RCM 0 is saved as rcm_0.rcm
[   0.2845 ] RCM 1 is saved as rcm_1.rcm
[   0.2845 ] List of rcm files are saved in rcm_list.xml
[   0.2845 ]
[   0.2845 ] Signing RCM messages
[   0.2869 ] tegrasign_v3.py --key None --list rcm_list.xml --pubkeyhash pub_key.key
[   0.2870 ] Assuming zero filled SBK key
[   0.4438 ] Copying signature to RCM mesages
[   0.4466 ] tegrarcm_v2 --chip 0x18 0 --updatesig rcm_list_signed.xml
[   0.4568 ]
[   0.4568 ] Boot Rom communication
[   0.4590 ] tegrarcm_v2 --chip 0x18 0 --rcm rcm_list_signed.xml --skipuid
[   0.4600 ] RCM version 0X180001
[   0.4860 ] Boot Rom communication completed
[   1.4938 ]
[   2.4998 ] tegrarcm_v2 --isapplet
[   2.5027 ] Applet version 01.00.0000
[   2.5802 ]
[   2.5821 ] Retrieving EEPROM data
[   2.5822 ] tegrarcm_v2 --oem platformdetails eeprom cvm /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   2.5827 ] Applet version 01.00.0000
[   2.6729 ] Saved platform info in /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/cvm.bin
[   3.7716 ]
Board ID(3310) version(B02) sku(1000) revision(B.0)
copying bctfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/P3310_A00_8GB_lpddr4_A02_l4t.cfg)... done.
copying misc_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-misc-si-l4t.cfg)... done.
copying pinmux_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-pinmux-quill-p3310-1000-c03.cfg)... done.
copying pmic_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-pmic-quill-p3310-1000-c04.cfg)... done.
copying pmc_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-pad-quill-p3310-1000-c03.cfg)... done.
copying prod_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-prod-quill-p3310-1000-c03.cfg)... done.
copying scr_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/minimal_scr.cfg)... done.
copying scr_cold_boot_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/mobile_scr.cfg)... done.
copying bootrom_config(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/tegra186-mb1-bct-bootrom-quill-p3310-1000-c03.cfg)... done.
copying dev_params(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/BCT/emmc.cfg)... done.
Existing bootloader(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/nvtboot_cpu.bin) reused.
copying initrd(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/l4t_initrd.img)... done.
Making Boot image... done.
/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py --chip 0x18 --key  --cmd sign boot.img kernel
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0003 ] Generating signature
[   0.0023 ] tegrasign_v3.py --getmode mode.txt --key
[   0.0024 ] Assuming zero filled SBK key : not reading
[   0.0024 ] sign_type   : 0
[   0.0024 ] header_magic: 414e4452
[   0.0195 ] tegrahost_v2 --chip 0x18 --align 1_boot.img
[   0.0203 ]
[   0.0222 ] tegrahost_v2 --chip 0x18 0 --appendsigheader 1_boot.img zerosbk
[   0.0235 ]
[   0.0264 ] tegrasign_v3.py --key  --list 1_boot_sigheader.img_list.xml --pubkeyhash pub_key.key
[   0.0265 ] Assuming zero filled SBK key : not reading
[   0.0360 ] tegrahost_v2 --chip 0x18 0 --updatesigheader 1_boot_sigheader.img.encrypt 1_boot_sigheader.img.hash zerosbk
[   0.0370 ]
[   0.0373 ] Signed file: /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/temp_user_dir/boot_sigheader.img.encrypt
l4t_sign_image.sh: Generate header for boot_sigheader.img.encrypt
l4t_sign_image.sh: chip 0x18: Don't need to do anything
l4t_sign_image.sh: Generate 16-byte-size-aligned base file for boot_sigheader.img.encrypt
l4t_sign_image.sh: the signed file is /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/temp_user_dir/boot_sigheader.img.encrypt
done.
Existing sosfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/mb1_recovery_prod.bin) reused.
copying tegraboot(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/nvtboot.bin)... done.
Existing cpu_bootloader(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/nvtboot_cpu.bin) reused.
Existing mb2blfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/nvtboot_recovery.bin) reused.
Existing mtspreboot(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/preboot_d15_prod_cr.bin) reused.
Existing mts(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/mce_mts_d15_prod_cr.bin) reused.
Existing mb1file(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/mb1_prod.bin) reused.
Existing bpffile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/bpmp.bin) reused.
copying bpfdtbfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2.dtb)... done.
Existing scefile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/camera-rtcpu-sce.img) reused.
Existing spefile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/spe.bin) reused.
copying wb0boot(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/warmboot.bin)... done.
Existing tosfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/tos-trusty.img) reused.
Existing eksfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/eks.img) reused.
./flash.sh: line 2661: [: : integer expression expected
copying dtbfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/kernel/dtb/tegra186-quill-p3310-1000-c03-00-base.dtb)... done.
Copying nv_boot_control.conf to rootfs
/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py --chip 0x18 --key  --cmd sign kernel_tegra186-quill-p3310-1000-c03-00-base.dtb kernel_dtb
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0003 ] Generating signature
[   0.0024 ] tegrasign_v3.py --getmode mode.txt --key
[   0.0024 ] Assuming zero filled SBK key : not reading
[   0.0024 ] sign_type   : 808464433
[   0.0024 ] header_magic: d00dfeed
[   0.0042 ] tegrahost_v2 --chip 0x18 --align 1_kernel_tegra186-quill-p3310-1000-c03-00-base.dtb
[   0.0050 ]
[   0.0067 ] tegrahost_v2 --chip 0x18 0 --appendsigheader 1_kernel_tegra186-quill-p3310-1000-c03-00-base.dtb zerosbk
[   0.0076 ]
[   0.0099 ] tegrasign_v3.py --key  --list 1_kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb_list.xml --pubkeyhash pub_key.key
[   0.0100 ] Assuming zero filled SBK key : not reading
[   0.0167 ] tegrahost_v2 --chip 0x18 0 --updatesigheader 1_kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt 1_kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.hash zerosbk
[   0.0175 ]
[   0.0178 ] Signed file: /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/temp_user_dir/kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt
l4t_sign_image.sh: Generate header for kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt
l4t_sign_image.sh: chip 0x18: Don't need to do anything
l4t_sign_image.sh: Generate 16-byte-size-aligned base file for kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt
l4t_sign_image.sh: the signed file is /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/temp_user_dir/kernel_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt
done.
Reusing existing system.img...
done.
Existing tbcfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/cboot.bin) reused.
copying tbcdtbfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/kernel/dtb/tegra186-quill-p3310-1000-c03-00-base.dtb)... done.
copying cfgfile(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/t186ref/cfg/flash_l4t_t186.xml) to flash.xml... done.
Existing flasher(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/nvtboot_recovery_cpu.bin) reused.
Existing flashapp(/home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/tegraflash.py) reused.
*** Updating [APP] with system.img ***
./flash.sh: line 3316: [: : integer expression expected
./tegraflash.py --bl nvtboot_recovery_cpu.bin  --chip 0x18 --applet mb1_recovery_prod.bin --sdram_config P3310_A00_8GB_lpddr4_A02_l4t.cfg --misc_config tegra186-mb1-bct-misc-si-l4t.cfg --pinmux_config tegra186-mb1-bct-pinmux-quill-p3310-1000-c03.cfg --pmic_config tegra186-mb1-bct-pmic-quill-p3310-1000-c04.cfg --pmc_config tegra186-mb1-bct-pad-quill-p3310-1000-c03.cfg --prod_config tegra186-mb1-bct-prod-quill-p3310-1000-c03.cfg --scr_config minimal_scr.cfg --scr_cold_boot_config mobile_scr.cfg --br_cmd_config tegra186-mb1-bct-bootrom-quill-p3310-1000-c03.cfg --dev_params emmc.cfg  --cfg  flash.xml --bins "mb2_bootloader nvtboot_recovery.bin; mts_preboot preboot_d15_prod_cr.bin; mts_bootpack mce_mts_d15_prod_cr.bin; bpmp_fw bpmp.bin; bpmp_fw_dtb tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2.dtb; tlk tos-trusty.img; eks eks.img; bootloader_dtb tegra186-quill-p3310-1000-c03-00-base.dtb"  --odmdata 0x1090000  --cmd "write APP system.img; reboot"
Welcome to Tegra Flash
version 1.0.0
Type ? or help for help and q or quit to exit
Use ! to execute system commands

[   0.0046 ] Generating RCM messages
[   0.0064 ] tegrarcm_v2 --listrcm rcm_list.xml --chip 0x18 0 --download rcm mb1_recovery_prod.bin 0 0
[   0.0071 ] RCM 0 is saved as rcm_0.rcm
[   0.0073 ] RCM 1 is saved as rcm_1.rcm
[   0.0073 ] List of rcm files are saved in rcm_list.xml
[   0.0073 ]
[   0.0073 ] Signing RCM messages
[   0.0092 ] tegrasign_v3.py --key None --list rcm_list.xml --pubkeyhash pub_key.key
[   0.0092 ] Assuming zero filled SBK key
[   0.0129 ] Copying signature to RCM mesages
[   0.0148 ] tegrarcm_v2 --chip 0x18 0 --updatesig rcm_list_signed.xml
[   0.0155 ]
[   0.0155 ] Boot Rom communication
[   0.0173 ] tegrarcm_v2 --chip 0x18 0 --rcm rcm_list_signed.xml
[   0.0190 ] BootRom is not running
[   5.0900 ]
[   6.0960 ] tegrarcm_v2 --isapplet
[   6.0989 ] Applet version 01.00.0000
[   6.1733 ]
[   6.1755 ] tegrasign_v3.py --getmode mode.txt --key None
[   6.1755 ] Assuming zero filled SBK key
[   6.1755 ] Parsing partition layout
[   7.5159 ] tegraparser_v2 --pt flash.xml.tmp
[   7.5209 ]
[   7.5210 ] Creating list of images to be signed
[   7.5253 ] tegrahost_v2 --chip 0x18 0 --partitionlayout flash.xml.bin --list images_list.xml zerosbk
[   8.4651 ]
[   8.4653 ] Generating signatures
[   8.4680 ] tegrasign_v3.py --key None --list images_list.xml --pubkeyhash pub_key.key
[   8.4682 ] Assuming zero filled SBK key
[  10.3296 ] Reading BCT from device for further operations
[  10.3296 ] Generating blob
[  10.3345 ] tegrahost_v2 --chip 0x18 --align blob_nvtboot_recovery_cpu.bin
[  10.3379 ]
[  10.3422 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_nvtboot_recovery_cpu.bin zerosbk
[  10.3466 ]
[  10.3516 ] tegrasign_v3.py --key None --list blob_nvtboot_recovery_cpu_sigheader.bin_list.xml --pubkeyhash pub_key.key
[  10.3518 ] Assuming zero filled SBK key
[  10.3769 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_nvtboot_recovery_cpu_sigheader.bin.encrypt blob_nvtboot_recovery_cpu_sigheader.bin.hash zerosbk
[  10.4535 ]
[  10.4584 ] tegrahost_v2 --chip 0x18 --align blob_nvtboot_recovery.bin
[  10.4592 ]
[  10.4611 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_nvtboot_recovery.bin zerosbk
[  10.4622 ]
[  10.4642 ] tegrasign_v3.py --key None --list blob_nvtboot_recovery_sigheader.bin_list.xml --pubkeyhash pub_key.key
[  10.4642 ] Assuming zero filled SBK key
[  10.4685 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_nvtboot_recovery_sigheader.bin.encrypt blob_nvtboot_recovery_sigheader.bin.hash zerosbk
[  10.5379 ]
[  10.5429 ] tegrahost_v2 --chip 0x18 --align blob_preboot_d15_prod_cr.bin
[  10.5460 ]
[  10.5504 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_preboot_d15_prod_cr.bin zerosbk
[  10.5540 ]
[  10.5593 ] tegrasign_v3.py --key None --list blob_preboot_d15_prod_cr_sigheader.bin_list.xml --pubkeyhash pub_key.key
[  10.5596 ] Assuming zero filled SBK key
[  10.5731 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_preboot_d15_prod_cr_sigheader.bin.encrypt blob_preboot_d15_prod_cr_sigheader.bin.hash zerosbk
[  10.6996 ]
[  10.7042 ] tegrahost_v2 --chip 0x18 --align blob_mce_mts_d15_prod_cr.bin
[  10.7064 ]
[  10.7094 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_mce_mts_d15_prod_cr.bin zerosbk
[  10.7158 ]
[  10.7179 ] tegrasign_v3.py --key None --list blob_mce_mts_d15_prod_cr_sigheader.bin_list.xml --pubkeyhash pub_key.key
[  10.7180 ] Assuming zero filled SBK key
[  10.7423 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_mce_mts_d15_prod_cr_sigheader.bin.encrypt blob_mce_mts_d15_prod_cr_sigheader.bin.hash zerosbk
[  10.7940 ]
[  10.7988 ] tegrahost_v2 --chip 0x18 --align blob_bpmp.bin
[  10.8021 ]
[  10.8062 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_bpmp.bin zerosbk
[  10.8119 ]
[  10.8169 ] tegrasign_v3.py --key None --list blob_bpmp_sigheader.bin_list.xml --pubkeyhash pub_key.key
[  10.8170 ] Assuming zero filled SBK key
[  10.8571 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_bpmp_sigheader.bin.encrypt blob_bpmp_sigheader.bin.hash zerosbk
[  10.9638 ]
[  10.9688 ] tegrahost_v2 --chip 0x18 --align blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2.dtb
[  10.9695 ]
[  10.9714 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2.dtb zerosbk
[  10.9729 ]
[  10.9750 ] tegrasign_v3.py --key None --list blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2_sigheader.dtb_list.xml --pubkeyhash pub_key.key
[  10.9750 ] Assuming zero filled SBK key
[  10.9844 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2_sigheader.dtb.encrypt blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2_sigheader.dtb.hash zerosbk
[  11.0701 ]
[  11.0752 ] tegrahost_v2 --chip 0x18 --align blob_tos-trusty.img
[  11.0785 ]
[  11.0829 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_tos-trusty.img zerosbk
[  11.0882 ]
[  11.0935 ] tegrasign_v3.py --key None --list blob_tos-trusty_sigheader.img_list.xml --pubkeyhash pub_key.key
[  11.0937 ] Assuming zero filled SBK key
[  11.1245 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_tos-trusty_sigheader.img.encrypt blob_tos-trusty_sigheader.img.hash zerosbk
[  11.2046 ]
[  11.2096 ] tegrahost_v2 --chip 0x18 --align blob_eks.img
[  11.2125 ]
[  11.2170 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_eks.img zerosbk
[  11.2205 ]
[  11.2259 ] tegrasign_v3.py --key None --list blob_eks_sigheader.img_list.xml --pubkeyhash pub_key.key
[  11.2261 ] Assuming zero filled SBK key
[  11.2395 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_eks_sigheader.img.encrypt blob_eks_sigheader.img.hash zerosbk
[  11.3674 ]
[  11.3721 ] tegrahost_v2 --chip 0x18 --align blob_tegra186-quill-p3310-1000-c03-00-base.dtb
[  11.3727 ]
[  11.3745 ] tegrahost_v2 --chip 0x18 0 --appendsigheader blob_tegra186-quill-p3310-1000-c03-00-base.dtb zerosbk
[  11.3754 ]
[  11.3775 ] tegrasign_v3.py --key None --list blob_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb_list.xml --pubkeyhash pub_key.key
[  11.3775 ] Assuming zero filled SBK key
[  11.3846 ] tegrahost_v2 --chip 0x18 0 --updatesigheader blob_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt blob_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.hash zerosbk
[  11.5256 ]
[  11.5289 ] tegrahost_v2 --chip 0x18 --generateblob blob.xml blob.bin
[  11.5301 ] number of images in blob are 9
[  11.5304 ] blobsize is 4541416
[  11.5305 ] Added binary blob_nvtboot_recovery_cpu_sigheader.bin.encrypt of size 225392
[  11.5329 ] Added binary blob_nvtboot_recovery_sigheader.bin.encrypt of size 122272
[  11.5333 ] Added binary blob_preboot_d15_prod_cr_sigheader.bin.encrypt of size 58256
[  11.5336 ] Added binary blob_mce_mts_d15_prod_cr_sigheader.bin.encrypt of size 2197920
[  11.5342 ] Added binary blob_bpmp_sigheader.bin.encrypt of size 534816
[  11.5345 ] Added binary blob_tegra186-a02-bpmp-quill-p3310-1000-c04-00-te770d-ucm2_sigheader.dtb.encrypt of size 617920
[  11.5349 ] Added binary blob_tos-trusty_sigheader.img.encrypt of size 407360
[  11.5350 ] Added binary blob_eks_sigheader.img.encrypt of size 1440
[  11.5351 ] Added binary blob_tegra186-quill-p3310-1000-c03-00-base_sigheader.dtb.encrypt of size 375888
[  11.5365 ]
[  11.5366 ] Sending bootloader and pre-requisite binaries
[  11.5384 ] tegrarcm_v2 --download blob blob.bin
[  11.5389 ] Applet version 01.00.0000
[  11.6119 ] Sending blob
[  11.6119 ] [................................................] 100%
[  12.1951 ]
[  12.1972 ] tegrarcm_v2 --boot recovery
[  12.1977 ] Applet version 01.00.0000
[  12.2744 ]
[  13.2801 ] tegrarcm_v2 --isapplet
[  13.5716 ]
[  17.6674 ] tegradevflash_v2 --iscpubl
[  17.6705 ] Bootloader version 01.00.0000
[  17.7005 ] Bootloader version 01.00.0000
[  17.7018 ]
[  17.7018 ] Writing partition
[  17.7037 ] tegradevflash_v2 --write APP /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/system.img
[  17.7042 ] Bootloader version 01.00.0000
[  17.7483 ] Writing partition APP with /home/user/nvidia-sdkmanager-ws/nvidia/nvidia_sdk/JetPack_4.6.4_Linux_JETSON_TX2_TARGETS/Linux_for_Tegra/bootloader/system.img
[  17.7488 ] [................................................] 100%
[ 1550.5561 ]
[ 1550.5566 ] Coldbooting the device
[ 1550.5586 ] tegradevflash_v2 --reboot coldboot
[ 1550.5591 ] Bootloader version 01.00.0000
[ 1550.5891 ]
*** The [APP] has been updated successfully. ***
```
