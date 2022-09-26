/******************************************************************************************************************************************************************
*                                                 本驱动基于高通msm-4.4开发                                                                                       *
*******************************************************************************************************************************************************************/
cps_wls_4038.dtsi ：IIC设备树注册文件，需要注册到对应的dtsi文件里.(示例：kernel\msm-4.4\arch\arm\boot\dts\qcom\msm8998-qrd-skuk.dtsi)
cps_wls_charger.c ：驱动代码.c(示例：放入目录kernel\msm-4.4\drivers\power\supply\qcom下)
cps_wls_charger.h ：驱动代码.h(示例：放入目录kernel\msm-4.4\drivers\power\supply\qcom下)
Kconfig ：添加驱动代码配置说明
Makefile ：添加编译驱动代码的makefile编译配置

在线升级时需要将bootloader.hex和firmware.hex可以通过adb push 到路径/data/misc/cps/文件夹下，路径可在驱动内修改。
App_Hex和Bootloader_Hex下面的三个文件为hex的三种不同格式，根据系统需要选用。
