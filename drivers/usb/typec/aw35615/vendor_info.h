/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef VENDOR_INFO_H
#define VENDOR_INFO_H
#include "vif_macros.h"

/* Leave any fields that do not apply as their defaults */
/*
 * All fields marked as unimplemented are supported by device, but are not
 * part of the example code. Features can be implemented on request.
 */

/*
 * Change the number of PDOs enabled to the correct value. If you enable PPS
 * PDOs make sure that firmware is built with PPS enabled flag.
 */

#define $VIF_Specification "Revision 1.00, Version 1.0"
#define $VIF_Producer "USB-IF Vendor Info File Generator, Version 0.0.0.1"
#define $Vendor_Name "AWINIC"
#define $Model_Part_Number "aw35615"
#define $Product_Revision "0.2.5"
#define $TID "0"
#define VIF_Product_Type "0"
#define $Port_Label "1"


/* 0: Consumer, 1: Consumer/Provider, 2: Provider/Consumer, 3: Provider, 4: DRP */
#define PD_Port_Type 4
/* 0: Src, 1: Snk, 2: DRP */
#define Type_C_State_Machine 2

/* VIF Product */
#define Connector_Type 2              /* 0: Type-A, 1: Type-B, 2: Type-C */
#define USB_PD_Support YES
#define Port_Battery_Powered NO
#define BC_1_2_Support 0

/* General PD Settings Tab */
#define PD_Specification_Revision 2         /* Revision 3.0 */
#define SOP_Capable YES                     /* Always YES */
#define SOP_P_Capable YES
#define SOP_PP_Capable NO
#define SOP_P_Debug_Capable NO              /* Not Currently Implemented */
#define SOP_PP_Debug_Capable NO             /* Not Currently Implemented */
#define USB_Comms_Capable YES
#define DR_Swap_To_DFP_Supported YES
#define DR_Swap_To_UFP_Supported YES
#define Unconstrained_Power YES
#define VCONN_Swap_To_On_Supported NO
#define VCONN_Swap_To_Off_Supported NO
#define Responds_To_Discov_SOP_UFP YES
#define Responds_To_Discov_SOP_DFP YES
#define Attempts_Discov_SOP YES

/* USB Type-C Tab */
#define Type_C_Implements_Try_SRC NO        /* only one shall be enabled by */
#define Type_C_Implements_Try_SNK YES        /* the port at the same time, try src or snk*/
#define Rp_Value 2                          /* 0: Def 1: 1.5A 2: 3A Advertised*/
#define Type_C_Supports_Audio_Accessory YES
#define Type_C_Is_VCONN_Powered_Accessory NO
#define Type_C_Is_Debug_Target_SRC YES
#define Type_C_Is_Debug_Target_SNK YES
#define Type_C_Can_Act_As_Host NO           /* Not Controlled by this driver */
#define Type_C_Host_Speed 0                 /* Not Controlled by this driver */
#define Type_C_Can_Act_As_Device NO         /* Not Controlled by this driver */
#define Type_C_Device_Speed 0               /* Not Controlled by this driver */
#define Type_C_Is_Alt_Mode_Controller NO    /* Not Controlled by this driver */
#define Type_C_Is_Alt_Mode_Device NO        /* Not Controlled by this driver */
#define Type_C_Power_Source 0               /* Not Controlled by this driver */
#define Type_C_BC_1_2_Support 0             /* Not Controlled by this driver */
#define Type_C_Battery_Powered NO           /* Not Controlled by this driver */
#define Type_C_Port_On_Hub NO               /* Not Controlled by this driver */
#define Captive_Cable NO
#define Type_C_Sources_VCONN NO

/* Source Tab */
#define PD_Power_as_Source 5000
#define USB_Suspend_May_Be_Cleared YES

#define Sends_Pings NO                      /* Not Currently Implemented */

#define NUMBER_OF_SRC_PDOS_ENABLED  1

#define Src_PDO_Supply_Type1 0              //; 0: Fixed
#define Src_PDO_Peak_Current1 0             //; 0: 100% IOC
#define Src_PDO_Voltage1 100                //; 5V
#define Src_PDO_Max_Current1 100            //; 1.0A
#define Src_PDO_Min_Voltage1 0              //; 0V
#define Src_PDO_Max_Voltage1 0              //; 0V
#define Src_PDO_Max_Power1 0                //; 0W

#define Src_PDO_Supply_Type2 0              //; 0: Fixed
#define Src_PDO_Peak_Current2 0             //; 0: 100% IOC
#define Src_PDO_Voltage2 0                  //; 9V
#define Src_PDO_Max_Current2 0              //; 3A
#define Src_PDO_Min_Voltage2 0              //; 0V
#define Src_PDO_Max_Voltage2 0              //; 0V
#define Src_PDO_Max_Power2 0                //; 0W

#define Src_PDO_Supply_Type3 0              //; 0: Fixed
#define Src_PDO_Peak_Current3 0             //; 0: 100% IOC
#define Src_PDO_Voltage3 0                  //; 12V
#define Src_PDO_Max_Current3 0              //; 3A
#define Src_PDO_Min_Voltage3 0              //; 0V
#define Src_PDO_Max_Voltage3 0              //; 0V
#define Src_PDO_Max_Power3 0                //; 0W

#define Src_PDO_Supply_Type4 0              //; 0: Fixed
#define Src_PDO_Peak_Current4 0             //; 0: 100% IOC
#define Src_PDO_Voltage4 0                  //; 15V
#define Src_PDO_Max_Current4 0              //; 3A
#define Src_PDO_Min_Voltage4 0              //; 0V
#define Src_PDO_Max_Voltage4 0              //; 0V
#define Src_PDO_Max_Power4 0                //; 0W

#define Src_PDO_Supply_Type5 0              //; 0: Fixed
#define Src_PDO_Peak_Current5 0             //; 0: 100% IOC
#define Src_PDO_Voltage5 0                  //; 20V
#define Src_PDO_Max_Current5 0              //; 3A
#define Src_PDO_Min_Voltage5 0              //; 0V
#define Src_PDO_Max_Voltage5 0              //; 0V
#define Src_PDO_Max_Power5 0                //; 0W

#define Src_PDO_Supply_Type6 3              //; 3: Augmented
#define Src_PDO_Peak_Current6 0             //; 0: 100% IOC
#define Src_PDO_Voltage6 0
#define Src_PDO_Max_Current6 0              //; 3A
#define Src_PDO_Min_Voltage6 0              //; 3.3V
#define Src_PDO_Max_Voltage6 0              //; 21V
#define Src_PDO_Max_Power6 0                //; 0W

#define Src_PDO_Supply_Type7 0              //; 0: Fixed
#define Src_PDO_Peak_Current7 0             //; 0: 100% IOC
#define Src_PDO_Voltage7 0
#define Src_PDO_Max_Current7 0
#define Src_PDO_Min_Voltage7 0              //; 0V
#define Src_PDO_Max_Voltage7 0              //; 0V
#define Src_PDO_Max_Power7 0                //; 0W

/* Sink Tab */
#define PD_Power_as_Sink 10000                //; 10W
#define No_USB_Suspend_May_Be_Set NO
#define GiveBack_May_Be_Set NO
#define Higher_Capability_Set NO

#define NUMBER_OF_SNK_PDOS_ENABLED  1

#define Snk_PDO_Supply_Type1 0              //; 0: Fixed
#define Snk_PDO_Voltage1 100                //; 5V
#define Snk_PDO_Op_Current1 200             //; 100mA
#define Snk_PDO_Min_Voltage1 0              //; 0 V
#define Snk_PDO_Max_Voltage1 0              //; 0 V
#define Snk_PDO_Op_Power1 0                 //; 0 W

#define Snk_PDO_Supply_Type2 0              //; 0: Fixed
#define Snk_PDO_Voltage2 0                  //; 9V
#define Snk_PDO_Op_Current2 60               //; 50mA
#define Snk_PDO_Min_Voltage2 53              //; 100mV
#define Snk_PDO_Max_Voltage2 53              //; 100mV
#define Snk_PDO_Op_Power2 0                 //; 0 W

#define Snk_PDO_Supply_Type3 0              //; 0: Fixed
#define Snk_PDO_Voltage3 0
#define Snk_PDO_Op_Current3 0
#define Snk_PDO_Min_Voltage3 0              //; 0 V
#define Snk_PDO_Max_Voltage3 0              //; 0 V
#define Snk_PDO_Op_Power3 0                 //; 0 W

#define Snk_PDO_Supply_Type4 0              //; 0: Fixed
#define Snk_PDO_Voltage4 0
#define Snk_PDO_Op_Current4 0
#define Snk_PDO_Min_Voltage4 0              //; 0 V
#define Snk_PDO_Max_Voltage4 0              //; 0 V
#define Snk_PDO_Op_Power4 0                 //; 0 W

#define Snk_PDO_Supply_Type5 0              //; 0: Fixed
#define Snk_PDO_Voltage5 0
#define Snk_PDO_Op_Current5 0
#define Snk_PDO_Min_Voltage5 0              //; 0 V
#define Snk_PDO_Max_Voltage5 0              //; 0 V
#define Snk_PDO_Op_Power5 0                 //; 0 W

#define Snk_PDO_Supply_Type6 0              //; 0: Fixed
#define Snk_PDO_Voltage6 0
#define Snk_PDO_Op_Current6 0
#define Snk_PDO_Min_Voltage6 0              //; 0 V
#define Snk_PDO_Max_Voltage6 0              //; 0 V
#define Snk_PDO_Op_Power6 0                 //; 0 W

#define Snk_PDO_Supply_Type7 0              //; 0: Fixed
#define Snk_PDO_Voltage7 0
#define Snk_PDO_Op_Current7 0
#define Snk_PDO_Min_Voltage7 0              //; 0 V
#define Snk_PDO_Max_Voltage7 0              //; 0 V
#define Snk_PDO_Op_Power7 0                 //; 0 W

/* Dual Role Tab */
#define Accepts_PR_Swap_As_Src YES
#define Accepts_PR_Swap_As_Snk YES
#define Requests_PR_Swap_As_Src NO
#define Requests_PR_Swap_As_Snk NO

/* SOP Discovery - Part One Tab */
#define Structured_VDM_Version_SOP 1        //; 1: V2.0
#define XID_SOP 0
#define Data_Capable_as_USB_Host_SOP NO
#define Data_Capable_as_USB_Device_SOP NO
#define Product_Type_UFP_SOP 0                  //; 0: Undefined
#define Product_Type_DFP_SOP 0                  //; 0: Undefined
#define Modal_Operation_Supported_SOP NO
#define Connector_Type 2
#define USB_VID_SOP 0x344f
#define PID_SOP 0x0000
#define bcdDevice_SOP 0x0000

/* SOP Discovery - Part Two Tab */
#define SVID_fixed_SOP YES
#define Num_SVIDs_min_SOP 1
#define Num_SVIDs_max_SOP 1                 /* Currently Implements Up To 1 */

#define SVID1_SOP 0x344f
#define SVID1_modes_fixed_SOP YES
#define SVID1_num_modes_min_SOP 1
#define SVID1_num_modes_max_SOP 1           /* Currently Implements Up To 1 */

#define SVID1_mode1_enter_SOP YES

/* AMA Tab */
#define AMA_HW_Vers 0x0
#define AMA_FW_Vers 0x0
#define AMA_VCONN_reqd NO
#define AMA_VCONN_power 0                   //; 0: 1W
#define AMA_VBUS_reqd NO
#define AMA_Superspeed_Support 0            //; 0: USB 2.0 only

/* Project related defines */
#define Attempt_DR_Swap_to_Ufp_As_Src        NO
#define Attempt_DR_Swap_to_Dfp_As_Sink       NO
#define Attempt_Vconn_Swap_to_Off_As_Src     NO
#define Attempt_Vconn_Swap_to_On_As_Sink     NO
#define Attempts_DiscvId_SOP_P_First         NO
/* Display port related configuration */

/* Automatically enter display port mode if found, Auto VDM required */
#define DisplayPort_Auto_Mode_Entry      NO
#define DisplayPort_Enabled              NO
/* Prefer to be sink when DFP_D & UFP_D capable device attached */
#define DisplayPort_Preferred_Snk        NO
#define DisplayPort_UFP_Capable          YES  /* DP Sink */
#define DisplayPort_DFP_Capable          YES  /* DP Source */
#define DisplayPort_Signaling            YES  /* YES == 0x1, DP Standard signaling */
#define DisplayPort_Receptacle           1    /* DP is presented in Type-C 0 plug, 1 - Receptacle */
#define DisplayPort_USBr2p0Signal_Req    NO   /* USB r2.0 isgnaling required */
/* Only Pin Assignment C,D,E supported */
#define DisplayPort_DFP_Pin_Mask         (DP_PIN_ASSIGN_C | DP_PIN_ASSIGN_D | DP_PIN_ASSIGN_E)
#define DisplayPort_UFP_Pin_Mask         DP_PIN_ASSIGN_NS
#define DisplayPort_UFP_PinAssign_Start  'C'
#endif /* VENDOR_INFO_H */
