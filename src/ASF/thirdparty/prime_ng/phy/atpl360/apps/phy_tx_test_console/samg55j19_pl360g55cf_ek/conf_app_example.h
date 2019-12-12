/**
 * \file
 *
 * \brief CONF_APP_EXAMPLE : Example configuration for PLC Application
 *
 * Copyright (c) 2019 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#ifndef CONF_EXAMPLE_H
#define CONF_EXAMPLE_H

/* PHY Data message start address. */
#define ADDR_APP_PHY_MESSAGE_DATA (IFLASH_ADDR + IFLASH_SIZE - (2 * IFLASH_LOCK_REGION_SIZE))

/* If coupling 11 is chosen */
/* #define CONF_APP_ENABLE_C11_CFG */

/* Configure Coupling Parameters for PL360 device ************************************************************************************************/
/* Number of TX levels */
#define NUM_TX_LEVELS                        8

/* Coupling configuration for PRIME CHANNEL 1 */
/* Recommend to use ATPLCOUP007v2/ATPLCOUP008v2 coupling board */
#define MAX_RMS_HI_VALUES_CHN_1              {1725, 1522, 1349, 1202, 1071, 957, 855, 764}
#define MAX_RMS_VLO_VALUES_CHN_1             {4874, 4427, 3986, 3555, 3157, 2795, 2470, 2184}
#define TH_HI_VALUES_CHN_1                   {0, 0, 0, 0, 0, 0, 0, 0, 1467, 1292, 1145, 1019, 910, 811, 725, 648}
#define TH_VLO_VALUES_CHN_1                  {0, 0, 0, 0, 0, 0, 0, 0, 8479, 7515, 6665, 5874, 5192, 4576, 4030, 3557}
#define PREDIST_COEF_HI_VALUES_CHN_1         {0x756E, 0x7396, 0x730A, 0x72EB, 0x72B2, 0x7433, 0x755E, 0x75D7, 0x769E, 0x76A4, 0x77C3, 0x7851, 0x7864, 0x78A0, \
					      0x78BA, 0x7918, 0x79B6, 0x79E9, 0x7ACC, 0x7B06, 0x7B30, 0x7B27, 0x7C1E, 0x7B96, 0x7A76, 0x7B12, 0x7AFD, 0x7C40, \
					      0x7C5E, 0x7B48, 0x7B8A, 0x7C64, 0x7C42, 0x7BCD, 0x7AFD, 0x7A5F, 0x7A03, 0x7A9D, 0x7A1A, 0x7A4A, 0x79FC, 0x7984, \
					      0x7A0D, 0x79CC, 0x792E, 0x780D, 0x7676, 0x75E4, 0x747A, 0x7251, 0x707E, 0x6E96, 0x6E30, 0x6D44, 0x6DBD, 0x6C9A, \
					      0x6C3C, 0x6CF8, 0x6CA4, 0x6CDF, 0x6C59, 0x6B2C, 0x6CB9, 0x6C1F, 0x6B6D, 0x6BF5, 0x6AF0, 0x6A55, 0x6955, 0x674F, \
					      0x6841, 0x685D, 0x670F, 0x6904, 0x6967, 0x6B01, 0x6C31, 0x6C2A, 0x6D82, 0x6F58, 0x6E62, 0x6F18, 0x6EE7, 0x7069, \
					      0x717B, 0x7120, 0x7170, 0x72FB, 0x7491, 0x75B3, 0x75A2, 0x7664, 0x784A, 0x7A52, 0x7B51, 0x7D5A, 0x7FFF}
#define PREDIST_COEF_VLO_VALUES_CHN_1        {0x7FFF, 0x7F2B, 0x7E38, 0x7CD3, 0x7B38, 0x7972, 0x77D6, 0x7654, 0x74AE, 0x7288, 0x70C0, 0x6E9A, 0x6D24, 0x6B80, \
					      0x6A2F, 0x6852, 0x674E, 0x65DA, 0x652E, 0x637E, 0x6292, 0x6142, 0x60CC, 0x5FF8, 0x5F6D, 0x5EC2, 0x5E6F, 0x5E55, \
					      0x5E43, 0x5E02, 0x5E5B, 0x5EB3, 0x5F4A, 0x5FD7, 0x604C, 0x60FC, 0x61F3, 0x6297, 0x63A9, 0x643D, 0x654A, 0x6634, \
					      0x675C, 0x6824, 0x6910, 0x69A4, 0x6A73, 0x6B6F, 0x6C15, 0x6CCD, 0x6D64, 0x6E4B, 0x6ED3, 0x6F44, 0x6F85, 0x70A1, \
					      0x70AF, 0x71B2, 0x7149, 0x71F3, 0x7203, 0x7279, 0x71FB, 0x72B4, 0x7281, 0x72A4, 0x7262, 0x72BD, 0x7295, 0x72CC, \
					      0x729E, 0x7288, 0x7244, 0x7279, 0x726C, 0x7230, 0x71B9, 0x70D8, 0x7045, 0x7052, 0x6F8D, 0x6F3D, 0x6EB0, 0x6E6A, \
					      0x6E76, 0x6E1C, 0x6D7A, 0x6D84, 0x6D50, 0x6D45, 0x6CF2, 0x6CA9, 0x6C92, 0x6CBA, 0x6C69, 0x6C27, 0x6C02}
#define IFFT_GAIN_HI_INI_CHN_1               81
#define IFFT_GAIN_VLO_INI_CHN_1              256
#define IFFT_GAIN_HI_MIN_CHN_1               40
#define IFFT_GAIN_VLO_MIN_CHN_1              128
#define IFFT_GAIN_HI_MAX_CHN_1               128
#define IFFT_GAIN_VLO_MAX_CHN_1              281

#define DACC_CFG_TABLE_CHN_1                {0x00000000, 0x00002120, 0x0000073F, 0x00003F3F, 0x00000333, 0x00000000, 0x546000FF, 0x1A1A1A1A, \
					     0x00002020, 0x00000044, 0x0FD20005, 0x00000355, 0x0F000000, 0x001020F0, 0x00000355, 0x0F000000, 0x001020FF}

/* Coupling configuration for PRIME CHANNEL 2 - 8 */
/* Recommend to use ATPLCOUP006v2 coupling board */
#define MAX_RMS_HI_VALUES_CHN_2              {779, 690, 613, 544, 484, 431, 383, 341}
#define MAX_RMS_VLO_VALUES_CHN_2             {3595, 3208, 2840, 2493, 2177, 1903, 1666, 1465}
#define TH_HI_VALUES_CHN_2                   {0, 0, 0, 0, 0, 0, 0, 0, 661, 586, 520, 461, 410, 365, 325, 290}
#define TH_VLO_VALUES_CHN_2                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_3              {1500, 1332, 1184, 1053, 937, 834, 742, 661}
#define MAX_RMS_VLO_VALUES_CHN_3             {3754, 3358, 2973, 2611, 2283, 2000, 1753, 1544}
#define TH_HI_VALUES_CHN_3                   {0, 0, 0, 0, 0, 0, 0, 0, 1269, 1127, 1002, 892, 794, 708, 631, 562}
#define TH_VLO_VALUES_CHN_3                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_4              {1625, 1447, 1290, 1150, 1024, 913, 815, 726}
#define MAX_RMS_VLO_VALUES_CHN_4             {4012, 3591, 3179, 2795, 2444, 2136, 1874, 1648}
#define TH_HI_VALUES_CHN_4                   {0, 0, 0, 0, 0, 0, 0, 0, 1374, 1224, 1089, 971, 867, 773, 690, 615}
#define TH_VLO_VALUES_CHN_4                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_5              {1653, 1471, 1310, 1167, 1040, 926, 825, 736}
#define MAX_RMS_VLO_VALUES_CHN_5             {4071, 3655, 3252, 2875, 2529, 2227, 1966, 1742}
#define TH_HI_VALUES_CHN_5                   {0, 0, 0, 0, 0, 0, 0, 0, 1401, 1247, 1110, 987, 881, 784, 699, 623}
#define TH_VLO_VALUES_CHN_5                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_6              {1674, 1489, 1326, 1183, 1055, 940, 838, 747}
#define MAX_RMS_VLO_VALUES_CHN_6             {4090, 3684, 3293, 2927, 2591, 2297, 2041, 1819}
#define TH_HI_VALUES_CHN_6                   {0, 0, 0, 0, 0, 0, 0, 0, 1428, 1271, 1132, 1008, 899, 801, 714, 636}
#define TH_VLO_VALUES_CHN_6                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_7              {1564, 1393, 1242, 1108, 988, 882, 787, 703}
#define MAX_RMS_VLO_VALUES_CHN_7             {4278, 3860, 3455, 3079, 2735, 2433, 2166, 1935}
#define TH_HI_VALUES_CHN_7                   {0, 0, 0, 0, 0, 0, 0, 0, 1328, 1184, 1055, 940, 840, 749, 668, 595}
#define TH_VLO_VALUES_CHN_7                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define MAX_RMS_HI_VALUES_CHN_8              {1268, 1132, 1010, 902, 805, 719, 642, 573}
#define MAX_RMS_VLO_VALUES_CHN_8             {4630, 4158, 3709, 3297, 2923, 2595, 2311, 2064}
#define TH_HI_VALUES_CHN_8                   {0, 0, 0, 0, 0, 0, 0, 0, 1073, 958, 855, 764, 682, 610, 544, 485}
#define TH_VLO_VALUES_CHN_8                  {0, 0, 0, 0, 0, 0, 0, 0, 100000, 100000, 100000, 100000, 100000, 100000, 100000, 100000}

#define PREDIST_COEF_HI_VALUES_CHN_2_8       {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}
#define PREDIST_COEF_VLO_VALUES_CHN_2_8      {0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, \
					      0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF, 0x7FFF}
#define IFFT_GAIN_HI_INI_CHN_2_8             120
#define IFFT_GAIN_VLO_INI_CHN_2_8            256
#define IFFT_GAIN_HI_MIN_CHN_2_8             60
#define IFFT_GAIN_VLO_MIN_CHN_2_8            128
#define IFFT_GAIN_HI_MAX_CHN_2_8             256
#define IFFT_GAIN_VLO_MAX_CHN_2_8            256

#if defined(CONF_APP_ENABLE_C11_CFG) || BOARD == PL360G55CF_EK
/* Recommended configuration to use for DACC in coupling 11 reference design */
#define DACC_CFG_TABLE_CHN_2_8               {0x00000000, 0x21202120, 0x073F073F, 0x3F3F3F3F, 0x00000FFF, 0x00000000, 0xA8C000FF, 0x1B1B1B1B, \
					      0x10101010, 0x00001111, 0x04380006, 0x000003AA, 0xF0000000, 0x001020FF, 0x000003AA, 0xF0000000, 0x001020FF}
#else
#define DACC_CFG_TABLE_CHN_2_8               {0x00000000, 0x10102120, 0x033F073F, 0x3F3F3F3F, 0x00000FFF, 0x00000000, 0xA8C000FF, 0x1B1B1B1B, \
					      0x10101010, 0x00001111, 0x04380006, 0x000003AA, 0xF0000000, 0x001020F0, 0x00000355, 0x0F000000, 0x001020FF}
#endif
/*************************************************************************************************************************************************/

#endif /* CONF_EXAMPLE_H */
