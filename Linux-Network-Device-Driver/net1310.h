/**
  * File Name: "net1310.h"
  * Description:
  * - This program is Parameters for cc1310_netdev Module
  *
  * Programmed by Ji-Hyeok, Yang
  * Copyright (c) 2018 Advanced Networking Technology Lab. 
  *           (YU-ANTL), Yeungnam University, All right reserved.
  *
  * ========================================================
  * Version Control (Explain updates in detail)
  * ========================================================
  * Updated By Date (YYYY/MM/DD) Version Remarks
  * V.0.0  2018/4/30  Ji-Hyeok, Yang  Create Header File
  *
  * ========================================================
  **/

#ifndef NET1310_H
#define NET1310_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioctl.h>

#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include<linux/workqueue.h>
/**********************************************************************************
                        Include File for Network Device Driver
**********************************************************************************/

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>

/**********************************************************************************
                       Device Informations & Common Defines
**********************************************************************************/
#define DEVICE_NAME                     "net1310"

/**********************************************************************************
                           SPI Frame Sync byte(2Bytes)
**********************************************************************************/
#define HCI_SOF_BYTE1                   0xAA //SOF = Start of Frame
#define HCI_SOF_BYTE2                   0x7E

/**********************************************************************************
                          SPI Frame Control Field(1Byte)
     1bytes = Frame Type(3Bits) + Command Type(2bits) + Command Sub-Type(3bits)
**********************************************************************************/
// Frame Type(3bits)
#define HCI_FRAME_FC_FT_REQ_GET         0b000 //0
#define HCI_FRAME_FC_FT_RES_GET         0b001 //1
#define HCI_FRAME_FC_FT_REQ_SET         0b010 //2
#define HCI_FRAME_FC_FT_RES_SET         0b011 //3
#define HCI_FRAME_FC_FT_NACK            0b100 //4
#define HCI_FRAME_FC_FT_DUMMY           0b111 //7

// Command Type(2bits)
#define HCI_FRAME_FC_CT_SYS             0b00
#define HCI_FRAME_FC_CT_MAC_PARAM       0b01
#define HCI_FRAME_FC_CT_MAC_TRX         0b10

// Command Sub Type(3bits)
// SYS
#define HCI_FRAME_FC_CST_SYS_RESET      0b000

// MAC
#define HCI_FRAME_FC_CST_MAC_EXE        0b000
#define HCI_FRAME_FC_CST_MAC_RF         0b001
#define HCI_FRAME_FC_CST_MAC_MODE       0b010


// MAC TRX
#define HCI_FRAME_FC_CST_MAC_TX         0b000
#define HCI_FRAME_FC_CST_MAC_RX         0b001
#define HCI_FRAME_FC_CST_MAC_DUMMY_TX   0b010


/**********************************************************************************
							ACK Set Response Type
**********************************************************************************/
#define HCI_FRAME_ACK_OK                0
#define HCI_FRAME_ACK_BUF_FULL          1

/**********************************************************************************
                               Linux Kernel SPI  
**********************************************************************************/
#define HCI_FRAME_SOF_SIZE              2
#define HCI_FRAME_FC_SIZE               1
#define HCI_FRAME_LENGTH_SIZE           1
#define HCI_FRAME_PAYLOAD_SIZE			255
#define HCI_FRAME_SIZE                  (HCI_FRAME_SOF_SIZE + HCI_FRAME_FC_SIZE + HCI_FRAME_LENGTH_SIZE + HCI_FRAME_PAYLOAD_SIZE)
#define SPI_DATA_FRAME              0
#define SPI_CMD_FRAME               1

/**********************************************************************************
									MAC SDU
**********************************************************************************/

#define MAC_SDU_HEADER_SIZE				2
#define MAC_SDU_FRAME_SIZE				242
#define MAC_SDU_PAYLOAD_SIZE			(MAC_SDU_FRAME_SIZE - MAC_SDU_HEADER_SIZE)


/**********************************************************************************
                                Struct Type Def
**********************************************************************************/

/**** cc1310_spidev internal structs ****/
struct cc1310_priv {
	struct spi_device *spi;
	int irq;

	/* Thread Handler */
	struct task_struct *pThrHdlr;
	struct task_struct *pThrHdlr_rx;
	/* Internal Data/CMD Queue */
	struct list_head tx_data_queue;
	struct list_head rx_data_queue;
    struct list_head rx_cmd_queue;


	/* Mutex for tx/rx Queue */
	struct mutex tx_data_queue_lock;
	struct mutex rx_data_queue_lock;
    struct mutex rx_cmd_queue_lock;
};

struct spi_data {
	unsigned char frame_type : 3;
	unsigned char cmd_type : 2;
	unsigned char cmd_sub_type : 3;
	unsigned char len;

	char  buf[HCI_FRAME_PAYLOAD_SIZE];

	struct list_head list;
};

struct ack_response_management {
	bool condition;
	char response;
};

/**** enum type defines ****/
typedef enum {
	RF_IDLE = 0,
	RF_BUSY,
}operation_state;

typedef enum {
	TYPE_AP = 0,
	TYPE_AP_RE,            // RE: Range Extender
	TYPE_NODE,
	TYPE_NODE_RE,          // RE: Range Extender
} station_info_type_table;

typedef enum {
	RF_MODE_TRANS = 0,
	RF_MODE_MAC,
} common_station_info_rf_mode_table;

typedef enum {
	RF_PARAM_PHY_FREQ_917MHZ = 0,
	RF_PARAM_PHY_FREQ_918MHZ,
	RF_PARAM_PHY_FREQ_919MHZ,
	RF_PARAM_PHY_FREQ_920MHZ,
	RF_PARAM_PHY_FREQ_921MHZ,
	RF_PARAM_PHY_FREQ_922MHZ,
	RF_PARAM_PHY_FREQ_923MHZ
} rf_params_phy_freq_table;

typedef enum {
	RF_PARAM_PHY_DR_625BPS = 0,
	RF_PARAM_PHY_DR_2_5KBPS,
	RF_PARAM_PHY_DR_50KBPS,
	RF_PARAM_PHY_DR_100KBPS,
	RF_PARAM_PHY_DR_200KBPS,
	RF_PARAM_PHY_DR_300KBPS,
	RF_PARAM_PHY_DR_400KBPS,
	RF_PARAM_PHY_DR_500KBPS,
    RF_PARAM_PHY_DR_4MBPS
} rf_params_phy_dr_table;


typedef enum {
	RF_PARAM_PHY_TXPWR_N10DBM = 0,
	RF_PARAM_PHY_TXPWR_0DBM,
	RF_PARAM_PHY_TXPWR_1DBM,
	RF_PARAM_PHY_TXPWR_2DBM,
	RF_PARAM_PHY_TXPWR_3DBM,
	RF_PARAM_PHY_TXPWR_4DBM,
	RF_PARAM_PHY_TXPWR_5DBM,
	RF_PARAM_PHY_TXPWR_6DBM,
	RF_PARAM_PHY_TXPWR_7DBM,
	RF_PARAM_PHY_TXPWR_8DBM,
	RF_PARAM_PHY_TXPWR_9DBM,
	RF_PARAM_PHY_TXPWR_10DBM,
	RF_PARAM_PHY_TXPWR_11DBM,
	RF_PARAM_PHY_TXPWR_12DBM,
	RF_PARAM_PHY_TXPWR_12_5DBM,
	RF_PARAM_PHY_TXPWR_14DBM
} rf_params_phy_txpwr_table;

/**** spi communication structs with cc1310 ****/
struct HCI_Frame_FC {
	unsigned char frame_type : 3;
	unsigned char cmd_type : 2;
	unsigned char cmd_sub_type : 3;
};

struct HCI_Frame {
	char sof[HCI_FRAME_SOF_SIZE];
	struct HCI_Frame_FC fc;
	unsigned char length;
	char payload[HCI_FRAME_PAYLOAD_SIZE];
};

struct MAC_SDU
{
	unsigned char mflag;
	unsigned char size;
	unsigned char payloadR[MAC_SDU_PAYLOAD_SIZE];
};
struct rf_phy_params
{
	unsigned char cent_freq;
	unsigned char data_rate;
	unsigned char tx_power;
};

struct operation_state {
	char op_state;
};

struct rssi_report {
	char rssi;
};

struct rf_info {
	unsigned char type;       // AP / Node
};

struct mac_params {
    unsigned int super_frame_size;
    unsigned int csma_period_size;
};

/**** cc1310 status struct ****/
struct cc1310_module_state {
	bool driver_nops;
	bool end_thread;
	bool int_fired;
	
	struct operation_state state;			// idle, busy
	struct rf_phy_params rf_phy_state;		// cent_freq, data_rate, tx_power
	struct rf_info rf_info;					// type(AP, Node), rf_mode(trans, mac)
};

struct my_work
{
	struct work_struct work;
	unsigned long long data;
};



/**********************************************************************************
                                Function Proto Type
**********************************************************************************/



static int net1310_open(struct net_device *pDev);
static int net1310_close(struct net_device *pDev);
static netdev_tx_t net1310_xmit(struct sk_buff *pSkb, struct net_device *pDev);
static int net1310_ioctl(struct net_device *pDev, struct ifreq *pIfr, int cmd);
static int net1310_mac_addr(struct net_device *pDev, void *p);
static void net1310_destruct(struct net_device *pDev);
static void net1310_setup(struct net_device *pDev);

#endif /* CC1310_SPIDEV_H */

