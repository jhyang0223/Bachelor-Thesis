/**
  * File Name : "net1310"
  * Description:
  * - This program is spi network device driver for cc1310
  *
  * Programmed by Ji-Hyeok Yang
  * Copyright (c) 2018 Advanced Networking Technology Lab.
  *           (YU-ANTL), Yeungnam University, All right reserved.
  *
  * ========================================================
  * Version Control  (Explain updates in detail)
  * Updated By Date  (YYYY/MM/DD) Version Remarks
  * v0.0    2018/04/25  Ji-Hyeok, Yang  Create Project.
  *
  *
  *
  * ========================================================
  *
  **/
#include"net1310.h"
//#define DEBUG FALSE
#define STATION 
/************************************************************
                    Grobal Variable
************************************************************/
dev_t cc1310_dev_t;
struct cc1310_priv *priv;
struct cc1310_module_state module_state, tmp_module_state;
struct ack_response_management tx_ack_management;
struct ack_response_management rx_ack_management;
struct ack_response_management cmd_ack_management;
static struct net_device *pDevCleanup = NULL;
DECLARE_WAIT_QUEUE_HEAD(tx_wq_head);
DECLARE_WAIT_QUEUE_HEAD(rx_wq_head);
DECLARE_WAIT_QUEUE_HEAD(rx_cmd_wq_head);
static struct workqueue_struct *txWorkQ;
/**
*  Function Name: init_module_state
*  Input arguments (condition):
	void
*  Processing in function (in pseudo code style):
*    1) initialize module state
*  Function Return:
*   1) void
*/
static void init_module_state(void)
{	module_state.int_fired = false;
	module_state.end_thread = false;
	module_state.driver_nops = false;
	module_state.state.op_state = RF_IDLE;
	/* will fix */
	module_state.rf_info.type = TYPE_AP;
	memset(&(module_state.rf_phy_state), -1, sizeof(struct rf_phy_params));
	memset(&tmp_module_state, 0, sizeof(struct cc1310_module_state));
}/**
*  Function Name: reset_module_state
*  Input arguments (condition):
void
*  Processing in function (in pseudo code style):
*    1) reset module state
*  Function Return:
*   1) void
*/
static void reset_module_state(void)
{	module_state.int_fired = true;
	module_state.end_thread = true;
	module_state.driver_nops = true;
	module_state.state.op_state = RF_IDLE;
	memset(&(module_state.rf_phy_state), -1, sizeof(struct rf_phy_params));
	memset(&tmp_module_state, 0, sizeof(struct cc1310_module_state));
}/**
*  Function Name: queue_lock
*  Input arguments (condition):
	struct list_head *target : Queue Header
*  Processing in function (in pseudo code style):
*    1) Queue_Lock By Queue
*  Function Return:
*   1) void
*/
static void queue_lock(struct list_head *target)
{	if (target == &priv->tx_data_queue)
		mutex_lock(&priv->tx_data_queue_lock);
	else if (target == &priv->rx_data_queue)
		mutex_lock(&priv->rx_data_queue_lock);
	else if (target == &priv->rx_cmd_queue)
		mutex_lock(&priv->rx_cmd_queue_lock);
}/**
*  Function Name: queue_unlock
*  Input arguments (condition):
struct list_head *target : Queue Header
*  Processing in function (in pseudo code style):
*    1) Queue_unLock By Queue
*  Function Return:
*   1) void
*/
static void queue_unlock(struct list_head *target)
{	if (target == &priv->tx_data_queue)
		mutex_unlock(&priv->tx_data_queue_lock);
	else if (target == &priv->rx_data_queue)
		mutex_unlock(&priv->rx_data_queue_lock);
	else if (target == &priv->rx_cmd_queue)
		mutex_unlock(&priv->rx_cmd_queue_lock);
}/**
*  Function Name: queue_empty
*  Input arguments (condition):
struct list_head *target : Queue Header
*  Processing in function (in pseudo code style):
*    1) Confirm Queue State
*  Function Return:
*   1) true : Empty
	2) false : Not Empty
*/
static bool queue_empty(struct list_head *target)
{	bool ret;
	//queue_lock(target);
	if (list_empty(target))
		ret = true;
	else
		ret = false;
	//queue_unlock(target);
	return ret;
}/**
*  Function Name: enqueue_data
*  Input arguments (condition):
struct list_head *target : Queue Header
struct spi_data *data : send spi_data
*  Processing in function (in pseudo code style):
*    1) enqueue spi data TxQueue
*  Function Return:
*   1) return 1 : success
*/
static int enqueue_data(struct list_head *target, struct spi_data *data)
{#ifdef DEBUG
	struct spi_data *obj2;
	int cnt = 0;
#endif
	queue_lock(target);
	list_add_tail(&(data->list), target);
#ifdef DEBUG
	list_for_each_entry(obj2, target, list)
	{
		cnt++;
		if (target == &priv->tx_data_queue)
			printk("cc1310: enqueue to (tx_queue) len: %d\n", obj2->len);
		else if (target == &priv->rx_data_queue)
			printk("cc1310: enqueue to (rx_queue) len: %d\n", obj2->len);
	}
	if (target == &priv->tx_data_queue)
		printk("cc1310: tx_queue num entries : %d", cnt);
	else if (target == &priv->rx_data_queue)
		printk("cc1310: rx_queue num entries : %d", cnt);
	printk("\n");
#endif
	queue_unlock(target);
	return 1;
}/**
*  Function Name: dequeue_data
*  Input arguments (condition):
struct list_head *target : Queue Header
struct spi_data *data : receive spi_data
*  Processing in function (in pseudo code style):
*    1) dequeue spi data RxQueue
*  Function Return:
*   1) return len : dequeue data len
*/
static int dequeue_data(struct list_head *target, struct spi_data **data)
{	int len = -1;
	struct list_head *pos, *q;
	queue_lock(target);
	if (list_empty(target))
	{
		printk("cc1310: dequeue_data() - empty queue");
		queue_unlock(target);
		return len;
	}
	list_for_each_safe(pos, q, target)
	{
		*data = list_entry(pos, struct spi_data, list);
		if (*data != NULL)
		{
			len = (*data)->len;
#ifdef DEBUG
			if (target == &priv->tx_data_queue)
				printk("cc1310: dequeue from (tx_queue)");
			else if (target == &priv->rx_data_queue)
				printk("cc1310: dequeue from (rx_queue)");
			printk("\n");
#endif
			list_del(pos);
		}
		break;
	}
	queue_unlock(target);
	return len;
}/**
*  Function Name: spi_data_rw
*  Input arguments (condition):
struct spi_device *spi : spi_device 
char *tx_buf : tx_buffer for spi communication
char *rx_buf : rx_buffer for spi communication
int len : spi communication size
*  Processing in function (in pseudo code style):
*    1) spi_communication wrapper
*  Function Return:
*   1) return 0 : error
*/
static int spi_data_rw(struct spi_device *spi, char *tx_buf, char *rx_buf, int len)
{	int ret;
	struct spi_transfer t = {
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = len,
		.cs_change = 0,
	};
	struct spi_message m;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
#ifdef DEBUG
	printk("cc1310: spi_data_rw() send %d Byte via spi_sync()", t.len);
#endif
	ret = spi_sync(spi, &m);
	if (ret)
		dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
	return ret;
}/**
*  Function Name: Packet_to_Upper
*  Input arguments (condition):
void *param : thread param
*  Processing in function (in pseudo code style):
*    1) Packet to Upper Layer Interface
*  Function Return:
*   1) return 0 : thread exit
*/
static int Packet_to_Upper(void *param)
{//	struct my_work *myWork = (struct my_work *)work;
	struct list_head *target = &priv->rx_data_queue;
	struct list_head *pos, *q;
	struct spi_data *temp;
	struct sk_buff *pSkb;
	unsigned char big_data[1500];
	int totalLen =0;
	struct MAC_SDU *ms_temp;
	int mflag=0;
	int src_addr=-1;
	memset(big_data, 0, 1500);
	allow_signal(SIGKILL);
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop())
	{
		wait_event_interruptible(rx_wq_head, !queue_empty(&priv->rx_data_queue) || module_state.end_thread);
		if (!module_state.end_thread)
		{
			queue_lock(target);
			if (list_empty(target))
			{
#ifdef DEBUG
				printk("net1310 : Packet_to_Upper() - empty queue");
#endif // DEBUG
			}
			else
			{
				list_for_each_safe(pos, q, target)
				{
					temp = list_entry(pos, struct spi_data, list);
					if (temp != NULL)
					{
						ms_temp = (struct MAC_SDU*)temp->buf;
						mflag = ms_temp->mflag;
						memcpy(&big_data[totalLen], ms_temp->payloadR, ms_temp->size);
						totalLen += ms_temp->size;
						list_del(pos);
					}
					if (mflag == 1)
					{
						pSkb = netdev_alloc_skb_ip_align(pDevCleanup, totalLen);
						skb_copy_to_linear_data(pSkb, &big_data[0], totalLen);
						skb_put(pSkb, totalLen);
						pSkb->protocol = eth_type_trans(pSkb, pDevCleanup);
						netif_rx_ni(pSkb);
						pDevCleanup->stats.rx_packets++;
						pDevCleanup->stats.rx_bytes += totalLen;
						memset(big_data, 0, 1500);
						totalLen = 0;
						break;
					}
				}
			}
#ifdef DEBUG
			printk("net1310 : Packet_to_Upper()");
#endif
			queue_unlock(target);
		}
	}
//	kfree(myWork);
	return 0;
}/**
*  Function Name: spi_frame_parsing
*  Input arguments (condition):
struct HCI_FRAME *rx_frame : 
*  Processing in function (in pseudo code style):
*    1) rx HCI frame parsing 
*  Function Return:
*   1) void
*/
static void spi_frame_parsing(struct HCI_Frame *rx_frame)
{	struct spi_data *data;
	struct my_work *myWork;
	if (rx_frame->fc.frame_type == HCI_FRAME_FC_FT_RES_SET)
	{
		data = (struct spi_data*)kmalloc(sizeof(struct spi_data), GFP_KERNEL);
		//Data
		if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_TRX)
		{
			//RX Data
			if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_RX)
			{
				if (rx_frame->length > 0)
				{
					data->len = rx_frame->length;
					memcpy(data->buf, rx_frame->payload, data->len);
					if (data->buf != NULL)
					{
						enqueue_data(&priv->rx_data_queue, data);
						if (rx_frame->payload[0] == 1)
						{
//							myWork = (struct my_work *)kmalloc(sizeof(struct my_work), GFP_KERNEL);
//							myWork->data = &priv->rx_data_queue;
//							INIT_WORK(&(myWork->work), Packet_to_Upper);
//							queue_work(rxWorkQ, &(myWork->work));
						}
						wake_up(&rx_wq_head);
						//kfree(data);
					}
					else
						kfree(data);
				}
			}
			//TX Data Ack
			else if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_TX)
			{
				tx_ack_management.response = rx_frame->payload[0];
				printk("spi_parsing : tx_ack_management.response: %d", tx_ack_management.response);
				if(tx_ack_management.response < 4) // Dummy Tx 만 보내야 할 때
					tx_ack_management.condition = true;
				else
					tx_ack_management.condition = false;
//				wake_up(&rx_cmd_wq_head);
			}
		}
		// Command ack
		else if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_PARAM)
		{
			if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_EXE)
			{
				cmd_ack_management.response = rx_frame->payload[0];
				cmd_ack_management.condition = true;
				wake_up(&rx_cmd_wq_head);
			}
			else if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_RF)
			{
				cmd_ack_management.response = rx_frame->payload[0];
				cmd_ack_management.condition = true;
				wake_up(&rx_cmd_wq_head);
			}
			else if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_MODE)
			{
				cmd_ack_management.response = rx_frame->payload[0];
				cmd_ack_management.condition = true;
				wake_up(&rx_cmd_wq_head);
			}
			if (cmd_ack_management.response == HCI_FRAME_ACK_OK)
			{
				if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_PARAM && rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_RF)
					memcpy(&(module_state.rf_phy_state), &(tmp_module_state.rf_phy_state), sizeof(struct rf_phy_params));
				else if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_PARAM && rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_EXE)
					module_state.state.op_state = tmp_module_state.state.op_state;
				else if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_PARAM && rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_MODE)
					memcpy(&(module_state.rf_info), &(tmp_module_state.rf_info), sizeof(struct rf_info));
			}
		}
	}
	else if (rx_frame->fc.frame_type == HCI_FRAME_FC_FT_RES_GET)
	{
		data = (struct spi_data*)kmalloc(sizeof(struct spi_data), GFP_KERNEL);
		if (rx_frame->fc.cmd_type == HCI_FRAME_FC_CT_MAC_PARAM)
		{
			if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_RF)
			{
				data->len = rx_frame->length;
				memcpy(data->buf, rx_frame->payload, sizeof(struct rf_phy_params));
			}
			else if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_MODE)
			{
				data->len = rx_frame->length;
				memcpy(data->buf, rx_frame->payload, sizeof(struct rf_info));
			}
			else if (rx_frame->fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_EXE)
			{
				data->len = rx_frame->length;
				memcpy(data->buf, rx_frame->payload, sizeof(struct operation_state));
			}
		}
		enqueue_data(&priv->rx_cmd_queue, data);
		wake_up(&rx_cmd_wq_head);
	}
	else if (rx_frame->fc.frame_type == HCI_FRAME_FC_FT_NACK)
	{
		tx_ack_management.response = rx_frame->payload[0];
		tx_ack_management.condition = true;
		wake_up(&rx_cmd_wq_head);
	}
}/**
*  Function Name: spi_comm_cntr
*  Input arguments (condition):
void *param : thread param
*  Processing in function (in pseudo code style):
*    1) SPI Communication Thread
*  Function Return:
*   1) return 0 : thread exit
*/
static int spi_comm_cntr(void *param)
{	int ret, len = 0;
	struct spi_data *data = NULL;
	struct HCI_Frame spi_tx_frame, spi_rx_frame;
	struct HCI_Frame spi_dummy_tx_frame;
#ifdef DEBUG
	int i = 0;
#endif
	/*Dummy Tx Initialize*/
	memset(&spi_dummy_tx_frame, 0, sizeof(struct HCI_Frame));
	spi_dummy_tx_frame.sof[0] = HCI_SOF_BYTE1;
	spi_dummy_tx_frame.sof[1] = HCI_SOF_BYTE2;
	spi_dummy_tx_frame.fc.frame_type = HCI_FRAME_FC_FT_REQ_SET;
	spi_dummy_tx_frame.fc.cmd_type = HCI_FRAME_FC_CT_MAC_TRX;
	spi_dummy_tx_frame.fc.cmd_sub_type = HCI_FRAME_FC_CST_MAC_DUMMY_TX;
	spi_dummy_tx_frame.length = 0;
	tx_ack_management.condition = false;
	tx_ack_management.response = 5;
	allow_signal(SIGKILL);
	set_current_state(TASK_INTERRUPTIBLE);
	while (!kthread_should_stop())
	{
#ifdef DEBUG
		printk("cc1310: Thread Wait For\n");
#endif
		wait_event_interruptible(tx_wq_head, !queue_empty(&priv->tx_data_queue) || module_state.end_thread || module_state.int_fired);
#ifdef DEBUG
		printk("cc1310: Thread Wake Up\n");
#endif
		if (!module_state.end_thread)
		{
			module_state.int_fired = false;
			if (queue_empty(&priv->tx_data_queue))
			{
				spi_tx_frame.sof[0] = HCI_SOF_BYTE1;
				spi_tx_frame.sof[1] = HCI_SOF_BYTE2;
				spi_tx_frame.fc.frame_type = HCI_FRAME_FC_FT_DUMMY;
			}
			else if((!queue_empty(&priv->tx_data_queue)) && tx_ack_management.condition == false)
			{
				len = dequeue_data(&priv->tx_data_queue, &data);
				if (len < 0)
				{
					printk("cc1310: Thread dequeue err(len: %d)", len);
					continue;
				}
				else
				{
					spi_tx_frame.sof[0] = HCI_SOF_BYTE1;
					spi_tx_frame.sof[1] = HCI_SOF_BYTE2;
					memcpy(&(spi_tx_frame.fc), data, HCI_FRAME_FC_SIZE + HCI_FRAME_LENGTH_SIZE);
					if (len > 0)
					{
						memcpy(spi_tx_frame.payload, data->buf, len);
						kfree(data);
						data = NULL;
					}
				}
			}
			else if(tx_ack_management.condition == true)
			{
				spi_tx_frame = spi_dummy_tx_frame;
			}
#ifdef DEBUG
			printk("tx frame_type: %d\n", spi_tx_frame.fc.frame_type);
			printk("tx cmd_type: %d\n", spi_tx_frame.fc.cmd_type);
			printk("tx cmd_sub_type: %d\n", spi_tx_frame.fc.cmd_sub_type);
#endif
			//Sync Byte Send
			ret = spi_data_rw(priv->spi, (char*)&spi_tx_frame, (char*)&spi_rx_frame, HCI_FRAME_SIZE);
			if (spi_rx_frame.sof[0] == HCI_SOF_BYTE1 && spi_rx_frame.sof[1] == HCI_SOF_BYTE2 && !ret)
			{
#ifdef DEBUG
				printk("**************************************");
				printk("sync_byte: %X %X\n", spi_rx_frame.sof[0], spi_rx_frame.sof[1]);
				printk("frame_type: %d\n", spi_rx_frame.fc.frame_type);
				printk("cmd_type: %d\n", spi_rx_frame.fc.cmd_type);
				printk("cmd_sub_type: %d\n", spi_rx_frame.fc.cmd_sub_type);
				printk("length: %d\n", spi_rx_frame.length);
				printk("payload\n");
				
				if (spi_rx_frame.fc.frame_type != HCI_FRAME_FC_FT_DUMMY)
				{
					if (spi_rx_frame.fc.cmd_sub_type == HCI_FRAME_FC_CST_MAC_RX)
					{
						for (i = 0; i < 5; i++)
							printk("%3x", spi_rx_frame.payload[i]);
					}
					else if (spi_rx_frame.fc.cmd_sub_type != HCI_FRAME_FC_CST_MAC_RX)
					{
						for (i = 0; i < spi_rx_frame.length; i++)
							printk("%3x", spi_rx_frame.payload[i]);
					}
				}
				
				printk("**************************************");
#endif
				if (spi_rx_frame.fc.frame_type != HCI_FRAME_FC_FT_DUMMY)
					spi_frame_parsing(&spi_rx_frame);
			}
			else
				printk("cc1310: (SOF err) %X, %X, ret %d", spi_rx_frame.sof[0], spi_rx_frame.sof[1], ret);
			memset(&spi_tx_frame, 0, sizeof(struct HCI_Frame));
			memset(&spi_rx_frame, 0, sizeof(struct HCI_Frame));
			mdelay(3); //
		}
	}
	return 0;
}/**
*  Function Name: cc1310_isr
*  Input arguments (condition):
int irq : irq number
void *data : received data by interrupt
*  Processing in function (in pseudo code style):
*    1) Process Interrupt
*  Function Return:
*   1) return IRQ_HANDLED : return IRQ Handle flag
*/
static irqreturn_t cc1310_isr(int irq, void *data)
{	if (!module_state.driver_nops)
	{
		if (queue_empty(&priv->tx_data_queue) && !module_state.int_fired)
		{
			module_state.int_fired = true;
			wake_up(&tx_wq_head);
#ifdef DEBUG
			printk("cc1310: isr() - tx_queue empty!");
#endif
		}
	}
	return IRQ_HANDLED;
}static const struct net_device_ops net1310_ops = 
{    .ndo_open = net1310_open,
    .ndo_stop = net1310_close,
    .ndo_start_xmit = net1310_xmit,
    .ndo_do_ioctl = net1310_ioctl,
    .ndo_set_mac_address = net1310_mac_addr,
};
/**
*  Function Name: boardSetting
*  Input arguments (condition):
struct work_struct *work : Frame for Setting Board
void *data : received data by interrupt
*  Processing in function (in pseudo code style):
*    1) Setting boardSetting
*  Function Return:
*/
static void boardSetting(struct work_struct *work)
{	struct my_work *myWork = (struct my_work*)work;
	struct spi_data *data = (struct spi_data *)myWork->data;
	enqueue_data(&priv->tx_data_queue, data);
	wake_up(&tx_wq_head);
	cmd_ack_management.condition = false;
	wait_event_interruptible(rx_cmd_wq_head, cmd_ack_management.condition);
}/**
*  Function Name: net1310_open
*  Input arguments (condition):
	struct net_device *pDev : network device to open
*  Processing in function (in pseudo code style):
*   1) network device open and start netif enqueue
*  Function Return:
	0 : success
*/
static int net1310_open(struct net_device *pDev)
{    struct spi_device *spi = priv->spi;
	struct spi_data *data = NULL;
	struct operation_state state;
	struct my_work *myWork;
	int rc, irq_type;
	spi->irq = gpio_to_irq(23);
	irq_type = irq_get_trigger_type(spi->irq);
	if (!irq_type)
		irq_type = IRQF_TRIGGER_HIGH;
    //cc1310_spi_open() section
    init_module_state();
    
    INIT_LIST_HEAD(&priv->tx_data_queue);
    INIT_LIST_HEAD(&priv->rx_data_queue);
    INIT_LIST_HEAD(&priv->rx_cmd_queue);
    priv->pThrHdlr = kthread_run(spi_comm_cntr, NULL, "spi_comm_cntr");
   if(IS_ERR(priv->pThrHdlr))
        return -1;
	priv->pThrHdlr_rx = kthread_run(Packet_to_Upper, NULL, "Packet_to_Upper");
	if (IS_ERR(priv->pThrHdlr_rx))
		return -1;
	rc = devm_request_irq(&spi->dev, spi->irq, cc1310_isr, IRQF_SHARED | irq_type, dev_name(&spi->dev), priv);
	if (rc)
	{
		printk("devm_request_irq");
		return -EINVAL;
	}
#ifdef DEBUG
    printk("net1310 : Open Module & Communication Thread Create\n");
#endif
    //Irq Section
    
	//IOC Configuration Section
	data = (struct spi_data *)kmalloc(sizeof(struct spi_data), GFP_KERNEL);
	if (data == NULL)
		return 0;
	//op_state_init
	state.op_state = RF_BUSY;
	//op_state_init end
		tmp_module_state.state.op_state = state.op_state;
	data->frame_type = HCI_FRAME_FC_FT_REQ_SET;
	data->cmd_type = HCI_FRAME_FC_CT_MAC_PARAM;
	data->cmd_sub_type = HCI_FRAME_FC_CST_MAC_EXE;
	data->len = 1;
	memcpy(data->buf, &state, sizeof(struct operation_state));
	myWork = (struct my_work *)kmalloc(sizeof(struct my_work), GFP_KERNEL);
	myWork->data = &(*data);
	INIT_WORK(&(myWork->work), boardSetting);
	queue_work(txWorkQ, &(myWork->work));
	//end setOPState
	netif_start_queue(pDev);
	printk("net1310 open complete !\n");
    return 0;
}/**
*  Function Name: net1310_close
*  Input arguments (condition):
struct net_device *pDev : network device to close
*  Processing in function (in pseudo code style):
*    1) network device close and stop netif enqueue
*  Function Return:
0 : success
*/
static int net1310_close(struct net_device *pDev)
{    netif_stop_queue(pDev);
	struct spi_data* data;
	// reset CC1310 RF Module
	data = (struct spi_data*)kmalloc(sizeof(struct spi_data), GFP_KERNEL);
	data->frame_type = HCI_FRAME_FC_FT_REQ_SET;
	data->cmd_type = HCI_FRAME_FC_CT_SYS;
	data->cmd_sub_type = HCI_FRAME_FC_CST_SYS_RESET;
	data->len = 0;
#ifdef DEBUG
	printk("cc1310: release enqueue Reset Command");
#endif
	module_state.driver_nops = true; // for Interrupt Ignore
	enqueue_data(&priv->tx_data_queue, data);
	wake_up(&tx_wq_head);
	mdelay(30); // Delay for Reset Command Transmit
				//thread stop
	if (priv->pThrHdlr)
	{
#ifdef DEBUG
		printk("cc1310: release Delete Thread");
#endif
		reset_module_state();
		wake_up(&tx_wq_head);
		kthread_stop(priv->pThrHdlr);
		printk("cc1310: Release Module & Communication Thread Killed\n");
		printk("\n");
	}
	else
	{
		printk("cc1310: No Kernel Thread To Kill\n");
		printk("\n");
	}
	if (priv->pThrHdlr_rx)
	{
#ifdef DEBUG
		printk("cc1310: release Delete Thread");
#endif
		reset_module_state();
		wake_up(&tx_wq_head);
		kthread_stop(priv->pThrHdlr_rx);
		printk("cc1310: Release Packet To Upper Killed\n");
		printk("\n");
	}
	else
	{
		printk("cc1310: No Kernel Thread To Kill\n");
		printk("\n");
	}
    return 0;
}/**
*  Function Name: net1310_xmit
*  Input arguments (condition):
	struct sk_buff *pSkb: received Socket buffer from upper layer
	struct net_device *pDev :  network device to use
*  Processing in function (in pseudo code style):
*    1) received Socket buffer from upper layer and send to under layer
*  Function Return:
	NETDEV_TX_OK : xmit is ok
*/
static netdev_tx_t net1310_xmit(struct sk_buff *pSkb, struct net_device *pDev)
{	struct MAC_SDU *myFrame;
	struct spi_data *data;
	int totalLen = pSkb->len;
	int sumOfLen = 0;
	int i = 0;
	int k = 0;
	netif_stop_queue(pDev);
/*	
	if ((pSkb->data[12] == 0x86) && (pSkb->data[13] == 0xdd))
    {
        printk("ipv6 packet drop");
		goto no_enqueue;
    }
*/
	if (((pSkb->data[12] == 0x80) && (pSkb->data[13] == 0x00))||((pSkb->data[12] == 0x86) && (pSkb->data[13] == 0xdd)))
	{
		if ((pSkb->data[36] == 0x07) && (pSkb->data[37] == 0x6c))
        {
            printk("ipv4 SSDP packet drop");
			goto no_enqueue;
        }
		else if ((pSkb->data[36] == 0x14) && (pSkb->data[37] == 0xeb))
        {
            printk("ipv4 LLMNR packet drop");
			goto no_enqueue;
        }
		else if ((pSkb->data[36] == 0x44) && (pSkb->data[37] == 0x5c))
        {
            printk("ipv4 DB-LSP");
			goto no_enqueue;
        }
	}
		for (k = 0; k <= ((totalLen - 1) / MAC_SDU_PAYLOAD_SIZE); k++)
	{
		myFrame = (struct MAC_SDU*)kmalloc(sizeof(struct MAC_SDU), GFP_KERNEL);
		if (k == ((totalLen - 1) / MAC_SDU_PAYLOAD_SIZE))
		{
#ifdef DEBUG
			printk("net1310 : net1310_xmit _ make mflag =1 packet");
#endif // DEBUG
			myFrame->mflag = 1;
			myFrame->size = totalLen - sumOfLen;
		}
		else
		{
#ifdef DEBUG
			printk("net1310 : net1310_xmit _ make mflag =0 packet");
#endif // DEBUG
			myFrame->mflag = 0;
			myFrame->size = MAC_SDU_PAYLOAD_SIZE;
		}
		for (i = 0; i < myFrame->size; i++)
		{
			myFrame->payloadR[i] = pSkb->data[i + sumOfLen];
		}
		sumOfLen += myFrame->size;
		if (module_state.state.op_state == RF_IDLE)
		{
			printk("op_state :%d", module_state.state.op_state);
			goto no_enqueue;
		}
		while (1)
		{
			data = (struct spi_data*)kmalloc(sizeof(struct spi_data), GFP_KERNEL);
			//		ret = copy_from_user(data->buf, buf, count);
			memcpy(data->buf, myFrame, sizeof(struct MAC_SDU));
			data->frame_type = HCI_FRAME_FC_FT_REQ_SET;
			data->cmd_type = HCI_FRAME_FC_CT_MAC_TRX;
			data->cmd_sub_type = HCI_FRAME_FC_CST_MAC_TX;
			data->len = (myFrame->size) + MAC_SDU_HEADER_SIZE;
			enqueue_data(&priv->tx_data_queue, data);
			wake_up(&tx_wq_head);
//			tx_ack_management.condition = false;
			break;
		}
		pDevCleanup->stats.tx_packets++;
		pDevCleanup->stats.tx_bytes += (myFrame->size) + MAC_SDU_HEADER_SIZE;
		kfree(myFrame);
	}
	kfree_skb(pSkb);
	netif_start_queue(pDev);
	return NETDEV_TX_OK;
no_enqueue:
	return NETDEV_TX_OK;
}static int net1310_ioctl(struct net_device *pDev, struct ifreq *pIfr, int cmd)
{    return -ENOIOCTLCMD;
}static int net1310_mac_addr(struct net_device *pDev, void *p)
{    const struct sockaddr *addr = p;
    memcpy(pDev->dev_addr, addr->sa_data, ETH_ALEN);
    return 0;
}/**
*  Function Name: net1310_destruct
*  Input arguments (condition):
struct net_device *pDev :  network device to free
*  Processing in function (in pseudo code style):
*    1) free network device
*  Function Return:
	void
*/
static void net1310_destruct (struct net_device *pDev)
{    free_netdev(pDev);
}/**
*  Function Name: net1310_setup
*  Input arguments (condition):
struct net_device *pDev :  network device to setup
*  Processing in function (in pseudo code style):
*    1) set up network device
*  Function Return:
void
*/
static void net1310_setup (struct net_device *pDev)
{    char addr[6] = {0xBC,0xEE,0x7B, 11, 11, 15};
#ifdef STATION
    addr[5] = 0x14;
#endif
    
    ether_setup(pDev);
    pDev->netdev_ops = &net1310_ops;
    pDev->destructor = net1310_destruct;
    pDev->mtu = 1500;
    memcpy(pDev->dev_addr, addr, pDev->addr_len);
}/**
*  Function Name: net1310_probe
*  Input arguments (condition):
struct spi_device *spi : to probe spi
*  Processing in function (in pseudo code style):
*    1) network device probe
*  Function Return:
	return not zero : error
*/
static int net1310_probe(struct spi_device *spi)
{	int rc, irq_type;
	int size = sizeof(struct cc1310_priv);
	struct net_device *pDev;
	priv = (struct cc1310_priv *)kmalloc(sizeof(struct cc1310_priv), GFP_KERNEL);
	mutex_init(&priv->tx_data_queue_lock);
	mutex_init(&priv->rx_data_queue_lock);
	mutex_init(&priv->rx_cmd_queue_lock);
	module_state.end_thread = true;
	module_state.driver_nops = true;
	spi->irq = gpio_to_irq(23);
	if (!spi->irq)
	{
		dev_err(&spi->dev, "no IRQ specified\n");
		return -EINVAL;
	}
	dev_set_drvdata(&spi->dev, priv);
	priv->spi = spi;
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	spi_setup(spi);
	printk("net1310 : SPI Setup Complete\n");
	printk("net1310 : SPI speed: %d, mode: %d, bits_per_word: %d\n", spi->max_speed_hz, spi->mode, spi->bits_per_word);
	//net1310_setup(pDev);
	//netdev section
	pDev = alloc_netdev(0, "HaLow%d", NET_NAME_UNKNOWN, net1310_setup);
	pDevCleanup = 0x00;
	register_netdev(pDev);
	pDevCleanup = pDev;
	//workQ init
	txWorkQ = create_workqueue("tx_workQ");
//	rxWorkQ = create_workqueue("rx_workQ");
}static int net1310_remove(struct spi_device *spi)
{    kfree(priv);
    return 0;
}static const struct of_device_id net1310_of_match[] =
{    {.compatible = "ti,cc1310",},
    {},
};
MODULE_DEVICE_TABLE(of, net1310_of_match);
static const struct spi_device_id net1310_device_id[] = 
{    {.name = "net1310",},
    {},
};
MODULE_DEVICE_TABLE(spi, net1310_device_id);
static struct spi_driver net1310_driver =
{    .id_table = net1310_device_id,
    .driver = 
    {
        .of_match_table = of_match_ptr(net1310_of_match),
        .name = DEVICE_NAME,
    },
    .probe = net1310_probe,
    .remove = net1310_remove,
};
static int __init net1310_init(void)
{    int status;
    status = spi_register_driver(&net1310_driver);
    printk("YangJiHyeok : Calling net1310_init");
    return status;
}module_init(net1310_init);
static void __init net1310_exit(void)
{    spi_unregister_driver(&net1310_driver);
    printk("YangJiHyeok : Calling net1310_exit");
}module_exit(net1310_exit);
MODULE_ALIAS("spi:net1310");
MODULE_DESCRIPTION("CC1310 Sub-1GHz WiFi Extender");
MODULE_LICENSE("GPL v2");

