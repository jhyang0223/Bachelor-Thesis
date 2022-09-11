/**
* File Name : "MAC.c"
* Description:
* - This program is for TDD RF Full-duplex protocol
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
#include "MAC.h"
/***************************************
             Global Variable
 ***************************************/
unsigned char cc1310_TxOverheadByte[9] = {11, 11, 11, 11, 11, 11};
unsigned short cc1310_TxOverheadUs[9] = {440, 360, 360, 300, 300, 290};
double IndexToBPuS[9] = {0.05, 0.1, 0.2, 0.3, 0.4, 0.5};
unsigned int Super_Frame_Size = 0;
unsigned short mBeacon_Interval = 0;
unsigned short mGaurd_Interval = 0;
unsigned short mTDMA_Start_Time = 0;
unsigned short mTDMA_Period = 0;
unsigned short mTDMA_End_Time = 0;
unsigned short mTDMA_Slot_Size = 0;
unsigned short numTDMA_Slot = 0;
/***************************************
                 Macro
 ***************************************/
#define getTransUsDelay(len, bps) (unsigned int)(((len + cc1310_TxOverheadByte[bps]) * 8 / IndexToBPuS[bps]) + cc1310_TxOverheadUs[bps])
#define UsToMiniSlot(millisecond) (unsigned short)(ceil((millisecond) / 500.0))
 /**
 *  Function Name: calcCheckSum
 *  Input arguments (condition):
	unsigned char *ptr : Frame information
	int size :frame size
	int *BOOL : compare received checksum
 *  Processing in function (in pseudo code style):
 *    1) calculate CheckSum
 *  Function Return:
 *   1) return calculated Checksum
 */
unsigned short calChecksum(unsigned char *ptr, int size, int *BOOL)
{    unsigned short result = 0;
    unsigned int carryResult = 0;
    unsigned char *pCTemp;
    unsigned char temp1 = 0, temp2 = 0;
    unsigned int temp_32 = 0;
    unsigned int carry = 0;
    DataFrame *temp_F = (DataFrame *)ptr;
    unsigned short originCheck = temp_F->checkSum;
    temp_F->checkSum = 0;
    if ((size % 2) == 0)
    {
        pCTemp = new unsigned char[size];
        memset(pCTemp, 0, size);
        for (int i = 0; i < size; i++)
            pCTemp[i] = ptr[i];
    }
    else
        return 1;
    for (int i = 0; i < size; i += 2)
    {
        temp1 = pCTemp[i];
        temp2 = pCTemp[i + 1];
        temp_32 = temp1;
        temp_32 = (temp_32 << 8) + (unsigned int)temp2;
        carryResult += temp_32;
        if (carryResult >= 0x00010000)
        {
            carry = (carryResult & 0xFFFF0000) >> 16;
            carryResult -= (carryResult & 0xFFFF0000);
            carryResult += carry;
        }
    }
    result = (unsigned short)(carryResult & 0x0000FFFF);
    result = ~result;
    *BOOL = (originCheck == result);
    return result;
}/**
*  Function Name: MACThread
*  Input arguments (condition):
	void *arg : thread argument
*  Processing in function (in pseudo code style):
* - This Thread is for TDD RF Full-duplex protocol

*  Function Return:
*   1) return void
*/
void *MACThread(void *arg)
{    Thread_Params thread_params = *(Thread_Params*)arg;
    HCI_Frame_Queue mac_req_queue = thread_params.mac_req_queue;
    HCI_Frame_Queue mac_res_queue = thread_params.mac_res_queue;
    HCI_Frame_Queue_Entry *ret_hci_frame_entry = NULL;
    Event_Handle queue_moni_th_wake_evt = thread_params.queue_moni_th_wake_evt;
    //HCI_Frame mac_req_hci_frame; //, mac_res_hci_frame;
    Event_Handle mac_th_wake_evt = thread_params.mac_th_wake_evt;
    Shared_MAC_Params *share_mac_params = thread_params.shared_mac_params;
    RF_Packet_Recv_Queue_Entry *recv_packet_entry;
    //variable
    unsigned int evt;
    unsigned char first_flag = 1;
    unsigned int i = 0;
    RF_Params_Phy phy_params;
    Control_Phase phase = BeaconPhase;
    Super_Frame_Size = SUPER_FRAME_SIZE;
    mBeacon_Interval = SUPER_FRAME_SIZE/MINI_SLOT_SIZE;
    mGaurd_Interval = UsToMiniSlot(GUARD_INTERVAL);
    mTDMA_Start_Time = UsToMiniSlot(getTransUsDelay(sizeof(BeaconFrame), RF_PARAM_PHY_DR_500KBPS) + 1000);
    mTDMA_Period = (mBeacon_Interval - mGaurd_Interval) - mTDMA_Start_Time;
    mTDMA_End_Time = mTDMA_Start_Time + mTDMA_Period;
    // Slot Size
    mTDMA_Slot_Size = UsToMiniSlot(getTransUsDelay(sizeof(DataFrame), RF_PARAM_PHY_DR_500KBPS)); // 10 -> 11
    numTDMA_Slot = mTDMA_Period / mTDMA_Slot_Size;
    phy_params.freq_tbl_idx = RF_PARAM_PHY_FREQ_917MHZ;
    phy_params.dr_tbl_idx = RF_PARAM_PHY_DR_500KBPS;
    phy_params.tx_pwr_tbl_idx = RF_PARAM_PHY_RE_TXPWR_26DBM;
    setPhyRFCtlIF(phy_params);
    dealyRFCtlIF(RF_getCurrentTime() + RF_convertUsToRatTicks(4 * MINI_SLOT_SIZE));
    while(1)
    {
#ifdef RPI_AP
            share_mac_params->mac_mode = MAC_MODE_AP;
#else
            share_mac_params->mac_mode = MAC_MODE_NODE;
#endif
        evt = Event_pend(mac_th_wake_evt, Event_Id_NONE, MAC_THREAD_BUSY_TRANS_WAKE_EVT + MAC_THREAD_BUSY_MAC_WAKE_EVT, BIOS_WAIT_FOREVER);
        if(evt & MAC_THREAD_BUSY_MAC_WAKE_EVT) // Busy MAC Mode
        {
            if(share_mac_params->mac_mode == MAC_MODE_AP)
            {
                BeaconFrame beacon_frame;
                DataFrame tx_data_frame, *rx_data_frame = NULL;
                memset(&beacon_frame, 0, sizeof(BeaconFrame));
                memset(&tx_data_frame, 0, sizeof(DataFrame));
                unsigned long long next_beacon_time = 0, cur_beacon_start_time = 0, tdma_start_time = 0, tdma_slot_time = 0;
                while(1)
                {
                    if(phase == BeaconPhase)
                    {
                        beacon_frame.MH.frame_control = Beacon;
                        beacon_frame.MH.src_addr = AP_ADDR;
                        beacon_frame.MH.dst_addr = BROAD_CAST_ADDR;
                        beacon_frame.mBeacon_interval = mBeacon_Interval;
                        beacon_frame.tsi.mTslotPosition = mTDMA_Start_Time;
                        beacon_frame.tsi.startTDMAslot = 1;
                        beacon_frame.tsi.mTslotSize = mTDMA_Slot_Size;
                        beacon_frame.tsi.numTslot = numTDMA_Slot;
                        if(first_flag)
                        {
                            first_flag = 0;
                            next_beacon_time = (unsigned long long)RF_getCurrentTime() + (unsigned long long)RF_convertUsToRatTicks(Super_Frame_Size);
                            dealyRFCtlIF((unsigned int)next_beacon_time);
                            continue;
                        }
                        cur_beacon_start_time = next_beacon_time;
                        onDebugPIN0();
                        txRFCtlIF((char*)&beacon_frame, sizeof(BeaconFrame), NULL);
                        offDebugPIN0();
                        next_beacon_time = cur_beacon_start_time + (unsigned long long)RF_convertUsToRatTicks(Super_Frame_Size);
                        phase = TransmissionPhase;
                    }
                    else if(phase == TransmissionPhase)
                    {
                        tdma_slot_time = tdma_start_time = cur_beacon_start_time + (unsigned long long)RF_convertUsToRatTicks(mTDMA_Start_Time * MINI_SLOT_SIZE);
                        tx_data_frame.MH.frame_control = Data;
                        tx_data_frame.MH.src_addr = AP_ADDR;
                        tx_data_frame.MH.dst_addr = ST_ADDR;
                        dealyRFCtlIF((unsigned int)tdma_start_time);
                        for(i = 0; i < numTDMA_Slot/2; i++)
                        {
                            if(!(Queue_empty(mac_req_queue.handle)))
                            {
                                tx_data_frame.MH.dst_addr = ST_ADDR;
                                Semaphore_pend(mac_req_queue.sema, BIOS_WAIT_FOREVER);
                                ret_hci_frame_entry = Queue_dequeue(mac_req_queue.handle);
                                (*(mac_req_queue.size))--;
                                Semaphore_post(mac_req_queue.sema);
                                memcpy(tx_data_frame.dummy, &(ret_hci_frame_entry->hci_frame.payload), ret_hci_frame_entry->hci_frame.length);
                                //tx_data_frame.more_flag = ?;
                                free(ret_hci_frame_entry);
                                toggleLED0();
                            }
                            else
                            {
                                tx_data_frame.MH.dst_addr = 0;
                            }
                            onDebugPIN0();
                            txRFCtlIF((char*)&tx_data_frame, sizeof(DataFrame), NULL);
//                            toggleLED0();
                            offDebugPIN0();
                            tdma_slot_time += (unsigned long long)RF_convertUsToRatTicks(mTDMA_Slot_Size * MINI_SLOT_SIZE);
                            onDebugPIN1();
                            rxRFCtlIF(NULL, (unsigned int)(tdma_slot_time + (unsigned long long)RF_convertUsToRatTicks(getTransUsDelay(sizeof(DataFrame), RF_PARAM_PHY_DR_500KBPS))), RUN);
                            offDebugPIN1();
                            while(!(Queue_empty(pakcet_recv_queue.handle)))
                            {
                                //Semaphore_pend(pakcet_recv_queue.sema, BIOS_WAIT_FOREVER);
                                recv_packet_entry = Queue_dequeue(pakcet_recv_queue.handle);
                                pakcet_recv_queue.size--;
                                //Semaphore_post(pakcet_recv_queue.sema);
                                if(recv_packet_entry->packet_len == sizeof(DataFrame))
                                {
                                    rx_data_frame = (DataFrame*)recv_packet_entry->packet;
                                    if(rx_data_frame->MH.dst_addr == AP_ADDR)
                                    {
                                        ret_hci_frame_entry = (HCI_Frame_Queue_Entry *)malloc(sizeof(HCI_Frame_Queue_Entry));
                                        ret_hci_frame_entry->hci_frame.fc.frame_type = HCI_FRAME_FC_FT_RES_SET;
                                        ret_hci_frame_entry->hci_frame.fc.cmd_type = HCI_FRAME_FC_CT_MAC_TRX;
                                        ret_hci_frame_entry->hci_frame.fc.cmd_sub_type = HCI_FRAME_FC_CST_MAC_RX;
                                        ret_hci_frame_entry->hci_frame.length = rx_data_frame->dummy[1]+MAC_SDU_HEADER_SIZE;
                                        memcpy(ret_hci_frame_entry->hci_frame.payload, rx_data_frame->dummy, MAC_SDU_FRAME_SIZE);
                                        Semaphore_pend(mac_res_queue.sema, BIOS_WAIT_FOREVER);
                                        Queue_enqueue(mac_res_queue.handle, &(ret_hci_frame_entry->elem));
                                        (*(mac_res_queue.size))++;
                                        Semaphore_post(mac_res_queue.sema);
                                        toggleLED1();
                                        Event_post(queue_moni_th_wake_evt, QUEUE_MONITOR_WAKE_EVT);
                                    }
                                }
                                free(recv_packet_entry->packet);
                                free(recv_packet_entry);
                            }
                            tdma_slot_time += (unsigned long long)RF_convertUsToRatTicks(mTDMA_Slot_Size * MINI_SLOT_SIZE);
//                            if(RF_getCurrentTime() >= (unsigned int)tdma_slot_time)
//                                toggleLED1();
                            onDebugPIN1();
                            dealyRFCtlIF((unsigned int)tdma_slot_time);
                            offDebugPIN1();
                        }
                        phase = BeaconPhase;
                        dealyRFCtlIF((unsigned int)next_beacon_time);
                    }
                    if(Event_pend(mac_th_wake_evt, Event_Id_NONE, MAC_THREAD_IDLE_EVT, BIOS_NO_WAIT) & MAC_THREAD_IDLE_EVT)
                    {
                        first_flag = 1;
                        break;
                    }
                }// AP while
            }// if AP
            else if(share_mac_params->mac_mode == MAC_MODE_NODE)
            {
                BeaconFrame beacon_frame;
                DataFrame tx_data_frame, *rx_data_frame = NULL;
                memset(&beacon_frame, 0, sizeof(BeaconFrame));
                memset(&tx_data_frame, 0, sizeof(DataFrame));
                unsigned int beacon_duration_time =  getTransUsDelay(sizeof(BeaconFrame), BASIC_BITRATE);
                unsigned long long beacon_rcv_time = 0, next_sf_start_time = 0, tdma_start_time = 0, tdma_slot_time = 0;
                while(1)
                {
                    if(phase == BeaconPhase)
                    {
                        rxRFCtlIF(NULL, NULL, RUN);
                        while(!(Queue_empty(pakcet_recv_queue.handle)))
                        {
                            Semaphore_pend(pakcet_recv_queue.sema, BIOS_WAIT_FOREVER);
                            recv_packet_entry = Queue_dequeue(pakcet_recv_queue.handle);
                            pakcet_recv_queue.size--;
                            Semaphore_post(pakcet_recv_queue.sema);
                            if(recv_packet_entry->packet_len == sizeof(BeaconFrame))
                            {
                                //beacon_rcv_time = (unsigned long long)(RF_getCurrentTime());
                                beacon_rcv_time = (unsigned long long)(RF_getCurrentTime() - RF_convertUsToRatTicks(258));
                                memcpy(&beacon_frame, recv_packet_entry->packet, sizeof(BeaconFrame));
                                next_sf_start_time = beacon_rcv_time + (unsigned long long)RF_convertUsToRatTicks((beacon_frame.mBeacon_interval * MINI_SLOT_SIZE) - beacon_duration_time);
                                phase = TransmissionPhase;
                            }
                            free(recv_packet_entry->packet);
                            free(recv_packet_entry);
                        }
                    }
                    else if(phase == TransmissionPhase)
                    {
                        tdma_slot_time = tdma_start_time = beacon_rcv_time + (unsigned long long)RF_convertUsToRatTicks((beacon_frame.tsi.mTslotPosition * MINI_SLOT_SIZE)- beacon_duration_time);
                        tx_data_frame.MH.frame_control = Data;
                        tx_data_frame.MH.src_addr = ST_ADDR;
                        tx_data_frame.MH.dst_addr = AP_ADDR;
                        dealyRFCtlIF((unsigned int)tdma_start_time);
                        for(i = 0; i < beacon_frame.tsi.numTslot/2; i++)
                        {
                            onDebugPIN1();
                            rxRFCtlIF(NULL, (unsigned int)(tdma_slot_time + (unsigned long long)RF_convertUsToRatTicks(getTransUsDelay(sizeof(DataFrame), RF_PARAM_PHY_DR_500KBPS))), RUN);
                            offDebugPIN1();
                            onDebugPIN0();
                            while(!(Queue_empty(pakcet_recv_queue.handle)))
                            {
                                Semaphore_pend(pakcet_recv_queue.sema, BIOS_WAIT_FOREVER);
                                recv_packet_entry = Queue_dequeue(pakcet_recv_queue.handle);
                                pakcet_recv_queue.size--;
                                Semaphore_post(pakcet_recv_queue.sema);
                                if(recv_packet_entry->packet_len == sizeof(DataFrame))
                                {
                                    rx_data_frame = (DataFrame*)recv_packet_entry->packet;
                                    if(rx_data_frame->MH.dst_addr == ST_ADDR)
                                    {
                                       ret_hci_frame_entry = (HCI_Frame_Queue_Entry *) malloc(sizeof(HCI_Frame_Queue_Entry));
                                       ret_hci_frame_entry->hci_frame.fc.frame_type = HCI_FRAME_FC_FT_RES_SET;
                                       ret_hci_frame_entry->hci_frame.fc.cmd_type = HCI_FRAME_FC_CT_MAC_TRX;
                                       ret_hci_frame_entry->hci_frame.fc.cmd_sub_type = HCI_FRAME_FC_CST_MAC_RX;
                                       ret_hci_frame_entry->hci_frame.length = rx_data_frame->dummy[1]+MAC_SDU_HEADER_SIZE;
                                       memcpy(ret_hci_frame_entry->hci_frame.payload, rx_data_frame->dummy, MAC_SDU_FRAME_SIZE);
                                       Semaphore_pend(mac_res_queue.sema, BIOS_WAIT_FOREVER);
                                       Queue_enqueue(mac_res_queue.handle, &(ret_hci_frame_entry->elem));
                                       (*(mac_res_queue.size))++;
                                       Semaphore_post(mac_res_queue.sema);
                                       Event_post(queue_moni_th_wake_evt, QUEUE_MONITOR_WAKE_EVT);
                                       toggleLED1();
                                    }
                                }
                                free(recv_packet_entry->packet);
                                free(recv_packet_entry);
                            }
                            if(!(Queue_empty(mac_req_queue.handle)))
                            {
                                tx_data_frame.MH.dst_addr = AP_ADDR;
                                Semaphore_pend(mac_req_queue.sema, BIOS_WAIT_FOREVER);
                                ret_hci_frame_entry = Queue_dequeue(mac_req_queue.handle);
                                (*(mac_req_queue.size))--;
                                Semaphore_post(mac_req_queue.sema);
                                memcpy(tx_data_frame.dummy, &(ret_hci_frame_entry->hci_frame.payload), (ret_hci_frame_entry->hci_frame.length));
                                free(ret_hci_frame_entry);
                                toggleLED0();
                            }
                            else
                            {
                                tx_data_frame.MH.dst_addr = 0;
                            }
                            tdma_slot_time += (unsigned long long)RF_convertUsToRatTicks(mTDMA_Slot_Size * MINI_SLOT_SIZE);
                            offDebugPIN0();
                            onDebugPIN0();
                            //memcpy(tx_data_frame.dummy,rx_data_frame->dummy,241);
                            //txRFCtlIF((char*)&tx_data_frame, sizeof(DataFrame), tdma_slot_time);
                            txRFCtlIF((char*)&tx_data_frame, sizeof(DataFrame), NULL);
                            offDebugPIN0();
                            tdma_slot_time += (unsigned long long)RF_convertUsToRatTicks(mTDMA_Slot_Size * MINI_SLOT_SIZE);
                            onDebugPIN0();
                            dealyRFCtlIF((unsigned int)tdma_slot_time);
                            offDebugPIN0();
                        }
                        phase = BeaconPhase;
                        onDebugPIN1();
                        dealyRFCtlIF((unsigned int)next_sf_start_time - GUARD_INTERVAL);
                        offDebugPIN1();
                    }
                }// station while
            } // else if Station*/
        } // if MAC busy
    }
}/**
*  Function Name: initMACThread
*  Input arguments (condition):
Thread_Params *thread_params : thread parameter
*  Processing in function (in pseudo code style):
* - initialize MACThread

*  Function Return:
*   1) return void
*/
void initMACThread(Thread_Params *thread_params)
{    pthread_t           thread;
    pthread_attr_t      attrs;
    struct sched_param  pri_param;
    int                 retc;
    int                 detachState;
    /* Set priority and stack size attributes */
    pthread_attr_init(&attrs);
    pri_param.sched_priority = 1;
    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }
    pthread_attr_setschedparam(&attrs, &pri_param);
    retc |= pthread_attr_setstacksize(&attrs, 2048);    // THREADSTACKSIZE = 2048 1792
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }
    // Create MACThread
    retc = pthread_create(&thread, &attrs, MACThread, (void*)thread_params);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
}