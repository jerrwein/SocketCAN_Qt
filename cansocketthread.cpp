#include <QtTest/QtTest>
#include <QTime>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <pthread.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>

#include "canstructures.h"
#include "cansocketthread.h"
#include "Fifo.h"

QTime GLB_QTime;

CANSocketThread::CANSocketThread(QObject *parent) : QThread(parent), waitTimeout(0), m_quitThread(false)
{
    m_ReadTimeouts = 0;
    m_WriteTimeouts = 0;

    m_bBusyProcTrans = false;
    m_SocketNameToOpen = "can0";
    m_CanSocket = -1;
    m_ProcessingThreadActive = false;

//    FiFo * m_pFifo = new FiFo;

 //   DWORD dw = QThread::currentThreadId();
  //  qDebug("1.) Constructor thread ID = %lX", QThread::currentThreadId());
    qDebug() << "2.) Constructor thread ID = " << QThread::currentThreadId();


  //  m_quitThread = false;
}

CANSocketThread::~CANSocketThread()
{
    m_SerialMutex.lock();
    m_quitThread = true;
    // Wake up the serial thread for the 'quit' operation
    m_SerialWaitCond.wakeOne();
    m_SerialMutex.unlock();
    wait();

 //   if (m_pFifo)
 //       delete m_pFifo;
}

bool CANSocketThread::RunProcessingThread(bool bEnable)
{
    m_ProcessingThreadActive = bEnable;
    return true;
}

bool CANSocketThread::Send_NMT(uint8_t u8_val)
{
    struct can_frame 	tx_frame;
    int                 nbytes;
    int                 errsv;

    if (m_CanSocket == -1)
        return false;

    tx_frame.can_id  = TX_NMT_COB_ID;
    tx_frame.data[0] = u8_val;
    tx_frame.data[1] = IC650_NODE_ID;
    tx_frame.can_dlc = 2;

    nbytes = write (m_CanSocket, &tx_frame, sizeof(struct can_frame));
    errsv = errno;
    if (nbytes != sizeof(struct can_frame))
    {
        emit this->GeneralResponse (tr("Send_NMT(): BW=%1, LE=%2\n").arg(nbytes).arg(errsv));
        return false;
    }
    return true;
}

bool CANSocketThread::Send_Heartbeat (uint8_t u8_mode)
{
    CAN_DATA           tx_data;

    if (m_CanSocket == -1)
        return false;

    tx_data.CanFrame.can_id  = IC650_TX_HEARTBEAT_COB_ID;
    tx_data.CanFrame.data[0] = u8_mode;
    tx_data.CanFrame.can_dlc = 1;

    // Insert data/frame into FIFO
    if (1 != m_CanTxFifo.InsertElement(&tx_data))
    {
        emit this->GeneralResponse ("Failed to queue CAN data in Tx FIFO\n");
        qDebug() << "  Failed to queue Tx FIFO...";
        return false;
    }
    else
    {
        qDebug() << "Successfully queued Tx FIFO...";
        return true;
    }
}

bool CANSocketThread::Send_ReadSDO (uint16_t index,uint8_t sub_index)
{
    struct can_frame 	tx_frame;
    int                 nbytes;
    int                 errsv;

    if (m_CanSocket == -1)
        return false;

    tx_frame.can_id  = IC650_TX_SDO_COB_ID;
    tx_frame.data[0] = IC650_SDO_RD_CMD;
    tx_frame.data[1] = (uint8_t)(index & 0x00ff);
    tx_frame.data[2] = (uint8_t)((index & 0xff00) >> 8);
    tx_frame.data[3] = sub_index;
    tx_frame.can_dlc = 4;

    nbytes = write (m_CanSocket, &tx_frame, sizeof(struct can_frame));
    errsv = errno;
    if (nbytes != sizeof(struct can_frame))
    {
        emit this->GeneralResponse (tr("Send_NMT(): BW=%1, LE=%2\n").arg(nbytes).arg(errsv));
        return false;
    }
    return true;
}

bool CANSocketThread::Send_WriteSDO (uint16_t index, uint8_t sub_index, uint8_t numBytes, uint8_t data[])
{
    struct can_frame 	tx_frame;
    int                 nBytesSent;
    int                 errsv;

    if (m_CanSocket == -1)
        return false;

    tx_frame.can_id  = IC650_TX_SDO_COB_ID;
    tx_frame.data[1] = (uint8_t)(index & 0x00ff);
    tx_frame.data[2] = (uint8_t)((index & 0xff00) >> 8);
    tx_frame.data[3] = sub_index;

    switch (numBytes)
    {
        case 1:
            tx_frame.data[0] = IC650_SDO_WRT_1_CMD;
            tx_frame.data[4] = data[0];
            tx_frame.can_dlc = 5;
            break;
        case 2:
            tx_frame.data[0] = IC650_SDO_WRT_2_CMD;
            tx_frame.data[4] = data[0];
            tx_frame.data[5] = data[1];
            tx_frame.can_dlc = 6;
            break;
        case 3:
            tx_frame.data[0] = IC650_SDO_WRT_3_CMD;
            tx_frame.data[4] = data[0];
            tx_frame.data[5] = data[1];
            tx_frame.data[6] = data[2];
            tx_frame.can_dlc = 7;
            break;
        case 4:
            tx_frame.data[0] = IC650_SDO_WRT_4_CMD;
            tx_frame.data[4] = data[0];
            tx_frame.data[5] = data[1];
            tx_frame.data[6] = data[2];
            tx_frame.data[7] = data[3];
            tx_frame.can_dlc = 8;
            break;
        default:
            return false;
    }

    // Write CAN frame
    nBytesSent = write (m_CanSocket, &tx_frame, sizeof(struct can_frame));
    errsv = errno;
    if (nBytesSent != sizeof(struct can_frame))
    {
        emit this->GeneralResponse (tr("Send_NMT(): BW=%1, LE=%2\n").arg(nBytesSent).arg(errsv));
        return false;
    }
    return true;
}

bool CANSocketThread::OpenCanSocket(QString newSocketName)
{
    // Update PORT info
    m_SocketNameToOpen = newSocketName;

    int                 nRes;
    struct ifreq        if_req;
    struct sockaddr_can sock_addr;

    if ((m_CanSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        emit GeneralResponse (tr("Failed to open socket %1.\n").arg(newSocketName));
        return false;
    }

    strcpy (if_req.ifr_name, "can0");
    ioctl (m_CanSocket, SIOCGIFINDEX, &if_req);

    sock_addr.can_family  = AF_CAN;
    sock_addr.can_ifindex = if_req.ifr_ifindex;

    if (bind(m_CanSocket, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) < 0)
    {
        emit GeneralResponse (tr("Failed to bind socket %1.\n").arg(newSocketName));
        return false;
    }

    // Set socket to non-blocking I/O
    int flags = fcntl (m_CanSocket, F_GETFL, 0);
    if (flags < 0)
    {
        emit GeneralResponse (tr("Error getting CANSocket flags %1.\n").arg(newSocketName));
        return false;
    }
    flags = flags | O_NONBLOCK;
    nRes = fcntl (m_CanSocket, F_SETFL, flags);
    if (0 != nRes)
    {
        emit GeneralResponse (tr("Failed to set CANSocket non-blocking flags %1.\n").arg(newSocketName));
        return false;
    }

    // Setup for timestamps
    // Enable timestamps
    int opt_val = 1;
    int opt_len = sizeof(opt_val);
    nRes = setsockopt (m_CanSocket, SOL_SOCKET, SO_TIMESTAMP, &opt_val, opt_len);
    if (nRes < 0)
    {
        emit GeneralResponse (tr("Failed to set CANSocket timestamp options %1.\n").arg(newSocketName));
        return false;
    }

    emit GeneralResponse (tr("Successfully opened %1 socket.\n").arg(newSocketName));

    return true;
}

#if 0
void CANSocketThread::transaction( int canSocket, int transID, int waitTimeout, const QString &request)
{
//    QTime           time;
 //   m_time.start();
    qDebug() << "transaction(0): Initial entry into routine at: " << GLB_QTime.elapsed() << "ms. TID: " << QThread::currentThreadId();
// xxx    QMutexLocker locker(&m_SerialMutex);
    m_SerialMutex.lock();
    qDebug() << "transaction(1): Aquired lock at: " << GLB_QTime.elapsed() << "ms. TID: " << QThread::currentThreadId();
}
#endif

void CANSocketThread::run()
{
//    QTime           time;
  //  bool            bCurrentPortNameChanged = false;
 //   static long         lTransProcessed = 0;
 //   int                 currentTransID;
     QString            qstrSocketNameOpen = "None";
     int                nBytesRcvd, nbytesTransmitted;
     int                errsv;
     long               msgsRcvd = 0;
     long               msgsProcessed = 0;
     long               txMsgsProcessed = 0;
     struct can_frame 	rxmsg;
     CAN_DATA           rx_data, CanFifoData, CanTxFifoData;

    //-----------------------------------------
    // initialize the message-header structure
    //-----------------------------------------
    //	struct msghdr	mymsghdr = { 0 };
    struct msghdr	mymsghdr;
    struct msghdr	*msgp       = &mymsghdr;
    struct cmsghdr	*cmsg;
    struct iovec	my_iov;
    unsigned char	tm_cbuf[40] = { 0 };
    int             tm_cbuf_len = sizeof(tm_cbuf);

    // Get current time
    struct timeval	tv_base, tv_recv;
    unsigned long long	stamp_base, stamp_recv;
    unsigned long long	stamp_delta;
    if ( gettimeofday( &tv_base, NULL ) < 0 )
    {
        emit this->GeneralResponse("Get base time NOT successful!.");
        return;
    }
    stamp_base = tv_base.tv_sec * 1000000LL + tv_base.tv_usec;

    // Setup for timestamps
    my_iov.iov_base = &rxmsg;
    my_iov.iov_len = sizeof(rxmsg);

    mymsghdr.msg_name       = NULL;
    mymsghdr.msg_namelen	= 0;
    mymsghdr.msg_iov        = &my_iov;
    mymsghdr.msg_iovlen     = 1;
    // mymsghdr.msg_control	= NULL;
    mymsghdr.msg_control	= tm_cbuf;
    // mymsghdr.msg_controllen	= 0;
    mymsghdr.msg_controllen	= tm_cbuf_len;
    mymsghdr.msg_flags      = 0;

//   FiFo *pLocalFifo = new FiFo;
 //    FiFo localFifo;
    bool    bCanSocketReady = false;
    bool    bRxDataTaken;

     GLB_QTime.restart();

    qDebug() << "  ST: Kicking off thread..." << GLB_QTime.elapsed() << "ms - TID=" << QThread::currentThreadId();

    while (!m_quitThread)
    {
        if (qstrSocketNameOpen != m_SocketNameToOpen)
        {
            bCanSocketReady = false;
            qDebug() << "  CT: Port name change detected";
            if (OpenCanSocket(m_SocketNameToOpen))
            {
                qstrSocketNameOpen = m_SocketNameToOpen;
              //  m_CanSocketActive = true;
                m_CanReqReady = true;
                bCanSocketReady = true;
            }
            else
            {
                emit this->GeneralResponse("Failure opening port, exiting.");
              //  m_CanSocketActive = false;
                return;
            }
        }
        if (m_ProcessingThreadActive)
        {
            if (bCanSocketReady)
            {
                if (m_CanTxFifo.IsDataAvailable())
                {
                    txMsgsProcessed++;
                    if (1 != m_CanTxFifo.GetElement(&CanTxFifoData))
                    {
                        emit this->GeneralResponse ("Failed to retreive CAN data from transmit FIFO\n");
                    }
                    else
                    {
                        nbytesTransmitted = write (m_CanSocket, &(CanTxFifoData.CanFrame), sizeof(struct can_frame));
                        if (nbytesTransmitted != sizeof(struct can_frame))
                        {
                            errsv = errno;
                            emit this->GeneralResponse (tr("Send_NMT(): BW=%1, LE=%2\n").arg(nbytesTransmitted).arg(errsv));
                        }
                    }
                }

                do      // Bring in all available frames
                {
                    // Check for incoming traffic with timestamps
                    nBytesRcvd = recvmsg (m_CanSocket, &mymsghdr, MSG_DONTWAIT);
                    if (0 < nBytesRcvd)
                    {
                        bRxDataTaken = true;
                        if (nBytesRcvd != sizeof(struct can_frame))
                        {
                            emit this->GeneralResponse (tr("Rx(1) message(%1) bytes: %2\n").arg(msgsRcvd).arg(nBytesRcvd));
                        }
                        else
                        {
    #if 1
                            // Apply system macros for accessing the ancilliary data
                            for (cmsg=CMSG_FIRSTHDR(msgp); cmsg != NULL; cmsg = CMSG_NXTHDR(msgp,cmsg))
                            {
                                if ((cmsg->cmsg_level == SOL_SOCKET) && (cmsg->cmsg_type == SO_TIMESTAMP))
                                {
                                    memcpy (&tv_recv, CMSG_DATA(cmsg), sizeof(tv_recv));
                                    stamp_recv = tv_recv.tv_sec * 1000000LL + tv_recv.tv_usec;
                                    stamp_delta = stamp_recv - stamp_base;
                                    qDebug("Rx delta = %lld\n", stamp_delta);
                                    break;
                                }
                                else
                                {
                                    stamp_delta = 0ll;
                                }
                            }
                            if (cmsg == NULL)
                            {
                                /* Error: SO_TIMESTAMP not enabled or similar */
                                emit this->GeneralResponse (tr("Error: SO_TIMESTAMP not enabled or similar\n"));
                            }
    #endif
                            rx_data.usec_delta = stamp_delta;
                            rx_data.tv_usec = 100;
                            rx_data.CanFrame = rxmsg;
                            msgsRcvd++;
                            emit this->ProcessCanResponse("Incoming CAN data", msgsRcvd, rx_data);
                   //         emit this->GeneralResponse (tr("Rx(2) message(%1) bytes: %2\n").arg(msgsRcvd).arg(nBytesRcvd));

                            // Insert data/frame into FIFO
                            if (1 != m_CanRxFifo.InsertElement(&rx_data))
                            {
                                emit this->GeneralResponse ("Failed to queue received CAN data in FIFO\n");
                                qDebug() << "Failed to queue FIFO...";
                            }
                            else
                            {
                               qDebug() << "Successfully queued Rx FIFO...";
                            }
                        }
                    }
                    else
                    {
                        bRxDataTaken = false;
                    }
                }
                while (bRxDataTaken);

                // Process FIFOed data
                if (m_CanRxFifo.IsDataAvailable())
                {
                    msgsProcessed++;
                    if (1 != m_CanRxFifo.GetElement(&CanFifoData))
                    {
                        emit this->GeneralResponse ("Failed to retrieve CAN data from FIFO\n");
                    }
                    else
                    {
                        ProcessRecvdFrame (&CanFifoData, m_CanSocket);
                    }
                }
            } // end if (bCanSocketReady)

            QThread::msleep(1);

               // Clear BUSY
            //   m_BusyMutex.lock();
            //   m_bBusyProcTrans = false;
            //   m_BusyMutex.unlock();

               // Loop can now run
              // bReady = true;

            //   qDebug() << "  ST: Calling Wait() with lock release at" << GLB_QTime.elapsed() << "ms - TID=" << QThread::currentThreadId();
               // Wait for next transaction request - mutex will be re-locked atomically with wait()
               // m_SerialWaitCond.wait(&m_SerialMutex);
           //    while (!m_SerialWaitCond.wait(&m_SerialMutex, 1000))
            //   {
            //       qDebug() << "  ST: Wait(100) timed out at " << GLB_QTime.elapsed() << "ms - TID=" << QThread::currentThreadId();
            //   }
            //   qDebug() << "  ST: Thread resuming with locked mutex at" << GLB_QTime.elapsed() << "ms - TID=" << QThread::currentThreadId();
        }

    };

}

int CANSocketThread::ProcessRecvdFrame(CAN_DATA *pCanData, int CAN_sock)
{
    bool    bVerbose = false;
    int     i;
    static SEG_TRANSFER_INFO 	segXferInfo;

#if 0

    printf("Rx: ID=0x%03X, DLC=%d \n", pCanFrame->can_id, pCanFrame->can_dlc);
    for (i =0; i<pCanFrame->can_dlc; i++)
    {
         printf("0x%02X ", pCanFrame->data[i]);
    }
    printf("\n");
#endif

    if (IC650_RX_SDO_COB_ID == pCanData->CanFrame.can_id)
    {
        if (bVerbose)
        {
            printf("  Rx: ID=0x%03X, DLC=%d \n", pCanData->CanFrame.can_id, pCanData->CanFrame.can_dlc);
            printf("    ");
            for (i =0; i<pCanData->CanFrame.can_dlc; i++)
            {
                printf("0x%02X ", pCanData->CanFrame.data[i]);
            }
            printf("\n");
        }

        if (IC650_SDO_SEG_RD_TOGGLE_RESP == (pCanData->CanFrame.data[0] & IC650_CMD_SERVER_CMD_SPEC_MASK))
        {
            // Process Toggle-X Frame
            if (0 == ProcessRecvdSegmentedSDO (pCanData, &segXferInfo, false /* verbosity */))
            {
                if (IC650_CMD_SEG_LAST_FRAME_MASK & pCanData->CanFrame.data[0])
                {
                    if (bVerbose)
                    {
                        printf("ProcessRecvdSegmentedSDO(SEG_TOG-0): bytes taken = %d\n", segXferInfo.u32_BytesTaken);
                    }
                    // Todo: Process segments
                    if (segXferInfo.u32_BytesTaken != segXferInfo.u32_BytesExpected)
                    {
                        printf("Segmented Xfer - MISMATCH in bytes received, expected = %d\n", segXferInfo.u32_BytesExpected);
                    }
                    else
                    {
                        if (bVerbose)
                        {
                            printf("Segmented Xfer - LAST frame received\n");
                            printf("Data: %s\n", segXferInfo.data);
                        }
                        if (IC650_SDO_MANUF_SW_VERSION == segXferInfo.primary_index)
                        {
                            printf("P-SDO(SW-VER): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, segXferInfo.primary_index);
                            printf("P-SDO(SW-VER): Version: %s\n", segXferInfo.data);
                    //        QString qstr = "Version: " + QString::fromStdString(segXferInfo.data);
                            QString qstr = "Version: " + QString::fromStdString(reinterpret_cast<char *>(segXferInfo.data)) + "\n";
                            //                         reinterpret_cast<char *>
                            emit this->GeneralResponse (qstr);
                        }
                    }
                }
                else
                {
                    // Toggle polarity flag
                    segXferInfo.u8_toggle_state ^= IC650_CMD_SEG_TOGGLE_MASK;
                    //	printf("Segmented Xfer - Send TOG-(0x%02X) request\n", segXferInfo.u8_toggle_state);
                    if (0 != SendSegmentedReadSDO (segXferInfo.u8_toggle_state, CAN_sock, bVerbose))
                    {
                        printf("Segmented Xfer TOG-X seg request failed!!!\n");
                    }
                }
            }
        }
        else if (!(IC650_CMD_EXPED_TRANSFER_BIT & pCanData->CanFrame.data[0]))	/* 0x02 */
        {
            if (bVerbose)
            {
                printf("Segmented Xfer starting\n");
            }
            segXferInfo.primary_index = (((uint16_t)pCanData->CanFrame.data[2]) << 8)
                          + ((uint16_t)pCanData->CanFrame.data[1]);
            segXferInfo.sub_index = pCanData->CanFrame.data[3];
            segXferInfo.u32_BytesExpected = (((uint16_t)pCanData->CanFrame.data[7]) << 24)
                              + (((uint16_t)pCanData->CanFrame.data[6]) << 16)
                              + (((uint16_t)pCanData->CanFrame.data[5]) << 8)
                              + ((uint16_t)pCanData->CanFrame.data[4]);
            segXferInfo.u32_BytesTaken = 0;
            segXferInfo.u8_toggle_state = 0x00;
            // Clear receive buffer
            for (i=0; i<25; i++)
            {
                segXferInfo.data[i] = 0;
            }
            //pSegInfo->TransferPhase = SEG_TOG_0_FRAME;
            if (bVerbose)
            {
                printf("ProcessRecvdFrame(INITIAL_RESP): bytes expected = %d\n", segXferInfo.u32_BytesExpected);
            }
            //printf("Expedited Xfer - Send TOG-0 request\n");
            if (0 != SendSegmentedReadSDO (segXferInfo.u8_toggle_state, CAN_sock, bVerbose))
            {
                printf("Segmented Xfer 1st. seg request failed!!!\n");
            }
        }
        else
        {
            ProcessRecvdSDO (pCanData);
        }
    }
    else if (IC650_RX_PDO1_COB_ID == pCanData->CanFrame.can_id)
    {
        ProcessRecvdPDO1 (pCanData);
    }
    else if (IC650_RX_PDO2_COB_ID == pCanData->CanFrame.can_id)
    {
        ProcessRecvdPDO2 (pCanData);
    }
    else if (IC650_RX_PDO3_COB_ID == pCanData->CanFrame.can_id)
    {
        ProcessRecvdPDO3 (pCanData);
    }
#if 0
    else if (IC650_RX_HEARTBEAT_COB_ID == pCanData->CanFrame.can_id)
    {
        IC650_ProcessRecvdHeartbeat (pCanData, CAN_sock);
    }
    else
    {
        printf("PF: What ??? ID: 0x%03X\n", pCanData->CanFrame.can_id);
    }
#endif
    return 0;
}

int CANSocketThread::ProcessRecvdSegmentedSDO (CAN_DATA *pCanData, SEG_TRANSFER_INFO *pSegInfo, bool bVerbose)
{
 //   uint32_t 	u32_val;
    int		n;

    uint8_t u8_unused = (IC650_CMD_SEG_UNUSED_BYTES_MASK & pCanData->CanFrame.data[0]) >> 1;

    if (bVerbose)
    {
        printf("ProcessRecvdSegmentedSDO(): bytes unused = %d\n", u8_unused);
    }

    // Check TOGGLE polarity
    if (pSegInfo->u8_toggle_state != (pCanData->CanFrame.data[0] & IC650_CMD_SEG_TOGGLE_MASK))
    {
        printf("ProcessRecvdSegmentedSDO(): Unexpected Toggle bit = 0x%02X\n", (pCanData->CanFrame.data[0] & IC650_CMD_SEG_TOGGLE_MASK));
        printf("ProcessRecvdSegmentedSDO(): Expected Toggle bit = 0x%02X\n", pSegInfo->u8_toggle_state);
        // ToDo: Cancel segment transfer
        return -1;
    }
    for (n=0; n<(7-u8_unused); n++)
    {
        pSegInfo->data[pSegInfo->u32_BytesTaken+n] = pCanData->CanFrame.data[n+1];
    }
    pSegInfo->u32_BytesTaken += (7-u8_unused);

    return 0;
}

int CANSocketThread::SendSegmentedReadSDO (uint8_t u8_tog, int CAN_sock, bool bVerbose)
{
    struct can_frame 	tx_frame;
    int                 nbytes;
    int                 err_last;

    tx_frame.can_id  = IC650_TX_SDO_COB_ID;
    tx_frame.data[0] = IC650_SDO_SEG_RD_CMD | u8_tog;
//	tx_frame.data[1] = 0x00;
//	tx_frame.data[2] = 0x00;
//	tx_frame.data[3] = 0x00;
//	tx_frame.data[4] = 0x00;
//	tx_frame.data[5] = 0x00;
//	tx_frame.data[6] = 0x00;
//	tx_frame.data[7] = 0x00;
    tx_frame.can_dlc = 1;

    if (bVerbose)
    {
        int i;
        printf("  Tx: ID=0x%03X, DLC=%d\n", tx_frame.can_id, tx_frame.can_dlc);
        printf("    ");
        for (i =0; i<tx_frame.can_dlc; i++)
        {
            printf("0x%02X ", tx_frame.data[i]);
        }
        printf("\n");
    }

    nbytes = write (CAN_sock, &tx_frame, sizeof(struct can_frame));
    err_last = errno;
    if (nbytes != sizeof(struct can_frame))
    {
        printf("BW: %d, LE #: %d (%s)\n", nbytes, err_last, strerror(err_last));
        return -1;
    }
    else
    {
    //	printf("Segmented BW: %d\n", nbytes);
    }
    return 0;
}

int CANSocketThread::ProcessRecvdPDO1 (CAN_DATA *pCanData)
{
 //   uint16_t u16_val;
//    uint16_t u8_val;
//    uint16_t primary_index = (((uint16_t)pCanData->CanFrame.data[2]) << 8)
 //                  + ((uint16_t)pCanData->CanFrame.data[1]);

    emit this->ProcessCanResponse_PDO(pCanData->CanFrame.can_id, *pCanData);
    return 0;
}

int CANSocketThread::ProcessRecvdPDO2 (CAN_DATA *pCanData)
{
    emit this->ProcessCanResponse_PDO(pCanData->CanFrame.can_id, *pCanData);
    return 0;
}

int CANSocketThread::ProcessRecvdPDO3 (CAN_DATA *pCanData)
{
    emit this->ProcessCanResponse_PDO(pCanData->CanFrame.can_id, *pCanData);
    return 0;
}

int CANSocketThread::ProcessRecvdSDO (CAN_DATA *pCanData)
{
    float f32_val;
    uint16_t u16_val;
    uint16_t u8_val;
    uint16_t primary_index = (((uint16_t)pCanData->CanFrame.data[2]) << 8)
                   + ((uint16_t)pCanData->CanFrame.data[1]);

    if (IC650_SDO_RD_CMD == (0xf0 & pCanData->CanFrame.data[0]))
    {
    //    void ProcessCanResponse_PDO(long cob_id, const CAN_DATA &rx_data);
        emit this->ProcessCanResponse_SDO(primary_index, *pCanData);
        switch (primary_index)
        {
            case IC650_SDO_IDENTITY:
                printf("P-SDO(IDNT): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
                break;
            case IC650_SDO_AC_VOLT_X16:
                printf("P-SDO(AC_X16): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
                u16_val = (((uint16_t)pCanData->CanFrame.data[5]) << 8)
                        + ((uint16_t)pCanData->CanFrame.data[4]);
            //	printf("P-SDO(AC_X16): AC: 0x%04X, AC/16 (%d.%02f)\n", u16_val, u16_val>>4, (float)(u16_val&0x000f)/16.0f);
                f32_val = (float)(u16_val) / 16.0f;
                printf("P-SDO(AC_X16): AC: %3.3f\n", f32_val);
                break;
            case IC650_SDO_CHARGER_TEMPERATURE:
                printf("P-SDO(CHRG_TEMP): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
                u8_val = pCanData->CanFrame.data[4];
                printf("P-SDO(CHRG_TEMP): temp: 0x%02X (%dd C.)\n", u8_val, u8_val);
                break;
            case IC650_SDO_EXTENDED_STATUS:
                printf("P-SDO(EXT_STS): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
                break;
            default:
                printf("P-SDO(---): ID: 0x%03X, SDO_RD (%04X)\n", pCanData->CanFrame.can_id, primary_index);
        };
    }
    else
    {
        printf("PF: ID: 0x%03X, ??? (%04X)\n", pCanData->CanFrame.can_id, primary_index);
    }
    return 0;
}
