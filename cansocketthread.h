#ifndef CANSOCKETTHREAD_H
#define CANSOCKETTHREAD_H

#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QTime>

#include <stdint.h>
#include <linux/can.h>
#include "canstructures.h"
#include "Fifo.h"

#define IC650_NODE_ID			(0x0A)

#define IC650_PRE_OPERATIONAL	(0x7F)
#define IC650_OPERATIONAL		(0x05)

#define TX_NMT_COB_ID               (0x000)
#define IC650_TX_HEARTBEAT_COB_ID	(0x701)
#define IC650_TX_HEARTBEAT_MODE     (0x00)

#define IC650_TX_SDO_COB_ID         (0x60A)
#define IC650_RX_SDO_COB_ID         (0x58A)

#define IC650_RX_PDO1_COB_ID	(0x28A)
#define IC650_RX_PDO2_COB_ID	(0x38A)
#define IC650_RX_PDO3_COB_ID	(0x48A)

#define IC650_NMT_START			(0x01)
#define IC650_NMT_STOP			(0x02)
#define IC650_NMT_PRE_OP		(0x80)
#define IC650_NMT_RESET_NODE	(0x81)
#define IC650_NMT_RESET_COMM	(0x82)

#define IC650_SDO_RD_CMD                (0x40)
#define IC650_SDO_WRT_1_CMD             (0x2F)
#define IC650_SDO_WRT_2_CMD             (0x2B)
#define IC650_SDO_WRT_3_CMD             (0x27)
#define IC650_SDO_WRT_4_CMD             (0x23)

#define IC650_SDO_SEG_RD_CMD            (0x60)
#define IC650_SDO_SEG_RD_TOGGLE_RESP	(0x00)

#define IC650_SDO_MANUF_SW_VERSION      (0x100A)
#define IC650_SDO_IDENTITY              (0x1018)
#define IC650_SDO_AC_VOLT_X16           (0x2200)
#define IC650_SDO_DC_VOLT_Q8P8          (0x2101)
#define IC650_SDO_CHARGER_TEMPERATURE	(0x2050)
#define IC650_SDO_EXTENDED_STATUS       (0x2006)

#define IC650_SDO_BATTERY_STATUS        (0x6000)
#define IC650_SDO_VOLTAGE_LIMIT_REQ     (0x2271)
#define IC650_SDO_CHARGE_CURRENT_LIMIT  (0x6070)


// Segmented transfer specific
#define IC650_CMD_EXPED_TRANSFER_BIT	(0x02)
#define IC650_CMD_SERVER_CMD_SPEC_MASK	(0xE0)
#define IC650_CMD_SEG_TOGGLE_MASK       (0x10)
#define IC650_CMD_SEG_UNUSED_BYTES_MASK	(0x0E)
#define IC650_CMD_SEG_LAST_FRAME_MASK	(0x01)

#define SUB_INDX_0  (0)

#if 0
typedef struct
{
    struct can_frame 	CanFrame;
    unsigned long long	usec_delta;
    unsigned long		tv_usec;
} CAN_DATA;

Q_DECLARE_METATYPE(CAN_DATA);
#endif

Q_DECLARE_METATYPE(CAN_DATA);

class CANSocketThread : public QThread
{
  Q_OBJECT
public:
//    CANSocketThread();
    CANSocketThread(QObject *parent = 0);
    ~CANSocketThread();

    void OpenSocket(const QString &socketName);
    bool RunProcessingThread(bool bEnable);

    bool Send_NMT(uint8_t u8_val);
    bool Send_ReadSDO (uint16_t index, uint8_t sub_index);
//    bool Send_WriteSDO (uint16_t index, uint8_t sub_index, uint32_t data);
    bool Send_WriteSDO (uint16_t index, uint8_t sub_index, uint8_t numBytes, uint8_t data[]);
    bool Send_Heartbeat (uint8_t u8_mode);

 //   void transaction(int canSocket, int transID, int waitTimeout, const QString &request);
    void run();

signals:
    void GeneralResponse(const QString &s);
    void GeneralError(const QString &s);
//    void GeneralTimeout(const QString &s, long read_to, long write_to);
//    void ProcessResponse(const QString &s);
//    void SocketOpenResponse(const QString &s, bool bStatus);

//    void ProcessCanResponse(const QString &s, const struct can_frame &rx_frame);
    void ProcessCanResponse(const QString &s, long lMsgID, const CAN_DATA &rx_data);

    void ProcessCanResponse_PDO(long cob_id, const CAN_DATA &rx_data);
    void ProcessCanResponse_SDO(int sdo_index, const CAN_DATA &rx_data);


private:
   QString  m_SocketNameToOpen;
   bool     m_ProcessingThreadActive;
   bool     m_CanReqReady;
   int      m_CanSocket;
   QString  request;
   int      waitTimeout;

   FiFo     m_CanRxFifo;
   FiFo     m_CanTxFifo;

   QMutex   m_SerialMutex;

   bool     m_bBusyProcTrans;
   QMutex   m_BusyMutex;

   QWaitCondition m_SerialWaitCond;
   bool     m_quitThread;

   long     m_ReadTimeouts;
   long     m_WriteTimeouts;

   int      m_ThisTransID;
//   SCPI_Commands m_eThisCMD;

private:
   bool SubmitRequest (int canSocket, QByteArray requestData, int waitTimeout, int nTransID);
   void GetResponse (int canSocket, int waitTimeout, int nTransID);
   bool EmptyRxBuffer(int canSocket);
   bool OpenCanSocket(QString newSocketName);
   int  ProcessRecvdFrame(CAN_DATA *pCanData, int CAN_sock);
   int  ProcessRecvdSegmentedSDO (CAN_DATA *pCanData, SEG_TRANSFER_INFO *pSegInfo, bool bVerbose);
   int  SendSegmentedReadSDO (uint8_t u8_tog, int CAN_sock, bool bVerbose);
   int  ProcessRecvdPDO1 (CAN_DATA *pCanData);
   int  ProcessRecvdPDO2 (CAN_DATA *pCanData);
   int  ProcessRecvdPDO3 (CAN_DATA *pCanData);
   int  ProcessRecvdSDO (CAN_DATA *pCanData);
};
#endif // CANSOCKETTHREAD_H
