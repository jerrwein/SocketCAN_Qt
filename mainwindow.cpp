#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtTest/QtTest>
#include <QThread>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    m_CanThreadActive = false;
    m_HeatbeatActive = false;
    m_NMT_Operational = false;

    // Connect all sgnals to slots
    qRegisterMetaType<CAN_DATA>("CAN_DATA");
    connect(&m_CANSocketThread, SIGNAL(GeneralResponse(QString)), this, SLOT(UI_ShowGeneralStatus(QString)));
    connect(&m_CANSocketThread, SIGNAL(ProcessCanResponse(QString, long, CAN_DATA)), this, SLOT(UI_ProcessCanResponse(QString, long, const CAN_DATA)));
    connect(&m_CANSocketThread, SIGNAL(ProcessCanResponse_PDO(long, CAN_DATA)), this, SLOT(UI_ProcessCanResponse_PDO(long, const CAN_DATA)));
    connect(&m_CANSocketThread, SIGNAL(ProcessCanResponse_SDO(int, CAN_DATA)), this, SLOT(UI_ProcessCanResponse_SDO(int, const CAN_DATA)));

    connect(&m_UiTimer, SIGNAL(timeout()), this, SLOT(on_TimerTick()));
    m_UiTimer.setInterval(500);

    qDebug() << "Starting serial thread...";
    m_CANSocketThread.start();
    qDebug() << "Started serial thread.";

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_TimerTick(void)
{
    qDebug() << "on_TimerTick() - submitting H-B request...";
    m_CANSocketThread.Send_Heartbeat(IC650_OPERATIONAL);
}

void MainWindow::UI_ShowGeneralStatus(const QString &qstrReport)
{
    this->ui->GeneralStatus_textEdit->insertPlainText(qstrReport);
    this->ui->GeneralStatus_textEdit->moveCursor(QTextCursor::End);
}

void MainWindow::UI_ProcessCanResponse_PDO(long cob_id, const CAN_DATA &rx_data)
{
    QString qstr_data;
    uint8_t u8_val;
    uint16_t u16_val;
    float f32_val;

    switch (cob_id)
    {
        case IC650_RX_PDO1_COB_ID:
            qstr_data = QByteArray::number(rx_data.CanFrame.can_dlc);
            qstr_data += " - 0x" + QString("%1").arg(rx_data.CanFrame.data[0], 2, 16, QChar('0'));
            if (1 < rx_data.CanFrame.can_dlc)
            {
                for (int i=1; i<rx_data.CanFrame.can_dlc; i++)
                    qstr_data += " 0x" + QString("%1").arg(rx_data.CanFrame.data[i], 2, 16, QChar('0'));
            }
            qstr_data += "\n";
            this->ui->CAN_PDO1_lineEdit->setText(qstr_data);
            break;
        case IC650_RX_PDO2_COB_ID:
            qstr_data = QByteArray::number(rx_data.CanFrame.can_dlc);
            qstr_data += " - 0x" + QString("%1").arg(rx_data.CanFrame.data[0], 2, 16, QChar('0'));
            if (1 < rx_data.CanFrame.can_dlc)
            {
                for (int i=1; i<rx_data.CanFrame.can_dlc; i++)
                    qstr_data += " 0x" + QString("%1").arg(rx_data.CanFrame.data[i], 2, 16, QChar('0'));
            }
            qstr_data += "\n";
            this->ui->CAN_PDO2_lineEdit->setText(qstr_data);
            // Charger Status
            u8_val = (uint16_t)rx_data.CanFrame.data[0];
            this->ui->CAN_PDO3_ChargeStatus_lineEdit->setText(QByteArray::number(u8_val, 16));
            // A-H returned last charge
            u16_val = (((uint16_t)rx_data.CanFrame.data[2]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[1]);
            f32_val = (float)(u16_val);
            this->ui->CAN_PDO3_AmpHoursReturned_lineEdit->setText(QByteArray::number(f32_val));
            // State of Charge
            u8_val = (uint16_t)rx_data.CanFrame.data[3];
            this->ui->CAN_PDO3_StateOfCharge_lineEdit->setText(QByteArray::number(u8_val, 16));
            break;
        case IC650_RX_PDO3_COB_ID:
            qstr_data = QByteArray::number(rx_data.CanFrame.can_dlc);
            qstr_data += " - 0x" + QString("%1").arg(rx_data.CanFrame.data[0], 2, 16, QChar('0'));
            if (1 < rx_data.CanFrame.can_dlc)
            {
                for (int i=1; i<rx_data.CanFrame.can_dlc; i++)
                    qstr_data += " 0x" + QString("%1").arg(rx_data.CanFrame.data[i], 2, 16, QChar('0'));
            }
            qstr_data += "\n";
            this->ui->CAN_PDO3_lineEdit->setText(qstr_data);
            // Charger current
            u16_val = (((uint16_t)rx_data.CanFrame.data[1]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[0]);
            f32_val = (float)(u16_val) / 16.0f;
            this->ui->CAN_PDO3_ChargeCurrent_lineEdit->setText(QByteArray::number(f32_val));
            // Battery Voltage
            u16_val = (((uint16_t)rx_data.CanFrame.data[3]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[2]);
            f32_val = (float)(u16_val) / 256.0f;
            this->ui->CAN_PDO3_BatteryVoltage_lineEdit->setText(QByteArray::number(f32_val));
            // Extended Charge Status
            u16_val = (((uint16_t)rx_data.CanFrame.data[5]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[4]);
            this->ui->CAN_PDO3_ExtStatus_lineEdit->setText(QByteArray::number(u16_val, 16));
            break;
    }
}

void MainWindow::UI_ProcessCanResponse_SDO(int sdo_index, const CAN_DATA &rx_data)
{
    uint8_t u8_val;
    uint16_t u16_val;
    uint32_t u32_val;
    float f32_val;
    switch (sdo_index)
    {
        case IC650_SDO_AC_VOLT_X16:
            u16_val = (((uint16_t)rx_data.CanFrame.data[5]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[4]);
            f32_val = (float)(u16_val) / 16.0f;
            this->ui->CAN_SDO_AcLineVoltage_lineEdit->setText(QByteArray::number(f32_val));
            break;
        case IC650_SDO_DC_VOLT_Q8P8:
            u16_val = (((uint16_t)rx_data.CanFrame.data[5]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[4]);
            f32_val = (float)(u16_val) / 256.0f;
            this->ui->CAN_SDO_DcVoltage_lineEdit->setText(QByteArray::number(f32_val));
            break;
        case IC650_SDO_BATTERY_STATUS:
            u8_val = rx_data.CanFrame.data[4];
            this->ui->CAN_SDO_BatteryStatus_lineEdit->setText(QByteArray::number(u8_val, 16));
            break;
        case IC650_SDO_VOLTAGE_LIMIT_REQ:
            u32_val = (((uint32_t)rx_data.CanFrame.data[7]) << 24)
                    + (((uint32_t)rx_data.CanFrame.data[6]) << 16)
                    + (((uint32_t)rx_data.CanFrame.data[5]) << 8)
                    + ((uint32_t)rx_data.CanFrame.data[4]);
            f32_val = (float)(u32_val) / 1024.0f;
            this->ui->CAN_SDO_ChargeVoltageLimit_lineEdit->setText(QByteArray::number(f32_val));
            break;
        case IC650_SDO_CHARGE_CURRENT_LIMIT:
            u16_val = (((uint16_t)rx_data.CanFrame.data[5]) << 8)
                    + ((uint16_t)rx_data.CanFrame.data[4]);
            f32_val = (float)(u16_val) / 16.0f;
            this->ui->CAN_SDO_ChargeCurrentLimit_lineEdit->setText(QByteArray::number(f32_val));
            break;
        case IC650_SDO_CHARGER_TEMPERATURE:
            u8_val = (uint16_t)rx_data.CanFrame.data[4];
            f32_val = (float)u8_val;
            this->ui->CAN_SDO_ChargerTemperature_lineEdit->setText(QByteArray::number(f32_val));

        default:
            break;
    }
}

void MainWindow::UI_ProcessCanResponse(const QString &qs, long lMsgID, const CAN_DATA &rx_data)
{
//  QString qstr_msg = QByteArray::number(rx_data.usec_delta, 10) + " CAN Frame ID: " + QByteArray::number(rx_data.CanFrame.can_id, 16) + ", DLC: " + QByteArray::number(rx_data.CanFrame.can_dlc, 10) + "\n";
    QString qstr_msg = QByteArray::number(rx_data.usec_delta/1000000ll, 10) + "." +  QByteArray::number(rx_data.usec_delta%1000000ll, 10) + " CAN Frame ID: " + QByteArray::number(rx_data.CanFrame.can_id, 16) + ", DLC: " + QByteArray::number(rx_data.CanFrame.can_dlc, 10) + "\n";
//    printf("\nRx delta = %lld.%06lld\n", pCanData->usec_delta/1000000ll, pCanData->usec_delta%1000000ll);

    this->ui->CanTraffic_textEdit->insertPlainText(qstr_msg);
   // this->ui->CAN_Incoming_textEdit->moveCursor(QTextCursor::End);

//    QString qstr_data = "0x" + QByteArray::number(rx_data.CanFrame.data[0], 16);
    QString qstr_data = "0x" + QString("%1").arg(rx_data.CanFrame.data[0], 2, 16, QChar('0'));
    if (1 < rx_data.CanFrame.can_dlc)
    {
        for (int i=1; i<rx_data.CanFrame.can_dlc; i++)
            qstr_data += " 0x" + QString("%1").arg(rx_data.CanFrame.data[i], 2, 16, QChar('0'));
    }
    qstr_data += "\n";

  //  QString("%1").arg(rx_data.CanFrame.data[i], 2, 16, QChar('0'))

    this->ui->CanTraffic_textEdit->insertPlainText(qstr_data);
    this->ui->CanTraffic_textEdit->moveCursor(QTextCursor::End);

    qDebug("II.) %ld: CAN Frame ID: 0x%02X, DLC: %d", lMsgID, rx_data.CanFrame.can_id, rx_data.CanFrame.can_dlc);
}

void MainWindow::on_Heartbeat_pushButton_clicked()
{
    if (!m_HeatbeatActive)
    {
        m_UiTimer.start();
        this->ui->Heartbeat_pushButton->setText("Heartbeat Stop");
        m_HeatbeatActive = true;
    }
    else
    {
        m_UiTimer.stop();
        this->ui->Heartbeat_pushButton->setText("Heartbeat Start");
        m_HeatbeatActive = false;
    }
}

void MainWindow::on_NMT_pushButton_clicked()
{
    if (!m_NMT_Operational)
    {
        m_CANSocketThread.Send_NMT(IC650_NMT_START);
        this->ui->NMT_pushButton->setText("NMT-Stop");
        m_NMT_Operational = true;
    }
    else
    {
        m_CANSocketThread.Send_NMT(IC650_NMT_STOP);
        this->ui->NMT_pushButton->setText("NMT-Start");
        m_NMT_Operational = false;
    }
}

void MainWindow::on_CanThread_pushButton_clicked()
{
    if (!m_CanThreadActive)
    {
        m_CANSocketThread.RunProcessingThread(true);
        m_CanThreadActive = true;
        this->ui->CanThread_pushButton->setText("Stop CAN Thread");
    }
    else
    {
        m_CANSocketThread.RunProcessingThread(false);
        m_CanThreadActive = false;
        this->ui->CanThread_pushButton->setText("Start CAN Thread");
    }
}

void MainWindow::on_CAN_SDO_GetAll_pushButton_clicked()
{
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_AC_VOLT_X16, SUB_INDX_0);
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_DC_VOLT_Q8P8, SUB_INDX_0);
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_CHARGER_TEMPERATURE, SUB_INDX_0);
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_EXTENDED_STATUS, SUB_INDX_0);
}

void MainWindow::on_CAN_SDO_BatteryStatus_pushButton_clicked()
{
    this->ui->CAN_SDO_BatteryStatus_lineEdit->setText("----");
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_BATTERY_STATUS, SUB_INDX_0);
}

void MainWindow::on_CAN_SDO_VoltageLimit_pushButton_clicked()
{
    this->ui->CAN_SDO_ChargeVoltageLimit_lineEdit->setText("----");
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_VOLTAGE_LIMIT_REQ, SUB_INDX_0);
}

void MainWindow::on_CAN_SDO_CurrentLimit_pushButton_clicked()
{
    this->ui->CAN_SDO_ChargeCurrentLimit_lineEdit->setText("----");
    m_CANSocketThread.Send_ReadSDO (IC650_SDO_CHARGE_CURRENT_LIMIT, SUB_INDX_0);
}

void MainWindow::on_CAN_SDO_SetBatteryStatus_pushButton_clicked()
{
    uint8_t     data[8];
    uint16_t    u16_val;
    QString     qstr;

    qstr = this->ui->CAN_SDO_BatteryStatus_lineEdit->text();
    u16_val = qstr.toUShort();

    data[0] = (uint8_t)(u16_val & 0x00ff);
    m_CANSocketThread.Send_WriteSDO(IC650_SDO_BATTERY_STATUS, SUB_INDX_0, 1, data);
}

void MainWindow::on_CAN_SDO_SetVoltageLimit_pushButton_clicked()
{
    uint8_t     data[8];
    uint32_t    u32_val;
    float       f32_val;
    QString     qstr;

    qstr = this->ui->CAN_SDO_ChargeVoltageLimit_lineEdit->text();
    f32_val = qstr.toFloat() * 1024.0;
    u32_val = (uint32_t)f32_val;

    data[0] = (uint8_t)(u32_val & 0x000000ff);
    data[1] = (uint8_t)((u32_val & 0x0000ff00) >> 8);
    data[2] = (uint8_t)((u32_val & 0x00ff0000) >> 16);
    data[3] = (uint8_t)((u32_val & 0xff000000) >> 24);

    m_CANSocketThread.Send_WriteSDO(IC650_SDO_VOLTAGE_LIMIT_REQ, SUB_INDX_0, 4, data);
}

void MainWindow::on_CAN_SDO_SetCurrentLimit_pushButton_clicked()
{
    uint8_t     data[8];
    uint32_t    u16_val;
    float       f32_val;
    QString     qstr;

    qstr = this->ui->CAN_SDO_ChargeCurrentLimit_lineEdit->text();
    f32_val = qstr.toFloat() * 16.0;
    u16_val = (uint16_t)f32_val;

    data[0] = (uint8_t)(u16_val & 0x00ff);
    data[1] = (uint8_t)((u16_val & 0xff00) >> 8);

    m_CANSocketThread.Send_WriteSDO(IC650_SDO_CHARGE_CURRENT_LIMIT, SUB_INDX_0, 2, data);
}

void MainWindow::on_CAN_SDO_SetAll_pushButton_clicked()
{
    uint8_t     data[8];
    float       f32_val;
    uint8_t     u8_val;
    uint16_t    u16_val;
    uint32_t    u32_val;
    QString     qstr;

    qstr = this->ui->CAN_SDO_ChargeVoltageLimit_lineEdit->text();
    f32_val = qstr.toFloat() * 1024.0;
    u32_val = (uint32_t)f32_val;

    data[0] = (uint8_t)(u32_val & 0x000000ff);
    data[1] = (uint8_t)((u32_val & 0x0000ff00) >> 8);
    data[2] = (uint8_t)((u32_val & 0x00ff0000) >> 16);
    data[3] = (uint8_t)((u32_val & 0xff000000) >> 24);

    m_CANSocketThread.Send_WriteSDO(IC650_SDO_VOLTAGE_LIMIT_REQ, SUB_INDX_0, 4, data);

    QThread::usleep(5000);

    qstr = this->ui->CAN_SDO_ChargeCurrentLimit_lineEdit->text();
    f32_val = qstr.toFloat() * 16.0;
    u16_val = (uint16_t)f32_val;

    data[0] = (uint8_t)(u16_val & 0x00ff);
    data[1] = (uint8_t)((u16_val & 0xff00) >> 8);

    m_CANSocketThread.Send_WriteSDO(IC650_SDO_CHARGE_CURRENT_LIMIT, SUB_INDX_0, 2, data);

    QThread::usleep(5000);

    qstr = this->ui->CAN_SDO_BatteryStatus_lineEdit->text();
    u8_val = (uint8_t)qstr.toUInt();

    data[0] = u8_val;
    m_CANSocketThread.Send_WriteSDO(IC650_SDO_BATTERY_STATUS, SUB_INDX_0, 1, data);
}
