#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <linux/can.h>
#include "canstructures.h"
#include "cansocketthread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_TimerTick(void);

    void UI_ShowGeneralStatus(const QString &qstrReport);
    void UI_ProcessCanResponse(const QString &qs, long lMsgID, const CAN_DATA &rx_data);
    void UI_ProcessCanResponse_PDO(long cob_id, const CAN_DATA &rx_data);
    void UI_ProcessCanResponse_SDO(int sdo_index, const CAN_DATA &rx_data);

    void on_Heartbeat_pushButton_clicked();
    void on_NMT_pushButton_clicked();
    void on_CanThread_pushButton_clicked();


    void on_CAN_SDO_GetAll_pushButton_clicked();

    void on_CAN_SDO_BatteryStatus_pushButton_clicked();

    void on_CAN_SDO_VoltageLimit_pushButton_clicked();

    void on_CAN_SDO_CurrentLimit_pushButton_clicked();

    void on_CAN_SDO_SetBatteryStatus_pushButton_clicked();

    void on_CAN_SDO_SetVoltageLimit_pushButton_clicked();

    void on_CAN_SDO_SetCurrentLimit_pushButton_clicked();

    void on_CAN_SDO_SetAll_pushButton_clicked();

private:
    Ui::MainWindow  *ui;
     QTimer         m_UiTimer;

    bool    m_CanThreadActive;
    bool    m_HeatbeatActive;
    bool    m_NMT_Operational;

    CANSocketThread m_CANSocketThread;
};

#endif // MAINWINDOW_H
