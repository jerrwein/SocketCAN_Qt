#ifndef FIFO_H
#define FIFO_H

#include "cansocketthread.h"

#define RX_FIFO_SIZE 12

class FiFo
{
public:
    FiFo();
    ~FiFo();

public:
    int 	IsDataAvailable(void);
    int 	HowManyElementsAvailable(void);
    int 	InsertElement(CAN_DATA *pCanData);
    int 	InsertData(CAN_DATA CanData[], int nLen);
    int 	GetElement(CAN_DATA *pCanData);

private:
    unsigned char   m_IndexNextIn;
    unsigned char   m_IndexNextOut;
    unsigned int	m_Status;
    CAN_DATA        m_CanData[RX_FIFO_SIZE];
};

Q_DECLARE_METATYPE(FiFo);

#endif // FIFO_H
