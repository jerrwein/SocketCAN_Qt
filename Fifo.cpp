#include <linux/can.h>
#include <linux/can/raw.h>
#include "canstructures.h"
#include "cansocketthread.h"
#include "Fifo.h"

//#define RX_FIFO_SIZE 12

FiFo::FiFo()
{
    short nIndex;

    m_IndexNextIn = 0;
    m_IndexNextOut = 0;
    for (nIndex=0; nIndex<RX_FIFO_SIZE; nIndex++)
    {
        m_CanData[nIndex].CanFrame.can_id = 0;
        m_CanData[nIndex].usec_delta = 0ll;
    }
    m_Status = 0;
}

FiFo::~FiFo()
{
    m_IndexNextIn = 0;
    m_IndexNextOut = 0;
}

int FiFo::IsDataAvailable(void)
{
    if (m_IndexNextIn != m_IndexNextOut)
        return 1;
    else
        return 0;
}

int FiFo::HowManyElementsAvailable(void)
{
    if (m_IndexNextOut <= m_IndexNextIn)
        return (m_IndexNextIn - m_IndexNextOut);
    else
        return (RX_FIFO_SIZE - (m_IndexNextOut - m_IndexNextIn));
}

int FiFo::InsertElement(CAN_DATA *pCanData)
{
    short nIndexNextIn;

    /* Are we about to crush existing data? */
    nIndexNextIn = (m_IndexNextIn + 1) % RX_FIFO_SIZE;
    if (nIndexNextIn != m_IndexNextOut)
    {
        m_CanData[m_IndexNextIn] = *pCanData;
        m_IndexNextIn = (unsigned char)nIndexNextIn;
        return 1;
    }
    else
    {
        /* Fifo fill failure status bit */
        // FIFO_TxCommStatus |= 0x01;
        return 0;
    }
}

int FiFo::InsertData(CAN_DATA CanData[], int nLen)
{
    int n, nRes;
    for (n=0; n<nLen; n++)
    {
        nRes = InsertElement (&CanData[n]);
        if (1 != nRes)
        {
            /* Fifo fill failure status bit */
            // IFO_TxCommStatus |= 0x01;
            return n;
        }
    }
    return nLen;
}

int FiFo::GetElement(CAN_DATA *pCanData)
{
    if (m_IndexNextIn != m_IndexNextOut)
    {
        *pCanData = m_CanData[m_IndexNextOut];
        m_IndexNextOut = (unsigned char)((m_IndexNextOut+1) % RX_FIFO_SIZE);
        return 1;
    }
    else
    {
        /* Nothing to give */
        return 0;
    }
}

