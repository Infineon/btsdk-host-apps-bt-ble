/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*
 * Sample MCU application for using WICED HCI protocol. Common main app.
 */

#include <QCloseEvent>
#include "hci_control_api.h"
#include <QDateTime>
#include <QFileDialog>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>

extern void TraceHciPkt(BYTE type, BYTE *buffer, USHORT length, USHORT serial_port_index,int iSpyInstance);

extern "C"
{
void app_host_init();

int app_host_port_write(uint8_t *data, uint32_t len)
{
    return g_pMainWindow->PortWrite(data, len);
}

void app_host_log(const char * fmt, ...)
{
    va_list         vargs;
    char            log_buffer[1024];

    va_start(vargs, fmt);
    vsprintf(log_buffer, fmt, vargs);
    va_end(vargs);

    g_pMainWindow->Log(log_buffer);
}

}

MainWindow *g_pMainWindow = NULL ;
bool m_bClosing = false;

class EventFilter : public QObject
{
protected:
    bool eventFilter(QObject *obj, QEvent *event) override;
};

bool EventFilter::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == g_pMainWindow->ui->tabDualA2DP && event->type() == QEvent::EnabledChange)
    {
        if (g_pMainWindow->ui->tabDualA2DP->isEnabled())
        {
            g_pMainWindow->m_audio_format = (g_pMainWindow->m_settings.value("AudioSrcFormatMp3DualA2DP", true).toBool()) ? 1 : 0;
        }
        else
        {
            g_pMainWindow->m_audio_format = (g_pMainWindow->m_settings.value("AudioSrcFormatMp3", true).toBool()) ? 1 : 0;
        }
    }
    return true;
}

// Initialize UI and variables
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    m_paired_icon(":/paired.png"),
    m_settings("clientcontrol.ini",QSettings::IniFormat),
    m_fp_logfile(NULL),
    scripting(false),
    ui(new Ui::MainWindow)
{
    iSpyInstance = 0;
    app_host_init();
    ui->setupUi(this);

    qApp->setStyleSheet("QGroupBox {  border: 1px solid gray;}");

    g_pMainWindow = this;

    connect(ui->btnClear, SIGNAL(clicked()), this, SLOT(btnClearClicked()));
    connect(ui->btnFindLogFile, SIGNAL(clicked()), this, SLOT(btnFindLogfileClicked()));
    connect(ui->btnLogToFile, SIGNAL(clicked(bool)), this, SLOT(btnLogToFileClicked(bool)));
    connect(ui->btnAddTrace, SIGNAL(clicked()), this, SLOT(btnAddTraceClicked()));

    connect(this, SIGNAL(HandleWicedEvent(unsigned int,unsigned int,unsigned char*)), this, SLOT(onHandleWicedEvent(unsigned int,unsigned int ,unsigned char*)), Qt::QueuedConnection);
    connect(this, SIGNAL(HandleTrace(QString*)), this, SLOT(processTrace(QString*)), Qt::QueuedConnection);
    connect(this, SIGNAL(ScrollToTop()), this, SLOT(processScrollToTop()), Qt::QueuedConnection);
    connect(this, SIGNAL(ListClear()), this, SLOT(processClear()), Qt::QueuedConnection);

    qApp->setStyleSheet("QGroupBox {border: 1px solid gray; border-radius: 9px; margin-top: 0.5em;}");
    qApp->setStyleSheet("QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px;}");

    QIcon icon;
    icon.addFile(":/wiced.png");
    if(!icon.isNull())
    {
        QApplication::setWindowIcon(icon);
    }

    // Initialize each profile/feature tab
    InitDm();
    InitAudioSrc();
    InitAudioSrc_DualA2DP();
    InitAVRCTG();
    InitAudioSnk();
    InitAVRCCT();
    InitAG();
    InitHF();
    InitSPP();
    InitHIDH();
    InitBLEHIDD();
    InitGATT();
    InitHK();
    InitiAP2();
    InitBSG();
    InitPBC();
    InitBATTC();
    InitFINDMEL();
    InitDemo();
    InitOPS();
    InitGATT_DB();
    InitANP();
    InitLecoc();
    InitLED_Demo();
    InitOTPClient();
    InitMAPClient();

    ListClear();

    Log("Instructions:");
    Log("1.  Plug the WICED Evaluation Board into the computer using a USB cable.");
    Log("2.  Build and download an embedded application to the WICED evaluation board.");
    Log("3.  Select the serial (COM) port for the WICED Evaluation Board and open the port.");
    Log("    This is usually enumerated 'WICED HCI UART' on Windows or Linux PCs.");
    Log("    The UI will be enabled when the Client Control app is able to communicate with the embedded BT app.");
    Log("4.  For more information on application tabs, select a tab and click on the help (?) icon.");

    ScrollToTop();

    // Tab index 18 and higher are not used currently, remove then from UI
    for(int i = 0; i < 8; i++)
        ui->tabMain->removeTab(18);

    EventFilter *evtFilter = new EventFilter();
    ui->tabDualA2DP->installEventFilter(evtFilter);
}

MainWindow::~MainWindow()
{
    delete ui;
}

// indication of app shutdown
void MainWindow::closeEvent (QCloseEvent *event)
{
    closeEventDm(event);
    event->accept();
}

void MainWindow::showEvent(QShowEvent *ev)
{
    QMainWindow::showEvent(ev);
    QTimer::singleShot(500, this, SLOT(window_shown()));
}

// When window is shown, if COM port or baud rate is specified in command line,
// use it to open the port
void MainWindow::window_shown()
{
    if(!str_cmd_port.isEmpty() && !str_cmd_baud.isEmpty())
    {
        on_btnOpenPort_clicked();
    }

    if (scripting)
        CreateScriptThread();
}

// Handle WICED HCI events
void MainWindow::onHandleWicedEvent(unsigned int opcode, unsigned int len, unsigned char *p_data)
{
    // send event to modules, DM should be first
    onHandleWicedEventDm(opcode,p_data,len);
    onHandleWicedEventAudioSrc(opcode, p_data, len);
    onHandleWicedEventAudioSrc_DualA2DP(opcode, p_data, len);
    onHandleWicedEventHF(opcode, p_data, len);
    onHandleWicedEventSPP(opcode, p_data, len);
    onHandleWicedEventAG(opcode, p_data, len);
    onHandleWicedEventBLEHIDD(opcode, p_data, len);
    onHandleWicedEventHIDH(opcode, p_data, len);
    onHandleWicedEventAVRCCT(opcode, p_data, len);
    onHandleWicedEventAVRCTG(opcode, p_data, len);
    onHandleWicedEventHK(opcode, p_data, len);
    onHandleWicedEventiAP2(opcode, p_data, len);
    onHandleWicedEventGATT(opcode, p_data, len);
    onHandleWicedEventAudioSnk(opcode, p_data, len);
    onHandleWicedEventBSG(opcode, p_data, len);
    onHandleWicedEventPBC(opcode, p_data, len);
    onHandleWicedEventBATTC(opcode, p_data, len);
    onHandleWicedEventFINDMEL(opcode, p_data, len);
    onHandleWicedEventDemo(opcode, p_data, len);
    onHandleWicedEventOPS(opcode, p_data, len);
    onHandleWicedEventGATT_DB(opcode, p_data, len);
    onHandleWicedEventANP(opcode, p_data, len);
    onHandleWicedEventLECOC(opcode, p_data, len);
    onHandleWicedEventOTPClient(opcode, p_data, len);
    onHandleWicedEventMAPClient(opcode, p_data, len);
    onHandleWicedEventHciDfu(opcode, p_data, len);
    // free event data, allocated in Dm module when event arrives
    if (p_data)
        free(p_data);
}

/*************** Tracing UI ************************/

// Clear traces
void MainWindow::onClear()
{
    ui->lstTrace->clear();
}

// add trace to window
void MainWindow::processTrace(QString * trace)
{
    // Keep a max of 50 lines of traces in windows, otherwise
    // it slows down the rendering.
    if(ui->lstTrace->count() > 50)
    {
        QListWidgetItem *pRemove = ui->lstTrace->takeItem(0);
        delete pRemove;
    }

    ui->lstTrace->addItem(*trace);
    ui->lstTrace->scrollToBottom();
    ui->lstTrace->scrollToItem(ui->lstTrace->item( ui->lstTrace->count()));

    if (ui->btnLogToFile->isChecked() && m_fp_logfile)
    {
        fprintf(m_fp_logfile, "%s\n", trace->toStdString().c_str());
        fflush(m_fp_logfile);
    }

    delete trace;
}

void MainWindow::processScrollToTop()
{
    ui->lstTrace->scrollToTop();
}

void MainWindow::processClear()
{
    ui->lstTrace->clear();
}

// common Log method to send traces to app UI
void MainWindow::Log(const char * fmt, ...)
{
    va_list cur_arg;
    va_start(cur_arg, fmt);
    char trace[1000];
    memset(trace, 0, sizeof(trace));
    vsprintf(trace, fmt, cur_arg);

    // send to spy before the time stamp
    TraceHciPkt(0, (BYTE *)trace, strlen(trace), 0, iSpyInstance);

    QString s = QDateTime::currentDateTime().toString("MM-dd-yyyy hh:mm:ss.zzz: ") + trace;
    va_end(cur_arg);

    // add trace to file and UI screen in UI thread
    emit HandleTrace(new QString(s));
}

// User button to clear traces
void MainWindow::btnClearClicked()
{
    ui->lstTrace->clear();
}

// Log traces to file
void MainWindow::btnFindLogfileClicked()
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("Log File"), "", tr("All Files (*.*)"));
    ui->edLogFile->setText(fileName);
}

// Log to file
void MainWindow::btnLogToFileClicked(bool checked)
{
    if (checked)
    {
        if (NULL == (m_fp_logfile = fopen(ui->edLogFile->text().toStdString().c_str(), "w")))
        {
            Log("Error opening logfile: %d", 0/*errno*/);
            ui->btnLogToFile->setChecked(false);
        }
    }
    else
    {
        // stop logging to file
        if (m_fp_logfile)
        {
            fclose(m_fp_logfile);
            m_fp_logfile = NULL;
        }
    }
}

// Add custom trace to app traces
void MainWindow::btnAddTraceClicked()
{
    Log(ui->edTrace->text().toStdString().c_str());
}

/******************** Utility functions ***********************/
USHORT MainWindow::GetHandle(QString &str)
{
    BYTE buf[2];
    int num_digits = GetHexValue(buf, 2, str);
    if (num_digits == 2)
        return (buf[0] << 8) + buf[1];
    else
        return buf[0];
}

DWORD MainWindow::GetHexValue(LPBYTE buf, DWORD buf_size, QString &str)
{
    char szbuf[100];
    char *psz = szbuf;
    BYTE *pbuf = buf;
    DWORD res = 0;

    memset(buf, 0, buf_size);

    strncpy(szbuf, str.toStdString().c_str(), 100);

    if (strlen(szbuf) == 1)
    {
        szbuf[2] = 0;
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    else if (strlen(szbuf) == 3)
    {
        szbuf[4] = 0;
        szbuf[3] = szbuf[2];
        szbuf[2] = szbuf[1];
        szbuf[1] = szbuf[0];
        szbuf[0] = '0';
    }
    for (DWORD i = 0; i < strlen(szbuf); i++)
    {
        if (isxdigit(psz[i]) && isxdigit(psz[i + 1]))
        {
            *pbuf++ = (ProcNibble(psz[i]) << 4) + ProcNibble(psz[i + 1]);
            res++;
            i++;
        }
    }
    return res;
}

BYTE MainWindow::ProcNibble (char n)
{
    if ((n >= '0') && (n <= '9'))
    {
        n -= '0';
    }
    else if ((n >= 'A') && (n <= 'F'))
    {
        n = ((n - 'A') + 10);
    }
    else if ((n >= 'a') && (n <= 'f'))
    {
        n = ((n - 'a') + 10);
    }
    else
    {
        n = (char)0xff;
    }
    return (n);
}

void GetTagDesc(char *desc, BYTE tag)
{
    switch (tag)
    {
    case 0x01: sprintf(&desc[strlen(desc)], "Flags"); break;
    case 0x02: sprintf(&desc[strlen(desc)], "MORE UUID16"); break;
    case 0x03: sprintf(&desc[strlen(desc)], "UUID16"); break;
    case 0x04: sprintf(&desc[strlen(desc)], "MORE UUID32"); break;
    case 0x05: sprintf(&desc[strlen(desc)], "UUID32"); break;
    case 0x06: sprintf(&desc[strlen(desc)], "MORE UUID128"); break;
    case 0x07: sprintf(&desc[strlen(desc)], "UUID128"); break;
    case 0x08: sprintf(&desc[strlen(desc)], "Name(Short)"); break;
    case 0x09: sprintf(&desc[strlen(desc)], "Name"); break;
    case 0x0A: sprintf(&desc[strlen(desc)], "TxPower"); break;
    case 0x0C: sprintf(&desc[strlen(desc)], "BdAddr"); break;
    case 0x0D: sprintf(&desc[strlen(desc)], "COD"); break;
    case 0xFF: sprintf(&desc[strlen(desc)], "Manufacturer"); break;
    default: sprintf(&desc[strlen(desc)], "(0x%02X)", tag); break;
    }
}


DWORD MainWindow::qtmin(DWORD len, DWORD bufLen)
{
    return (len < bufLen) ? len : bufLen;
}

void MainWindow::DumpData(char *description, void* p, unsigned int length, unsigned int max_lines)
{
    char    buff[100];
    unsigned int    i, j;
    char    full_buff[3000];

    if (p != NULL)
    {
        for (j = 0; j < max_lines && (32 * j) < length; j++)
        {
            for (i = 0; (i < 32) && ((i + (32 * j)) < length); i++)
            {
                sprintf(&buff[3 * i], "%02x \n", ((UINT8*)p)[i + (j * 32)]);
            }
            if (j == 0)
            {
                strncpy(full_buff, description, 3000-1);
                strcat(full_buff, buff);
                //qDebug(full_buff);
            }
            else
            {
                //qDebug(buff);
            }
        }
    }
}


// returns device name if present in data
void DecodeEIR_Hostmode(LPBYTE p_data, DWORD len, char * szName, int name_len)
{
    BYTE tag;
    int tag_len=0;
    int data_len = (int)len;

    memset(szName, 0, name_len);
 //   p_data++;
    while (data_len >= 2)
    {
        tag_len = (int)(unsigned int)*p_data++;
        tag = *p_data++;
        if (tag == 8 || tag == 9)
        {
            if (tag_len < name_len-1)
            {
                strncpy(szName, (char*)p_data, tag_len-1);
                if (g_pMainWindow)
                    g_pMainWindow->Log("name = %s", szName);
            }
            return;
        }
        p_data += tag_len - 1;
        data_len -= tag_len + 1;
    }
}

void MainWindow::DecodeEIR(LPBYTE p_data, DWORD len, char * szName, int name_len)
{
    char trace[1024] = {0};
    //static char bd_name[100]={0};
    BYTE tag;
    int i, tag_len;
    int data_len = (int)len;

    //szName[0]=0;
    memset(szName, 0, name_len);
    while (data_len >= 2)
    {
        //trace[0] = '\0';
        memset(trace, 0, 1024);

        tag_len = (int)(unsigned int)*p_data++;
        tag = *p_data++;
        sprintf(trace, " -Tag:");
        GetTagDesc(trace, tag);
        sprintf(&trace[strlen(trace)], ":");
        switch (tag)
        {
        case 0x02: // UUID16 More
        case 0x03: // UUID16
            unsigned short uuid16;

            for (i = 0; i < tag_len - 1; i += 2)
            {
                uuid16 = *p_data++;
                uuid16 |= (*p_data++ << 8);
                sprintf(&trace[strlen(trace)], "%04X ", (uint32_t) uuid16);
            }
            break;

        case 0x04: // UUID32 More
        case 0x05: // UUID32

            unsigned long uuid32;
            for (i = 0; i < tag_len - 1; i += 2)
            {
                uuid32 = *(char *)p_data++;
                uuid32 |= (*(char *)p_data++ << 8);
                uuid32 |= (*(char *)p_data++ << 16);
                uuid32 |= (*(char *)p_data++ << 24);
                sprintf(&trace[strlen(trace)], "%08X ", (uint32_t) uuid32);
            }
            break;

        case 0x08: // Shortened name
        case 0x09: // Name


            for (i = 0; i < tag_len - 1; i++)
            {
                if (i < (name_len-1))
                    szName[i] = *p_data;
                sprintf(&trace[strlen(trace)], "%c", *p_data++);
            }
            break;

        default:


            for (i = 0; i < tag_len - 1; i++)
                sprintf(&trace[strlen(trace)], "%02X ", *p_data++);
            break;
        }
        data_len -= tag_len + 1;

        Log(trace);
    }
}

void MainWindow::DumpMemory(BYTE * p_buf, int length)
{
    char trace[1024];
    int i;

    memset(trace, 0, sizeof(trace));
    for (i = 0; i < length; i++)
    {
        sprintf(&trace[strlen(trace)], "%02X ", p_buf[i + 3]);
        if (i && (i % 32) == 0)
        {
            Log(trace);
            memset(trace, 0, sizeof(trace));
        }
    }
    if (i % 32)
    {
        Log(trace);
    }
}

#ifdef Q_OS_MAC
void MainWindow::HandleHidHAudioStart(LPBYTE p_data, DWORD len) {}
void MainWindow::HandleHidHAudioStop(LPBYTE p_data, DWORD len) {}
void MainWindow::HandleHidHAudioRxData(LPBYTE p_data, DWORD len) {}
#endif

void MainWindow::on_btnHelpTab_clicked()
{
    if(ui->tabAG->isVisible())
        on_btnHelpAG_clicked();

    if(ui->tabHF->isVisible())
        on_btnHelpHF_clicked();

    if(ui->tabAVSRC->isVisible())
        on_btnHelpAVSRC_clicked();

    if(ui->tabAVRCCT->isVisible())
        on_btnHelpAVRC_CT_clicked();

    if(ui->tabAVRCTG->isVisible())
        on_btnHelpAVRC_TG_clicked();

    if(ui->tabHIDD->isVisible())
        on_btnHelpHIDD_clicked();

    if(ui->tabHIDH->isVisible())
        on_btnHelpHIDH_clicked();

    if(ui->tabSPP->isVisible())
        on_btnHelpSPP_clicked();

    if(ui->tabIAP2->isVisible())
        on_btnHelpIAP2_clicked();

    if(ui->tabHK->isVisible())
        on_btnHelpHK_clicked();

    if(ui->tabGATT->isVisible())
        on_btnHelpGATT_clicked();

    if(ui->tabAVSink->isVisible())
        on_btnHelpAVK_clicked();

    if(ui->tabBSG->isVisible())
        on_btnHelpBSG_clicked();

    if(ui->tabPBC->isVisible())
        on_btnHelpPBC_clicked();

    if(ui->tabGATT_DB->isVisible())
        on_btnHelpGATT_DB_clicked();

    if(ui->tabBATTC->isVisible())
        on_btnHelpBATTC_clicked();

    if(ui->tabFINDMEL->isVisible())
        on_btnHelpFindMe_clicked();

    if(ui->tabOPS->isVisible())
        on_btnHelpOPPS_clicked();

    if(ui->tabDemo->isVisible())
        on_btnHelpDemo_clicked();

    if(ui->tabAlertNotfn->isVisible())
        on_btnHelpANP_clicked();
}


void MainWindow::on_cbBLEHIDDebug_currentIndexChanged(int index)
{
    (void) index;
    Log("Route debug message to %s", ui->cbBLEHIDDebug->currentText().toStdString().c_str());
    EnableAppTraces();
}
