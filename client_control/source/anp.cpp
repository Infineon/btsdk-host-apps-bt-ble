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
 * Sample MCU application for Alert Notification profile using WICED HCI protocol.
 */


#include "app_include.h"
extern "C"
{
#include "app_host.h"
}

typedef struct
{
    uint8_t type;
    const char * name;
} ans_types;

// All alert categories
ans_types types [] =
{
    {ANP_ALERT_CATEGORY_ID_SIMPLE_ALERT , "Simple Alert" },
    {ANP_ALERT_CATEGORY_ID_EMAIL , "Email" },
    {ANP_ALERT_CATEGORY_ID_NEWS, "News"},
    {ANP_ALERT_CATEGORY_ID_NEWS, "Call"},
    {ANP_ALERT_CATEGORY_ID_MISSED_CALL, "Missed Call"},
    {ANP_ALERT_CATEGORY_ID_SMS_OR_MMS, "SMS or MMS"},
    {ANP_ALERT_CATEGORY_ID_VOICE_MAIL, "Voice Mail"},
    {ANP_ALERT_CATEGORY_ID_SCHEDULE_ALERT, "Schedule Alert"},
    {ANP_ALERT_CATEGORY_ID_HIGH_PRI_ALERT, "High Priority Alert"},
    {ANP_ALERT_CATEGORY_ID_INSTANT_MESSAGE, "Instant Message"}
};

// New Alert commands
static ans_types commandsNewAlerts [3] =
{
    {ANP_ALERT_CONTROL_CMD_ENABLE_NEW_ALERTS , "Enable New Alerts" },
    {ANP_ALERT_CONTROL_CMD_DISABLE_NEW_ALERTS, "Disable New Alerts"},
    {ANP_ALERT_CONTROL_CMD_NOTIFY_NEW_ALERTS_IMMEDIATE, "Notify on New Alerts"},
};

// Unread Alert commands
static ans_types commandsUnreadAlerts [3] =
{
    {ANP_ALERT_CONTROL_CMD_ENABLE_UNREAD_STATUS , "Enable Unread Alerts" },
    {ANP_ALERT_CONTROL_CMD_DISABLE_UNREAD_ALERTS, "Disable Unread Alerts"},
    {ANP_ALERT_CONTROL_CMD_NOTIFY_UNREAD_ALERTS_IMMEDIATE, "Notify on Unread Alerts"},
};

// Initialize app
void MainWindow::InitANP(void)
{
    /************ ANS **************************/
    // Create a list box of all alert categories. Initially uncheck and disable all items in list
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i ++ )
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setData(0, types[i].type);
        item->setText(types[i].name);
        item->setCheckState(Qt::Unchecked);
        Qt::ItemFlags flag = item->flags() | Qt::ItemIsUserCheckable;
        flag &= ~Qt::ItemIsEnabled;
        item->setFlags(flag);
        ui->listAlertCategories->insertItem(i, item);
    }

    // Disable UI buttons
    // ANS
    ui->btnANSSetAlert->setEnabled(false);
    ui->btnGenerateAlert->setEnabled(false);
    ui->btnCancelAlert->setEnabled(false);
    ui->radioANSSetUnreadAlerts->setEnabled(false);
    ui->radioANSSetNewAlerts->setEnabled(false);


    /************ ANC **************************/
    // Create a list box of all alert categories. Initially uncheck and disable all items in list
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i ++ )
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setData(0, types[i].type);
        item->setText(types[i].name);
        item->setCheckState(Qt::Unchecked);
        Qt::ItemFlags flag = item->flags() | Qt::ItemIsUserCheckable;
        flag &= ~Qt::ItemIsEnabled;
        item->setFlags(flag);
        ui->listAlertCategoriesANC->insertItem(i, item);
    }

    // Create a list box of all alert categories. Initially uncheck and disable all items in list
    for(int i = 0; i < 3; i ++ )
    {
        ui->cbANCControlAlertType->insertItem(i, commandsNewAlerts[i].name, commandsNewAlerts[i].type);
    }

    // ANC
    ui->btnACSReadAlerts->setEnabled(false);
    ui->btnANCControlAlerts->setEnabled(false);
    ui->btnANCEnableNewAlerts->setEnabled(false);
    ui->btnANCUnreadAlerts->setEnabled(false);
    ui->radioANCReadNewAlerts->setEnabled(false);
    ui->radioANCReadUnreadAlerts->setEnabled(false);
    ui->cbANCControlAlertType->setEnabled(false);
}


// Handle WICED HCI events
void MainWindow::onHandleWicedEventANP(unsigned int opcode, unsigned char *p_data, unsigned int len)
{
    switch (HCI_CONTROL_GROUP(opcode))
    {
    case HCI_CONTROL_GROUP_ANC: // client
        HandleANCEvents(opcode, p_data, len);
        break;
    case HCI_CONTROL_GROUP_ANS: // server
        HandleANSEvents(opcode, p_data, len);
        break;
    }
}

/********************ANS (service)***************************/

// Handle WICED HCI events for ANS
void MainWindow::HandleANSEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{

    app_host_ans_event(opcode, p_data, len);

    switch (opcode)
    {
    // app startup
    case HCI_CONTROL_ANS_EVENT_ANS_ENABLED:
    {
        // Emable list box with all supported categories
        for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
        {
            QListWidgetItem *item = ui->listAlertCategories->item(i);
            uint16_t category = 1 << types[i].type;
            if(g_app.m_ans_supported_alerts & category)
            {
                Qt::ItemFlags flag = item->flags() | Qt::ItemIsEnabled;
                item->setFlags(flag);
                item->setCheckState(Qt::Checked);
            }
        }
        ui->btnANSSetAlert->setEnabled(true);
        ui->radioANSSetUnreadAlerts->setEnabled(true);
        ui->radioANSSetNewAlerts->setEnabled(true);

    }
        break;

    case HCI_CONTROL_ANS_EVENT_CONNECTION_UP:
    {
        // On connection up, disable set alert buttons
        if(g_app.m_ans_connected)
        {
            ui->btnANSSetAlert->setEnabled(false);

        }
    }

        break;
    case HCI_CONTROL_ANS_EVENT_CONNECTION_DOWN:
    {
        // On connection down, enable Set alert buttons
        // Disable Generate Alert and Clear Alert buttons
        if(!g_app.m_ans_connected)
        {
            ui->btnANSSetAlert->setEnabled(true);

            ui->btnGenerateAlert->setEnabled(false);
            ui->btnCancelAlert->setEnabled(false);
        }
    }

        break;

    case HCI_CONTROL_ANS_EVENT_COMMAND_STATUS:

        break;
    }
}

// On new alert radio button, check supported new alert categories
void MainWindow::on_radioANSSetNewAlerts_clicked()
{

    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        QListWidgetItem *item = ui->listAlertCategories->item(i);
        uint16_t category = 1 << types[i].type;
        if(g_app.m_ans_supported_new_alerts & category)
        {
            item->setCheckState(Qt::Checked);
        }
        else if(g_app.m_ans_supported_alerts & category)
        {
            item->setCheckState(Qt::Unchecked);

        }
    }
}

// On unread alert radio button, check supported unread alert categories
void MainWindow::on_radioANSSetUnreadAlerts_clicked()
{
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        QListWidgetItem *item = ui->listAlertCategories->item(i);
        uint16_t category = 1 << types[i].type;
        if(g_app.m_ans_supported_unread_alerts & category)
        {
            item->setCheckState(Qt::Checked);
        }
        else if(g_app.m_ans_supported_alerts & category)
        {
            item->setCheckState(Qt::Unchecked);
        }
    }

}


// When user clicks Set Alert button, find the checked items categories
// for either new or unread alerts and set the alert for new or underad
// depending on radio box selected.
void MainWindow::on_btnANSSetAlert_clicked()
{
    uint16_t updated = 0;
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        uint16_t category = 1 << types[i].type;
        if(g_app.m_ans_supported_alerts & category)
        {
            QListWidgetItem *item = ui->listAlertCategories->item(i);
            if(item->checkState() == Qt::Checked)
            {
                updated |= category;
            }
        }
    }

    if(ui->radioANSSetNewAlerts->isChecked())
    {
        g_app.m_ans_supported_new_alerts = updated;
        wiced_hci_bt_anp_alert_data_t data;
        data.alert = g_app.m_ans_supported_new_alerts;
        wiced_hci_ans_command_set_supported_new_alert_category(&data);
    }
    else
    {
        g_app.m_ans_supported_unread_alerts = updated;
        wiced_hci_bt_anp_alert_data_t data;
        data.alert = g_app.m_ans_supported_unread_alerts;
        wiced_hci_ans_command_set_supported_unread_alert_category(&data);

    }
}

// When user clicks Generate Alert button, find the allowed categories,
// find among them the one that is selected and send alert
void MainWindow::on_btnGenerateAlert_clicked()
{
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        uint8_t category = 1 << types[i].type;
        if(g_app.m_ans_supported_alerts & category)
        {
            QListWidgetItem *item = ui->listAlertCategories->item(i);
            if(item->isSelected())
            {
                wiced_hci_bt_anp_alert_category_data_t data;
                data.alert_category = types[i].type;
                wiced_hci_ans_command_generate_alert(&data);
            }
        }
    }

}

// When user clicks Clear Alert button, find the allowed categories,
// find among them the one that is selected and send cancel alert
void MainWindow::on_btnCancelAlert_clicked()
{
    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        uint8_t category = 1 << types[i].type;
        if(g_app.m_ans_supported_alerts & category)
        {
            QListWidgetItem *item = ui->listAlertCategories->item(i);
            if(item->isSelected())
            {
                wiced_hci_bt_anp_alert_category_data_t data;
                data.alert_category = types[i].type;
                wiced_hci_ans_command_clear_alert(&data);
            }
        }
    }
}

// When user selects a item in list, first diable Generate and Cancel alert buttons
// If the selected item is enabled (i.e. allowed by embedded app), and checked (i.e. suuuported by server),
// then enable the buttons if ANS is connected
void MainWindow::on_listAlertCategories_itemClicked(QListWidgetItem *item)
{
    ui->btnGenerateAlert->setEnabled(false);
    ui->btnCancelAlert->setEnabled(false);

    if(item->flags() & Qt::ItemIsEnabled)
    {
        if(item->checkState() == Qt::Checked)
        {
            if(g_app.m_ans_connected)
            {
                ui->btnGenerateAlert->setEnabled(true);
                ui->btnCancelAlert->setEnabled(true);
            }
        }
    }
}


/********************ANC (client)***************************/

// Handle WICED HCI events for ANC Client
void MainWindow::HandleANCEvents(DWORD opcode, LPBYTE p_data, DWORD len)
{
    app_host_anc_event(opcode, p_data, len);

    switch (opcode)
    {
    // on HCI_CONTROL_ANC_EVENT_ANC_ENABLED if server
    // is connected, enable app UI
    case HCI_CONTROL_ANC_EVENT_ANC_ENABLED:
        if(g_app.m_anc_connected)
        {
            ui->btnACSReadAlerts->setEnabled(true);
            ui->btnANCControlAlerts->setEnabled(true);
            ui->btnANCEnableNewAlerts->setEnabled(true);
            ui->btnANCUnreadAlerts->setEnabled(true);
            ui->radioANCReadNewAlerts->setEnabled(true);
            ui->radioANCReadUnreadAlerts->setEnabled(true);
            ui->cbANCControlAlertType->setEnabled(true);
        }
        break;

        // on HCI_CONTROL_ANC_EVENT_ANC_DISABLED if server
        // is connected, disable app UI
    case HCI_CONTROL_ANC_EVENT_ANC_DISABLED:
        if(!g_app.m_anc_connected)
        {
            ui->btnACSReadAlerts->setEnabled(false);
            ui->btnANCControlAlerts->setEnabled(false);
            ui->btnANCEnableNewAlerts->setEnabled(false);
            ui->btnANCUnreadAlerts->setEnabled(false);
            ui->radioANCReadNewAlerts->setEnabled(false);
            ui->radioANCReadUnreadAlerts->setEnabled(false);
            ui->cbANCControlAlertType->setEnabled(false);
            g_app.m_anc_selected_alerts = 0;
        }
        break;

        // Change the button text depending on Alert status
     case HCI_CONTROL_ANC_EVENT_ENABLE_NEW_ALERTS:
     case HCI_CONTROL_ANC_EVENT_DISABLE_NEW_ALERTS:
     case HCI_CONTROL_ANC_EVENT_ENABLE_UNREAD_ALERTS:
     case HCI_CONTROL_ANC_EVENT_DISABLE_UNREAD_ALERTS:
        {
            if(g_app.m_b_new_alerts_enabled)
                ui->btnANCEnableNewAlerts->setText("Disable New Alerts");
            else
                ui->btnANCEnableNewAlerts->setText("Enable New Alerts");

            if(g_app.m_b_unread_alerts_enabled)
                ui->btnANCUnreadAlerts->setText("Disable Unread Alerts");
            else
                ui->btnANCUnreadAlerts->setText("Enable Unread Alerts");
        }
        [[fallthrough]];
        // On HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_NEW_ALERTS
        // populate the list box with supported new alerts
    case HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_NEW_ALERTS:
        {
            if(ui->radioANCReadNewAlerts->isChecked())
                on_radioANCReadNewAlerts_clicked();
        }
        [[fallthrough]];
        // On HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_UNREAD_ALERTS
        // populate the list box with supported unread alerts
    case HCI_CONTROL_ANC_EVENT_SERVER_SUPPORTED_UNREAD_ALERTS:
        {
            if(ui->radioANCReadUnreadAlerts->isChecked())
                on_radioANCReadUnreadAlerts_clicked();
        }
        break;


    }

}

// When radio button for new alerts is clicked, check
// the items enabled for New alerts supported by server
void MainWindow::on_radioANCReadNewAlerts_clicked()
{
    g_app.m_anc_selected_alerts = 0;

    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        QListWidgetItem *item = ui->listAlertCategoriesANC->item(i);
        uint8_t category = 1 << types[i].type;
        if(g_app.m_anc_server_supported_new_alerts & category)
        {
            Qt::ItemFlags flag = item->flags() | Qt::ItemIsEnabled;
            item->setFlags(flag);
            item->setCheckState(Qt::Checked);
            g_app.m_anc_selected_alerts |= category;
        }
        else
        {
            Qt::ItemFlags flag = item->flags() & ~Qt::ItemIsEnabled;
            item->setFlags(flag);
            item->setCheckState(Qt::Unchecked);
        }
    }

    ui->cbANCControlAlertType->clear();
    for(int i = 0; i < 3; i ++ )
    {
        ui->cbANCControlAlertType->insertItem(i, commandsNewAlerts[i].name, commandsNewAlerts[i].type);
    }

    ui->cbANCControlAlertType->setCurrentIndex(0);
}

// When radio button for unread alerts is clicked, check
// the items enabled for Unread alerts supported by server
void MainWindow::on_radioANCReadUnreadAlerts_clicked()
{
    g_app.m_anc_selected_alerts = 0;

    for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        QListWidgetItem *item = ui->listAlertCategoriesANC->item(i);
        uint8_t category = 1 << types[i].type;
        if(g_app.m_anc_server_supported_unread_alerts & category)
        {
            Qt::ItemFlags flag = item->flags() | Qt::ItemIsEnabled;
            item->setFlags(flag);
            item->setCheckState(Qt::Checked);
            g_app.m_anc_selected_alerts |= category;
        }
        else
        {
            Qt::ItemFlags flag = item->flags() & ~Qt::ItemIsEnabled;
            item->setFlags(flag);
            item->setCheckState(Qt::Unchecked);
        }
    }

    ui->cbANCControlAlertType->clear();
    for(int i = 0; i < 3; i ++ )
    {
        ui->cbANCControlAlertType->insertItem(i, commandsUnreadAlerts[i].name, commandsUnreadAlerts[i].type);
    }

    ui->cbANCControlAlertType->setCurrentIndex(0);
}

// When read Alert button is clicked,
// read New or Unread alerts depending on radio button selection
void MainWindow::on_btnACSReadAlerts_clicked()
{
    if(ui->radioANCReadNewAlerts->isChecked())
        wiced_hci_anc_command_read_server_supported_new_alerts();
    else
        wiced_hci_anc_command_read_server_supported_unread_alerts();

}
void MainWindow::on_listAlertCategoriesANC_itemClicked(QListWidgetItem *item)
{
    uint8_t category = 0;

    if(item->flags() & Qt::ItemIsEnabled)
    {
        category = 0;
        for(int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
        {
            QListWidgetItem *itemlocal = ui->listAlertCategoriesANC->item(i);
            if(itemlocal->isSelected())
            {
                category = i;
                break;
            }
        }

        if(item->checkState() == Qt::Checked)
        {
            g_app.m_anc_selected_alerts |= (1 << category);
        }
        else if (item->checkState() == Qt::Unchecked)
        {
            g_app.m_anc_selected_alerts &= ~(1 << category);
        }
    }
}

// When Control Alert is clicked, send the command for the selected drop down command in
// combo box for selected category in list. If no category is selected in list, send command for
// ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED categories
void MainWindow::on_btnANCControlAlerts_clicked()
{
    wiced_hci_bt_anc_control_alert_data_t data;
    uint8_t category = 0;
    uint8_t num_of_selected = 0;

    if (g_app.m_anc_selected_alerts == 0)
    {
        Log("No Alert Type Selected");
        return;
    }

    /* Allow only Either Single Alert Type or All supported Alert Types */
    for (int i = 0; i < ANP_NOTIFY_CATEGORY_COUNT; i++)
    {
        if (g_app.m_anc_selected_alerts & (1 << i))
        {
           num_of_selected++;
           category = i;
        }
    }

    if ((g_app.m_anc_selected_alerts == g_app.m_anc_server_supported_new_alerts) ||
        (g_app.m_anc_selected_alerts == g_app.m_anc_server_supported_unread_alerts))
    {
        category = ANP_ALERT_CATEGORY_ID_ALL_CONFIGURED;
    }
    else if (num_of_selected != 1)
    {
        Log("Either Select Single Alert Type or All supported Alert Types");
        return;
    }

    QVariant v = ui->cbANCControlAlertType->currentData();
    data.cmd_id = (uint8_t) v.toInt();
    data.alert_category = category;

    wiced_hci_anc_command_control_alerts(&data);
}

// Enable/disable new alerts
void MainWindow::on_btnANCEnableNewAlerts_clicked()
{
    if(!g_app.m_b_unread_alerts_enabled)
        wiced_hci_anc_enable_new_alerts();
    else
        wiced_hci_anc_disable_new_alerts();
}

// Enable/disable unread alerts
void MainWindow::on_btnANCUnreadAlerts_clicked()
{
    if(!g_app.m_b_unread_alerts_enabled)
        wiced_hci_anc_enable_unread_alerts();
    else
        wiced_hci_anc_disable_unread_alerts();
}


/***********************************************************/

void MainWindow::on_btnHelpANP_clicked()
{
    onClear();
    Log("Alert Notification Profile help topic:");
    Log("");
    Log("Apps : anc (Client), ans (Service)");
    Log("");
    Log("The ans or anc app should be compiled after enabling flag TEST_HCI_CONTROL in the app makefile.mk ");
    Log("Download ANS app on one WICED Platform and ANC app on another WICED Platform.");
    Log("The app should automatically connect.");
    Log("From Client Control tab of ANP/ANS, configure the required alerts or generate alerts.");
    Log("From Client Control tab of ANP/ANC, query the server supported alerts and control them.");

    ScrollToTop();
}
