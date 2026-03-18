/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionClear;
    QAction *actionJoint_1;
    QAction *actionJoint_2;
    QAction *actionJoint_3;
    QAction *actionJoint_4;
    QAction *actionJoint_5;
    QAction *actionJoint_6;
    QAction *actionJoint_7;
    QAction *actionAuto_Scaling;
    QAction *actionWindow_Width;
    QAction *actionZoom_In;
    QAction *actionZoom_Out;
    QAction *actionZoom_Reset;
    QAction *actionLoging;
    QAction *actionSave_Data;
    QAction *actionLine_Width;
    QAction *actionService;
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QMenuBar *menubar;
    QMenu *menuDisplay;
    QMenu *menuSetting;
    QMenu *menuCommand;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(800, 600);
        actionClear = new QAction(MainWindow);
        actionClear->setObjectName("actionClear");
        actionJoint_1 = new QAction(MainWindow);
        actionJoint_1->setObjectName("actionJoint_1");
        actionJoint_1->setCheckable(true);
        actionJoint_1->setChecked(true);
        actionJoint_2 = new QAction(MainWindow);
        actionJoint_2->setObjectName("actionJoint_2");
        actionJoint_2->setCheckable(true);
        actionJoint_2->setChecked(true);
        actionJoint_3 = new QAction(MainWindow);
        actionJoint_3->setObjectName("actionJoint_3");
        actionJoint_3->setCheckable(true);
        actionJoint_3->setChecked(true);
        actionJoint_4 = new QAction(MainWindow);
        actionJoint_4->setObjectName("actionJoint_4");
        actionJoint_4->setCheckable(true);
        actionJoint_4->setChecked(true);
        actionJoint_5 = new QAction(MainWindow);
        actionJoint_5->setObjectName("actionJoint_5");
        actionJoint_5->setCheckable(true);
        actionJoint_5->setChecked(true);
        actionJoint_6 = new QAction(MainWindow);
        actionJoint_6->setObjectName("actionJoint_6");
        actionJoint_6->setCheckable(true);
        actionJoint_6->setChecked(true);
        actionJoint_7 = new QAction(MainWindow);
        actionJoint_7->setObjectName("actionJoint_7");
        actionJoint_7->setCheckable(true);
        actionJoint_7->setChecked(true);
        actionAuto_Scaling = new QAction(MainWindow);
        actionAuto_Scaling->setObjectName("actionAuto_Scaling");
        actionAuto_Scaling->setCheckable(true);
        actionAuto_Scaling->setChecked(true);
        actionWindow_Width = new QAction(MainWindow);
        actionWindow_Width->setObjectName("actionWindow_Width");
        actionZoom_In = new QAction(MainWindow);
        actionZoom_In->setObjectName("actionZoom_In");
        actionZoom_Out = new QAction(MainWindow);
        actionZoom_Out->setObjectName("actionZoom_Out");
        actionZoom_Reset = new QAction(MainWindow);
        actionZoom_Reset->setObjectName("actionZoom_Reset");
        actionLoging = new QAction(MainWindow);
        actionLoging->setObjectName("actionLoging");
        actionLoging->setCheckable(true);
        actionSave_Data = new QAction(MainWindow);
        actionSave_Data->setObjectName("actionSave_Data");
        actionLine_Width = new QAction(MainWindow);
        actionLine_Width->setObjectName("actionLine_Width");
        actionService = new QAction(MainWindow);
        actionService->setObjectName("actionService");
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setSpacing(1);
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(1, 1, 1, 1);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1119, 24));
        menuDisplay = new QMenu(menubar);
        menuDisplay->setObjectName("menuDisplay");
        menuSetting = new QMenu(menubar);
        menuSetting->setObjectName("menuSetting");
        menuCommand = new QMenu(menubar);
        menuCommand->setObjectName("menuCommand");
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuDisplay->menuAction());
        menubar->addAction(menuSetting->menuAction());
        menubar->addAction(menuCommand->menuAction());
        menuDisplay->addAction(actionAuto_Scaling);
        menuDisplay->addAction(actionClear);
        menuDisplay->addAction(actionJoint_1);
        menuDisplay->addAction(actionJoint_2);
        menuDisplay->addAction(actionJoint_3);
        menuDisplay->addAction(actionJoint_4);
        menuDisplay->addAction(actionJoint_5);
        menuDisplay->addAction(actionJoint_6);
        menuDisplay->addAction(actionJoint_7);
        menuSetting->addAction(actionWindow_Width);
        menuSetting->addAction(actionLine_Width);
        menuSetting->addAction(actionZoom_In);
        menuSetting->addAction(actionZoom_Out);
        menuSetting->addAction(actionZoom_Reset);
        menuSetting->addAction(actionLoging);
        menuCommand->addAction(actionSave_Data);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Robot Monitor", nullptr));
        actionClear->setText(QCoreApplication::translate("MainWindow", "Clear", nullptr));
        actionJoint_1->setText(QCoreApplication::translate("MainWindow", "Joint 1", nullptr));
        actionJoint_2->setText(QCoreApplication::translate("MainWindow", "Joint 2", nullptr));
        actionJoint_3->setText(QCoreApplication::translate("MainWindow", "Joint 3", nullptr));
        actionJoint_4->setText(QCoreApplication::translate("MainWindow", "Joint 4", nullptr));
        actionJoint_5->setText(QCoreApplication::translate("MainWindow", "Joint 5", nullptr));
        actionJoint_6->setText(QCoreApplication::translate("MainWindow", "Jiont 6", nullptr));
        actionJoint_7->setText(QCoreApplication::translate("MainWindow", "Joint 7", nullptr));
        actionAuto_Scaling->setText(QCoreApplication::translate("MainWindow", "Auto Scaling", nullptr));
        actionWindow_Width->setText(QCoreApplication::translate("MainWindow", "Window Width", nullptr));
        actionZoom_In->setText(QCoreApplication::translate("MainWindow", "Zoom In", nullptr));
        actionZoom_Out->setText(QCoreApplication::translate("MainWindow", "Zoom Out", nullptr));
        actionZoom_Reset->setText(QCoreApplication::translate("MainWindow", "Zoom Reset", nullptr));
        actionLoging->setText(QCoreApplication::translate("MainWindow", "Logging", nullptr));
        actionSave_Data->setText(QCoreApplication::translate("MainWindow", "Save Data", nullptr));
        actionLine_Width->setText(QCoreApplication::translate("MainWindow", "Line Width", nullptr));
        actionService->setText(QCoreApplication::translate("MainWindow", "Stop Service", nullptr));
        menuDisplay->setTitle(QCoreApplication::translate("MainWindow", "Display", nullptr));
        menuSetting->setTitle(QCoreApplication::translate("MainWindow", "Setting", nullptr));
        menuCommand->setTitle(QCoreApplication::translate("MainWindow", "Command", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
