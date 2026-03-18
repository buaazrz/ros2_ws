/********************************************************************************
** Form generated from reading UI file 'RobotGUIMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOTGUIMAINWINDOW_H
#define UI_ROBOTGUIMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout;
    QFrame *frame;
    QFormLayout *formLayout;
    QLabel *label;
    QFrame *frame_2;
    QGridLayout *gridLayout_2;
    QTabWidget *tabWidget_2;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_3;
    QPlainTextEdit *plainTextEdit;
    QWidget *tab_3;
    QFormLayout *formLayout_3;
    QWidget *JointMotionController;
    QWidget *tab_6;
    QWidget *tab_7;
    QWidget *tab_8;
    QWidget *tab_4;
    QPushButton *pushButton;
    QLabel *label_2;
    QComboBox *comboBox;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(834, 726);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout = new QGridLayout(centralwidget);
        gridLayout->setSpacing(1);
        gridLayout->setObjectName("gridLayout");
        gridLayout->setContentsMargins(1, 1, 1, 1);
        frame = new QFrame(centralwidget);
        frame->setObjectName("frame");
        frame->setMinimumSize(QSize(0, 0));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        formLayout = new QFormLayout(frame);
        formLayout->setObjectName("formLayout");
        label = new QLabel(frame);
        label->setObjectName("label");
        label->setAlignment(Qt::AlignCenter);

        formLayout->setWidget(0, QFormLayout::SpanningRole, label);


        gridLayout->addWidget(frame, 0, 0, 1, 1);

        frame_2 = new QFrame(centralwidget);
        frame_2->setObjectName("frame_2");
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame_2->sizePolicy().hasHeightForWidth());
        frame_2->setSizePolicy(sizePolicy);
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        gridLayout_2 = new QGridLayout(frame_2);
        gridLayout_2->setSpacing(1);
        gridLayout_2->setObjectName("gridLayout_2");
        gridLayout_2->setContentsMargins(1, 1, 1, 1);
        tabWidget_2 = new QTabWidget(frame_2);
        tabWidget_2->setObjectName("tabWidget_2");
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(tabWidget_2->sizePolicy().hasHeightForWidth());
        tabWidget_2->setSizePolicy(sizePolicy1);
        tab_5 = new QWidget();
        tab_5->setObjectName("tab_5");
        verticalLayout_3 = new QVBoxLayout(tab_5);
        verticalLayout_3->setSpacing(1);
        verticalLayout_3->setObjectName("verticalLayout_3");
        verticalLayout_3->setContentsMargins(1, 1, 1, 1);
        plainTextEdit = new QPlainTextEdit(tab_5);
        plainTextEdit->setObjectName("plainTextEdit");
        plainTextEdit->setReadOnly(true);

        verticalLayout_3->addWidget(plainTextEdit);

        tabWidget_2->addTab(tab_5, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName("tab_3");
        formLayout_3 = new QFormLayout(tab_3);
        formLayout_3->setObjectName("formLayout_3");
        tabWidget_2->addTab(tab_3, QString());
        JointMotionController = new QWidget();
        JointMotionController->setObjectName("JointMotionController");
        tabWidget_2->addTab(JointMotionController, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName("tab_6");
        tabWidget_2->addTab(tab_6, QString());
        tab_7 = new QWidget();
        tab_7->setObjectName("tab_7");
        tabWidget_2->addTab(tab_7, QString());
        tab_8 = new QWidget();
        tab_8->setObjectName("tab_8");
        tabWidget_2->addTab(tab_8, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName("tab_4");
        tabWidget_2->addTab(tab_4, QString());

        gridLayout_2->addWidget(tabWidget_2, 2, 0, 1, 2);

        pushButton = new QPushButton(frame_2);
        pushButton->setObjectName("pushButton");

        gridLayout_2->addWidget(pushButton, 1, 0, 1, 2);

        label_2 = new QLabel(frame_2);
        label_2->setObjectName("label_2");
        label_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout_2->addWidget(label_2, 0, 0, 1, 1);

        comboBox = new QComboBox(frame_2);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName("comboBox");

        gridLayout_2->addWidget(comboBox, 0, 1, 1, 1);

        gridLayout_2->setColumnStretch(0, 10);
        gridLayout_2->setColumnStretch(1, 40);

        gridLayout->addWidget(frame_2, 0, 1, 1, 1);

        gridLayout->setColumnStretch(0, 20);
        gridLayout->setColumnStretch(1, 50);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 834, 23));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget_2->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Robot GUI", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Robot", nullptr));
        plainTextEdit->setPlainText(QCoreApplication::translate("MainWindow", "Please choose a controller to activate", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_5), QCoreApplication::translate("MainWindow", "NONE", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_3), QCoreApplication::translate("MainWindow", "ForwardController", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(JointMotionController), QCoreApplication::translate("MainWindow", "JointMotionController", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_6), QCoreApplication::translate("MainWindow", "CartesianMotionController", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_7), QCoreApplication::translate("MainWindow", "CartesianTrajectoryController", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_8), QCoreApplication::translate("MainWindow", "AdmittanceController", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_4), QCoreApplication::translate("MainWindow", "ForceDragController", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "Activate", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "Controller:", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("MainWindow", "NONE", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("MainWindow", "ForwardController", nullptr));
        comboBox->setItemText(2, QCoreApplication::translate("MainWindow", "JointMotionController", nullptr));
        comboBox->setItemText(3, QCoreApplication::translate("MainWindow", "CartesianMotionController", nullptr));
        comboBox->setItemText(4, QCoreApplication::translate("MainWindow", "CartesianTrajectoryController", nullptr));
        comboBox->setItemText(5, QCoreApplication::translate("MainWindow", "CartesianImpedancePDController", nullptr));
        comboBox->setItemText(6, QCoreApplication::translate("MainWindow", "AdmittanceController", nullptr));
        comboBox->setItemText(7, QCoreApplication::translate("MainWindow", "ForceDragController", nullptr));

    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTGUIMAINWINDOW_H
