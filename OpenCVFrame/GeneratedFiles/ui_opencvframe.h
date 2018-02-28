/********************************************************************************
** Form generated from reading UI file 'opencvframe.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OPENCVFRAME_H
#define UI_OPENCVFRAME_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_OpenCVFrameClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *OpenCVFrameClass)
    {
        if (OpenCVFrameClass->objectName().isEmpty())
            OpenCVFrameClass->setObjectName(QStringLiteral("OpenCVFrameClass"));
        OpenCVFrameClass->resize(600, 400);
        menuBar = new QMenuBar(OpenCVFrameClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        OpenCVFrameClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(OpenCVFrameClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        OpenCVFrameClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(OpenCVFrameClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        OpenCVFrameClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(OpenCVFrameClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        OpenCVFrameClass->setStatusBar(statusBar);

        retranslateUi(OpenCVFrameClass);

        QMetaObject::connectSlotsByName(OpenCVFrameClass);
    } // setupUi

    void retranslateUi(QMainWindow *OpenCVFrameClass)
    {
        OpenCVFrameClass->setWindowTitle(QApplication::translate("OpenCVFrameClass", "OpenCVFrame", 0));
    } // retranslateUi

};

namespace Ui {
    class OpenCVFrameClass: public Ui_OpenCVFrameClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OPENCVFRAME_H
