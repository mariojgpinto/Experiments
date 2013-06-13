/********************************************************************************
** Form generated from reading UI file 'Viewer.ui'
**
** Created: Wed 12. Jun 16:32:31 2013
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_VIEWER_H
#define UI_VIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ViewerClass
{
public:
    QAction *main_action_quit;
    QAction *main_action_preferences;
    QWidget *centralWidget;
    QWidget *main_widget_left;
    QWidget *main_widget_right;
    QFrame *line;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuPreferences;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *ViewerClass)
    {
        if (ViewerClass->objectName().isEmpty())
            ViewerClass->setObjectName(QString::fromUtf8("ViewerClass"));
        ViewerClass->resize(980, 420);
        main_action_quit = new QAction(ViewerClass);
        main_action_quit->setObjectName(QString::fromUtf8("main_action_quit"));
        main_action_preferences = new QAction(ViewerClass);
        main_action_preferences->setObjectName(QString::fromUtf8("main_action_preferences"));
        centralWidget = new QWidget(ViewerClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        main_widget_left = new QWidget(centralWidget);
        main_widget_left->setObjectName(QString::fromUtf8("main_widget_left"));
        main_widget_left->setGeometry(QRect(0, 0, 480, 360));
        main_widget_right = new QWidget(centralWidget);
        main_widget_right->setObjectName(QString::fromUtf8("main_widget_right"));
        main_widget_right->setGeometry(QRect(500, 0, 480, 360));
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(480, 0, 21, 361));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        ViewerClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(ViewerClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 980, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuPreferences = new QMenu(menuBar);
        menuPreferences->setObjectName(QString::fromUtf8("menuPreferences"));
        ViewerClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(ViewerClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        ViewerClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(ViewerClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        ViewerClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuPreferences->menuAction());
        menuFile->addAction(main_action_quit);
        menuPreferences->addAction(main_action_preferences);

        retranslateUi(ViewerClass);

        QMetaObject::connectSlotsByName(ViewerClass);
    } // setupUi

    void retranslateUi(QMainWindow *ViewerClass)
    {
        ViewerClass->setWindowTitle(QApplication::translate("ViewerClass", "Viewer", 0, QApplication::UnicodeUTF8));
        main_action_quit->setText(QApplication::translate("ViewerClass", "Quit", 0, QApplication::UnicodeUTF8));
        main_action_preferences->setText(QApplication::translate("ViewerClass", "Preferences", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("ViewerClass", "File", 0, QApplication::UnicodeUTF8));
        menuPreferences->setTitle(QApplication::translate("ViewerClass", "Preferences", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ViewerClass: public Ui_ViewerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_VIEWER_H
