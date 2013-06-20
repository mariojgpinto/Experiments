/********************************************************************************
** Form generated from reading UI file 'Preferences.ui'
**
** Created: Fri 14. Jun 15:02:18 2013
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PREFERENCES_H
#define UI_PREFERENCES_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Preferences
{
public:
    QWidget *centralwidget;
    QLabel *preferences_label_min_depth;
    QLabel *preferences_label_max_depth;
    QSlider *preferences_slider_min_depth;
    QCheckBox *preferences_check_box_diff;
    QSlider *preferences_slider_max_depth;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *Preferences)
    {
        if (Preferences->objectName().isEmpty())
            Preferences->setObjectName(QString::fromUtf8("Preferences"));
        Preferences->resize(600, 400);
        centralwidget = new QWidget(Preferences);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        preferences_label_min_depth = new QLabel(centralwidget);
        preferences_label_min_depth->setObjectName(QString::fromUtf8("preferences_label_min_depth"));
        preferences_label_min_depth->setGeometry(QRect(10, 10, 121, 16));
        preferences_label_max_depth = new QLabel(centralwidget);
        preferences_label_max_depth->setObjectName(QString::fromUtf8("preferences_label_max_depth"));
        preferences_label_max_depth->setGeometry(QRect(10, 60, 111, 16));
        preferences_slider_min_depth = new QSlider(centralwidget);
        preferences_slider_min_depth->setObjectName(QString::fromUtf8("preferences_slider_min_depth"));
        preferences_slider_min_depth->setGeometry(QRect(100, 10, 491, 20));
        preferences_slider_min_depth->setValue(5);
        preferences_slider_min_depth->setOrientation(Qt::Horizontal);
        preferences_check_box_diff = new QCheckBox(centralwidget);
        preferences_check_box_diff->setObjectName(QString::fromUtf8("preferences_check_box_diff"));
        preferences_check_box_diff->setGeometry(QRect(10, 30, 70, 17));
        preferences_check_box_diff->setLayoutDirection(Qt::LeftToRight);
        preferences_slider_max_depth = new QSlider(centralwidget);
        preferences_slider_max_depth->setObjectName(QString::fromUtf8("preferences_slider_max_depth"));
        preferences_slider_max_depth->setGeometry(QRect(100, 60, 491, 20));
        preferences_slider_max_depth->setValue(30);
        preferences_slider_max_depth->setOrientation(Qt::Horizontal);
        Preferences->setCentralWidget(centralwidget);
        menubar = new QMenuBar(Preferences);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 600, 21));
        Preferences->setMenuBar(menubar);
        statusbar = new QStatusBar(Preferences);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Preferences->setStatusBar(statusbar);

        retranslateUi(Preferences);

        QMetaObject::connectSlotsByName(Preferences);
    } // setupUi

    void retranslateUi(QMainWindow *Preferences)
    {
        Preferences->setWindowTitle(QApplication::translate("Preferences", "Preferences GUI", 0, QApplication::UnicodeUTF8));
        preferences_label_min_depth->setText(QApplication::translate("Preferences", "Min Depth: 0.5", 0, QApplication::UnicodeUTF8));
        preferences_label_max_depth->setText(QApplication::translate("Preferences", "Max Depth: 1.0", 0, QApplication::UnicodeUTF8));
        preferences_check_box_diff->setText(QApplication::translate("Preferences", "Lock Diff", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Preferences: public Ui_Preferences {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PREFERENCES_H
