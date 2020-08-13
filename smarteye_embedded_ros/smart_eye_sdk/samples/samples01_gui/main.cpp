#include "emSmarteyeUI.h"
#include <QApplication>

#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(":/ico/eye_scan_48px.ico"));

    MainWindow w;
    w.show();

    return a.exec();
}
