#ifndef EMSMARTEYE_H
#define EMSMARTEYE_H

#include <iostream>
#include <string>
#include <thread>
#include <memory>
#include <chrono>

// Qt
#include <QMainWindow>
#include <QString>
#include <QButtonGroup>
#include <QFileDialog>
#include <QFile>
#include <QStandardItemModel>
#include <QStringListModel>
#include <QTimer>
#include <QProgressBar>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// OpenCV
#include <opencv2/opencv.hpp>

//Thirdparty
#include "emController.h"

//define
typedef pcl::PointXYZRGB        PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    QThread *threadShowimg;
    QTimer m_heart;
    emController* emDemo;

    QStringListModel *Model;
    QStandardItemModel *ItemModel;

    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer;
    PointCloudT::Ptr cloud;
    PointCloud_EM::Ptr emCloud;


private:
    Ui::MainWindow *ui;
    QProgressBar * pProgressBar;

    int m_currentIndex;

    QTimer *timer;

    int roiStartX;
    int roiStartY;
    int roiEndX;
    int roiEndY;

    int deepZmin;
    int deepZmax;

    void display_1();
    void display_2();

    QGraphicsScene *scene;
    QGraphicsPixmapItem *pixmapItem;

    void init();
    int  addRow();
    void addnRow(int nRow);
    void deleteAllRow();
    void addItemContent(int row, int column, QString content);
    void addMessage(int column,QString contexTIM, QString contexMsg);
    QImage pixmapScale(const QImage& image, const double & index);
    void SavePPMFile(uint32_t ui32Width, uint32_t ui32Height,unsigned char* g_pMonoImageBufs, unsigned int times);

signals:
    void sOutputLog(QString info);

private slots:
    void showTime();
    void slotShowLog();
    void slot_HeartTimeOut();
    void slot_ScanDevice();
    void slot_InitDevice();
    void slot_ControlStatu();
    void slot_SettingTableChanged (int, int);
    void slot_RunSingal();
    void slot_RunMultip();
    void slot_Run3D();
    void slot_Run2D();

    void slot_LoadConfigIni();
    void slot_SyncConfigIni();
    void slot_RecvConfigIni();

    void slot_SysStatus();

    void lowerValueChangedSlotz(int);
    void upperValueChangedSlotz(int);
    void lowerValueChangedSlotz();
    void upperValueChangedSlotz();

    void lowerValueChangedSlotw(int);
    void upperValueChangedSlotw(int);
    void lowerValueChangedSlotw();
    void upperValueChangedSlotw();

    void lowerValueChangedSloth(int);
    void upperValueChangedSloth(int);
    void lowerValueChangedSloth();
    void upperValueChangedSloth();

    void setExposureTime();
    void switchRunMode(bool);

    void outputLog(QString);
};

#endif // MAINWINDOW_H
