#include "emSmarteyeUI.h"
#include "ui_emSmarteyeUI.h"

#include "qeventloop.h"
#include <QStatusBar>

#include "emPointCloudOperation.h"

#include <qdebug.h>

#define IMG_WIDTH   2048
#define IMG_HEIGHT  1536
#define IMG_CH      4

#define ExTest  1
#define TIMERS

//////////////////////////////////////////////////////////////////////////////////////
//全局函数定义区
void OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM* pFrame);
void SavePPMFile(uint32_t ui32Width, uint32_t ui32Height,unsigned char* g_pMonoImageBufs, unsigned int times);

//全局常量定义区
void * m_Device_1 = NULL;

static unsigned int recvBufID = 0;
static bool ImgBuffReady = false;
static bool bRunTypeContinue = false;
static const int  ipMax = 32;
static bool statu3Dmodle = true;
static QImage ImageParallax(IMG_WIDTH,IMG_HEIGHT, QImage::Format_Indexed8);

static unsigned char ImgBuffer[IMG_WIDTH*IMG_HEIGHT*4] = {0};
static unsigned char ImgBufferVideo[IMG_WIDTH*IMG_HEIGHT] = {0};
static unsigned char ImgBufferGray[IMG_WIDTH*IMG_HEIGHT] = {0};
///////////////////////////////////////////////////////////////////////////////////////

using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle ("emSmarteyeUI");

    init();

    pixmapItem = new QGraphicsPixmapItem();

    m_currentIndex = 0;
    emDemo = new emController();

    timer = new QTimer(this);       //新建一个定时器对象
    connect(timer, &QTimer::timeout, this, &MainWindow::showTime);
//    timer->start(500);
    //创建子线程
    connect(&m_heart,&QTimer::timeout,this,&MainWindow::slot_HeartTimeOut);
    m_heart.setInterval(200);
    m_heart.start();
}

MainWindow::~MainWindow()
{
    if(m_Device_1 != NULL)
        emDemo->emCloseDevice(m_currentIndex);
    if(emDemo != NULL)
        delete emDemo;
    delete ui;
}

void MainWindow::init()
{   
    ui->tableWidget->setColumnWidth(0,125);
    ui->radioButton_singal->setChecked(true);
    ui->pushButton_sw->setEnabled(false);
    ui->groupBox_resMode->setEnabled(false);
    ui->checkBox_2D->setEnabled(false);
//    ui->checkBox_2D->setEnabled(false);
    ui->label_gray->setScaledContents(true);
    //将header最后的空间补全，仅仅是通过延伸最后一个单元格实现的，而没有单元格平分
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->resizeRowsToContents();
    //设置表头高度
    ui->tableWidget->horizontalHeader()->setFixedHeight(30);
    ui->lineEdit_exptime->setText("12000");
    //double slider
    ui->spinBox_zl->setValue(10);
    ui->spinBox_zr->setValue(2000);
    ui->horizontalSlider_z->setHandleMovementMode(QxtSpanSlider::NoOverlapping);
    ui->horizontalSlider_z->setMaximum(3500);
    ui->horizontalSlider_z->setLowerValue(10);
    ui->horizontalSlider_z->setUpperValue(2000);
    connect(ui->horizontalSlider_z,SIGNAL(lowerValueChanged(int)),this,SLOT(lowerValueChangedSlotz(int)));
    connect(ui->horizontalSlider_z,SIGNAL(upperValueChanged(int)),this,SLOT(upperValueChangedSlotz(int)));
    connect(ui->spinBox_zl,SIGNAL(editingFinished()),this,SLOT(lowerValueChangedSlotz()));
    connect(ui->spinBox_zr,SIGNAL(editingFinished()),this,SLOT(upperValueChangedSlotz()));

    ui->spinBox_wl->setValue(1);
    ui->spinBox_wr->setValue(2048);
    ui->horizontalSlider_w->setHandleMovementMode(QxtSpanSlider::NoOverlapping);
    ui->horizontalSlider_w->setMaximum(2048);
    ui->horizontalSlider_w->setLowerValue(1);
    ui->horizontalSlider_w->setUpperValue(2048);
    connect(ui->horizontalSlider_w,SIGNAL(lowerValueChanged(int)),this,SLOT(lowerValueChangedSlotw(int)));
    connect(ui->horizontalSlider_w,SIGNAL(upperValueChanged(int)),this,SLOT(upperValueChangedSlotw(int)));
    connect(ui->spinBox_wl,SIGNAL(editingFinished()),this,SLOT(lowerValueChangedSlotw()));
    connect(ui->spinBox_wr,SIGNAL(editingFinished()),this,SLOT(upperValueChangedSlotw()));

    ui->spinBox_hl->setValue(1);
    ui->spinBox_hr->setValue(1536);
    ui->horizontalSlider_h->setHandleMovementMode(QxtSpanSlider::NoOverlapping);
    ui->horizontalSlider_h->setMaximum(1536);
    ui->horizontalSlider_h->setLowerValue(1);
    ui->horizontalSlider_h->setUpperValue(1536);
    connect(ui->horizontalSlider_h,SIGNAL(lowerValueChanged(int)),this,SLOT(lowerValueChangedSloth(int)));
    connect(ui->horizontalSlider_h,SIGNAL(upperValueChanged(int)),this,SLOT(upperValueChangedSloth(int)));
    connect(ui->spinBox_hl,SIGNAL(editingFinished()),this,SLOT(lowerValueChangedSloth()));
    connect(ui->spinBox_hr,SIGNAL(editingFinished()),this,SLOT(upperValueChangedSloth()));

    roiStartX = 1;
    roiStartY = 1;
    roiEndX   = IMG_WIDTH;
    roiEndY   = IMG_HEIGHT;

    deepZmin = 1;
    deepZmax = 1500;

    connect(ui->lineEdit_exptime, SIGNAL(returnPressed()), this, SLOT(setExposureTime()));
    connect(ui->tableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(slot_SettingTableChanged(int, int)),Qt::UniqueConnection);///*Double*//*cellClicked*/
    connect(ui->tableWidget,SIGNAL(itemChanged(QTableWidgetItem*)),ui->tableWidget,SLOT(resizeRowsToContents()));
    connect(ui->radioButton_singal,SIGNAL(clicked()),this,SLOT(slot_RunSingal()));
    connect(ui->radioButton_mul,SIGNAL(clicked()),this,SLOT(slot_RunMultip()));
    connect(ui->checkBox_2D, SIGNAL(clicked(bool)), this, SLOT(switchRunMode(bool)));
    connect(this, SIGNAL(sOutputLog(QString)), this, SLOT(outputLog(QString)));
    connect(ui->pushButton_SystemStu, SIGNAL(clicked()), this, SLOT(slot_SysStatus()));

    statu3Dmodle = true;

    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    emCloud = PointCloud_EM_Ptr(new PointCloud_EM());
    emCloud->height = IMG_HEIGHT;
    emCloud->width = IMG_WIDTH;
    emCloud->resize(IMG_HEIGHT * IMG_WIDTH);

    viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer",false));
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");//显示点
    viewer->setBackgroundColor(92.0 / 255, 92.0 / 255, 184.0 / 255, 0);
    viewer->resetCamera();

    ui->QVTKWidget3DViewer->SetRenderWindow((viewer->getRenderWindow()));
    viewer->setupInteractor(ui->QVTKWidget3DViewer->GetInteractor(),ui->QVTKWidget3DViewer->GetRenderWindow());
    ui->QVTKWidget3DViewer->update();
/////////////////////////////////////////////////////////jok/////////////////////////////////////////////////////////
    QStatusBar * pStatusBar = new QStatusBar();
    setStatusBar(pStatusBar);
    pProgressBar = new QProgressBar();
    QLabel *  pLabel= new QLabel();
    pLabel->setText("请稍候...");
    pProgressBar->setRange(0,100);
    pProgressBar->setValue(30);
    pStatusBar->addWidget(pLabel);//添加到状态栏的左边
    pStatusBar->addWidget(pProgressBar);

    //QStatusBar
    QLabel *per1 = new QLabel("©2020-2025 shs. All Rights Reserved.", this);
    statusBar()->addPermanentWidget(per1); //现实永久信息
}

void OnTestCallBackFun(PNP_FRAME_CALLBACK_PARAM* pFrame)
{
    static int recvBufID_Old = 0;
    recvBufID = pFrame->nFrameID;

    if(recvBufID_Old != recvBufID)
    {
//        printf("recived ok... \n");
        ImgBuffReady = true;
        recvBufID_Old = pFrame->nFrameID;
        if(statu3Dmodle)
            memcpy(ImgBuffer,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
        else
            memcpy(ImgBufferVideo,(unsigned char*)pFrame->pImgBuf,pFrame->pBufferSize);
    }
}

void MainWindow::slotShowLog()
{
    qDebug() << "__FILE" ;
}

void MainWindow::slot_SettingTableChanged (int row, int col)
{
    if(ui->tableWidget->item (row,col)->checkState ()==Qt::Checked)
    {
        ui->tableWidget->item(row,col)->setTextColor (Qt::green);
        QString str_ip = ui->tableWidget->item(row,col)->text();
        if(emDemo->emDevChangeIp(str_ip.toLatin1().data(), row) >= 0)
            pProgressBar->setValue(30);
        else    //show failed info
            return;
        //初始化网络链接
        slot_InitDevice();
    }
    else
    {
        ui->tableWidget->item(row,col)->setTextColor(Qt::gray);
    }
}

/*插入一行*/
int MainWindow::addRow()
{
    int row = ui->tableWidget->rowCount();
    ui->tableWidget->insertRow(row);
    ui->tableWidget->setRowHeight(row, 21);
    return row;
}

/*插入n行*/
void MainWindow::addnRow(int nRow)
{
    int row;
    for(int i = 0;i < nRow;i ++)
    {
        row = ui->tableWidget->rowCount();
        ui->tableWidget->insertRow(row);
        ui->tableWidget->setRowHeight(row, 20);
    }
}

void MainWindow::deleteAllRow()
{
    int row = ui->tableWidget->rowCount();
    for(int i = 0;i<row;i++)
        ui->tableWidget->removeRow(i);
    ui->tableWidget->clearContents();
}

/*给单元格添加内容*/
void MainWindow::addItemContent(int row, int column, QString content)
{
   QTableWidgetItem *item = new QTableWidgetItem(content);
   ui->tableWidget->setItem(row, column, item);
}

/*add message*/
void MainWindow::addMessage(int column,QString contexTIM, QString contexMsg)
{
    int row = addRow();

    QTableWidgetItem *check = new QTableWidgetItem(contexTIM);
    check->setCheckState (Qt::Unchecked);
    ui->tableWidget->setItem(row,column,check); //插入复选框

    addItemContent(row,column + 1,contexMsg);
}

void MainWindow::showTime()
{
    //ADD YOUR CODE HERE ...
}

void MainWindow::slot_HeartTimeOut()
{
//    static int img_count = 0;
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
#ifdef XILINX_ZYNQMP_MONOCULAR_ALGRITHIM
                display_2();
#else
                display_1();
#endif
            }

            if(index == 1)
            {
#ifdef XILINX_ZYNQMP_MONOCULAR_ALGRITHIM
                display_2();
#else
                display_1();
#endif
            }
        }
    }
}

QImage Mat2QImage(cv::Mat const& src)
{
     cv::Mat temp; // make the same cv::Mat
     cvtColor(src, temp,CV_GRAY2RGB); // cvtColor Makes a copt, that what i need
     QImage dest((const uchar *) temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
     dest.bits(); // enforce deep copy, see documentation
     // of QImage::QImage ( const uchar * data, int width, int height, Format format )
     return dest;
}

void MainWindow::display_1()
{
    if(!ImgBuffReady)
        return;

    ImgBuffReady = false;   
    if(statu3Dmodle)
    {
        cloud->clear();
    #ifdef ExTest
        emDemo->emExchangeParallaxToPointCloudEx(ImgBuffer, ImgBufferGray, emCloud);
    #else
        emCloud->height = roiRowEnd - roiRowStart +1;
        emCloud->width = roiColEnd-roiColStart+1;
        emCloud->resize(emCloud->height * emCloud->width);
        emDemo->emExchangeParallaxToPointCloud(ImgBuffer, ImgBufferGray, emCloud);
    #endif
        convert2PCLPointCloud(emCloud, cloud);

        //Save PointCloud Data
        if(ui->checkBox_SavePcloud->isChecked())
        {
            static uint32_t index = 0;
            char img_file[100];
            sprintf(img_file, "3dview_round_%d.ply", (int)index++);
            pcl::io::savePLYFile(img_file,*cloud);
        }
        //Auto save gray images
        if(ui->checkBox_SaveGrayImg->isChecked())
        {
            static uint32_t cTimes = 0;
    #ifdef ExTest
            SavePPMFile(IMG_WIDTH, IMG_HEIGHT, ImgBufferGray, cTimes++);
    #else
            SavePPMFile(roiEndX-roiStartX+1, roiEndY - roiStarYt +1, ImgBufferGray, cTimes++);
    #endif
        }
        //Auto loading gray image
        if(ui->tabWidget_view->currentIndex() == 1)
        {

    #ifdef ExTest
            QImage *img = new QImage(IMG_WIDTH, IMG_HEIGHT, QImage::Format_Indexed8);
            memcpy(img->bits(), ImgBufferGray, IMG_WIDTH*IMG_HEIGHT);
            ui->label_gray->setPixmap(QPixmap::fromImage(*img).scaled(ui->label_gray->size(), Qt::KeepAspectRatio));
            ui->label_gray->update();
    #endif
        }

        if(ui->tabWidget_view->currentIndex() == 2)
        {
    #ifndef ExTest
            Mat gray1_mat(roiRowEnd - roiRowStart +1, roiColEnd-roiColStart+1, CV_8UC1, ImgBufferGray);//为了显示图片 先改为mat类型
            //imshow("去雾图像显示", gray1_mat);

            QImage img = Mat2QImage(gray1_mat);
            QPixmap pixmap=QPixmap::fromImage(img);
//            QGraphicsScene *scene=new QGraphicsScene;
//            QGraphicsPixmapItem *pixmapItem=new QGraphicsPixmapItem;
            pixmapItem=scene->addPixmap(pixmap);
            ui->graphicsView->setScene(scene);
            ui->graphicsView->show();
    #endif
        }
        //Update the pointcloude window
        if(!cloud->points.empty())
        {
    //        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> single_color(cloud,"z"); //z arrow
    //        viewer->updatePointCloud(cloud, single_color, "cloud");//显示点
            viewer->updatePointCloud(cloud, "cloud");//显示点
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
            viewer->resetCamera();

            ui->QVTKWidget3DViewer->update();
        }
    }
    else
    {
        Mat gray1_mat(roiEndY - roiStartY +1, roiEndX - roiStartX +1, CV_8UC1, ImgBufferGray);//为了显示图片 先改为mat类型

        QImage img = Mat2QImage(gray1_mat);
        QPixmap pixmap=QPixmap::fromImage(img);

//        pixmapItem=scene->addPixmap(pixmap);
//        ui->graphicsView_2d->setScene(scene);
//        ui->graphicsView_2d->show();
    }
    //outputLog(QString("FrameSum:%1").arg(QString::number(recvBufID)));
}

void MainWindow::display_2()
{
    //ADD YOUR CODE HERE ...
}

void MainWindow::SavePPMFile(uint32_t ui32Width, uint32_t ui32Height,unsigned char* g_pMonoImageBufs, unsigned int times)
{
    char szName[512] = {0};

    static int nRawFileIndex = 0;
    FILE* phImageFile = NULL;
    sprintf(szName, "Frame_%d_%d.ppm", times,nRawFileIndex++);
    phImageFile = fopen(szName,"wb");
    if (phImageFile == NULL)
    {
        outputLog(QString("Save %1 failed!").arg(QString::fromLocal8Bit(szName)));
        return;
    }
    //Save mono image
    if(g_pMonoImageBufs != NULL)
    {
        fprintf(phImageFile, "P5\n" "%u %u 255\n", ui32Width, ui32Height);
        fwrite(g_pMonoImageBufs, 1, ui32Width * ui32Height, phImageFile);
        fclose(phImageFile);
        phImageFile = NULL;
        outputLog(QString("Save %1 successed!").arg(QString::fromLocal8Bit(szName)));
    }
    else
    {
        outputLog(QString("Save %1 failed!").arg(QString::fromLocal8Bit(szName)));
    }
    return;
}

//void SaveBMPFile(uint32_t ui32Width, uint32_t ui32Height,unsigned char* g_pMonoImageBufs, unsigned int times)
//{
//    cv::Mat tempMat = Mat(ui32Height, ui32Width,);
//}

QImage MainWindow::pixmapScale(const QImage& image, const double & index)
{
    QImage r_image;
    r_image = image.scaled(image.width()*index, image.height()*index, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    return r_image;
}

void MainWindow::slot_ScanDevice()
{
    if(m_Device_1 != NULL)
    {
        deleteAllRow();
        emDemo->emCloseDevice(m_currentIndex);
    }
    outputLog(QString().sprintf("开始扫描设备..."));
    if(emDemo->emScanDevice(false) >= 0)
        pProgressBar->setValue(10);
    else
        outputLog(QString().sprintf("扫描失败，请检查网络是否链接."));

    for(int i = 0;i<ipMax;i++)
    {
        if(strlen(emDemo->emGuiDevScanInfo[i]->IP) != 0)
        {
            QString str = QString(" MAC: %1\n SN: %2\n Version:%3\n Name:%4\n RemoteIP: %5\n") \
                    .arg(QString(emDemo->emGuiDevScanInfo[i]->MAC)) \
                                 .arg(QString(emDemo->emGuiDevScanInfo[i]->SN)) \
                                              .arg(QString(emDemo->emGuiDevScanInfo[i]->Version))   \
                                                        .arg(QString(emDemo->emGuiDevScanInfo[i]->Name)) \
                                                                .arg(QString(emDemo->emGuiDevScanInfo[i]->lIP))  ;
            addMessage(0,emDemo->emGuiDevScanInfo[i]->IP, str);
            outputLog(QString("远端设备信息:%1").arg(str));
        }           
    }
    pProgressBar->setValue(15);
}

void MainWindow::slot_InitDevice()
{
    RICB usrPara = {0.1, 0, NULL};
    int temp_device_num = ui->tableWidget->rowCount();
    outputLog(QString().sprintf("初始化设备..."));
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 != NULL)
                {
                    outputLog(QString().sprintf("设备已连接，请勿重复操作..."));
                    continue;
                }
                if(emDemo->emOpenDevice(m_Device_1, index, MSQ_KEY, true, false) < 0)
                {
                    outputLog(QString().sprintf("打开设备失败..."));
                    break;
                }
                pProgressBar->setValue(50);

#ifdef XILINX_ZYNQMP_MONOCULAR_ALGRITHIM
                emDemo->emRegisterImageCallback(index, (void*)&usrPara, OnTestCallBackFun_2);
#else
                outputLog(QString().sprintf("注册回调函数..."));
                emDemo->emRegisterImageCallback(index, (void*)&usrPara, OnTestCallBackFun);
#endif
                pProgressBar->setValue(60);
                slot_RunSingal();
                pProgressBar->setValue(70);
                ui->groupBox_resMode->setEnabled(true);
                slot_Run3D();
                pProgressBar->setValue(90);
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }
    ui->pushButton_sw->setEnabled(true);
    ui->pushButton_scan->setEnabled(false);
    pProgressBar->setValue(100);

    outputLog(QString().sprintf("初始化设备完成..."));
}

void MainWindow::slot_ControlStatu()
{
    static bool status_1 = false;
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }
                if(!status_1)
                {
#ifdef TIMERS
                    if(bRunTypeContinue || !statu3Dmodle)
                    {
                        status_1 = true;
                    }
                    emDemo->emDevStart(index);
#else
                    status_1 = true;
                    emDemo->emDevStart(index);
                    if(!bRunTypeContinue)
                        timer->start(800);
#endif
                    if(bRunTypeContinue)
                        ui->pushButton_sw->setText("Stop");
                    else
                        ui->pushButton_sw->setText("Capture");
                    outputLog(QString().sprintf("start..."));
                }
                else
                {
#ifdef TIMERS
                    if(bRunTypeContinue || !statu3Dmodle)
                    {
                        status_1 = false;
                    }
                    emDemo->emDevStop(index);
#else
                    status_1 = false;
                    if(!bRunTypeContinue)
                        timer->stop();
                    else
                        emDemo->emDevStop(index);
#endif
                    if(bRunTypeContinue)
                        ui->pushButton_sw->setText("Capture");
                    outputLog(QString().sprintf("stop..."));
                }
            }
            else
            {
               //ADD YOUR CODE HERE ...
            }
        }
    }
}

void MainWindow::slot_RunSingal()
{
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }

                emDemo->emSetOutputOnceOrMulti(index, ONCETIME);
                bRunTypeContinue = false;
                outputLog(QString().sprintf("当前模式为单次成像模式..."));
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }  
}

void MainWindow::slot_RunMultip()
{
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }

                emDemo->emSetOutputOnceOrMulti(index, CONTINUE);
                bRunTypeContinue = true;
                outputLog(QString().sprintf("当前模式为连续成像模式..."));
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }
}

void MainWindow::slot_Run3D()
{
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }

                if(emDemo->emSwitchRunMode(index, EM_NORMAL_3D) >= 0)
                    outputLog(QString().sprintf("设备当前输出为3D模式..."));
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }
}

void MainWindow::slot_Run2D()
{
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }

                if(emDemo->emSwitchRunMode(index, EM_2D_ORGLOOK) >= 0)
                    outputLog(QString().sprintf("设备当前输出为视频流模式..."));
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }
}

void MainWindow::switchRunMode(bool is2D)
{
    if(is2D)
    {
       slot_Run2D();
       statu3Dmodle = false;
    }
    else
    {
        slot_Run3D();
        statu3Dmodle = true;
    }
}

void MainWindow::slot_SysStatus()
{
    SysInfo sysinfo;
    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(ui->tableWidget->item (index,0)->checkState ()==Qt::Checked)
        {
            if(index == 0)
            {
                if(m_Device_1 == NULL)
                {
                    outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                    return;
                }
                emDemo->emGetSysStatusEx(index, sysinfo);
                emit sOutputLog(QString("embedded system status : systemMode: %1 \
                    frameRate: %2 \
                    inSideTemp: %3 \
                    tempStatu: %4 \
                    powerOnTime: %5 \
                    allFrameCnt: %6 \
                    expTime: %7 \
                    initDeviceFlag: %8").arg(QString::number(sysinfo.gsystemMode))    \
                                        .arg(QString::number(sysinfo.gframeRate))    \
                                        .arg(QString::number(sysinfo.ginSideTemp))    \
                                        .arg(QString::number(sysinfo.gtempStatu))    \
                                        .arg(QString::number(sysinfo.gpowerOnTime))    \
                                        .arg(QString::number(sysinfo.gallFrameCnt))    \
                                        .arg(QString::number(sysinfo.gexpTime))    \
                                        .arg(QString::number(sysinfo.ginitDeviceFlag)));
            }
            else
            {
                //ADD YOUR CODE HERE ...
            }
        }
    }
}

void MainWindow::outputLog(QString info)
{
    ui->plainTextEdit_log->appendPlainText(info);
    ui->plainTextEdit_log->moveCursor(QTextCursor::End);
    QApplication::processEvents();
}

void MainWindow::slot_LoadConfigIni()
{

    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(index == 0)
        {
            if(m_Device_1 == NULL)
            {
                outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                return;
            }

            emDemo->emLoadConfigIniFile(index, "./");
        }
        else if(index ==1)
        {
            //ADD YOUR CODE HERE ...
        }
    }
}

void MainWindow::slot_SyncConfigIni()
{

    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(index == 0)
        {
            if(m_Device_1 == NULL)
            {
                outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                return;
            }

            emDemo->emSyncLocalConfigIniFile(index, (char*)"./config.ini");
        }
        else if(index ==1)
        {
            //ADD YOUR CODE HERE ...
        }
    }
}

void MainWindow::slot_RecvConfigIni()
{

    int temp_device_num = ui->tableWidget->rowCount();
    for(int index = 0;index < temp_device_num;index ++)
    {
        if(index == 0)
        {
            if(m_Device_1 == NULL)
            {
                outputLog(QString().sprintf("设备句柄为空，请检查是否已经完成初始化..."));
                return;
            }

            emDemo->emRecoveryConfigIniFile(index);
        }
        else if(index ==1)
        {
            //ADD YOUR CODE HERE ...
        }
    }
}

//minz
void MainWindow::lowerValueChangedSlotz(int value)
{
    ui->spinBox_zl->setValue(value);
    ui->horizontalSlider_z->setLowerValue(value);
    deepZmin = value;
    emDemo->emSetzAxisValueOfPiontcloud(0, deepZmin, deepZmax);
}

void MainWindow::upperValueChangedSlotz(int value)
{
    ui->spinBox_zr->setValue(value);
    ui->horizontalSlider_z->setUpperValue(value);
    deepZmax = value;
    emDemo->emSetzAxisValueOfPiontcloud(0, deepZmin, deepZmax);
}

void MainWindow::lowerValueChangedSlotz()
{
    int value = ui->spinBox_zl->value();
    ui->horizontalSlider_z->setLowerValue(value);
    deepZmin = value;
    emDemo->emSetzAxisValueOfPiontcloud(0, deepZmin, deepZmax);
}

void MainWindow::upperValueChangedSlotz()
{
    int value = ui->spinBox_zr->value();
    ui->horizontalSlider_z->setUpperValue(value);
    deepZmax = value;
    emDemo->emSetzAxisValueOfPiontcloud(0, deepZmin, deepZmax);
}

//width
void MainWindow::lowerValueChangedSlotw(int value)
{
    ui->spinBox_wl->setValue(value);
    ui->horizontalSlider_w->setLowerValue(value);
    roiStartX = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::upperValueChangedSlotw(int value)
{
    ui->spinBox_wr->setValue(value);
    ui->horizontalSlider_w->setUpperValue(value);
    roiEndX = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::lowerValueChangedSlotw()
{
    int value = ui->spinBox_wl->value();
    ui->horizontalSlider_w->setLowerValue(value);
    roiStartX = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::upperValueChangedSlotw()
{
    int value = ui->spinBox_wr->value();
    ui->horizontalSlider_w->setUpperValue(value);
    roiEndX = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

//hight
void MainWindow::lowerValueChangedSloth(int value)
{
    ui->spinBox_hl->setValue(value);
    ui->horizontalSlider_h->setLowerValue(value);
    roiStartY = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::upperValueChangedSloth(int value)
{
    ui->spinBox_hr->setValue(value);
    ui->horizontalSlider_h->setUpperValue(value);
    roiEndY = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::lowerValueChangedSloth()
{
    int value = ui->spinBox_hl->value();
    ui->horizontalSlider_h->setLowerValue(value);
    roiStartY = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::upperValueChangedSloth()
{
    int value = ui->spinBox_hr->value();
    ui->horizontalSlider_h->setUpperValue(value);
    roiEndY = value;
    emDemo->emSetRoiValue(0, roiStartX, roiStartY, roiEndX-roiStartX+1, roiEndY-roiStartY+1);
}

void MainWindow::setExposureTime()
{
    bool ok;
    QString strExpT = ui->lineEdit_exptime->text();
    emDemo->emSetExposureTime(0, strExpT.toInt(&ok), 2);
}
