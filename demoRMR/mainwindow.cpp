#include "mainwindow.h"
#include "BugAlgorithm.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>

/** Custom variables -> move to interface later **/
#include "Odometry.h"
#include "Controller.h"
#include <fstream>
#include <iostream>

Odometry* control = new Odometry();
OdometryData odData = {0};
Controller* controller;
bool initialStart = true;
BugAlgorithm* bugAlg;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

   // tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna

    ipaddress="127.0.0.1";
//    ipaddress = "192.168.1.13";
//    cap.open("http://192.168.1.11:8000/stream.mjpg");

    ui->setupUi(this);
    datacounter=0;

//    timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));

    actIndex=-1;
    useCamera1=false;
    datacounter=0;

    // Constructor objects
    controller = new Controller(&robot, &odData, 4.5, 1.2);
//    controller->checkpoints.push_back({4.5, 1.85});
//    controller->checkpoints.push_back({0.0, 3.0});
    bugAlg = new BugAlgorithm({4.5, 1.85});
}

MainWindow::~MainWindow()
{
    delete ui;
    delete control;
    delete bugAlg;

    std::cout << "All closed and destroyed\n\n";
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)
    {
        std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1)
        {
            updateLaserPicture=0;
            pero.setColor(Qt::green);
            painter.setPen(pero);
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
}

void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}

int step = 0;

int MainWindow::processThisRobot(TKobukiData robotdata)
{
    // Set starting location and rotation
    if (initialStart)
    {
        odData.distLeftWheel = 0;
        odData.distRightWheel = 0;
        odData.leftWheelTicks = robotdata.EncoderLeft;
        odData.rightWheelTicks = robotdata.EncoderRight;
        odData.posX = 0;
        odData.posY = 0;
        odData.initRotation = robotdata.GyroAngle / 100.0;
        odData.rotation = robotdata.GyroAngle / 100.0;

        emit uiValuesChanged(0, 0, odData.rotation);
        initialStart = false;
        return 0;
    }
    if (!controller->b_finishReached)
    {
        control->readOdometry(robotdata, &odData, controller->fStopLidar);
//        if (bugAlg->isWallFollowing)
//        {
//            controller->controllerOutput.forwardSpeed = 0;
//            controller->controllerOutput.rotationSpeed = 0;
//            controller->controllerOutput.reached = 0;

            controller->regulateDynamic(bugAlg->getLidarData());
//        }
//        else
//        {
//            controller->regulate();
            controller->regulateWallFollow();
//        }
    }

    if(datacounter%5)
    {
        emit uiValuesChanged(odData.posX, odData.posY, odData.rotation);
    }
    datacounter++;

    return 0;

}

int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru

    //Laser data processing
    if (!controller->fRotating && !controller->b_finishReached)
    {
        bugAlg->updateRobotState(odData);
        bugAlg->updateLidar(laserData);
        bugAlg->proccess(controller->checkpoints);
    }
    // End laser data processing

    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia

    return 0;

}


int MainWindow::processThisCamera(cv::Mat cameraData)
{
    cameraData.copyTo(frame[(actIndex+1)%3]);
    actIndex=(actIndex+1)%3;
    updateLaserPicture=1;
    return 0;
}

void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
      /*  laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
      robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);*/
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters("http://" + ipaddress + ":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
//    robot.setCameraParameters("http://" + ipaddress + ":8000/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.robotStart();



    instance = QJoysticks::getInstance();

    /* Enable the virtual joystick */
  /*  instance->setVirtualJoystickRange(1);
    instance->setVirtualJoystickEnabled(true);
    instance->setVirtualJoystickAxisSensibility(0.7);*/
    //instance->
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    robot.setTranslationSpeed(500);
    /*std::vector<unsigned char> mess=robot.setTranslationSpeed(500);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);
  /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_6_clicked() //left
{
    robot.setRotationSpeed(3.14159/2);
  /*  std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_5_clicked()//right
{
    robot.setRotationSpeed(-3.14159/2);
   /* std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);
  /*  std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }*/
}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
