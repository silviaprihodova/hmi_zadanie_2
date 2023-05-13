#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#ifdef _WIN32
#include<windows.h>
#endif
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"

#include <QJoysticks.h>

struct Point{
    int x;
    int y;
};

namespace Ui {
class MainWindow;
}


///toto je trieda s oknom.. ktora sa spusti ked sa spusti aplikacia.. su tu vsetky gombiky a spustania...
class MainWindow : public QMainWindow
{
    Q_OBJECT


public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    cv::Mat frame_2[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

int processThisCamera(cv::Mat cameraData);
int processThisSkeleton(skeleton skeledata);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    int getMouseClickedX()
    {
        return this->mouseClickedX;
    }

    int getMouseClickedY()
    {
        return this->mouseClickedY;
    }

    void setMouseClickedY(int mouseClickedY)
    {
        this->mouseClickedY =mouseClickedY ;
    }

    void setMouseClickedX(int mouseClickedX)
    {
        this->mouseClickedX =mouseClickedX ;
    }


private:

    //--skuste tu nic nevymazat... pridavajte co chcete, ale pri odoberani by sa mohol stat nejaky drobny problem, co bude vyhadzovat chyby
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int updateSkeletonPicture;
     skeleton skeleJoints;
     int datacounter;
     QTimer *timer;

    void mousePressEvent(QMouseEvent *ev );

    void updateRobotState(long double encoderLeft, long double encoderRight);

    void floodFill(int sp);
    void findingPath(int sp);
    void clearMap();
    void robotMove();
    void expandWalls();

    int sign(double angle);

     int collision = 0;
     double D = 0;
     long double tickToMeter = 0.000085292090497737556558;
     double trans = 0.0;
     double rot = 0.0;
     double ramp_trans = 0.0, ramp_rot = 0.0;
     bool forward = true;
     vector<vector<int>>  points;
     vector<vector<int>>  pointsExpanded;
     vector<pair<int, int>> setPoints;
     std::list <Point> breakPoints;
     int mouseClickedX =0, mouseClickedY=0;
     int counterClicked = 0 ;
     int clickedStart = 0;
     int beginOfWindowX;
     int beginOfWindowY;
     int endOfWindowX;
     int endOfWindowY;
     int rows;
     int cols;
     int sizeY;
     int sizeX;
     int robotVisible = 0;
     int cameraVisible = 0;
     boolean clearingMap = false;
     boolean mision = false;
     int sleepCounter=0;
     int pom = 0;
     boolean clickedSecond = false;
     double robotOldSpeed = 0;
     double rotOldSpeed =0;
     int bat_charge =0;
     // video
     int frameWidth = 853; // sirka ako snima kamera obr
     int frameHeight = 480;
     int fps = 5;
     cv::VideoWriter videoWriter;

     long double wheelbase = 0.23;
     const unsigned short ENCODER_MAX = 65535;

     long double oldEncoderLeft;
     long double oldEncoderRight;

     struct RobotState {
         double x;
         double y;
         double angle;
     } state;

     struct Node {
         int row, col, val;
         Node(int r, int c, int v) : row(r), col(c), val(v) {}
     };


     enum directions {
         RIGHT, UP, LEFT,  DOWN
     };

     QJoysticks *instance;

     double forwardspeed = 0;//mm/s
     double rotationspeed = 0;//omega/s
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo




};

#endif // MAINWINDOW_H
