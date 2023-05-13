#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <cmath>
#include <QFile>
#include <QTextStream>
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QStringList>
#include <qevent.h>
#include "robot.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QImage>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <queue>

// PRIHODOVA S., SVEC D.

bool stop_switch = true;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{


    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
  //  ipaddress="192.168.1.15";//192.168.1.14toto je na niektory realny robot.. na lokal budete davat "127.0.0.1"
    ipaddress="127.0.0.1"; //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;

    // nacitanie mapy do pola
        ifstream file("C:/Users/41sil/Desktop/2. semester/HMI/hmi_zadanie2/SimulatorMap.txt");

        string line;
        while (getline(file, line)) {

            vector<int> row;

            for (char c : line)
            {
                if(c == '0')
                    row.push_back(0);
                else if(c == '1')
                    row.push_back(1);
            }

            points.push_back(row);
        }

//        for (int i = 0; i < points.size(); i++) {
//               for (int j = 0; j < points[i].size(); j++) {
//                   cout << points[i][j] << " ";
//               }
//               cout << endl;
//           }


        pointsExpanded = points; // nazov nezodpoveda skutocnosti, je to opacne points = expanded, pointsExpanded = not expanded, tak vyslo
        this->expandWalls();

        if(!file.is_open())
        {
          std::cout << "Map could not be opened" << std::endl;
        }
        else
        {
          std::cout << "Map was opened" << std::endl;
        }
        file.close();


        videoWriter.open("Video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(frameWidth, frameHeight));

        if (!videoWriter.isOpened())
        {
            cout<< "nepodarilo sa vytvorit video" << endl;
        }

    datacounter=0;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::red);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    QRect battery1;
    battery1.setHeight(rect.height()/15);
    battery1.setWidth(rect.width()/14);
    battery1.translate(rect.bottomRight().x()-rect.height()/15 - rect.height()/20, rect.bottomRight().y()-rect.height()/14-5);


    QRect battery2;
    battery2.setHeight(rect.height()/25);
    battery2.setWidth(rect.width()/200);
    battery2.translate(rect.bottomRight().x()-rect.height()/15 - rect.height()/17, rect.bottomRight().y()-rect.height()/14+3);

    QRect battery3;
    battery3.setHeight(rect.height()/20);
    battery3.setWidth(rect.width()/17);
    battery3.translate(rect.bottomRight().x()-rect.height()/15 - rect.height()/25, rect.bottomRight().y()-rect.height()/14);

    QRect battery_charge1;
    battery_charge1.setHeight(rect.height()/24);
    battery_charge1.setWidth(rect.width()/100);
    battery_charge1.translate(rect.bottomRight().x()-rect.height()/15 +rect.height()/35, rect.bottomRight().y()-rect.height()/14 +2);

    QRect battery_charge2;
    battery_charge2.setHeight(rect.height()/24);
    battery_charge2.setWidth(rect.width()/100);
    battery_charge2.translate(rect.bottomRight().x()-rect.height()/15+rect.height()/130, rect.bottomRight().y()-rect.height()/14 +2);


    QRect battery_charge3;
    battery_charge3.setHeight(rect.height()/24);
    battery_charge3.setWidth(rect.width()/100);
    battery_charge3.translate(rect.bottomRight().x()-rect.height()/15 - rect.height()/70, rect.bottomRight().y()-rect.height()/14 +2);


    QRect battery_charge4;
    battery_charge4.setHeight(rect.height()/24);
    battery_charge4.setWidth(rect.width()/100);
    battery_charge4.translate(rect.bottomRight().x()-rect.height()/15 - rect.height()/30, rect.bottomRight().y()-rect.height()/14 +2);



    QRect rect3;
    rect3.setHeight(rect.height()/2);
    rect3.setWidth(rect.width()*0.75);
    rect3.translate(rect.topLeft().x() + rect.width()*0.25/2, rect.topLeft().y() + rect3.height()/2);

    QRect rect2;
    rect2= ui->frame_2->geometry();//ziskate porametre stvorca, do ktoreho chcete kreslit

    // zvascenie pri maximalizacii
    rect2.setHeight(rect.height()/3);
    rect2.setWidth(rect.width()/3);
    rect2.translate(rect.bottomRight().x() - rect2.width(), rect.bottomRight().y() - rect2.height());

    //
    rows = points.size();
    cols = points[1].size();
    sizeY = (rect.height()/rows)+1;
    sizeX = (rect.width()/cols)+1;

    beginOfWindowX = rect.x();
    beginOfWindowY = rect.y();
    endOfWindowX = rect.right();
    endOfWindowY = rect.bottom();

    // vykreslenie mapy
    if(clickedStart == 1)
    {
        for (int i = 0; i < rows; i++) {
               for (int j = 0; j < cols; j++) {

                   if(pointsExpanded[i][j] == 0) {//|| pointsExpanded[i][j] >= 2){
                       painter.setPen(Qt::white);
                       painter.setBrush(Qt::white);

                   }
                   else if(pointsExpanded[i][j] == 1){
                       painter.setPen(Qt::black);
                       painter.setBrush(Qt::black);

                   }
                   else if(pointsExpanded[i][j] == -1){
                       painter.setPen(Qt::yellow);
                       painter.setBrush(Qt::yellow);

                   }
                    painter.drawRect((rect.x()+j*sizeX),(rect.y()+i*sizeY),sizeX, sizeY);
               }

        }

    }


    if (robotVisible == 1){
        long endLineX = state.x*sizeX/10+rect.x()+10*cos(state.angle);
        long endLineY = state.y*sizeY/10+rect.y()+10*sin(state.angle);
        painter.setPen(pero);
        painter.drawEllipse(QPoint(state.x*sizeX/10+rect.x(), state.y*sizeY/10+rect.y()),15*sizeX/10,15*sizeY/10);
        painter.drawLine(state.x*sizeX/10+rect.x(),state.y*sizeY/10+rect.y(), endLineX, endLineY);
    }

    QFont font("Arial", 40);
    QColor color(255, 0, 0, 128); // nastavenie farby na červenú s priehľadnosťou 50%

    int firstPoint = 1;

    if(updateLaserPicture==1) ///ak mam nove data z lidaru
    {
       updateLaserPicture=0;

        for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
        {
            // warning
            if ((copyOfLaserData.Data[k].scanDistance/10 < 40) && firstPoint == 1 && copyOfLaserData.Data[k].scanDistance != 0) // cca 20 polomer robota + 20 cm k prekazke warning
            {
                painter.setBrush(color);
                painter.setFont(font);
                painter.setPen(color);
                painter.drawRect(rect3);
                painter.drawText(rect3, Qt::AlignCenter, "WARNING");
                firstPoint += 1;
            Beep(1000, 500);
            }
        }


    }

    painter.setPen(QColor(192, 192, 192, 200));
    painter.setBrush(QColor(192, 192, 192, 200));
    painter.drawRect(battery1);
    painter.drawRect(battery2);

    painter.setPen(QColor(224, 224, 224, 200));
    painter.setBrush(QColor(224, 224, 224, 200));
    painter.drawRect(battery3);

   // printf("%d baterka\n",bat_charge);

    if(bat_charge <= 64){
        painter.setPen(QColor (255, 0, 0, 230));
        painter.setBrush(QColor (255, 0, 0, 230));
        painter.drawRect(battery_charge1);
    }else if (bat_charge > 64 && bat_charge <= 128){
        painter.setPen(QColor (0, 255, 0, 230));
        painter.setBrush(QColor (0, 255, 0, 230));
        painter.drawRect(battery_charge1);
        painter.drawRect(battery_charge2);

    }else if (bat_charge > 128 && bat_charge <= 192){
        painter.setPen(QColor (0, 255, 0, 230));
        painter.setBrush(QColor (0, 255, 0, 230));
        painter.drawRect(battery_charge1);
        painter.drawRect(battery_charge2);
        painter.drawRect(battery_charge3);

    }else if (bat_charge > 192 && bat_charge <= 255){
        painter.setPen(QColor (0, 255, 0, 230));
        painter.setBrush(QColor (0, 255, 0, 230));
        painter.drawRect(battery_charge1);
        painter.drawRect(battery_charge2);
        painter.drawRect(battery_charge3);
        painter.drawRect(battery_charge4);
    }


}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
//     ui->lineEdit_2->setText(QString::number(robotX));
//     ui->lineEdit_3->setText(QString::number(robotY));
//     ui->lineEdit_4->setText(QString::number(robotFi));
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{

     bat_charge = robotdata.Battery;
    ///tu mozete robit s datami z robota
    /// ale nic vypoctovo narocne - to iste vlakno ktore cita data z robota
    ///teraz tu posielam rychlosti na zaklade toho co setne joystick a vypisujeme data z robota(kazdy 5ty krat. ale mozete skusit aj castejsie). vyratajte si polohu. a vypiste spravnu
    /// tuto joystick cast mozete vklude vymazat,alebo znasilnit na vas regulator alebo ake mate pohnutky... kazdopadne, aktualne to blokuje gombiky cize tak
//    if(forwardspeed==0 && rotationspeed!=0)
//        robot.setRotationSpeed(rotationspeed);
//    else if(forwardspeed!=0 && rotationspeed==0)
//        robot.setTranslationSpeed(forwardspeed);
//    else if((forwardspeed!=0 && rotationspeed!=0))
//        robot.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
//    else
//        robot.setTranslationSpeed(0);


///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    sleepCounter++;
    updateRobotState(robotdata.EncoderLeft, robotdata.EncoderRight);
  //  cout<<state.x <<", " << state.y << endl;
    if(mision)
        this->robotMove();
    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{


    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru


    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    if(actIndex>-1 && mision)/// ak zobrazujem data z kamery ak aspon niektory frame vo vectore je naplneny
    {
        QImage image = QImage((uchar*)frame[actIndex].data, frameWidth, frameHeight, frame[actIndex].step, QImage::Format_RGB888);
        cv::Mat inputMat(image.height(), image.width(), CV_8UC3, (cv::Scalar*)image.scanLine(0)); //matica, format z kt vie cv pracovat

        videoWriter<<inputMat;
       // cout << "ukladanie obrazka" << endl;
    }
    return 0;
}

///toto je calback na data zo skeleton trackera, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z trackera
int MainWindow::processThisSkeleton(skeleton skeledata)
{
    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    updateSkeletonPicture=1;
    return 0;
}

void MainWindow::mousePressEvent (QMouseEvent *ev ) {

    if (mision)
        return;

    if(clickedStart == 1){
        setMouseClickedX(ev->x());
        setMouseClickedY(ev->y());

        if((getMouseClickedX() > beginOfWindowX && getMouseClickedX()< endOfWindowX) && (getMouseClickedY()> beginOfWindowY && getMouseClickedY()< endOfWindowY))
        {
            int indexX = (getMouseClickedX()-beginOfWindowX)/sizeX;
            int indexY = (getMouseClickedY()-beginOfWindowY)/sizeY;

            if(points[indexY][indexX] == 1)
            {
                cout << "Nedostiahnutelny bod" << endl;
                return;
            }

            setPoints.emplace_back(indexX,indexY);
            counterClicked++;
            pointsExpanded[indexY][indexX] = -1;

            if (counterClicked%2 == 0)
            {
                pom = 0;
                clickedSecond = true;
            }

//            std::cout<<"SP X "<< getMouseClickedX() <<std::endl;
//            std::cout<<"SP Y "<< getMouseClickedY() <<std::endl;

        }
    }
}

void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    //---simulator ma port 8889, realny robot 8000
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    robot.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&MainWindow::processThisSkeleton,this,std::placeholders::_1));
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();

    //ziskanie joystickov
    instance = QJoysticks::getInstance();
    clickedStart = 1;

    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );


    oldEncoderLeft = robotdata.EncoderLeft;
    oldEncoderRight = robotdata.EncoderRight;


}

void MainWindow::updateRobotState(long double encoderLeft, long double encoderRight) {

    // calculate the difference in encoder counts for each wheel
    long double deltaLeft = encoderLeft - oldEncoderLeft;
    long double deltaRight = encoderRight - oldEncoderRight;

    // handle encoder overflow by adding or subtracting the maximum encoder value
    if (deltaLeft > ENCODER_MAX / 2) {
        deltaLeft -= ENCODER_MAX;
    } else if (deltaLeft < -ENCODER_MAX / 2) {
       deltaLeft += ENCODER_MAX;
    }
    if (deltaRight > ENCODER_MAX / 2) {
        deltaRight -= ENCODER_MAX;
    } else if (deltaRight < -ENCODER_MAX / 2) {
        deltaRight += ENCODER_MAX;
    }

    // calculate the distance traveled by each wheel
    long double distLeft = deltaLeft * tickToMeter;
    long double distRight = deltaRight * tickToMeter;

    // calculate the average distance traveled by the robot
    long double distance = (distLeft + distRight) / 2;

    // calculate the change in angle of the robot
    long double deltaAngle = (distRight - distLeft) / wheelbase;

    if(distRight == distLeft){
        // update the robot's position and orientation
        state.x += (distance * cos(state.angle + deltaAngle / 2))*100;
        state.y += (distance * sin(state.angle + deltaAngle / 2))*100;
        state.angle += deltaAngle;

    }
    else{

        state.x += (((wheelbase*(distRight + distLeft))/(2*(distRight - distLeft)))*(sin(state.angle+deltaAngle)-sin(state.angle)))*100;
        state.y -= (((wheelbase*(distRight + distLeft))/(2*(distRight - distLeft)))*(cos(state.angle+deltaAngle)-cos(state.angle)))*100;
        state.angle += deltaAngle;
    }
    oldEncoderLeft = encoderLeft;
    oldEncoderRight = encoderRight;

}

void MainWindow::floodFill(int sp)
{
    int goalX = setPoints[sp+1].first;
    int goalY =  setPoints[sp+1].second;
    int startX = setPoints[sp].first;
    int startY = setPoints[sp].second;

    breakPoints.push_front(Point{(goalX+1)*10, (goalY+1)*10});
    cout << goalX << ", " << goalY << endl;

    int endValue = -111;

    int pathNumber = 2;
    int row =0; int col = 0;
    points[goalY][goalX] = endValue;

    points[startY][startX]= pathNumber;

    int val = 2;
        queue<Node> q;
        q.push(Node(startY, startX, val));

        while (!q.empty()) {
            Node node = q.front();
            q.pop();

            row = node.row;
            col = node.col;
            val = node.val;

            if (row < 0 || row >= points.size() || col < 0 || col >= points[0].size()) {
                continue;
            }

            if (points[row][col] == endValue) {
                points[row][col] = 999;
                break;
            }

            if (points[row][col] != 0 && points[row][col] != 2) {
                continue;
            }
            if (points[row][col] != 2) {
                points[row][col] = val;
            }
    //        cout << "inserting to: " << row  << " " << col << " with value: " << val << endl;
            q.push(Node(row-1, col, val+1));
            q.push(Node(row+1, col, val+1));
            q.push(Node(row, col-1, val+1));
            q.push(Node(row, col+1, val+1));
        }

}

void MainWindow::findingPath(int sp){

    int indexPathY= setPoints[sp+1].first;
    int indexPathX= setPoints[sp+1].second;

    int tmpXForSwitch = indexPathX;
    int tmpYForSwitch = indexPathY;
    int oldPathValue = points[indexPathX][indexPathY];

    int pom = 0;

    int direction = rand()%4;

    while(true){

        tmpXForSwitch = indexPathX;
        tmpYForSwitch = indexPathY;
        switch (direction) {
        case RIGHT:
            tmpXForSwitch +=1;
            break;
        case UP:
            tmpYForSwitch -=1;
            break;
        case LEFT:
            tmpXForSwitch -=1;
            break;
        case DOWN:
            tmpYForSwitch +=1;
            break;
        }

        int newPathValue = points[tmpXForSwitch][tmpYForSwitch];

        if (newPathValue == 1  || oldPathValue <= newPathValue || newPathValue == 0){
            direction = rand()%4;

            if(pom > 0)
            {
                //cout << indexPathY+1 << ", " << indexPathX+1 << endl;
                breakPoints.push_front(Point{(indexPathY+1)*10, (indexPathX+1)*10});
            }
            pom = 0;
            continue;
        }

        if (newPathValue == 2)
        {
            points[tmpXForSwitch][tmpYForSwitch] = 999;
            pointsExpanded[setPoints[sp+1].second][setPoints[sp+1].first] = -1;

            break;
        }

        pom++;
        indexPathX = tmpXForSwitch;
        indexPathY = tmpYForSwitch;
        points[tmpXForSwitch][tmpYForSwitch] = 999;
       // cout << tmpXForSwitch << ", " << tmpYForSwitch << endl;

        oldPathValue = newPathValue;
    }

}

int MainWindow::sign(double angle) {
    if (angle > 0) {
        return 1; // Kladné znamienko
    } else if (angle < 0) {
        return -1; // Záporné znamienko
    } else {
        return 0; // Nulový uhol
    }
}

void MainWindow::expandWalls(){
    int expansion = 2;

    vector<vector<int>> expandedMap(points.size(), vector<int>(points[0].size(), 0));

    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < points[0].size(); j++) {
            if (points[i][j] == 1) {
                for (int k = i-expansion; k <= i+expansion; k++) {
                    for (int l = j-expansion; l <= j+expansion; l++) {
                        if (k >= 0 && k < points.size() && l >= 0 && l < points[0].size()) {
                            expandedMap[k][l] = 1;
                        }
                    }
                }
            } else {
                expandedMap[i][j] = points[i][j];
            }
        }
    }

    points = expandedMap;

}

void  MainWindow::robotMove(){

    if (!(breakPoints.size() > 0))
    {
        if(clickedSecond)
        {
            pom++;
            if(pom == 1)
            {
                clickedSecond = false;
                sleepCounter = 0;
                return;
            }

        }

        if(sleepCounter > 500)
        {
            mision = false;
            cout<< "Ciel" << endl;
        }
        else{
            cout<< "Cakam" << endl;
        }

        robot.setTranslationSpeed(0);

        return;
    }

    Point nextPoint = breakPoints.front();
    int x1 = state.x;
    int y1 = state.y;
    int x2 = nextPoint.x;
    int y2 = nextPoint.y;

    double dx = x2 - x1;
    double dy = y2 - y1;

    double distanceThreshold = 1; // prah vzdialenosti
    double angleThreshold = 0.04 ; // prah uhla

    double distance = sqrt(pow(dx, 2) + pow(dy, 2));
    double angle = atan2(dy, dx) - state.angle;
    angle = fmod(angle+PI, PI*2)-PI;


    if (distance < distanceThreshold)
    {
        breakPoints.pop_front();
        return;
    }

    // otacanie
    double rotSpeed = (3.14159/3) * sign(angle);

    if ( fabs(angle) > angleThreshold) {
        robot.setRotationSpeed(rotSpeed);
        return;
    }


    // chod vpred
    double transSpeed = 20 * distance;
    transSpeed = transSpeed<500 ? transSpeed : 500;
    // rozbeh rampa
    double dif = transSpeed - robotOldSpeed;
    if (dif > 10)
        robotOldSpeed+=10;
    else
        robotOldSpeed = transSpeed;

    robot.setTranslationSpeed(robotOldSpeed);
    //robot.setTranslationSpeed(200);

}

void MainWindow::on_pushButton_2_clicked() //robot
{

    state.x = 50;
    state.y = 50;
    state.angle = 0;
    robotVisible = 1;
    setPoints.emplace_back((state.x)/sizeX,(state.y)/sizeY);
    ui->pushButton_2->setVisible(false);

}

void MainWindow::on_pushButton_3_clicked() //mision
{
    if (mision)
    {
        cout<< "Pockaj kym skonci misia" << endl;
        return;
     }

    this->clearMap();
    this->floodFill(counterClicked-1);
    this->findingPath(counterClicked-1);
    mision = true;

//    for (int i = 0; i < rows; i++) {
//       for (int j = 0; j < cols; j++) {
//           cout << points[i][j] << " ";
//       }
//       cout << endl;
//   }


}

void MainWindow::on_pushButton_6_clicked() //NEVYUZIVA SA //left
{
robot.setRotationSpeed(3.14159/2);

}

void MainWindow::on_pushButton_5_clicked() //NEVYUZIVA SA //right
{
 videoWriter.release();

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    if(stop_switch == true){
         stop_switch = false;
    }else{
         stop_switch = true;
    }

    robot.setTranslationSpeed(0);


}

void MainWindow::clearMap(){

    clearingMap = true;

    for (int i = 0; i < rows; i++) {
           for (int j = 0; j < cols; j++) {

               if(points[i][j] >= 2){
                   points[i][j] = 0;
               }

           }

    }

}


void MainWindow::on_pushButton_clicked() // camera
{
       for(int i = 0; i < points.size(); i++){
           for(int j = 0; j < points[0].size(); j++){
               if(pointsExpanded[i][j] == -1){
                   pointsExpanded[i][j] = 0;
               }
           }
       }

}

void MainWindow::getNewFrame()
{

}




