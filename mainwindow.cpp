#include "mainwindow.h"
#include "ui_mainwindow.h"

MarkerTriangle::MarkerTriangle() :
    c_rotation(1, 0, 0, 0),
    rotationChange(false)
{
    //default triangl
    c_vertices << Vector3d(0, 0, 0)
               << Vector3d(20, 0, 0)
               << Vector3d(0, 30, 0);

    //default rotation matrix //Доработать!
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            if(i==j)
                c_matrix(i,j) = 1;
            else
                c_matrix(i,j) = 0;
        }
    }

}

void MarkerTriangle::setModelPoint(int ind, Vector3d modelPoint)
{
    if(ind==0)
    {
            Vector3d translation = modelPoint - c_center;
            c_center = modelPoint;
            for (int i = 0; i < 3; ++i)
            {
                c_vertices[i] = c_vertices.at(i) - translation;
            }
    }
    else
    {
        c_vertices[ind-1] = modelPoint - c_center;
    }
}

void MarkerTriangle::setRotation(Quaterniond quat)
{
    c_rotation = quat;
    rotationChange = true;
}

void MarkerTriangle::setRotation(Vector3d angles)
{
    Quaterniond quat = Quaterniond(  AngleAxisd(angles(0), Vector3d(1, 0, 0))
                                     * AngleAxisd(angles(1), Vector3d(0, 1, 0))
                                     * AngleAxisd(angles(2), Vector3d(0, 0, 1)) );
    c_rotation = quat;
    rotationChange = true;
}

QVector<Vector3d> MarkerTriangle::edges()
{
    Matrix3d rotMat = rotationMatrix(); //Доработать
    QVector<Vector3d> points;
    for (int i = 0; i < 3; ++i)
    {
        points << c_center + c_translation + rotMat*c_vertices.at(i);
    }
    return points;
}

QVector<Vector2d> MarkerTriangle::imagePoints()
{
    QVector<Vector2d> points;
    QVector<Vector3d> worldPoints = edges();
    for (int i = 0; i < 3; ++i)
        points << worldPoints.at(i).eval().hnormalized();

    return points;
}

Matrix3d MarkerTriangle::rotationMatrix()
{
    if(rotationChange)
    {
        c_matrix = c_rotation.toRotationMatrix();
        rotationChange = false;
        return c_matrix;
    }
    else
        return c_matrix;
}

QVector<Vector3d> MarkerTriangle::modelPoins() const
{
    QVector<Vector3d> points;
    points << c_center;
    for (int i = 0; i < 3; ++i)
    {
        points << c_center + c_vertices.at(i);
    }
    return points;
}

//---------------------------------------------------------------------------------------------------------

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    modelAccepted(false),
    exoskeleton(0)
{
    srv = new SimpleServer(this);
    srv->hide();
    srv->setPort(5872);
    srv->setActive(true);
    connect(srv, SIGNAL(clientReceiveCommand(QTcpSocket*,QString,StringMap)),
            this, SLOT(onRequest(QTcpSocket*,QString,StringMap)));
    connect(srv, SIGNAL(clientConnect(QTcpSocket*)), this, SLOT(onClientConnect(QTcpSocket*)));
    connect(srv, SIGNAL(clientDisconnect(QTcpSocket*)), this, SLOT(onClientDisconnect(QTcpSocket*)));

    stsConnection = new QLabel("no clients");
    stsConnection->setMinimumWidth(200);

    settings = new QSettings("config.ini",QSettings::IniFormat);

    triangl = new MarkerTriangle();

    loadSettings();
    setMainWindow();

    for (int i = 0; i < 6; ++i)
    {
        updateTable(i);
    }

    estimate();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadSettings()
{
    for (int i = 0; i < 4; ++i)
    {
        QString pref = "model/point_" + QString::number(i);
        if(settings->contains(pref + "/x"))
        {
            Vector3d point;
            point(0) = settings->value(pref + "/x").toDouble();
            point(1) = settings->value(pref + "/y").toDouble();
            point(2) = settings->value(pref + "/z").toDouble();

            triangl->setModelPoint(i, point);
        }
    }

    if(settings->contains("translation/x"))
    {
        Vector3d translationt;
        translationt(0) = settings->value("translation/x").toDouble();
        translationt(1) = settings->value("translation/y").toDouble();
        translationt(2) = settings->value("translation/z").toDouble();

        triangl->setTranslation(translationt);
    }

    if(settings->contains("rotation/x"))
    {
        Vector3d rotationt;
        rotationt(0) = settings->value("rotation/x").toDouble();
        rotationt(1) = settings->value("rotation/y").toDouble();
        rotationt(2) = settings->value("rotation/z").toDouble();

        triangl->setRotation(rotationt);
    }

}

void MainWindow::setMainWindow()
{
    ui->setupUi(this);
    setWindowTitle("Pose Estimation: P3P");
    setStyleSheet("QLineEdit { font-size: 14px}");

    ui->statusBar->addWidget(stsConnection);

    handler = DisplayTable::handlerPoint();
    connect(handler, SIGNAL(valueChanged(int,int,double)), this, SLOT(onTableHandler(int,int,double)));


    DisplayTable *table = new DisplayTable("Model", 3, 4);
    table->setRawTitles("X, Y, Z");
    table->setColumTitles("center, A, B, C");

    table = new DisplayTable("Translation", 1, 3);
    table->setColumTitles("X, Y, Z");

    table = new DisplayTable("Rotation", 1, 3);
    table->setColumTitles("X, Y', Z''");

    table = new DisplayTable("Quaternion", 1, 4);
    table->setReadOnly(true);
    table->setColumTitles("W, X, Y, Z");
    table->setEditsMinimalWidth(50);

    table = new DisplayTable("World Coordinates 3D", 3, 4);
    table->setColumTitles("center, A, B, C");
    table->setRawTitles("X, Y, Z");
    table->setEditsMinimalWidth(70);

    table = new DisplayTable("Image Coordinates 2D", 2, 3);
    table->setColumTitles("A, B, C");
    table->setRawTitles("X, Y");

    table = new DisplayTable("Theia Kneip Solutions", 4, 7);
    table->setColumTitles("x tans, y trans, z trans, q.w, q.x, q.y, q.z");
    table->setRawTitles("SOL 1, SOL 2, SOL 3, SOL 4");
    table->setEditsMinimalWidth(50);

    table = new DisplayTable("OpenGV Kneip Solutions", 4, 10);
    table->setColumTitles("x tans, y trans, z trans, q.w, q.x, q.y, q.z, A er, B er, C er");
    table->setRawTitles("SOL 1, SOL 2, SOL 3, SOL 4");
    table->setEditsMinimalWidth(50);

    table = new DisplayTable("OpenGV Gao Solutions", 4, 10);
    table->setColumTitles("x tans, y trans, z trans, q.w, q.x, q.y, q.z, A er, B er, C er");
    table->setRawTitles("SOL 1, SOL 2, SOL 3, SOL 4");
    table->setEditsMinimalWidth(50);


    QVBoxLayout *projLa = new QVBoxLayout();
    projLa->addWidget(handler->table(0));
    projLa->addSpacing(30);
    projLa->addWidget(handler->table(1));
    projLa->addWidget(handler->table(2));
    projLa->addWidget(handler->table(3));
    projLa->addSpacing(30);
    projLa->addWidget(handler->table(4));
    projLa->addWidget(handler->table(5));
    projLa->addStretch(1);

    QVBoxLayout *solLa = new QVBoxLayout();
    solLa->addWidget(handler->table(6));
    solLa->addWidget(handler->table(7));
    solLa->addWidget(handler->table(8));
    solLa->addStretch(1);

    QHBoxLayout *mainLa = new QHBoxLayout();
    mainLa->addLayout(projLa);
    mainLa->addLayout(solLa);
    mainLa->addStretch(1);
    ui->centralWidget->setLayout(mainLa);
}

//void MainWindow::updateAndEstimate()
//{

//    for (int i = 0; i<3; i++)
//    {
//        worldPoints3D[i] = projectionMatrix * modelPoints3D[i].homogeneous();
//        imagePoints2D[i] = worldPoints3D[i].eval().hnormalized();

//        for(int j=0; j<3; j++)
//            worldPointsMap.value(i*10+j)->setText(QString::number(worldPoints3D[i](j)));
//        for(int j=0; j<2; j++)
//            imagePointsMap.value(i*10+j)->setText(QString::number(imagePoints2D[i](j)*SCALE_FACTOR));
//    }

//    std::vector<Matrix3d> rotations;
//    std::vector<Vector3d> translations;
//    PoseFromThreePoints(imagePoints2D, modelPoints3D, &rotations, &translations);

//    for(int i=0; i<rotations.size(); i++)
//    {
//        Vector3d angMat = rotations[i].eulerAngles(0, 1, 2);
//        for(int j=0; j<3; j++)
//        {
//            solutionsMap.value(i*10+j)->setText(QString::number(angMat(j)*180/M_PI, 'f', 0));
//            solutionsMap.value(i*10+3+j)->setText(QString::number(translations[i](j), 'f', 2));
//        }

//        Quaterniond qua(rotations[i]);
//        DisplayTable *table = handler->table("Rotation in Quaternions");
//        table->setValue(i*4+0, qua.w(), 'f', 3);
//        table->setValue(i*4+1, qua.x(), 'f', 3);
//        table->setValue(i*4+2, qua.y(), 'f', 3);
//        table->setValue(i*4+3, qua.z(), 'f', 3);
//    }


//}


//void MainWindow::estimataByGaoOpenGVAndSend(QTcpSocket *socket)
//{
//    std::vector<Matrix3d> rotations;
//    std::vector<Vector3d> translations;
//    PoseFromThreePoints(imagePoints2D, modelPoints3D, &rotations, &translations);


//    translation_t position;
//    rotation_t rotation;
//    points_t points;
//    bearingVectors_t bearingVectors;

//    CentralAbsoluteAdapter adapter(
//                bearingVectors,
//                points,
//                rotation );





//    StringMap resp;
//    QString str;
//    for(int i=0; i<rotations.size(); i++)
//    {
//        str.clear();
//        for(int l=0; l<3; l++)
//            for(int m=0; m<3; m++)
//                str.append(QString::number(rotations.at(i)(l,m), 'f', 3) + "|");
//        for(int l=0; l<3; l++)
//            str.append(QString::number(translations.at(i)(l), 'f', 3) + "|");
//        str.remove(str.size() - 1);
////        str.append(";");

//        resp[QString::number(i) + "sol"] = str;
//    }

    //    srv->sendCommandTo(socket, "solutions", resp);
//}

//void MainWindow::newEstimate()
//{
//    Vector3d modelPoints[3];
//    Vector2d imagePoints[3];

//    Matrix3d zeroRot;
//    for (int i = 0; i < 3; ++i)
//    {
//        for (int j = 0; j < 3; ++j)
//        {
//            if(i==j)
//                zeroRot(i, j) = 1;
//            else
//                zeroRot(i, j) = 0;
//        }
//    }

//    bearingVectors_t bearingVectors;
//    points_t points;
//    rotation_t rotation = zeroRot;
//    transformations_t p3p_kneip_transformations;

//    QVector<Vector3d> edges = triangl->edges();
//    QVector<Vector3d> model = triangl->modelPoins();


//    for (int i = 0; i < 3; ++i)
//    {
//        modelPoints[i] = model.at(i+1);
//        imagePoints[i] = edges.at(i).hnormalized();

//        bearingVectors.push_back(edges.at(i)/edges.at(i).norm());
//        points.push_back(modelPoints[i]);
//    }


//    absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors, points);
//    p3p_kneip_transformations = p3p_kneip(adapter);
////    openGVP3PKneip(adapter);


//    std::vector<Matrix3d> rotations;
//    std::vector<Vector3d> translations;
//    PoseFromThreePoints(imagePoints, modelPoints, &rotations, &translations);

//    for(int i=0; i<rotations.size(); i++)
//    {
//        for(int j=0; j<3; j++)
//            handler->table(6)->setValue(i*7+j, translations[i](j), 'f', 1);

//        Quaterniond quat(rotations[i]);
//        handler->table(6)->setValue(i*7+3, quat.w(), 'f', 3);
//        handler->table(6)->setValue(i*7+4, quat.x(), 'f', 3);
//        handler->table(6)->setValue(i*7+5, quat.y(), 'f', 3);
//        handler->table(6)->setValue(i*7+6, quat.z(), 'f', 3);
//    }

//    for(int i=0; i<p3p_kneip_transformations.size(); i++)
//    {
//        transformation_t trans = p3p_kneip_transformations.at(i);

//        for(int j=0; j<3; j++)
//            handler->table(7)->setValue(i*7+j, trans(j, 4), 'f', 1);

//        Matrix3d rotMat = trans.block(0, 0, 3, 3);
//        Quaterniond quat(rotMat);
//        handler->table(7)->setValue(i*7+3, quat.w(), 'f', 3);
//        handler->table(7)->setValue(i*7+4, quat.x(), 'f', 3);
//        handler->table(7)->setValue(i*7+5, quat.y(), 'f', 3);
//        handler->table(7)->setValue(i*7+6, quat.z(), 'f', 3);
//    }
//}

void MainWindow::updateTable(int ind)
{
    DisplayTable *table = handler->table(ind);

    switch (ind) {
    case 0:
    {
        QVector<Vector3d> points = triangl->modelPoins();
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                table->setValue(j*4+i, points.at(i)(j), 'f', 0);
            }
        }

        break;
    }
    case 1:
    {
        Vector3d trans = triangl->translation();
        for (int i = 0; i < 3; ++i)
        {
            table->setValue(i, trans(i), 'f', 0);
        }

        break;
    }
    case 2:
    {
        double w, x, y, z;
        double fi, teta, psi;
        int e;

        Quaterniond rotation = triangl->quaternion();
        w = rotation.w();
        x = rotation.x();
        y = rotation.y();
        z = rotation.z();
        e = 1;
        fi = atan2(2*(w*x-e*y*z),1-2*(x*x+y*y))*180/M_PI;
        teta = asin(2*(w*y+e*x*z))*180/M_PI;
        psi = atan2(2*(w*z-e*x*y),1-2*(y*y+z*z))*180/M_PI;

        table->setValue(0, fi, 'f', 0);
        table->setValue(1, teta, 'f', 0);
        table->setValue(2, psi, 'f', 0);

        break;
    }
    case 3:
    {
        Quaterniond rotation = triangl->quaternion();
        table->setValue(0, rotation.w(), 'f', 3);
        table->setValue(1, rotation.x(), 'f', 3);
        table->setValue(2, rotation.y(), 'f', 3);
        table->setValue(3, rotation.z(), 'f', 3);

        break;
    }
    case 4:
    {
        QVector<Vector3d> points;
        points << triangl->center() << triangl->edges();

        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                table->setValue(j*4+i, points.at(i)(j), 'f', 2);
            }
        }


        break;
    }
    case 5:
    {
        QVector<Vector2d> imagePoints2D;
        QVector<Vector3d> edges = triangl->edges();

        for (int i = 0; i < 3; ++i)
        {
            imagePoints2D << edges.at(i).hnormalized();
        }

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 2; ++j)
            {
                table->setValue(j*3+i, imagePoints2D.at(i)(j), 'f', 2);
            }
        }

        break;
    }
    default:
        break;
    }

    //    newEstimate();
}

void MainWindow::estimate()
{
    Vector3d modelPoints3D[3];
    Vector2d imagePoints2D[3];

    points_t points;
    bearingVectors_t bearingVectors;

    QVector<Vector3d> model = triangl->modelPoins();
//    QVector<Vector2d> image = triangl->imagePoints();
    for (int i = 0; i < 3; ++i)
    {
        Vector3d p = model.at(i+1);
//        Vector2d pIm = image.at(i);
        modelPoints3D[i] = p;

        points.push_back(p);

        Vector3d camIm = triangl->rotationMatrix()*p + triangl->translation();
        imagePoints2D[i] = Vector2d(camIm(0)/camIm(2), camIm(1)/camIm(2));
        double norm = camIm.norm();
        Vector3d bvector = camIm / norm;
        bearingVectors.push_back(bvector);        //bearingVectors[i] = bvector;
    }

    //----------------------------

    std::vector<Matrix3d> rotations;
    std::vector<Vector3d> translations;
    PoseFromThreePoints(imagePoints2D, modelPoints3D, &rotations, &translations);

    DisplayTable *table = handler->table("Theia Kneip Solutions");
    Quaterniond quat;
    for (size_t i = 0; i < rotations.size(); ++i)
    {
        quat = Quaterniond(rotations.at(i));
        for (int j = 0; j < 3; ++j)
        {
            table->setValue(i*7+j, translations.at(i)(j), 'f', 1);
            table->setValue(i*7+4+j, quat.vec()(j), 'f', 3);
        }
        table->setValue(i*7+3, quat.w(), 'f', 3);
    }

    //----------------------------

    CentralAbsoluteAdapter adapter = CentralAbsoluteAdapter(bearingVectors, points);
    transformations_t p3p_kneip_transformations;
    p3p_kneip_transformations = p3p_kneip(adapter);

    table = handler->table("OpenGV Kneip Solutions");
    for (int i = 0; i < p3p_kneip_transformations.size(); ++i)
    {
        transformation_t &trans = p3p_kneip_transformations.at(i);
        Matrix3d m = Matrix3d(trans.block(0,0,3,3).transpose());
        Vector3d t = Vector3d(-trans(0,3), -trans(1,3), -trans(2,3));

        //Vector3d s = m.transpose()*(t);

        quat = Quaterniond(m);
        for (int j = 0; j < 3; ++j)
        {
            table->setValue(i*10+j, (m*t)(j), 'f', 1);
            table->setValue(i*10+4+j, quat.vec()(j), 'f', 3);
        }
        table->setValue(i*10+3, quat.w(), 'f', 3);

        for (int j = 0; j < 3; ++j)
        {
            Vector3d newWPoint = m*model.at(j+1) + (m*t);
            Vector2d newIPoint = Vector2d(newWPoint(0)/newWPoint(2), newWPoint(1)/newWPoint(2));
            double diff = abs((newIPoint-imagePoints2D[j]).norm()/imagePoints2D[j].norm());

            table->setValue(i*10+7+j, diff, 'f', 1);
        }
    }

    //----------------------------

    transformations_t p3p_gao_transformations;
    p3p_gao_transformations = p3p_gao(adapter);

    table = handler->table("OpenGV Gao Solutions");
    for (int i = 0; i < p3p_gao_transformations.size(); ++i)
    {
        transformation_t &trans = p3p_gao_transformations.at(i);
        Matrix3d m = Matrix3d(trans.block(0,0,3,3).transpose());
        Vector3d t = Vector3d(-trans(0,3), -trans(1,3),-trans(2,3));

        quat = Quaterniond(m);
        for (int j = 0; j < 3; ++j)
        {
            table->setValue(i*10+j, (m*t)(j), 'f', 1);
            table->setValue(i*10+4+j, quat.vec()(j), 'f', 3);
        }

        table->setValue(i*10+3, quat.w(), 'f', 3);

        for (int j = 0; j < 3; ++j)
        {
            Vector3d newWPoint = m*model.at(j+1) + (m*t);
            Vector2d newIPoint = Vector2d(newWPoint(0)/newWPoint(2), newWPoint(1)/newWPoint(2));
            double diff = abs((newIPoint-imagePoints2D[j]).norm()/imagePoints2D[j].norm());

            table->setValue(i*10+7+j, diff, 'f', 1);
        }
    }

}

int MainWindow::indexFromString(QString str)
{
    switch (str)
    {
    case "A":
        return 0;
    case "B":
        return 1;
    case "":
        return 2;
    case "C":
        return 3;
    case "D":
        return 4;
    case "E":
        return 5;
    default:
        qDebug() << "No such point index: " + str;
        exit(0);
    }
}

bool MainWindow::isPointIndex(QString str)
{
    if(str=="A" || str=="B" ||str=="C" || str=="D" || str=="E")
        return true;
    else
        return false;
}

void MainWindow::onClientConnect(QTcpSocket *socket)
{
    stsConnection->setText(socket->peerAddress().toString() + " connected");
}

void MainWindow::onClientDisconnect(QTcpSocket *socket)
{
    stsConnection->setText("no clients");
    modelAccepted = false;
}

void MainWindow::onRequest(QTcpSocket *socket, QString cmd, StringMap array)
{
    if(cmd == "model")
    {
        currentSocket = socket;

        int count = array.count();
        if(count < 3)
        {
            StringMap resp;
            resp["0"] = "Not enough model points";
            srv->sendCommandTo(socket, "error", resp);
            return;
        }

//        QMap<int, Vector3d> modelPoints;
        QVector<Vector3d> points;
        QList<QString> list = array.keys();

//        int pointIndex;
        Vector3d pointVector;
        for(int i=0; i<count; i++)
        {
            QString key = list.at(i);
//            pointIndex = indexFromString(key);

            strList = array.value(key).split("|");
            for(int j=0; j<3; j++)
                pointVector(j) = strList.at(j).toDouble();

//            modelPoints[pointIndex] = pointVector;
            points << pointVector;
        }

        modelAccepted = true;
        if(exoskeleton==0)
            exoskeleton = new LEDExoskeleton(points);
        else
            exoskeleton->setModelPoints(points);
    }
    else if(cmd == "points")
    {
        if(!modelAccepted)
        {
            StringMap resp;
            resp["1"] = "Model not Accepted yet";
            srv->sendCommandTo(socket, "error", resp);
            return;
        }

        QVector<IndexedPoint> indPoints;

        QList<QString> list = array.keys();
        int count = array.count();

        long time;
        QString type = "none";
        Vector2d image;
        QList<QString> strList;
        for(int i=0; i<count; i++)
        {
            QString key = list.at(i);

            if(key=="type")
                type = key;
            else if(key=="time")
                time = array.value("time").toLong();
            else if(isPointIndex(key))
            {
                strList = array.value(key).split("|");
                image(0) = strList.at(0).toDouble();
                image(1) = strList.at(1).toDouble();

                indPoints << IndexedPoint(indexFromString(key), image);
            }
            else
            {
                StringMap resp;
                resp["2"] = "\"" + key + "\": the ponts key is not recognized";
                srv->sendCommandTo(socket, "error", resp);
            }
        }

        Matrix<double,3,4> solution;
        exoskeleton->estimate(indPoints, time, type);


//        QString str;

//        for(int l=0; l<3; l++)
//            for(int m=0; m<3; m++)
//                str.append(QString::number(rotations(l,m), 'f', 3) + "|");
//        for(int l=0; l<3; l++)
//            str.append(QString::number(translations(l), 'f', 3) + "|");
//        str.remove(str.size() - 1);

//        resp["main"] = str;
//        srv->sendCommandTo(currentSocket, "solutions", resp);
    }
    else
    {
        StringMap resp;
        resp["3"] = "\"" + cmd + "\": the message is not recognized";
        srv->sendCommandTo(socket, "error", resp);
    }
}

void MainWindow::onTableHandler(int tblInd, int editInd, double value)
{

    switch (tblInd) {
    case 0:
    {
        Vector3d point = triangl->modelPoins().at(editInd%4);
        point(editInd/4) = value;
        triangl->setModelPoint(editInd%4, point);

        QString pref = "model/point_" + QString::number(editInd%4);
        settings->setValue(pref + "/x", point(0));
        settings->setValue(pref + "/y", point(1));
        settings->setValue(pref + "/z", point(2));

        updateTable(4);
        updateTable(5);

        break;
    }
    case 1:
    {
        Vector3d trans = triangl->translation();
        trans(editInd) = value;
        triangl->setTranslation(trans);

        QString pref = "translation";
        settings->setValue(pref + "/x", trans(0));
        settings->setValue(pref + "/y", trans(1));
        settings->setValue(pref + "/z", trans(2));

        updateTable(4);
        updateTable(5);

        break;
    }
    case 2:
    {
        Vector3d rot;
        for (int i = 0; i < 3; ++i)
        {
            rot(i) = handler->table(tblInd)->value(i);
        }

        QString pref = "rotation";
        settings->setValue(pref + "/x", rot(0));
        settings->setValue(pref + "/y", rot(1));
        settings->setValue(pref + "/z", rot(2));

        triangl->setRotation(rot*M_PI/180);

        updateTable(3);
        updateTable(4);
        updateTable(5);

        break;
    }
    default:
        return;
        break;
    }

    estimate();
}



LEDExoskeleton::LEDExoskeleton(QVector<Vector3d> modelPoints) :
    c_center(0,0,0),
    c_translation(0,0,0),
    c_rotation(1, 0, 0, 0)
{
    for (int i = 0; i < modelPoints.count(); ++i)
    {
        c_vertices << modelPoints.at(i);
    }
}

void LEDExoskeleton::setModelPoints(QVector<Vector3d> modelPoints)
{
    c_vertices.clear();
    for (int i = 0; i < modelPoints.count(); ++i)
    {
        c_vertices << modelPoints.at(i);
    }
}

void LEDExoskeleton::estimate(QVector<IndexedPoint> indexedPoints,
                              long time,
                              Matrix<double,3,4> &solution)
{
    int pointsCount = indexedPoints.count();

    if(pointsCount < 3)
    {
        StringMap resp;
        resp["4"] = "Not enough image point";
        srv->sendCommandTo(socket, "error", resp);
        return;
    }

    //Наборы треугольников
    ImageTriangle imgTriangle;
    QVector<imgTriangle> triangls;
    if(pointsCount == 3)
    {
        imgTriangle.points << indexedPoints.at(0)
                           << indexedPoints.at(1)
                           << indexedPoints.at(2);
        triangls << imgTriangle;
    }
    else if(pointsCount == 4)
        for (int i = 0; i < 4; ++i)
        {
            imgTriangle.points.clear();
            for (int j = 0; j < 4; ++j)
            {
                if(j != i)
                    imgTriangle.points << IndexedPoint(indexedPoints.at(j));
            }
            triangls << imgTriangle;
        }
    else if(pointsCount == 5)
        for(int l=0; l<5; ++l)
            for(int m=l+1; m<5; ++m)
            {
                imgTriangle.points.clear();
                for(int n=0; n<5; ++n)
                    if(n!=l && n!=m)
                        imgTriangle.points << IndexedPoint(indexedPoints.at(n));
                triangls << imgTriangle;
            }

    //Набираем решения
    QVector<Matrix<double,3,4> > solutions;
    Vector3d modelPoints3D[3];
    Vector2d imagePoints2D[3];
    std::vector<Matrix3d> rotations;
    std::vector<Vector3d> translations;

    for (int i = 0; i < triangls.count(); ++i)
    {
        imgTriangle = triangls.at(i);
        for (int j = 0; j < 3; ++j)
        {
            IndexedPoint point = imgTriangle.points.at(j);
            int ind = point.index;

            imagePoints2D[j] = point.vector;
            modelPoints3D[j] = c_vertices.at(ind);
        }

        PoseFromThreePoints(imagePoints2D, modelPoints3D, &rotations, &translations);
        for (size_t j = 0; j < rotations.size(); ++j)
        {
            Matrix<double,3,4> sol;
            sol << rotations.at(j) << translations.at(j);
            solutions << sol;
        }
    }

    //Сравниваем решения


}
