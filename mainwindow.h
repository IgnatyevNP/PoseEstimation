#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <mylibrary.h>
#include <theiap3pkneip.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>

using namespace opengv;
using namespace absolute_pose;


struct IndexedPoint
{
    explicit IndexedPoint(int ind, Vector2d point) :
        index(ind),
        vector(point) {;}

    int index;
    Vector2d vector;
};

IndexedPoint::IndexedPoint(int ind, Vector2d point) :
    index(ind),
    vector(point)
{;}



class LEDExoskeleton : public QObject
{
    Q_OBJECT

public:
    explicit LEDExoskeleton(QVector<Vector3d> modelPoints);

private:
    struct ImageTriangle
    {
//        explicit ImageTriangle()

        QVector<IndexedPoint> points;
    };


    void setModelPoints(QVector<Vector3d> modelPoints);

    void estimate(QVector<IndexedPoint> indexedPoints,
                  long time,
                  Matrix<double,3,4> &solution);

    Vector3d c_center;
    QVector<Vector3d> c_vertices;
    Vector3d c_translation;
    Quaterniond c_rotation;


//    long lastTime;

//    Matrix3d c_matrix;
//    bool rotationChange;

};


class MarkerTriangle : public QObject
{
    Q_OBJECT

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//    enum RotationOrder
//    {
//        ro_XYX = 121,
//        ro_XYZ = 123,
//        ro_XZX = 131,
//        ro_XXY = 132,
//        ro_YXY = 212,
//        ro_YXZ = 213,
//        ro_YZX = 231,
//        ro_YZY = 232,
//        ro_ZXY = 312,
//        ro_ZXZ = 313,
//        ro_ZYX = 321,
//        ro_ZYZ = 323
//    };


    explicit MarkerTriangle();

    void setModelPoint(int ind, Vector3d modelPoint);
    void setCenter(Vector3d newCenter) {c_center = newCenter;}
    void setVertex(int ind, Vector3d vertex) {c_vertices[ind] = vertex;}

    void setTranslation(Vector3d trans) {c_translation = trans;}
    void setRotation(Quaterniond quat);
    void setRotation(Vector3d angles);

    QVector<Vector3d> modelPoins() const;
    Vector3d center() const {return c_center;}
    QVector<Vector3d> vertices() const {return c_vertices;}
    Vector3d translation() const {return c_translation;}
    Quaterniond quaternion() const {return c_rotation;}
    QVector<Vector3d> edges();
    QVector<Vector2d> imagePoints();

    Matrix3d rotationMatrix();

private:
    Vector3d c_center;
    QVector<Vector3d> c_vertices;
    Vector3d c_translation;
    Quaterniond c_rotation;

    Matrix3d c_matrix;
    bool rotationChange;

};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    enum PoseEstimationMethod
    {
        pem_KneipTheia,
        pem_KneipOpenGV,
        pem_GaoOpemGB
    };

private:
    Ui::MainWindow *ui;

    void loadSettings();
    void setMainWindow();

    void updateTable(int ind);
    void estimate();

    int indexFromString(QString str);
    bool isPointIndex(QString str);

    SimpleServer *srv;
    QLabel *stsConnection;
    QTcpSocket *currentSocket;

    QSettings *settings;
    const TableHandler *handler;

    MarkerTriangle *triangl;
    LEDExoskeleton *exoskeleton;

    bool modelAccepted;


//    void updateAndEstimate();
//    void estimateAndSend(QTcpSocket *socket);
//    transformations_t estimateByKneipTheiaGV(CentralAbsoluteAdapter);
//    transformations_t estimatePoseByGaoOpenGV(CentralAbsoluteAdapter);
//    void estimataByGaoOpenGVAndSend(QTcpSocket *socket);
//    void newEstimate();
//    void openGVP3PKneip(const CentralAbsoluteAdapter &adapter);
//    PoseEstimationMethod m_method;



private slots:
    void onClientConnect(QTcpSocket* socket);
    void onClientDisconnect(QTcpSocket* socket);

    void onTableHandler(int tblInd, int editInd, double value);


    void onRequest(QTcpSocket *socket, QString cmd, StringMap array);

};


#endif // MAINWINDOW_H
