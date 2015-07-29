#ifndef MYLIBRARY
#define MYLIBRARY

#include <QtWidgets>
#include <QTcpServer>
#include <QTcpSocket>

#include <Eigen/Dense>

using namespace Eigen;

QVector3D vector3DToQt(const Vector3d &vec);

//-----------------------------------------------------------------------------------------------
class TableHandler;

class DisplayTable : public QGroupBox
{
    Q_OBJECT

public :
    explicit DisplayTable(QString name, int raws, int colums);

    static const TableHandler* handlerPoint() {return &handler;}

    void setColumTitles(QString str);
    void setRawTitles(QString str);
    void setReadOnly(bool bl);

    void setValue(int edtiIndex, double value, char f, int prec);

    void setEditsMinimalWidth(int width);

    QString title() {return c_title;}
    int columCount() {return c_colums;}
    int rawCount() {return c_raws;}

    double value(int ind) const {return c_values.at(ind);}
    QLineEdit* editByIndex(int ind);



private:
    static TableHandler handler;

    QString c_title;
    int c_raws;
    int c_colums;
    QVector<double> c_values;

    QGridLayout *m_grid;
    QMap<int, QLineEdit*> editMap;

private slots:
    void onEditingFinished();

};

class TableHandler : public QObject
{
    Q_OBJECT

public:
    explicit TableHandler();

    void addNewTable(DisplayTable *table);
    void handle(DisplayTable *tbl, int ind, double value);

    DisplayTable* table(int ind) const {return displayMap.value(ind);}
    DisplayTable* table(QString title) const;

signals:
    void valueChanged(int tableInd, int editInd, double value);

private:
    static QMap<int, DisplayTable*> displayMap;

};

//-----------------------------------------------------------------------------------------------

typedef QMap<QString, QString> StringMap;

class SimpleServer : public QWidget
{
    Q_OBJECT

private:
    typedef QMap<QTcpSocket*, QString> SocketStringMap;

private:
    QTcpServer *srv;
    quint16 pport;
//    bool pconnected;
    SocketStringMap Buffers;

    void parseCommand(QTcpSocket *socket, QString txt);

private slots:
    void onClientConnect();
    void onClientDisconnect();
    void onClientRead();

public:
    explicit SimpleServer(QWidget *parent = 0);
    ~SimpleServer();

    QTcpServer* server() {return srv;}

    static QString getTextBefore(QString pat, QString &txt);

    quint16 port() {return pport;}
    void setPort(quint16 port);

    bool active() {return srv->isListening();}
    void setActive(bool active);
    bool connected() {return Buffers.count();}
    int clientCount() {return Buffers.count();}

    void sendCommandTo(QTcpSocket *socket, QString cmd, StringMap array);

signals:
    void clientConnect(QTcpSocket *socket);
    void clientDisconnect(QTcpSocket *socket);
    void clientReceiveCommand(QTcpSocket *socket, QString cmd, StringMap array);

public slots:

};


#endif // MYLIBRARY

