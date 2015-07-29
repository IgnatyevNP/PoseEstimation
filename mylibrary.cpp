#include "mylibrary.h"

QVector3D vector3DToQt(const Vector3d &vec)
{
    return QVector3D(vec(0), vec(1), vec(2));
}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------

TableHandler DisplayTable::handler;
DisplayTable::DisplayTable(QString name, int raws, int colums) : QGroupBox(name),
c_title(name),
c_raws(raws),
c_colums(colums),
c_values(c_raws*c_colums, 0)
{
    handler.addNewTable(this);

    m_grid = new QGridLayout(this);

    for(int i=0; i<c_raws; ++i)
        for(int j=0; j<c_colums; j++)
        {
            QLineEdit *edit = new QLineEdit();
            editMap[i*c_colums + j] = edit;
            m_grid->addWidget(edit, i+1, j+1);
            connect(edit, SIGNAL(editingFinished()), this, SLOT(onEditingFinished()));
        }
}

void DisplayTable::onEditingFinished()
{
    QLineEdit *edit = qobject_cast<QLineEdit*>(sender());
    int editInd = editMap.key(edit);
    double value = edit->text().toDouble();
    c_values[editInd] = value;
    handler.handle(this, editInd, value);
}

void DisplayTable::setColumTitles(QString str)
{
    QStringList strList = str.split(",");
    for(int i=0; i<strList.count(); i++)
    {
        m_grid->addWidget(new QLabel(strList.at(i).trimmed()), 0, i+1, Qt::AlignCenter);
    }
}

void DisplayTable::setRawTitles(QString str)
{
    QStringList strList = str.split(",");
    for(int i=0; i<strList.count(); i++)
    {
        m_grid->addWidget(new QLabel(strList.at(i).trimmed()), i+1, 0, Qt::AlignRight);
    }
}

void DisplayTable::setReadOnly(bool bl)
{
    foreach (QLineEdit* edit, editMap.values())
        edit->setReadOnly(bl);
}

void DisplayTable::setValue(int edtiIndex, double value, char f, int prec)
{
    editMap.value(edtiIndex)->setText(QString::number(value, f, prec));
    c_values[edtiIndex] = value;
}

void DisplayTable::setEditsMinimalWidth(int width)
{
    foreach (QLineEdit *edit, editMap.values())
        edit->setMinimumWidth(width);
}

QLineEdit *DisplayTable::editByIndex(int ind)
{
    if(editMap.contains(ind))
        return editMap.value(ind);
    else
        return 0;
}

//---------------------------------------------------------------------------------------------

QMap<int, DisplayTable*> TableHandler::displayMap;
TableHandler::TableHandler()
{
}

void TableHandler::addNewTable(DisplayTable *table)
{
    static int tableCount = 0;
    displayMap[tableCount++] = table;
}

void TableHandler::handle(DisplayTable *tbl, int ind, double value)
{
    int tblInd = displayMap.key(tbl);
    emit valueChanged(tblInd, ind, value);
}

DisplayTable* TableHandler::table(QString title) const
{
    foreach (DisplayTable *table, displayMap.values())
        if(table->title() == title)
            return table;

    qDebug() << "No such DisplayTable: " + title;
    exit(0);
}

//---------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------


SimpleServer::SimpleServer(QWidget *parent) :
    QWidget(parent)
{
    srv = new QTcpServer(this);
    pport = 0;
    connect(srv, SIGNAL(newConnection()), this, SLOT(onClientConnect()));
}

SimpleServer::~SimpleServer()
{
    setActive(false);
    delete srv;
}

void SimpleServer::onClientConnect()
{
    QTcpSocket *socket = srv->nextPendingConnection();
    Buffers[socket] = "";
    connect(socket, SIGNAL(disconnected()), this, SLOT(onClientDisconnect()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(onClientRead()));
    socket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    emit clientConnect(socket);
}

void SimpleServer::onClientDisconnect()
{
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    if (!socket)
        return;
    Buffers.remove(socket);
    emit clientDisconnect(socket);
    socket->deleteLater();
}

void SimpleServer::onClientRead()
{
    QTcpSocket *socket = qobject_cast<QTcpSocket*>(sender());
    QString data = socket->readAll();
    Buffers[socket] += data;

    QString cmd;
    do
    {
        cmd = getTextBefore("#end;", Buffers[socket]);
        if (!cmd.isEmpty())
            parseCommand(socket, cmd);
    }
    while (!cmd.isEmpty());
}

void SimpleServer::parseCommand(QTcpSocket *socket, QString txt)
{
    QString cmd = getTextBefore(":", txt);
    QString key, value;
    StringMap arr;
    while (!txt.isEmpty())
    {
        key = getTextBefore("=", txt);
        key = key.trimmed();
        value = getTextBefore(";", txt);
        value = value.trimmed();
        if (key.isEmpty())
            break;
        arr[key] = value;
    }
    emit clientReceiveCommand(socket, cmd, arr);
}

void SimpleServer::sendCommandTo(QTcpSocket *socket, QString cmd, StringMap array)
{
    QString res = cmd + ":";
    StringMap::iterator it;
    for (it=array.begin(); it!=array.end(); it++)
    {
        QString val = (*it);
        res += array.key(val) + "=" + val + ";";
    }
    res += "#end;";
    socket->write(res.toLatin1());
    socket->flush();
}

void SimpleServer::setActive(bool active)
{
    if (active && !this->active())
    {
        srv->listen(QHostAddress::Any, pport);
        if (!pport)
            pport = srv->serverPort();
    }
    else if (!active && this->active())
    {
        srv->close();
        Buffers.clear();
    }
}

void SimpleServer::setPort(quint16 port)
{
    pport = port;
    if (active())
    {
        setActive(false);
        setActive(true);
    }
}

QString SimpleServer::getTextBefore(QString pat, QString &txt)
{
    int p1 = txt.indexOf(pat);
    if (p1 == -1)
        return "";
    int p2 = txt.length() - p1 - pat.length();
    QString res = txt.left(p1);
    txt = txt.right(p2);
    return res;
}
