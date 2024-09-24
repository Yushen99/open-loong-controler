#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTabWidget>
#include <QDebug>

class HandWidget : public QWidget {
public:
    HandWidget(QWidget *parent = nullptr) : QWidget(parent) {}

    void setColor(const QColor &color) {
        handColor = color;
        update(); // 更新绘制
    }

protected:
    void paintEvent(QPaintEvent *event) override {
        Q_UNUSED(event);

        QPainter painter(this);

        // 设置画笔颜色和宽度
        painter.setPen(QPen(Qt::black, 2));

        // 设置手的颜色
        painter.setBrush(handColor);

        // 绘制手掌
        painter.drawEllipse(50, 50, 150, 200);

        // 绘制手指
        painter.drawLine(125, 150, 100, 100);
        painter.drawLine(125, 150, 150, 100);
        painter.drawLine(125, 150, 125, 75);
    }

private:
    QColor handColor = Qt::white; // 默认颜色为白色
};

#include <QApplication>
#include <QFile>
#include <QMainWindow>
#include <QMessageBox>
#include <QTabWidget>
#include <QWidget>
#include <QPainter>

void drawHand(QWidget *widget, const QColor &handColor) {
    QPainter painter(widget);
    painter.setRenderHint(QPainter::Antialiasing); // 抗锯齿
    painter.setPen(Qt::black); // 设置画笔颜色

    // 设置手的颜色
    painter.setBrush(handColor);

    // 绘制手掌
    painter.drawEllipse(50, 50, 150, 200);

    // 绘制手指
    painter.drawLine(125, 150, 100, 100);
    painter.drawLine(125, 150, 150, 100);
    painter.drawLine(125, 150, 125, 75);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // 加载 UI 文件
    QFile file(":/path/to/your/ui/file.ui");
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(nullptr, QObject::tr("Error"), QObject::tr("Cannot open file"));
        return 1;
    }

    // 创建主窗口并从 UI 文件中加载界面
    QMainWindow mainWindow;
    QTabWidget *tabWidget = nullptr;

    // 从 UI 文件中加载界面
    QUiLoader loader;
    QWidget *uiWidget = loader.load(&file, &mainWindow);
    file.close();
    if (!uiWidget) {
        QMessageBox::critical(nullptr, QObject::tr("Error"), QObject::tr("Cannot load UI"));
        return 1;
    }

    // 在界面中查找 Tab 标签
    tabWidget = uiWidget->findChild<QTabWidget*>("tabWidget");
    if (!tabWidget) {
        QMessageBox::critical(nullptr, QObject::tr("Error"), QObject::tr("Cannot find tabWidget"));
        return 1;
    }

    // 在 Tab 标签中添加手绘功能
    QWidget *handTab = tabWidget->findChild<QWidget*>("tabHand");
    if (handTab) {
        // 绘制手
        drawHand(handTab, Qt::blue); // 根据提供的数据设置手的颜色
    } else {
        QMessageBox::critical(nullptr, QObject::tr("Error"), QObject::tr("Cannot find tabHand"));
        return 1;
    }

    // 设置主窗口的中心部件为加载的界面
    mainWindow.setCentralWidget(uiWidget);
    mainWindow.show();

    return app.exec();
}
