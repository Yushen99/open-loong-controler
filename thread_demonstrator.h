#ifndef THREAD_DEMONSTRATOR_H
#define THREAD_DEMONSTRATOR_H
#include <QThread>
#include <QMutex>
#include <QDebug>
#include <iostream>
#include <QDateTime>
#include <atomic>
#include <QTimer>
#include <fstream>
#include <vector>

extern std::atomic<float> joint_larm_0;
extern std::atomic<float> joint_larm_1;
extern std::atomic<float> joint_larm_2;
extern std::atomic<float> joint_larm_3;
extern std::atomic<float> joint_larm_4;
extern std::atomic<float> joint_larm_5;
extern std::atomic<float> joint_larm_6;

extern std::atomic<float> joint_rarm_0;
extern std::atomic<float> joint_rarm_1;
extern std::atomic<float> joint_rarm_2;
extern std::atomic<float> joint_rarm_3;
extern std::atomic<float> joint_rarm_4;
extern std::atomic<float> joint_rarm_5;
extern std::atomic<float> joint_rarm_6;

extern std::atomic<float> joint_waist_0;
extern std::atomic<float> joint_waist_1;
extern std::atomic<float> joint_waist_2;
extern std::atomic<float> joint_head_0;
extern std::atomic<float> joint_head_1;

extern std::atomic<float> rcp_l_0;
extern std::atomic<float> rcp_l_1;
extern std::atomic<float> rcp_l_2;
extern std::atomic<float> rcp_l_3;
extern std::atomic<float> rcp_l_4;
extern std::atomic<float> rcp_l_5;
extern std::atomic<float> rcp_l_6;

extern std::atomic<float> rcp_r_0;
extern std::atomic<float> rcp_r_1;
extern std::atomic<float> rcp_r_2;
extern std::atomic<float> rcp_r_3;
extern std::atomic<float> rcp_r_4;
extern std::atomic<float> rcp_r_5;
extern std::atomic<float> rcp_r_6;

extern std::atomic<float> joint_lhand_0;
extern std::atomic<float> joint_lhand_1;
extern std::atomic<float> joint_lhand_2;
extern std::atomic<float> joint_lhand_3;
extern std::atomic<float> joint_lhand_4;
extern std::atomic<float> joint_lhand_5;
extern std::atomic<float> joint_rhand_0;
extern std::atomic<float> joint_rhand_1;
extern std::atomic<float> joint_rhand_2;
extern std::atomic<float> joint_rhand_3;
extern std::atomic<float> joint_rhand_4;
extern std::atomic<float> joint_rhand_5;

extern std::atomic<float> vcap_hand_0;
extern std::atomic<float> vcap_hand_1;

class ThreadDemonstrator : public QThread
{
    Q_OBJECT



public:
    ThreadDemonstrator(QObject *parent = nullptr);
    ~ThreadDemonstrator() override;

    enum OpMode{
        DEMONSTRATOR = 0,
        RCP = 1
    };

    void startThread();
    void run() override;
    void close();

    QTimer *fileWriteTimer;

public slots:
    void startRecording(const int mode);
    void stopRecording();
    void clearRecording();
    void saveRecording(const QString str);

signals:
    void feedback();

private:
    int opMode = DEMONSTRATOR;
    QMutex mutex;
    bool quit;
    bool recording = false;
    std::vector<std::vector<float>> data;
};


#endif // THREAD_DEMONSTRATOR_H
