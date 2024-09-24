#ifndef OPENLOONGCONTROLER_H
#define OPENLOONGCONTROLER_H

#include "robotsystem.h"
#include "thread_demonstrator.h"
//#include "thread_3DMouse.h"
//#include "thread_RGB.h"

#include "k_ORL2/k_OLR2.h"
#include "k_ORL2/k_OLR2_initialize.h"
#include "k_ORL2/k_OLR2_terminate.h"
#include "k_ORL2/rt_nonfinite.h"
#include "k_ORL2/rtGetInf.h"
#include "k_ORL2/rtGetNaN.h"
#include "k_ORL2/norm.h"
#include "k_ORL2/CoordinateTrans.h"

#include <QMainWindow>
#include <QMutex>
#include <QKeyEvent>
#include <unordered_map>
#include <QLabel>
#include <QSet>
#include <utility>
#include <QMap>
#include <atomic>
#include <QDateTime>
#include <QPainter>
#include <QPaintEvent>
#include <QLinearGradient>
#include <QRadialGradient>
#include <QDebug>
#include <QMatrix4x4>
#include <QVector3D>
#include <eigen3/Eigen/Dense>
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

extern std::atomic<uint8_t> R_value;
extern std::atomic<uint8_t> G_value;
extern std::atomic<uint8_t> B_value;

namespace Ui {
class OpenLoongControler;
}

class OpenLoongControler : public QMainWindow
{
    Q_OBJECT

    enum RobotPort{
      PORT_OCU = 8000,
      PORT_AUTONOMY_ROUTE = 8001,
      PORT_AUTONOMY_CELL = 8002,
      PORT_MOTION_EE = 8003,
      PORT_MOTION_JOINT = 8004,
      PORT_MOTION_CMD = 8005,
      PORT_HEAD = 8006,
      PORT_NUM = 7
    };

    enum Pos_Curr{
        POSITION = 0,
        CURRENT = 1
    };

    enum Add_Minus{
        ADD = 0,
        MINUS = 1
    };

    enum Joint_Info{
        ARM = 0,
        WAIST = 1,
        HEAD = 2,
        HAND = 3
    };

    struct LabelInfo{
        QLabel* label;
        Add_Minus add_minus;
        Joint_Info joint_info;
        int row;
        int col;
        LabelInfo() : label(nullptr), add_minus(ADD), joint_info(ARM), row(0), col(0) {}
        LabelInfo(QLabel* label, Add_Minus add_minus, Joint_Info joint_info, int r, int c)
                : label(label), add_minus(add_minus), joint_info(joint_info), row(r), col(c) {}
    };

    struct Point{
        int posX;
        int posY;
        int count;
    };

public:
    explicit OpenLoongControler(QWidget *parent = 0);
    ~OpenLoongControler();
    QTimer * clients_com_timer;
    QTimer * clients_remote_timer;
    QTimer * clients_agv;
    QTimer* keyRespondTimer;
    QTimer * show_timer;
    QTimer * demonstrator_timer;
    QTimer * demonstrator_run_timer;
    QTimer * rcp_timer;
    QTimer * rcp_run_timer;
    QTimer * draw_timer;
    RobotSystem * robot_system;

protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    bool eventFilter(QObject *watched, QEvent *event)override;

private slots:
    void on_pushButton_init_connect_clicked();
    void ShowInfomation();
    void ShowModeInfo();
    void on_pushButton_oneshot_manual_clicked();
    void on_radioButton_manual_pos_clicked();
    void on_radioButton_manual_curr_clicked();
    void handleKeyPress();
    void handleDemonstrator();
    void handleRCP();
    void drawHand_update();
    void handleDemonstrator_run();
    void handleRCP_run();
    void on_pushButton_manual_step_set_clicked();

    void on_pushButton_demon_continue_start_clicked();

    void on_pushButton_demon_continue_stop_clicked();

    void on_pushButton_demons_continue_clear_clicked();

    void on_pushButton_demon_continue_save_clicked();

    void on_pushButton_demon_continue_load_clicked();

    void on_pushButton_rcp_record_start_clicked();

    void on_pushButton_rcp_record_stop_clicked();

    void on_pushButton_rcp_record_save_clicked();

    void on_pushButton_rcp_record_load_clicked();

    void on_pushButton_rcp_record_clear_clicked();

    void on_pushButton_oneshot_demonstrator_clicked();

    void on_pushButton_oneshot_idle_clicked();

    void on_pushButton_oneshot_rcp_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_reset_clicked();

    void on_pushButton_enable_clicked();

    void on_pushButton_disable_clicked();

    void on_radioButton_filter_on_clicked();

    void on_radioButton_filter_off_clicked();

    void on_pushButton_demon_continue_replay_clicked();

    void on_radioButton_demonstrator_loop_clicked();

    void on_radioButton_demonstrator_notloop_clicked();

    void on_pushButton_rcp_manual_clicked();

    void on_pushButton_rcp_auto_clicked();

    void on_pushButton_rpc_setpset_clicked();

    void on_horizontalSlider_rcp_k_valueChanged(int value);

    void on_lineEdit_rcp_k_editingFinished();

    void on_pushButton_demon_continue_replay_stop_clicked();

    void on_pushButton_demon_continue_replay_continue_clicked();

    void on_pushButton_demon_continue_replay_end_clicked();

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_demon_continue_load2_clicked();

    void on_pushButton_rcp_init_clicked();

    void on_pushButton_rcp_record_run_clicked();

    void on_pushButton_rcp_record_modify_clicked();

    void on_pushButton_rcp_record_stamp_clicked();

    void on_pushButton_rcp_record_calibrate_clicked();

    void on_pushButton_rcp_lock_clicked();

    void on_pushButton_rcp_unlock_clicked();

private:
    Ui::OpenLoongControler *ui;

    ThreadDemonstrator thread_demonstrator;

    QMutex mutex;
    QMutex dataMutex;
    Pos_Curr posCurr = POSITION;
    bool bool_manualEnabled=false;
    bool bool_demonstrator_loop = false;
    bool rgb = false;
    short mouse_control_arm=0;
    //rcp
    double rcp_k=1;// 步长倍率
    float manual_step=0.02f;
    float stamp = 0.25; //rcp trajectory time stamp
    float rcp_step_rxyz=0.02f,rcp_step_x=0.02f,rcp_step_y=0.02f,rcp_step_z=0.02f;
    float delta_r_x = 0.0f, delta_r_y = 0.0f, delta_r_z = 10.0f;
    float delta_r_rx = 0.0f, delta_r_ry = 0.0f, delta_r_rz = 10.0f;
    Eigen::Matrix4f  matrix_stc_left, matrix_stc_right;
    //manual
    QList<int> pressedKeys;
    QMap<int, bool> keyPressedMap;
    std::unordered_map<int,struct LabelInfo> map_pos;
    std::unordered_map<int,struct LabelInfo> map_pos_ctrl;
    std::unordered_map<int,struct LabelInfo> map_curr;
    std::unordered_map<int,struct LabelInfo> map_curr_ctrl;
    std::unordered_map<int,struct LabelInfo> map_pos_alt;
    std::unordered_map<int,struct LabelInfo> map_curr_alt;
    std::unordered_map<int,struct LabelInfo> map_rcp;
    //demonstrate
    std::vector<std::vector<float>> demonstrate_data;
    std::vector<std::vector<float>> demonstrate_data_rcp;
    unsigned long curr_demonstrate_index=0;
    double playback_speed=1.0; // 倍速值
    double accumulated_time; // 累积时间
    //draw config
    static const int ImgHeight = 300;
    static const int ImgWidth = 730;
    int radius = 15;
    int countTable[ImgWidth*ImgHeight] = {0};
    QList<Point> point_list;
    double max_count;
    QImage dataImg; QImage heatImg;
    QRgb colorList[256];

    void initLabel();
    void drawHand();
    std::vector<float> interpolate(const std::vector<float>& start, const std::vector<float>& end, double t);
    void on_pushButton_rcp_record_modify_clicked_old();


signals:
    void startRecording(const int mode);
    void stopRecording();
    void clearRecording();
    void saveRecording(const QString str);

};

#endif // OPENLOONGCONTROLER_H


