#include "thread_demonstrator.h"

static std::ofstream ofs;

ThreadDemonstrator::ThreadDemonstrator(QObject *parent)
    :QThread(parent),quit(false)
{

}

ThreadDemonstrator::~ThreadDemonstrator(){
    mutex.lock();
    quit=true;
    mutex.unlock();
    wait();
    qInfo()<<"Thread Destoryed";
}

void ThreadDemonstrator::startThread(){
    if(!isRunning()){
        start();
    }
}

void ThreadDemonstrator::run(){
    qDebug()<<__PRETTY_FUNCTION__<<"() start";
    QThread::msleep(40);
    if(opMode==DEMONSTRATOR){
        std::cout<<"DEMONSTRATOR"<<std::endl;
        while(recording){
            //std::cout<<joint_larm_0<<" "<<joint_larm_1<<" "<<joint_larm_2<<" "<<joint_larm_3<<" write value "<<QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz").toStdString()<<std::endl;
            std::vector<float> data_temp{joint_larm_0,joint_larm_1,joint_larm_2,joint_larm_3,joint_larm_4,joint_larm_5,joint_larm_6,joint_rarm_0,joint_rarm_1,joint_rarm_2,joint_rarm_3,joint_rarm_4,joint_rarm_5,joint_rarm_6
                                        ,joint_lhand_0,joint_lhand_1,joint_lhand_2,joint_lhand_3,joint_lhand_4,joint_lhand_5,joint_rhand_0,joint_rhand_1,joint_rhand_2,joint_rhand_3,joint_rhand_4,joint_rhand_5,vcap_hand_0};
            //,joint_waist_0,joint_waist_1,joint_waist_2,joint_head_0,joint_head_1};
            //qInfo()<<"test";
            data.push_back(data_temp);
            QThread::msleep(10);
        }
    }else if(opMode == RCP){
        std::cout<<"PCR"<<std::endl;
        while(recording){
            std::vector<float> data_temp{rcp_l_0,rcp_l_1,rcp_l_2,rcp_l_3,rcp_l_4,rcp_l_5,rcp_l_6,rcp_r_0,rcp_r_1,rcp_r_2,rcp_r_3,rcp_r_4,rcp_r_5,rcp_r_6};
            data.push_back(data_temp);
            QThread::msleep(10);
        }
    }
}

void ThreadDemonstrator::startRecording(const int mode){
    recording = true;
    opMode = mode;
    std::cout<<"start recording"<<std::endl;
}

void ThreadDemonstrator::stopRecording(){
    recording=false;
    std::cout<<"stop recording"<<std::endl;
}

void ThreadDemonstrator::clearRecording(){
    data.clear();
}

void ThreadDemonstrator::saveRecording(QString str){
    std::cout<<str.toStdString()<<std::endl;
    //binary
//    std::ofstream file(str.toStdString(), std::ios::binary | std::ios::app);
//    if (file.is_open()) {
//        for (const auto &vec : data) {
//            file.write(reinterpret_cast<const char*>(&vec[0]), vec.size() * sizeof(float));
//        }
//        file.close();
//    } else {
//        qDebug() << "Failed to open file for writing.";
//    }
    //csv
    std::ofstream file(str.toStdString());
    if (file.is_open()) {
        for (const auto &vec : data) {
            for (size_t i = 0; i < vec.size(); ++i) {
                file << vec[i];
                if (i != vec.size()-1) {
                    file << ",";
                }
            }
            file << "\n";
        }
        file.close();

        data.clear();
    } else {
        qDebug() << "Failed to open file for writing.";
    }

    close();
}

void ThreadDemonstrator::close(){
    qDebug()<<__PRETTY_FUNCTION__<<"()"<<__LINE__;
    quit=true;
    wait();
    quit=false;
}
