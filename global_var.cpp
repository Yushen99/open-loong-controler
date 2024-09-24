#include <atomic>

std::atomic<float> joint_larm_0{0};
std::atomic<float> joint_larm_1{0};
std::atomic<float> joint_larm_2{0};
std::atomic<float> joint_larm_3{0};
std::atomic<float> joint_larm_4{0};
std::atomic<float> joint_larm_5{0};
std::atomic<float> joint_larm_6{0};

std::atomic<float> joint_rarm_0{0};
std::atomic<float> joint_rarm_1{0};
std::atomic<float> joint_rarm_2{0};
std::atomic<float> joint_rarm_3{0};
std::atomic<float> joint_rarm_4{0};
std::atomic<float> joint_rarm_5{0};
std::atomic<float> joint_rarm_6{0};

std::atomic<float> joint_waist_0{0};
std::atomic<float> joint_waist_1{0};
std::atomic<float> joint_waist_2{0};
std::atomic<float> joint_head_0{0};
std::atomic<float> joint_head_1{0};

std::atomic<float> rcp_l_0{0};
std::atomic<float> rcp_l_1{0};
std::atomic<float> rcp_l_2{0};
std::atomic<float> rcp_l_3{0};
std::atomic<float> rcp_l_4{0};
std::atomic<float> rcp_l_5{0};
std::atomic<float> rcp_l_6{0};

std::atomic<float> rcp_r_0{0};
std::atomic<float> rcp_r_1{0};
std::atomic<float> rcp_r_2{0};
std::atomic<float> rcp_r_3{0};
std::atomic<float> rcp_r_4{0};
std::atomic<float> rcp_r_5{0};
std::atomic<float> rcp_r_6{0};

std::atomic<float> joint_lhand_0{0};
std::atomic<float> joint_lhand_1{0};
std::atomic<float> joint_lhand_2{0};
std::atomic<float> joint_lhand_3{0};
std::atomic<float> joint_lhand_4{0};
std::atomic<float> joint_lhand_5{0};
std::atomic<float> joint_rhand_0{0};
std::atomic<float> joint_rhand_1{0};
std::atomic<float> joint_rhand_2{0};
std::atomic<float> joint_rhand_3{0};
std::atomic<float> joint_rhand_4{0};
std::atomic<float> joint_rhand_5{0};

std::atomic<float> vcap_hand_0{0};
std::atomic<float> vcap_hand_1{0};

std::atomic<bool> motion{false};
std::atomic<float> motionx{0};
std::atomic<float> motiony{0};
std::atomic<float> motionz{0};
std::atomic<float> motionrx{0};
std::atomic<float> motionry{0};
std::atomic<float> motionrz{0};

std::atomic<uint8_t> R_value{0};
std::atomic<uint8_t> G_value{0};
std::atomic<uint8_t> B_value{0};
