#include "negative_obs_detection/negative_obs_detection.h"

/* ---------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    ros::init(argc, argv, "vb_planner");
    NegObsDetect neg_detect;
    neg_detect.Loop();
    return 0;
}