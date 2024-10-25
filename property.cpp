#include "mcl.h"

Robot::Robot(cv::Mat mapInput){
    fieldmap_ = mapInput.clone();
}

void Robot::init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize){
    position_ = pos;
    orientation_ = orient;
    weight_ = w;
    n_particles_ = n;
    mapSize_ = mapSize;
    particles_ = initializeParticles(n_particles_, mapSize_);
}