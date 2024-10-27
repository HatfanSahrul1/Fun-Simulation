#include "mcl.h"

std::vector<Robot> Robot::initializeParticles(int numParticles, const cv::Size& mapSize) {
    std::vector<Robot> particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<> disX(50, mapSize.width - 50);
    std::uniform_real_distribution<> disY(50, mapSize.height - 50);
    std::uniform_real_distribution<> disTheta(0, 2 * CV_PI);
    
    for (int i = 0; i < numParticles; ++i) {
        Robot p(fieldmap_);
        p.position_ = cv::Point2f(disX(gen), disY(gen));
        p.orientation_ = disTheta(gen);
        p.weight_ = 1.0 / numParticles;  // uniform weight_
        particles.push_back(p);
    }
    return particles;
}

void Robot::moveParticles(std::vector<Robot>& particles, const cv::Size& mapSize, float& speed, float& orient) {
    
    for (auto& p : particles) {
        p.orientation_ += orient * CV_PI / 180.0;
        p.position_.x += speed * std::cos(p.orientation_);
        p.position_.y += speed * std::sin(p.orientation_);

        p.position_.x = std::max(0.0f, std::min((float)mapSize.width, p.position_.x));
        p.position_.y = std::max(0.0f, std::min((float)mapSize.height, p.position_.y));
    }
}

void Robot::activedParticleScan(){
    for(int i=0;i<particles_.size();i++){
        particles_[i].CreateFov();
        particles_[i].LineScan();
        particles_[i].weight_ += calculateCosineSimilarity(distance_, particles_[i].distance_);
    }
}