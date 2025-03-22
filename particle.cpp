#include "mcl.h"

std::vector<Robot> Robot::initializeParticles(int numParticles, const cv::Size& mapSize) {
    std::vector<Robot> particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    std::uniform_real_distribution<> disX(50, mapSize.width - 50);
    std::uniform_real_distribution<> disY(50, mapSize.height - 50);
    std::uniform_real_distribution<> disTheta(0, 2 * CV_PI);
    
    for (int i = 0; i < numParticles; ++i) {
        Robot p;
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

double Robot::CalculateWeight(cv::Mat& gradient, std::vector<cv::Point>& data_relative){
    double weightTotal = 1;

    for (const auto& p : particleSensor_) {
        uchar intensity = gradient.at<uchar>(p.x, p.y);
        weightTotal *= (double) intensity/255;
        std::cout<< "\np "<<p.x <<" " <<p.y;
        // std::cout<< weightTotal <<std::endl;
        cv::circle(display_, p, 5, 255, -1); 
    }
    // std::cout<<std::endl;

    return weightTotal;
}

void Robot::SetParticleSensor(std::vector<cv::Point>& data_relative){
    std::vector<cv::Point> points;

    for(const auto& p : data_relative){
        cv::Point result = RotatePoint(cv::Point(position_.x + p.x, position_.y + p.y), position_, orientation_);
        // std::cout<< "dr " << result<<std::endl;
        particleSensor_.push_back(result);
    }
    // std::cout<< "parc " << particleSensor_[0]<<std::endl;
        // particleSensor_ = points;
}

void Robot::activedParticleScan(){
    #pragma omp parallel for
    for(int i=0;i<particles_.size();i++){
        particles_[i].drawRobot(display_);
        // particles_[i].drawData(display_, data_relative_);
        particles_[i].SetParticleSensor(data_relative_);
        // particles_[i].CreateFov();
        // particles_[i].distance_ = particles_[i].LineDistance(fieldmap_);
        // particles_[i].DetectingLandmark();
        // particles_[i].weight_ += calculateCosineSimilarity(distance_, particles_[i].distance_);
        // particles_[i].weight_ += calculateCosineSimilarity(distance_, particles_[i].distance_, particles_[i].lm_);
        particles_[i].weight_ = particles_[i].CalculateWeight(gradient_, data_relative_);
        // std::cout<< particles_[i].weight_ <<" ";
    }
    std::cout<<std::endl;
}