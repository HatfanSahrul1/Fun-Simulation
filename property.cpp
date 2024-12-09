#include "mcl.h"

Robot::Robot(cv::Mat mapInput){
    fieldmap_ = mapInput;
    BinaryMatrix_ = ConvertToBinaryMatrix(fieldmap_);
    landmarks_ = {
        Landmark("X", {{475, 225}, {475, 425}}),
        Landmark("T", {{50, 110}, {50, 540}, {475, 50}, {475, 600}, {900, 110}, {900, 540}}),
        Landmark("L", {{50, 50}, {50, 600}, {145, 110}, {145, 540}, {805, 110}, {805, 540}, {900, 50}, {900, 600}})
    };
}

Robot::Robot(){

}

void Robot::init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize){
    position_ = pos;
    orientation_ = orient;
    weight_ = w;
    n_particles_ = n;
    mapSize_ = mapSize;
    particles_ = initializeParticles(n_particles_, mapSize_);
}

bool Robot::MainLoop(float move, float orient){
    display_ = fieldmap_.clone();
    cv::cvtColor(display_, display_, cv::COLOR_GRAY2BGR);

    drawParticles(display_, particles_);
    
    drawRobot(display_);
    CreateFov();
    // LineScan(BinaryMatrix_);
    distance_ = LineDistance(fieldmap_);
    DetectingLandmark();

    drawFov(display_);
    
    activedParticleScan();
    regularMove(mapSize_, move, orient);
    particles_ = resampleParticles(particles_);
    
    // if(!lm_.empty()){
    //     particles_ = resamplingLandmark(particles_);
    //     std::cout<<lm_[0].id<<" "<<lm_[0].distance<<std::endl;
    // }
    // std::cout<<particles_.size()<<std::endl;
    // printPoint();

    return true;
}

// Function to resample particles
std::vector<Robot> Robot::resampleParticles(const std::vector<Robot>& particles) {
    std::vector<Robot> newParticles;
    int numParticles = particles.size();
    
    // Normalize weights
    std::vector<Robot> mutableParticles = particles;  // Make a mutable copy for normalization
    normalizeWeights(mutableParticles);
    
    double maxWeight = getMaxWeight(particles);
    std::cout<<"weight : "<<maxWeight<<" rerata : "<<averageWeight(particles_)<<std::endl;
    // if(maxWeight < 0.95){
    //     return initializeParticles(n_particles_, mapSize_);
    // }

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);
    std::normal_distribution<> noise(0, 5.0);  // Position noise in pixels

    
    // Roulette-wheel selection
    for (int i = 0; i < numParticles; ++i) {
        double randChoice = dis(gen);
        double cumulativeWeight = 0.0;

        // std::cout<<"resampleParticles"<<numParticles<<std::endl;

        
        for (const auto& particle : mutableParticles) {
            cumulativeWeight += particle.weight_;
            if (randChoice <= cumulativeWeight) {
                Robot newParticle;
                
                newParticle.position_.x = particle.position_.x + noise(gen);
                newParticle.position_.y = particle.position_.y + noise(gen);
                
                newParticle.weight_ = 1.0 / numParticles;  // Reset new weight
                newParticles.push_back(newParticle);
                break;
            }
        }
    }
    return newParticles;
}

cv::Point2f Robot::getMeanPosition() const {
    float sumX = 0.0f;
    float sumY = 0.0f;
    int particleCount = particles_.size();

    for (const auto& particle : particles_) {
        sumX += particle.position_.x;
        sumY += particle.position_.y;
    }

    return cv::Point2f(sumX / particleCount, sumY / particleCount);
}

void Robot::DetectingLandmark(){
    lm_.clear();
    for(int i=0; i < landmarks_.size(); i++){
        for(int j=0; j < landmarks_[i].positions.size(); j++){
            if(landmarks_[i].isDetected(rotated_, landmarks_[i].positions[j])){
                DetectedLandmark det;
                det.id = landmarks_[i].getID();
                det.distance = GetDistance(cv::Point(landmarks_[i].positions[j].first, landmarks_[i].positions[j].second));
                lm_.push_back(det);
            }
        }
    }
}