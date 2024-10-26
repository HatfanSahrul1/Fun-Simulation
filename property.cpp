#include "mcl.h"

Robot::Robot(cv::Mat mapInput){
    fieldmap_ = mapInput;
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
    LineScan();

    drawFov(display_);
    
    activedParticleScan();
    regularMove(mapSize_, move, orient);
    particles_ = resampleParticles(particles_);
    // std::cout<<getMaxWeight(particles_)<<std::endl;  

    return true;
}

// Function to resample particles
std::vector<Robot> Robot::resampleParticles(const std::vector<Robot>& particles) {
    std::vector<Robot> newParticles;
    int numParticles = particles.size();
    
    double maxWeight = getMaxWeight(particles);
    std::cout<<"weight : "<<maxWeight<<std::endl;
    if(maxWeight < 0.75){
        return initializeParticles(n_particles_, mapSize_);
    }
    
    // Normalize weights
    std::vector<Robot> mutableParticles = particles;  // Make a mutable copy for normalization
    normalizeWeights(mutableParticles);
    
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
                Robot newParticle(fieldmap_);
                
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