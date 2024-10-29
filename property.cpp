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
    // std::cout<<particles_.size()<<std::endl;

    return true;
}

// Function to resample particles
std::vector<Robot> Robot::resampleParticles(const std::vector<Robot>& particles) {
    std::vector<Robot> newParticles;
    int numParticles = particles.size();
    
    // Normalize weights
    std::vector<Robot> mutableParticles = particles;  // Make a mutable copy for normalization
    normalizeWeights(mutableParticles);
    
    // Find maximum weight
    double maxWeight = getMaxWeight(mutableParticles);
    // std::cout << "Max weight: " << maxWeight << " Average weight: " << averageWeight(mutableParticles) << std::endl;

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);

    // Initialize index and beta
    int index = static_cast<int>(dis(gen) * numParticles);
    double beta = 0.0;

    for (int i = 0; i < numParticles; ++i) {
        // Increase beta by a random amount within 2*maxWeight
        beta += dis(gen) * 2.0 * maxWeight;

        // Move index forward until beta is less than or equal to the particle's weight
        while (beta > mutableParticles[index].weight_) {
            beta -= mutableParticles[index].weight_;
            index = (index + 1) % numParticles;
        }

        // Create a new particle around the selected particle's position with noise
        Robot newParticle(fieldmap_);
        std::normal_distribution<> noise(0, 10.0);  // Position noise in pixels
        newParticle.position_.x = mutableParticles[index].position_.x + noise(gen);
        newParticle.position_.y = mutableParticles[index].position_.y + noise(gen);
        newParticle.weight_ = 1.0 / numParticles;  // Reset new weight

        // Add the new particle to the resampled list
        newParticles.push_back(newParticle);
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