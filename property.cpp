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

Robot::Robot(){}

void Robot::init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize){
    position_ = pos;
    orientation_ = orient;
    weight_ = w;
    n_particles_ = n;
    mapSize_ = mapSize;
    particles_ = initializeParticles(n_particles_, mapSize_);

    src_sensor_ = cv::Mat::zeros(cv::Size(180*2, 200), CV_8UC3);
    gradient_ = cv::imread("/home/eros/images/inverted.jpg", cv::IMREAD_GRAYSCALE);
}

bool Robot::MainLoop(float move, float orient){
    Robot estimation;
    display_ = fieldmap_.clone();
    cv::cvtColor(display_, display_, cv::COLOR_GRAY2BGR);
    
    drawParticles(display_, particles_);
    
    drawRobot(display_);
    CreateFov();
    LineScan();
    distance_ = LineDistance(fieldmap_);
    // DetectingLandmark();
    GetDataRelative();
    drawFov(display_);
    // DrawIntersectionPoints(display_, detected_);
    estimation.position_ = getMeanPosition();
    estimation.orientation_ = getMeanOrient();
    estimation.drawRobot(display_);
    estimation.drawData(display_, data_relative_);
    std::cout << estimation.orientation_ << std::endl;
    
    activedParticleScan();
    regularMove(mapSize_, move, orient);
    particles_ = resampleParticlesFPS(particles_);
    
    // static int c = 0;
    // if(c != 0){
    //     particles_ = resampleParticlesFPS(particles_);
    // }else{
    //     particles_ = initializeParticles(n_particles_, mapSize_);
    // }
    // c = (c + 1) % 100;
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
    // std::cout<<"weight : "<<maxWeight<<" rerata : "<<averageWeight(particles_)<<std::endl;
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

std::vector<Robot> Robot::resampleParticlesFPS(const std::vector<Robot>& particles) {
    std::vector<Robot> newParticles;
    int numParticles = particles.size();
    
    // Normalize weights
    std::vector<Robot> mutableParticles = particles; // Make a mutable copy for normalization
    normalizeWeights(mutableParticles);

    double maxWeight = getMaxWeight(mutableParticles);
    // std::cout << "weight: " << maxWeight << " rerata: " << averageWeight(mutableParticles) << std::endl;

    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    std::normal_distribution<> noise(0.0, 5.0);  // Position noise in pixels

    // Fitness Proportionate Selection
    int index = static_cast<int>(dis(gen) * numParticles);
    double beta = 0.0;

    for (int i = 0; i < numParticles; ++i) {
        beta += dis(gen) * 2.0 * maxWeight;  // Random increment for beta

        while (beta > mutableParticles[index].weight_) {
            beta -= mutableParticles[index].weight_;
            index = (index + 1) % numParticles;  // Circular indexing
        }

        // Create new particle based on selected particle
        Robot newParticle;
        newParticle.position_.x = mutableParticles[index].position_.x + noise(gen);
        newParticle.position_.y = mutableParticles[index].position_.y + noise(gen);
        newParticle.weight_ = 1.0 / numParticles;  // Reset new weight
        newParticles.push_back(newParticle);
    }

    return newParticles;
}

std::vector<Robot> Robot::resampleParticlesSUS(const std::vector<Robot>& particles) {
    std::vector<Robot> newParticles;
    int numParticles = particles.size();

    // Normalize weights
    std::vector<Robot> mutableParticles = particles;  // Make a mutable copy for normalization
    normalizeWeights(mutableParticles);

    double maxWeight = getMaxWeight(mutableParticles);
    
    // Setup random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 1);
    std::normal_distribution<> noise(0, 5.0);  // Position noise in pixels

    int index = static_cast<int>(dis(gen) * numParticles);
    double beta = 0.0;

    for (int i = 0; i < numParticles; ++i) {
        beta += dis(gen) * 2.0 * maxWeight;

        while (beta > mutableParticles[index].weight_) {
            beta -= mutableParticles[index].weight_;
            index = (index + 1) % numParticles;
        }

        // Create a new particle with added noise
        Robot newParticle;
        newParticle.position_.x = mutableParticles[index].position_.x + noise(gen);
        newParticle.position_.y = mutableParticles[index].position_.y + noise(gen);
        newParticle.weight_ = 1.0 / numParticles;  // Reset new weight

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

float Robot::getMeanOrient() const {
    float sum = 0.0f;
    int particleCount = particles_.size();
    
    if (particleCount == 0) {
        return 0.0f; // atau nilai default lainnya
    }

    for (const auto& particle : particles_) {
        sum += particle.orientation_;
    }

    return sum / particleCount;
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