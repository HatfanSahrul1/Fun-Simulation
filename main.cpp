#include <iostream>
#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

struct Particle {
    double x;
    double y;
    double weight;
};

// Function to normalize particle weights
void normalizeWeights(std::vector<Particle>& particles) {
    double totalWeight = 0.0;
    for (const auto& particle : particles) {
        totalWeight += particle.weight;
    }
    for (auto& particle : particles) {
        particle.weight /= totalWeight;
    }
}

// Function to resample particles
std::vector<Particle> resampleParticles(const std::vector<Particle>& particles) {
    std::vector<Particle> newParticles;
    int numParticles = particles.size();
    
    // Normalize weights
    std::vector<Particle> mutableParticles = particles;  // Make a mutable copy for normalization
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
        
        for (const auto& particle : mutableParticles) {
            cumulativeWeight += particle.weight;
            if (randChoice <= cumulativeWeight) {
                Particle newParticle;
                newParticle.x = particle.x + noise(gen);
                newParticle.y = particle.y + noise(gen);
                newParticle.weight = 1.0 / numParticles;  // Reset new weight
                newParticles.push_back(newParticle);
                break;
            }
        }
    }
    return newParticles;
}

// Function to draw particles on an image
void drawParticles(cv::Mat& image, const std::vector<Particle>& particles, cv::Scalar color) {
    for (const auto& particle : particles) {
        cv::circle(image, cv::Point(static_cast<int>(particle.x), static_cast<int>(particle.y)), 3, color, -1);
    }
}

int main() {
    // Initialize initial particles
    std::vector<Particle> particles = {
        {100, 150, 0.1},
        {200, 250, 0.3},
        {300, 200, 0.2},
        {400, 100, 0.4}
    };

    // Create a blank image
    cv::Mat image = cv::Mat::zeros(400, 500, CV_8UC3);
    image = cv::Scalar(255, 255, 255);

    // Draw original particles (in blue)
    drawParticles(image, particles, cv::Scalar(255, 0, 0));
    
    // Resample particles
    std::vector<Particle> resampledParticles = resampleParticles(particles);
    
    // Draw resampled particles (in green)
    drawParticles(image, resampledParticles, cv::Scalar(0, 255, 0));

    // Show the image
    cv::imshow("Particle Resampling", image);
    cv::waitKey(0);
    return 0;
}
