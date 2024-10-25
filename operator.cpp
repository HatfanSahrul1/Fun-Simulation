#include "mcl.h"

cv::Point Robot::RotatePoint(const cv::Point2f& p, const cv::Point2f& pivot, double angle){
    double radians = angle;// *CV_PI / 180.0; // Konversi sudut dari derajat ke radian
    double cos_theta = std::cos(radians);
    double sin_theta = std::sin(radians);
    
    // Hitung titik baru setelah rotasi
    int x_new = cos_theta * (p.x - pivot.x) - sin_theta * (p.y - pivot.y) + pivot.x;
    int y_new = sin_theta * (p.x - pivot.x) + cos_theta * (p.y - pivot.y) + pivot.y;
    
    return cv::Point(x_new, y_new);
}

std::vector<double> Robot::getDistance(std::vector<cv::Point2f> points){
    std::vector<double> measured;

    for(const auto& pt : points){
        double dist = sqrt(pow(abs(pt.x - position_.x), 2) + pow(abs(pt.y - position_.y), 2));
        measured.push_back(dist);
    }
    return measured;
}
double Robot::euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2){
    // Jika kedua vektor kosong, anggap jaraknya 0 (mirip 100%)
    if (v1.empty() && v2.empty()) {
        return 0.0;
    }
    
    // Jika salah satu vektor kosong, anggap jaraknya besar
    if (v1.empty() || v2.empty()) {
        return std::numeric_limits<double>::max();  // Jarak maksimum
    }
    
    size_t size = std::min(v1.size(), v2.size());  // Bandingkan ukuran terkecil
    double sum = 0.0;
    
    for (size_t i = 0; i < size; ++i) {
        sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }

    return std::sqrt(sum);
}

// Fungsi untuk menghitung kesamaan antara dua vector outer yang berisi inner vector
double Robot::calculateSimilarity(const std::vector<std::vector<double>>& distances1, const std::vector<std::vector<double>>& distances2) {
    if (distances1.size() != distances2.size()) {
        std::cerr << "Ukuran outer vector tidak sama!" << std::endl;
        return 0.0;
    }
    
    double total_similarity = 0.0;
    
    for (size_t i = 0; i < distances1.size(); ++i) {
        double distance = euclideanDistance(distances1[i], distances2[i]);
    
        // Jika jarak maksimum (karena salah satu vektor kosong), set kemiripan jadi 0
        if (distance == std::numeric_limits<double>::max()) {
            total_similarity += 0.0;  // 0% kesamaan
        } else {
            total_similarity += 1.0 / (1.0 + distance);  // Semakin kecil jarak, semakin besar nilai kemiripan
        }
    }
    
    return total_similarity / distances1.size();  // Ambil rata-rata kesamaan
}

std::vector<cv::Point2f> Robot::FilterClose(std::vector<cv::Point2f> points) {
    std::vector<cv::Point2f> result;
    std::vector<bool> visited(points.size(), false);
    int threshold = 20;  // Threshold jarak x dan y
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;
        
        std::vector<cv::Point2f> closePoints;
        closePoints.push_back(points[i]);
        
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (!visited[j] && std::abs(points[j].x - points[i].x) < threshold 
                            && std::abs(points[j].y - points[i].y) < threshold) {
                closePoints.push_back(points[j]);
                visited[j] = true;
            }
        }
        
        // Hitung rata-rata untuk semua titik yang terlalu dekat
        if (closePoints.size() > 1) {
            float avgX = 0, avgY = 0;
            
            for (const auto& pt : closePoints) {
                avgX += pt.x;
                avgY += pt.y;
            }
            avgX /= closePoints.size();
            avgY /= closePoints.size();
            
            result.push_back(cv::Point2f(avgX, avgY));
        } else {
            result.push_back(points[i]);
        }
    }
    return result;
}