#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>

const cv::Scalar merah = cv::Scalar(0, 0, 255);
const cv::Scalar hijau = cv::Scalar(0, 255, 0);


class Robot
{
    private:
    public:
        int n_particles;
        cv::Point2f position;
        float orientation;
        float weight;
        std::vector<Robot> particles;
        cv::Size mapSize_;
        cv::Mat fov, fieldmap, inputPoint;

        std::vector<cv::Point> rotated;
        std::vector<std::vector<cv::Point2f>> detected;
        std::vector<std::vector<double>> distances;

        Robot(cv::Mat mapInput){
            fieldmap = mapInput.clone();
        }

        void init(cv::Point2f pos, float orient, float w, int n, cv::Size& mapSize){
            position = pos;
            orientation = orient;
            weight = w;
            n_particles = n;
            mapSize_ = mapSize;
            particles = initializeParticles(n_particles, mapSize_);
        }

        std::vector<Robot> initializeParticles(int numParticles, const cv::Size& mapSize) {
            std::vector<Robot> particles;
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> disX(0, mapSize.width);
            std::uniform_real_distribution<> disY(0, mapSize.height);
            std::uniform_real_distribution<> disTheta(0, 2 * CV_PI);
            
            for (int i = 0; i < numParticles; ++i) {
                Robot p(fieldmap);
                p.position = cv::Point2f(disX(gen), disY(gen));
                p.orientation = disTheta(gen);
                p.weight = 1.0 / numParticles;  // uniform weight
                particles.push_back(p);
            }
            return particles;
        }

        void moveParticles(std::vector<Robot>& particles, const cv::Size& mapSize, float& speed, float& orient) {
            
            for (auto& p : particles) {
                p.orientation += orient * CV_PI / 180.0;
                p.position.x += speed * std::cos(p.orientation);
                p.position.y += speed * std::sin(p.orientation);

                p.position.x = std::max(0.0f, std::min((float)mapSize.width, p.position.x));
                p.position.y = std::max(0.0f, std::min((float)mapSize.height, p.position.y));
            }
        }

        void regularMove(Robot& robot, const cv::Size& mapSize, float speed, float orient){
            robot.orientation += orient * CV_PI / 180.0;
            robot.position.x += speed * std::cos(robot.orientation);
            robot.position.y += speed * std::sin(robot.orientation);
            
            robot.position.x = std::max(0.0f, std::min((float)mapSize.width, robot.position.x));
            robot.position.y = std::max(0.0f, std::min((float)mapSize.height, robot.position.y));

            moveParticles(particles, mapSize, speed, orient);
        }

        
        void drawParticles(cv::Mat& map, std::vector<Robot>& particles) {
            for (const auto& p : particles) {
                cv::circle(map, p.position, 2, merah, -1);  // red particles
            }
            for(int i=0;i<particles.size();i++){
                particles[i].drawFov(map);
            }
        }

        void drawRobot(cv::Mat& map) {
            cv::circle(map, position, 5, hijau, -1);  // blue robotRobot
        }

        
        
        void drawFov(cv::Mat field){
            cv::Mat field2 = field.clone();
            std::vector<cv::Point2f> triangle_points;
            triangle_points.clear();
            triangle_points.push_back(position); // Sudut pertama
            triangle_points.push_back(cv::Point2f(position.x + 200, position.y - 180)); // Sudut kedua
            triangle_points.push_back(cv::Point2f(position.x + 200, position.y + 180)); // Sudut ketiga

            double angle = orientation;
            // std::cout<<angle<<std::endl;
            // Buat mask segitiga
            
            rotated.clear();
            rotated.push_back(triangle_points[0]);
            rotated.push_back(RotatePoint(triangle_points[1], triangle_points[0], angle));
            rotated.push_back(RotatePoint(triangle_points[2], triangle_points[0], angle));
           
            if(!field.empty()){
                cv::polylines(field2, rotated, true, cv::Scalar(255, 0, 255), 1);
            }

            
        }

        cv::Point RotatePoint(const cv::Point2f& p, const cv::Point2f& pivot, double angle) {
            double radians = angle;// *CV_PI / 180.0; // Konversi sudut dari derajat ke radian
            double cos_theta = std::cos(radians);
            double sin_theta = std::sin(radians);

            // Hitung titik baru setelah rotasi
            int x_new = cos_theta * (p.x - pivot.x) - sin_theta * (p.y - pivot.y) + pivot.x;
            int y_new = sin_theta * (p.x - pivot.x) + cos_theta * (p.y - pivot.y) + pivot.y;

            return cv::Point(x_new, y_new);
        }

        void LineScan(){
            cv::Mat image = fieldmap.clone();
    
            if (image.empty()) {
                std::cerr << "Error: Image not loaded." << std::endl;
                return;
            }

            // Find contours of the objects in the image (assumes binary image)
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            cv::Mat output = image.clone();
            std::vector<cv::Point2f> intersectionPoints, temp_points;

            cv::Point center = rotated[0]; // Titik pusat robot
            cv::Point p1 = rotated[1];
            cv::Point p2 = rotated[2];

            int num_lines = 15;

            detected.clear();
            distances.clear();

            for (int line_idx = 0; line_idx <= num_lines; ++line_idx) {
                float t = static_cast<float>(line_idx) / num_lines;  // Interpolasi antara sudut p1 dan p2
                cv::Point interpolated_point = p1 + t * (p2 - p1);
                cv::line(output, center, interpolated_point, cv::Scalar(255), 1);
                
                cv::LineIterator it(image, center, interpolated_point, 8);  

                temp_points.clear();  // Untuk menyimpan intersection sementara dari garis ini
                for (int i = 0; i < it.count; i++, ++it) {
                    if (image.at<uchar>(it.pos()) == 255) {  // Jika pixel putih (berpotongan dengan objek)
                        cv::Point2f intersectionPoint(it.pos());
                        temp_points.push_back(intersectionPoint);

                        cv::circle(output, intersectionPoint, 3, cv::Scalar(255), 1);
                    }
                }

                // Filter intersection points per line
                std::vector<cv::Point2f> filtered = FilterClose(temp_points);
                distances.push_back(getDistance(filtered));
                detected.push_back(filtered);
                // for (const auto& pt : filtered) {
                // }
            }

            // DrawIntersectionPoints(output, intersectionPoints);
            cv::imshow("Intersections", output);
            cv::imshow("check", fieldmap);
        }

        std::vector<double> getDistance(std::vector<cv::Point2f> points){
            std::vector<double> measured;

            for(const auto& pt : points){
                double dist = sqrt(pow(abs(pt.x - position.x), 2) + pow(abs(pt.y - position.y), 2));
                measured.push_back(dist);
            }

            return measured;
        }

        double euclideanDistance(const std::vector<double>& v1, const std::vector<double>& v2) {
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
        double calculateSimilarity(const std::vector<std::vector<double>>& distances1, const std::vector<std::vector<double>>& distances2) {
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


        void DrawIntersectionPoints(cv::Mat& image, const std::vector<cv::Point2f>& intersectionPoints) {
            // Buat salinan dari image asli untuk digambar
            cv::Mat output = image.clone();
            output = cv::Mat::zeros(image.size(), image.type());

            // Pastikan image dalam format BGR, jika tidak ubah dari grayscale ke BGR
            if (output.channels() == 1) {
                cv::cvtColor(output, output, cv::COLOR_GRAY2BGR);
            }

            // Tentukan warna dan ukuran titik
            cv::Scalar pointColor(0, 0, 255);  // Warna merah untuk titik (BGR)
            int radius = 3;                    // Radius dari titik
            int thickness = -1;                // -1 untuk menggambar lingkaran penuh

            // Loop melalui semua intersection points dan gambar pada image
            for (const auto& point : intersectionPoints) {
                cv::circle(output, point, radius, pointColor, thickness);
            }
            // Matching(output);
            // Tampilkan hasil gambar
            cv::imshow("Intersection Points", output);
        }

        std::vector<cv::Point2f> FilterClose(std::vector<cv::Point2f> points) {
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

        void activedParticleScan(){
            for(int i=0;i<particles.size();i++){
                particles[i].drawFov(particles[i].fieldmap);
                particles[i].LineScan();
                // p.printPoint();
            }
        }
    

        void printPoint(){
            for(const auto& p : distances){
                for(const auto& pc : p){
                    std::cout<<pc;
                }
                std::cout<<std::endl;
            }
            std::cout<<distances.size()<<std::endl;
        }

        void printSimilarity(){
            double sim;
            for(const auto& p : particles){
                sim = calculateSimilarity(distances, p.distances);
                std::cout<<sim<<std::endl;
            }
        }
};

void control(float &move, float &orient){
    if(cv::waitKey(0)=='w'){
        orient = 0.0f;
        move = 2.0f;
    }else if(cv::waitKey(0)=='s'){
        orient = 0.0f;
        move = -4.0f;
    }else if(cv::waitKey(0)=='a'){
        move = 0;
        orient = -1.0f;
    }else if(cv::waitKey(0)=='d'){
        move = 0;
        orient = 1.0f;   
    }else{
        move = 0;
        orient = 0;
    }
}

void drawMap(cv::Mat& map, const cv::Size& size){
    int width = size.width;
    int height = size.height;
    map = cv::Mat::zeros(height, width, CV_8UC1);
    
    cv::rectangle(map, cv::Point(50, 50), cv::Point(width - 50, height - 50), cv::Scalar(255, 255, 255), 2);

    std::vector<cv::Point2f> l = {cv::Point2f(width*2/3, 50), cv::Point2f(width*2/3, height * 3/8), cv::Point2f((width*2/3) + 70, height * 3/8)};
    std::vector<cv::Point2f> t = {cv::Point2f(width*3/5, height*2/3), cv::Point2f(width-50, height*2/3), 
                                    cv::Point2f(width*3/5, (height*2/3)-25), cv::Point2f(width*3/5, (height*2/3)+25)};
    std::vector<cv::Point2f> i = {cv::Point2f(width*4/10, 120), cv::Point2f(width*4/10, height-50)};

    // l
    cv::line(map, l[0], l[1], cv::Scalar(255), 2);
    cv::line(map, l[1], l[2], cv::Scalar(255), 2);

    // t 
    cv::line(map, t[0], t[1], cv::Scalar(255), 2);
    cv::line(map, t[2], t[3], cv::Scalar(255), 2);

    // i
    cv::line(map, i[0], i[1], cv::Scalar(255), 2);
}

int main(){
    cv::Size mapSize(950, 650);
    cv::Mat map, display;
    int n = 10;
    
    cv::Point2f position = cv::Point2f(mapSize.width / 2, mapSize.height / 2);
    float orientation = 20.0f;
    float weight = 1.0f;

    drawMap(map, mapSize);

    Robot myRobot(map);
    myRobot.init(position, orientation, weight, n, mapSize);
    std::vector<Robot> particles = myRobot.initializeParticles(n, mapSize);

    float updateMove = 0.0f, updateOrient = 0.0f;

    while (true){
        display = map.clone();
        cv::cvtColor(display, display, cv::COLOR_GRAY2BGR);

        myRobot.drawParticles(display, myRobot.particles);
        
        myRobot.drawRobot(display);
        myRobot.drawFov(display);
        myRobot.LineScan();
        myRobot.activedParticleScan();
        myRobot.printSimilarity();
        myRobot.regularMove(myRobot, mapSize, updateMove, updateOrient);
        cv::imshow("Monte Carlo Localization Visualization", display);
        // cv::imshow("sense", myRobot.fov);

        updateMove=0;
        updateOrient=0;
        control(updateMove, updateOrient);
        
        if (cv::waitKey(0)==27) break;
    }
    
    return 0;
}