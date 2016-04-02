
/*
This is supposed to take in a greyscale image (corrected to birds eye view)
of two (or really as many as you like) white lines and convert that to a laser_scan
msg
Takes in: sensor_msgs/image
Publishes: sensor_msgs/LaserScan
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <utility>
#include <math.h>
#include <cstdint>

using namespace sensor_msgs;

// A line, represented by a list of points
class Line{
    // Points that make up the line when joined sequentially
    std::vector<std::pair<int, int>> points;
    // How close a given point must be to at least one point on the line in
    // order to be added to the line
    float tolerance;
public:
    // Creates a new line with a given tolerance and a inital point somewhere on the line
    Line(float tolerance, std::pair<int, int> point){
        this->tolerance = tolerance;
        points.emplace_back(point);
    }
    // Adds a given point to the list of points next to the first point within
    // tolerance of it, returns false if no point on line found within tolerance
    bool addPoint(std::pair<int, int> point){
        for (int i = 0; i < points.size(); i++){
            if (distanceBetweenPoints(point, points[i]) < tolerance){
                points.emplace(points.begin() + i, point);
                return true;
            }
        }
        return false;
    }

    float distanceBetweenPoints(std::pair<int, int> p1, std::pair<int, int> p2){
        float dx = p1.first - p2.first;
        float dy = p1.second - p1.second;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }
};

// Initalized with an image, allows you to find lines in the image (getLines)
class ImageToLines{
    Image image;
    int subdivideFactor;
    std::vector<Line> lines;
public:
    ImageToLines(Image image, int subdivideFactor){
        this->image = image;
        // Make sure subdivide factor is odd, make it so if not
        if (subdivideFactor % 2 == 0){
            this->subdivideFactor = subdivideFactor - 1;
        } else {
            this->subdivideFactor = subdivideFactor;
        }
    }

    // Finds all the lines in the image (returns lines if they've been found already)
    std::vector<Line> getLinesFromImage() {
        if (lines.size() != 0){
            return lines;
        } else { // Find all lines in image
            int rows = floor(image.height / subdivideFactor);
            int columns = floor(image.width / subdivideFactor);
            for (int row = 0; row < rows; row++){
                for (int column = 0; column < columns; column++){
                    // Check for white squares along both diagonals
                    for (int subRowAndColumn = 0;
                        subRowAndColumn < subdivideFactor; subRowAndColumn++){
                        int x = row * subdivideFactor + subRowAndColumn;
                        int y = column * subdivideFactor + subRowAndColumn;
                        if (getPixelValue(x, y) > 125){
                            addPointToLines(std::make_pair(x, y));
                            break;
                        }
                    }
                }
            }
        }
        return lines;
    }

    // Adds the point to the first line it finds within tolerance to it,
    // or creates a new line with the point if no line was found
    void addPointToLines(std::pair<int, int> point){
        for (int line = 0; line < lines.size(); line++){
            if (lines[line].addPoint(point)){
                return;
            }
        }
        float tolerance = sqrt(2*pow(subdivideFactor, 2));
        Line line(tolerance, point);
        lines.emplace_back(line);
    }

    // Get the value of a pixel at a given x, y
    uint8_t getPixelValue(int x, int y){
        return image.data[y * image.width + x];
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_to_laserscan");
    ros::NodeHandle public_nh;
    ros::Rate loop_rate(5);

    // Get corrected image
    Image image;
    ros::Subscriber image_sub =
    public_nh.subscribe<Image>("corrected_image", 10,
                                        boost::function<void(Image)>
                                        ([&](Image image_from_subscriber){
                                            image = image_from_subscriber;
                                        }));

    LaserScan laserScan;
    // Main loop to run while ros is running
    while (ros::ok()){

    }
}
