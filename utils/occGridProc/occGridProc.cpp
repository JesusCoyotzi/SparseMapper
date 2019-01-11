#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

int main(int argc, char  *argv[]) {

        if (argc < 5) {
                std::cout << "Usage program inFile outFile mapResolution erosionRadius" << '\n';
        }

        std::string inputFileName(argv[1]);
        std::string outputFileName(argv[2]);
        float resolution = atof(argv[3]);
        float radius = atof(argv[4]);
        // size_t lastindex = inputFileName.find_last_of(".");
        // std::string yamlFile = inputFileName.substr(0, lastindex)+".yaml";
        // std::cout << "Config file: "<< yamlFile << '\n';
        // cv::FileStorage fs(yamlFile,cv::FileStorage::READ);
        // float resolution = fs["resolution"];
        std::cout << "Reading map " <<  inputFileName<<'\n';
        std::cout << "@ " << resolution << " m/pix\n";

        cv::Mat occGrid=cv::imread(argv[1]);
        int elementSize = radius/resolution;
        cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( elementSize, elementSize));
        cv::Mat processed;
        cv::morphologyEx( occGrid, processed, cv::MORPH_ERODE , element );

        cv::imshow("Occupancy grid",occGrid);
        cv::imshow("processed grid",processed);
        cv::waitKey(0);
        cv::imwrite(outputFileName,processed);
        return 0;
}
