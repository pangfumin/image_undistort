#include <iostream>
#include <algorithm>
#include "file-system-tools.h"

int main() {
    std::string folder ="/home/pang/data/dataset/segway_outdoor/cui_stereo_calib/newCamPics/image_to_calibrate";
    std::vector<std::string> image_filenames;
    /// image0
    common::getAllFilesInFolder(folder, &image_filenames);
    std::cout<<"image_filenames: " << image_filenames.size() << std::endl;



    std::vector<std::string> left,right;
    for (auto i : image_filenames) {
        std::string path, name;
        common::splitPathAndFilename(i, &path, &name);
        if (name.find("left") != std::string::npos) {
            left.push_back(name);
        }else if (name.find("right") != std::string::npos) {
            right.push_back(name);
        }
    }

    std::sort(left.begin(),left.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    std::sort(right.begin(),right.end(), [](std::string a, std::string b) {
        return !common::compareNumericPartsOfStrings(a,b);
    });

    for (int i= 0; i < left.size(); i++) {
        std::cout<< "\"" << left.at(i) << "\"" << std::endl;
        std::cout<< "\"" << right.at(i) << "\"" << std::endl;
    }
    return 0;
}