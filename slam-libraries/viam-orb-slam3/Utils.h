
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <iostream>
#define FILENAME_CONST 6

#define _USE_MATH_DEFINES
using namespace std;
using namespace boost::filesystem;

#ifndef UTILS_H
#define UTILS_H


namespace viam {
namespace utils {

enum class FileParserMethod { Recent, Closest };

// This function will be removed in a future update. Currently this is only
// used with a previous dataset
void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps);

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string argParser(int argc, char **argv, const string varName);

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string configMapParser(string map, string varName);


// Converts UTC time string to a double value.
double readTimeFromFilename(const string filename);

std::vector<std::string> listFilesInDirectoryForCamera(
    const std::string data_directory, const std::string extension,
    const std::string camera_name);

// take .both files from rdk and process them to use with ORBSLAM. this will be
// changed in
// https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-254
void decodeBOTH(std::string filename, cv::Mat &im, cv::Mat &depth);

// Find the next frame based off the current interest given a directory of
// data and time to search from
int parseDataDir(const std::vector<std::string> &files,
                 FileParserMethod interest, double configTime,
                 double *timeInterest);


// ---- Implementations
void LoadImagesRGBD(const string &pathSeq, const string &strPathTimes,
                    vector<string> &vstrImageFilenamesRGB,
                    vector<string> &vstrImageFilenamesD,
                    vector<double> &vTimeStamps) {
    // This function will be removed in a future update. Currently this is only
    // used with a previous dataset
    string pathCam0 = pathSeq + "/rgb";
    string pathCam1 = pathSeq + "/depth";
    std::ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageFilenamesRGB.reserve(5000);
    vstrImageFilenamesD.reserve(5000);
    while (!fTimes.eof()) {
        string s;
        getline(fTimes, s);
        if (!s.empty()) {
            stringstream ss;
            ss << s;
            vstrImageFilenamesRGB.push_back(pathCam0 + "/" + ss.str());
            vstrImageFilenamesD.push_back(pathCam1 + "/" + ss.str());
            string timestring = s.substr(0, s.find_last_of("."));
            std::string::size_type sz;
            double t = std::stod(timestring, &sz);
            vTimeStamps.push_back(t);
        }
    }
}

// find a specific input argument from rdk and write the value to a string.
// Returns empty if the argument is not found.
string argParser(int argc, char **argv, string strName) {
    // Possibly remove these in a future task
    string strVal;
    string currArg;
    size_t loc;
    for (int i = 0; i < argc; ++i) {
        currArg = string(argv[i]);
        loc = currArg.find(strName);
        if (loc != string::npos) {
            strVal = currArg.substr(loc + strName.size());
            break;
        }
    }
    return strVal;
}

// parse a config map for a specific variable name and return the value as a
// string. Returns empty if the variable is not found within the map.
string configMapParser(string map, string varName) {
    string strVal;
    size_t loc = string::npos;

    stringstream ss(map.substr(map.find("{") + 1, map.find("}") - 1));
    while (ss.good()) {
        string substr;
        getline(ss, substr, ',');
        loc = substr.find(varName);
        if (loc != string::npos) {
            strVal = substr.substr(loc + varName.size());
            break;
        }
    }

    return strVal;
}

// Converts UTC time string to a double value.
double readTimeFromFilename(string filename) {
    std::string::size_type sz;
    // Create a stream which we will use to parse the string
    std::istringstream ss(filename);

    // Create a tm object to store the parsed date and time.
    std::tm dt = {0};

    // Now we read from buffer using get_time manipulator
    // and formatting the input appropriately.
    ss >> std::get_time(&dt, "%Y-%m-%dT%H_%M_%SZ");
    double sub_sec =
        (double)std::stof(filename.substr(filename.find(".")), &sz);
    time_t thisTime = std::mktime(&dt);

    double myTime = (double)thisTime + sub_sec;
    return myTime;
}

std::vector<std::string> listFilesInDirectoryForCamera(
    std::string data_directory, std::string extension,
    std::string camera_name) {
    std::vector<std::string> file_paths;
    std::string currFile;
    for (const auto &entry : directory_iterator(data_directory)) {
        currFile = (entry.path()).stem().string();
        if (camera_name == currFile.substr(0, currFile.find("_data_"))) {
            file_paths.push_back(currFile);
        }
    }
    sort(file_paths.begin(), file_paths.end());
    return file_paths;
}

// take .both files from rdk and process them to use with ORBSLAM. this will be
// changed in
// https://viam.atlassian.net/jira/software/c/projects/DATA/boards/30?modal=detail&selectedIssue=DATA-254

void decodeBOTH(std::string filename, cv::Mat &im, cv::Mat &depth) {
    cv::Mat rawData;
    std::ifstream fin(filename + ".both");
    if (fin.peek() == std::ifstream::traits_type::eof()) {
        BOOST_LOG_TRIVIAL(error) << "Bad file, found EOF";
        return;
    }
    std::vector<char> contents((std::istreambuf_iterator<char>(fin)),
                            std::istreambuf_iterator<char>());
    char *buffer = &contents[0];
    int frameWidth;
    int frameHeight;
    int location = 0;
    long j;
    int nSize = contents.size();

    // exit if no frame width or height is found
    if (nSize < sizeof(__int64_t) * 2) return;
    memcpy(&j, buffer + location, sizeof(__int64_t));
    location = location + sizeof(__int64_t);
    frameWidth = (int)j;

    memcpy(&j, buffer + location, sizeof(__int64_t));
    location = location + sizeof(__int64_t);
    frameHeight = (int)j;
    // exit if depth map is not complete(assumes 8bit)
    if (nSize < (8 * frameWidth * frameHeight)) return;

    int depthFrame[frameWidth][frameHeight];
    // copy depth map into 2D vector
    for (int x = 0; x < frameWidth; x++) {
        for (int y = 0; y < frameHeight; y++) {
            memcpy(&j, buffer + location, sizeof(__int64_t));
            depthFrame[x][y] = (int)j;
            location = location + sizeof(__int64_t);
        }
    }
    if (nSize <= location) {
        return;
    }
    // grab location of the png(occurs after depth map completes)
    char *pngBuf = &contents[location];

    // format frames with opencv
    depth = cv::Mat(cv::Size(frameWidth, frameHeight), CV_16U, depthFrame,
                    cv::Mat::AUTO_STEP);
    rawData = cv::Mat(cv::Size(1, nSize - location), CV_8UC1, (void *)pngBuf,
                    cv::IMREAD_COLOR);
    im = cv::imdecode(rawData, cv::IMREAD_COLOR);
}

// Find the next frame based off the current interest given a directory of
// data and time to search from
int parseDataDir(const std::vector<std::string> &files,
                viam::utils::FileParserMethod interest, double configTime,
                double *timeInterest) {
    // Find the file closest to the configTime, used mostly in offline mode
    if (interest == viam::utils::FileParserMethod::Closest) {
        for (int i = 0; i < files.size() - 1; i++) {
            double fileTime = viam::utils::readTimeFromFilename(
                files[i].substr(files[i].find("_data_") + FILENAME_CONST));
            double delTime = fileTime - configTime;
            if (delTime > 0) {
                *timeInterest = fileTime;
                return i;
            }
        }
    }
    // Find the file generated most recently, used mostly in online mode
    else if (interest == viam::utils::FileParserMethod::Recent) {
        int i = files.size() - 2;
        double fileTime = viam::utils::readTimeFromFilename(
            files[i].substr(files[i].find("_data_") + FILENAME_CONST));
        double delTime = fileTime - configTime;
        if (delTime > 0) {
            *timeInterest = fileTime;
            return i;
        }
    }
    // if we do not find a file return -1 as an error
    return -1;
}

}
}

#endif // UTILS_H