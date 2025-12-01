#ifndef USERCONFIG_H
#define USERCONFIG_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Userconfig {
private:
    static const std::string FILE_NAME;

    // Helper to safely load data or return a default if missing/file error
    template <typename T>
    static T loadSafe(const std::string& key, T defaultValue) {
        cv::FileStorage fs(FILE_NAME, cv::FileStorage::READ);
        if (!fs.isOpened() || fs[key].empty()) {
            return defaultValue;
        }
        T value;
        fs[key] >> value;
        return value;
    }

    // Helper to write all 3 variables at once
    static void writeAll(int id, const cv::Mat& mat, const std::vector<std::vector<double>>& vec) {
        cv::FileStorage fs(FILE_NAME, cv::FileStorage::WRITE);
        fs << "activeTableID" << id;
        fs << "camToWorld" << mat;
        fs << "worldToRobot" << vec;
        fs.release();
    }

public:
    template <typename T>
    static T loadData(const std::string& key) {
        cv::FileStorage fs(FILE_NAME, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            throw std::runtime_error("Config file not found: " + FILE_NAME);
        }
        if (fs[key].empty()) {
            throw std::runtime_error("Key not found in config: " + key);
        }
        T value;
        fs[key] >> value;
        return value;
    }

    static void saveData(int activeTableID) {
        cv::Mat currentMat = loadSafe("camToWorld", cv::Mat::eye(3, 3, CV_64F));
        std::vector<std::vector<double>> currentVec = loadSafe("worldToRobot", std::vector<std::vector<double>>());

        writeAll(activeTableID, currentMat, currentVec);
    }

    static void saveData(const cv::Mat& camToWorld) {
        int currentID = loadSafe("activeTableID", 0);
        std::vector<std::vector<double>> currentVec = loadSafe("worldToRobot", std::vector<std::vector<double>>());

        writeAll(currentID, camToWorld, currentVec);
    }

    static void saveData(const std::vector<std::vector<double>>& worldToRobot) {
        int currentID = loadSafe("activeTableID", 0);
        cv::Mat currentMat = loadSafe("camToWorld", cv::Mat::eye(3, 3, CV_64F));

        writeAll(currentID, currentMat, worldToRobot);
    }
};

inline const std::string Userconfig::FILE_NAME = "config.yaml";

#endif
