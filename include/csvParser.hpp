#pragma once

#include "common.hpp"
#include "fast-cpp-csv-parser/csv.h"

class CSVParser
{
public:
    explicit CSVParser(const std::string& path)
    {
        const std::string inputDataPath = path + "/input.csv";

        loadInputCSV(inputDataPath);
    }

    const std::vector<Vec3D> getPosMarkers() const { return mPosMarkers; }
    const std::vector<Vec2D> getImgCoords() const { return mImgCoords; }
    const Eigen::Matrix<double, 3, 4> getOutputMat() const { return mOutputMat; }

private:
    std::vector<Vec3D> mPosMarkers;
    std::vector<Vec2D> mImgCoords;
    Eigen::Matrix<double, 3, 4> mOutputMat;

    void loadInputCSV(const std::string& inputDataPath)
    {
        io::CSVReader<5> inputData(inputDataPath);

        mPosMarkers.reserve(30);
        mImgCoords.reserve(30);

        std::string imgCoord_0, imgCoord_1, posMarker_0, posMarker_1, posMarker_2;
        bool firstLine = true;

        while (inputData.read_row(imgCoord_0, imgCoord_1, posMarker_0, posMarker_1, posMarker_2))
        {
            if (firstLine)
            {
                firstLine = false;
                continue;
            }

            mImgCoords.emplace_back(std::stod(imgCoord_0), std::stod(imgCoord_1));
            mPosMarkers.emplace_back(std::stod(posMarker_0), std::stod(posMarker_1), std::stod(posMarker_2));
        }
    }
}; // namespace std::filesystemlclassCSVParser