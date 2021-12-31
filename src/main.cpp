#include "common.hpp"
#include "csvParser.hpp"
#include "dlt.hpp"

using imgCoordinates = std::vector<Vec2D>;
using markerPositions = std::vector<Vec3D>;

int main()
{
    std::string resourcePath = "./resource/";

    CSVParser csv(resourcePath);

    const auto imgCoords = csv.getImgCoords();
    const auto posMarkers = csv.getPosMarkers();

    DLT dlt(imgCoords, posMarkers);
    DLT::Solution solution = dlt.run();

    return 0;
}