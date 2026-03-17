#include "BCDStrategy.hpp"
#include "Simulator.hpp"
#include <vector>
#include <set>
#include <algorithm>

using namespace std;

namespace {
    struct Segment {
        int y;
        int left;
        int right;
    };

    vector<Segment> getRowSegments(Map& map, int y, bool leftToRight) {
        vector<Segment> segments;
        int left = 1;
        int right = map.getCols() - 2;
        int x = left;

        while (x <= right) {
            while (x <= right && !map.isValid(x, y)) {
                ++x;
            }

            if (x > right) {
                break;
            }

            int segLeft = x;

            while (x <= right && map.isValid(x, y)) {
                ++x;
            }

            int segRight = x - 1;
            segments.push_back({y, segLeft, segRight});
        }

        if (!leftToRight) {
            reverse(segments.begin(), segments.end());
        }

        return segments;
    }
}

string BCDStrategy::getName() const {
    return "BCD";
}

void BCDStrategy::run(Simulator& simulator, const string& dockLabel, const string& runLabel) {
    Map& map = simulator.getMap();
    Robot& robot = simulator.getRobot();

    bool startFromLeft = (dockLabel == "Top-Left" || dockLabel == "Bottom-Left");
    bool startFromTop = (dockLabel == "Top-Left" || dockLabel == "Top-Right");

    int firstRow = startFromTop ? 1 : map.getRows() - 2;
    int lastRow = startFromTop ? map.getRows() - 2 : 1;
    int stepY = startFromTop ? 1 : -1;

    set<Point> cleanedCells;

    simulator.cleanCurrentCell();
    cleanedCells.insert(robot.pos);

    for (int y = firstRow, rowIndex = 0;
         (stepY > 0 ? y <= lastRow : y >= lastRow);
         y += stepY, ++rowIndex) {

        bool leftToRight = (rowIndex % 2 == 0) ? startFromLeft : !startFromLeft;
        vector<Segment> segments = getRowSegments(map, y, leftToRight);

        for (const Segment& seg : segments) {
            Point entry = leftToRight ? Point{seg.left, seg.y} : Point{seg.right, seg.y};
            Point exitPoint = leftToRight ? Point{seg.right, seg.y} : Point{seg.left, seg.y};
            int stepX = leftToRight ? 1 : -1;

            if (robot.pos != entry) {
                vector<Point> pathToEntry = map.getPathHome(robot.pos, entry);

                if (pathToEntry.empty() && robot.pos != entry) {
                    continue;
                }

                for (const Point& p : pathToEntry) {
                    simulator.moveRobot(p, true);
                }
            }

            if (robot.pos == entry && cleanedCells.count(robot.pos) == 0) {
                simulator.cleanCurrentCell();
                cleanedCells.insert(robot.pos);
            }

            for (int x = entry.x + stepX; ; x += stepX) {
                if ((stepX == 1 && x > exitPoint.x) || (stepX == -1 && x < exitPoint.x)) {
                    break;
                }

                simulator.moveRobot({x, seg.y}, false);
                cleanedCells.insert(robot.pos);
            }
        }
    }

    simulator.returnToDock();
    simulator.recordHistory(runLabel);
}
