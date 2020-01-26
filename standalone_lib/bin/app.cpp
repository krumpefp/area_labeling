#include <algorithm>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <optional>
#include <sstream>

#include "liblabeling.h"

using std::cin;
using std::cout;
using std::cerr;
using std::endl;

bool inputIsClosed(std::vector<double>& coords) {
    return coords[0] == coords[coords.size()-2] && coords[1] == coords[coords.size()-1];
}

liblabel::Polyline assemblePolyline(std::vector<double>& coords) {
    if (inputIsClosed(coords)) {
        // Remove the last point at the polyline otherwise will have a self loop
        // when an edge is introduced between the first and last vertex
        coords.pop_back();
        coords.pop_back();
    }
    assert(coords.size() % 2 == 0);
    std::vector<liblabel::Point> points;
    for(auto it = coords.begin(), end = coords.end(); it != end; ++it) {
        points.push_back({*it, *++it});
    }
    return {points};
}

struct Input {
    liblabel::Aspect aspect;
    liblabel::Polygon poly;
};

Input streamInput() {
    liblabel::Aspect aspect;

    cin >> aspect;
    std::cin.ignore();      // ignore the newline character after aspect input

    std::vector<double> coords;
    double tmp;

    std::string szTmp;
    std::getline(cin, szTmp);

    std::istringstream szStream(szTmp);
    std::copy(std::istream_iterator<double>(szStream),
        std::istream_iterator<double>(),
        std::back_inserter(coords));
    liblabel::Polyline outer = assemblePolyline(coords);

    std::vector<liblabel::Polyline> holes;
    for(std::string tmp; std::getline(cin, tmp);) {
        coords.clear();
        szStream = std::istringstream(tmp);
        std::copy(std::istream_iterator<double>(szStream),
            std::istream_iterator<double>(),
            std::back_inserter(coords));
        if(coords.size() == 0) {
            continue;
        }
        holes.push_back(assemblePolyline(coords));
    }

    return {aspect, {outer, holes}};
}

std::optional<Input> fileInput(std::string& filename) {
    std::ifstream inFile(filename, std::ios::in);
    if (!inFile.is_open()) {
        std::cout << "Input file " << filename << " could not be opened." << std::endl;
        return {};
    }

    liblabel::Aspect aspect;
    std::string szTmp;
    std::getline(inFile, szTmp);
    std::istringstream szStream(szTmp);
    szStream >> aspect;

    std::vector<double> coords;
    std::getline(inFile, szTmp);
    szStream = std::istringstream(szTmp);
    std::copy(std::istream_iterator<double>(szStream),
        std::istream_iterator<double>(),
        std::back_inserter(coords));
    liblabel::Polyline outer = assemblePolyline(coords);

    std::vector<liblabel::Polyline> holes;
    for(std::string tmp; std::getline(inFile, tmp);) {
        coords.clear();
        szStream = std::istringstream(tmp);
        std::copy(std::istream_iterator<double>(szStream),
            std::istream_iterator<double>(),
            std::back_inserter(coords));
        if(coords.size() == 0) {
            continue;
        }
        holes.push_back(assemblePolyline(coords));
    }

/*    std::cout << "Got a polygon of size " << std::to_string(outer.points.size())
              << " many nodes and " << std::to_string(holes.size())
              << " many holes" << std::endl;*/

    return Input{aspect, {outer, holes}};
}
Input interactiveInput() {
    liblabel::Aspect aspect;

    cout << "Give the label aspect ratio A = H/W: ";
    cin >> aspect;
    std::cin.ignore();      // ignore the newline character after aspect input

    std::vector<double> coords;
    double tmp;
    std::string szTmp;
    cout << "Give the sequence of coordinates defining the outer boundary: " << endl;
    std::getline(cin, szTmp);
    std::istringstream szStream(szTmp);
    std::copy(std::istream_iterator<double>(szStream),
        std::istream_iterator<double>(),
        std::back_inserter(coords));
    cout << "Outer Polygon contained " << coords.size() << " many elements." << endl;
    liblabel::Polyline outer = assemblePolyline(coords);

    std::vector<liblabel::Polyline> holes;
    char input = 'i';
    while(input == 'i') {
        cout << "Enter i to insert a hole (c to cancel): ";
        cin >> input;
        std::cin.ignore();      // ignore the newline character after aspect input
        if(input != 'i') continue;
        coords.clear();
        cout << "Give the sequence of coordinates defining the boundary of the hole: " << endl;
        std::getline(cin, szTmp);
        szStream = std::istringstream(szTmp);
        std::copy(std::istream_iterator<double>(szStream),
            std::istream_iterator<double>(),
            std::back_inserter(coords));
        holes.push_back(assemblePolyline(coords));
    }

    return {aspect, {outer, holes}};
}

int main(int argc, char** argv) {
    if(argc < 2) {
        cout << "Please use -i for interactive, -f for file or -s for streamed input."
             << endl;
    } else if ("-i" == std::string(argv[1])) {
        bool barrault = (argc == 3 && argv[2] == std::string("-b"));
        cout << "Get labeling parameters interactively!" << endl;
        Input input = interactiveInput();

        cout << "Let's compute a curved area label!" << endl;
        cout << "Aspect was " << input.aspect << endl;
        cout << "Input poly outer poly has " << input.poly.outer.points.size() << " points"
            << " and " << input.poly.holes.size() << " holes." << endl;

        if (barrault) 
            std::cout << "Starting labeling using Barrault-like procedure ..." << std::endl;
        else 
            std::cout << "Starting labeling using standard procedure ..." << std::endl;
        
        auto labelOp = barrault? liblabel::computeLabelBarrault(input.aspect, input.poly, true) : liblabel::computeLabel(input.aspect, input.poly, true);
        
        cout << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        if(labelOp.has_value()) {
            auto label = labelOp.value();
            cout << "Computed label was:\n"
                << "Center\t" << "(" << label.center.x << ", " << label.center.y << ")\n"
                << "Radii \t" << "low: " << label.rad_lower << " up: " << label.rad_upper << "\n"
                << "Angles\t" << "normal: " << label.normal << " extend: " << label.extend << endl;
            cout << "As tuple: " << endl;
            cout << "(" << label.center.x << ", " << label.center.y
                << ", " << label.rad_lower << ", " << label.rad_upper
                << ", " << label.normal << ", " << label.extend << ")"
                << endl;
        } else {
            cerr << "Label for the given input could not be constructed!" << endl;
        }
    } else if ("-s" == std::string(argv[1])) {
        bool barrault = (argc == 3 && argv[2] == std::string("-b"));
        Input input = streamInput();

        bool debug = false;
        auto labelOp = barrault? liblabel::computeLabelBarrault(input.aspect, input.poly, debug) : liblabel::computeLabel(input.aspect, input.poly, debug);

        cout << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        if(labelOp.has_value()) {
            auto label = labelOp.value();
            cout << "AreaLabel: "
                << "(" << label.center.x << ", " << label.center.y
                << ", " << label.rad_lower << ", " << label.rad_upper
                << ", " << label.normal << ", " << label.extend << ")"
                << endl;
        } else {
            cerr << "Label for the given input could not be constructed!" << endl;
        }
    } else if ("-f" == std::string(argv[1])) {
        if (argc < 3) {
            std::cout << "Please give an input file after specifying '-f' (-f input file name)" << std::endl;
        }
        std::string filename = std::string(argv[2]);
        bool barrault = (argc >= 4 && argv[3] == std::string("-b"));
        std::size_t candidates = 30;
        if (argc >= 5 && argv[3] == std::string("-c")) {
            candidates = std::stoi(argv[4]);
        }

        auto inputOpt = fileInput(filename);
        if (!inputOpt) {
            std::cout << "Could not read input file " << filename << ".\nExiting!" << std::endl;
        }
        Input input = inputOpt.value();

        bool debug = false;
        liblabel::Config conf;
        conf.numberOfPaths = candidates;
        auto labelOp = barrault? liblabel::computeLabelBarrault(input.aspect, input.poly, debug) : liblabel::computeLabel(input.aspect, input.poly, debug, conf);

        cout << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        if(labelOp.has_value()) {
            auto label = labelOp.value();
            cout << "AreaLabel: "
                << "(" << label.center.x << ", " << label.center.y
                << ", " << label.rad_lower << ", " << label.rad_upper
                << ", " << label.normal << ", " << label.extend << ")"
                << endl;
        } else {
            cerr << "Label for the given input could not be constructed!" << endl;
        }
    } else {
        cout << "Please use -i for interactive or -s for streamed input."
             << endl;
    }
    return 0;
}