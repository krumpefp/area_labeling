#include <algorithm>
#include <iostream>
#include <math.h>
#include <string>

#include "liblabeling.h"

#include "circle_apx.hpp"
#include "label_fit.hpp"
#include "longest_paths.hpp"
#include "segments_to_graph.hpp"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

const size_t TARGET_POLY_SIZE = 100;

namespace debug {
    using namespace std;
    void printPath(std::vector<liblabel::Point>& path) {
        for(auto p : path) {
            cout << "(" << p.x << "," << p.y << "), ";
        }
        cout << endl;
    }

    void printPolyline(liblabel::Polyline& pl) {
        for(auto p : pl.points) {
            cout << "(" << p.x << "," << p.y << "), ";
        }
        cout << endl;
    }
    void printPolygon(liblabel::Polygon& poly) {
        cout << "Outer boundary:" << endl;
        printPolyline(poly.outer);
        for(auto hole : poly.holes) {
            cout << "Hole: " << endl;
            printPolyline(hole);
        }
        cout << endl;
    }
}

namespace {
    // CGAL
    using K = CGAL::Exact_predicates_inexact_constructions_kernel;
    using KPoint = K::Point_2;
    using KSegment = K::Segment_2;
    using KPolygon = CGAL::Polygon_2<K>;
    using KPolyWithHoles = CGAL::Polygon_with_holes_2<K>;

    struct AugmentedSkeletonEdge {
        liblabel::Point src, trgt;
        double dist, clear;
    };

    using Path = std::vector<liblabel::Point>;
    using Skeleton = std::vector<AugmentedSkeletonEdge>;

    KPolyWithHoles constructPolygon(const liblabel::Polygon& poly);

    std::optional<std::vector<AugmentedSkeletonEdge>> constructSkeleton(const KPolyWithHoles&);

    std::vector<Path> computeLongestPaths(const std::vector<AugmentedSkeletonEdge>&, const liblabel::Aspect, const liblabel::Config&);

    std::vector<Path> findLongestPathsBarrault(const std::vector<AugmentedSkeletonEdge>&, size_t);

    std::optional<liblabel::AreaLabel> evaluatePaths(const std::vector<Path>&, const liblabel::Aspect, const KPolyWithHoles&, const liblabel::Config&);
}

std::optional<liblabel::AreaLabel> liblabel::computeLabelBarrault(
        liblabel::Aspect aspect,
        Polygon& poly,
        bool progress,
        liblabel::Config configuration
) {
    if(progress) std::cout << "Constructing the polygon ..." << std::endl;
    KPolyWithHoles ph = constructPolygon(poly);
    if(progress) std::cout << "... finished.\nOuter polygon was supsampled to "
                           << ph.outer_boundary().size() << " many points." << std::endl;

    // Construct the skeleton
    if(progress) std::cout << "Construncting the skeleton ..." << std:: endl;
    auto skeletonOp = constructSkeleton(ph);
    if(progress) std::cout << "... finished" << std:: endl;
    if(!skeletonOp.has_value()) {
        std::cout << "ERROR: Could not construct polygon skeleton!" << std::endl;
        return {};
    }
    if(progress) std::cout << "The computed skeleton contains " << skeletonOp.value().size() << " many edges" << std::endl;

    // Find candidate paths
    if(progress) std::cout << "Searching for 50 candidate paths ..." << std::endl;
    auto paths = findLongestPathsBarrault(skeletonOp.value(), 50);
    //auto paths = computeLongestPaths(skeletonOp.value(), aspect, configuration);
    if(progress) std::cout << "... finished. Found " << paths.size() << " candidate paths" << std::endl;
    if(paths.size() == 0) {
        std::cout << "ERROR: No candidate paths have been found!" << std::endl;
        return {};
    }

    // Evaluate paths
    if(progress) std::cout << "Evaluating paths ..." << std::endl;
    auto res = evaluatePaths(paths, aspect, ph, configuration);
    if(!res.has_value()) {
        if(progress) std::cout << "... finished without an result!" << std::endl;
    } else {
        if(progress) std::cout << "... finished" << std::endl;
    }

    return res;
}

std::optional<liblabel::AreaLabel> liblabel::computeLabel(
        liblabel::Aspect aspect,
        Polygon& poly,
        bool progress,
        liblabel::Config configuration
    ){
    if(progress) std::cout << "Constructing the polygon ..." << std::endl;
    KPolyWithHoles ph = constructPolygon(poly);
    if(progress) std::cout << "... finished.\nOuter polygon was supsampled to "
                           << ph.outer_boundary().size() << " many points." << std::endl;

    // Construct the skeleton
    if(progress) std::cout << "Construncting the skeleton ..." << std:: endl;
    auto skeletonOp = constructSkeleton(ph);
    if(progress) std::cout << "... finished" << std:: endl;
    if(!skeletonOp.has_value()) {
        return {};
    }
    if(progress) std::cout << "The computed skeleton contains " << skeletonOp.value().size() << " many edges" << std::endl;

    // Find candidate paths
    if(progress) std::cout << "Searching for longest paths ..." << std::endl;
    auto paths = computeLongestPaths(skeletonOp.value(), aspect, configuration);
    if(progress) std::cout << "... finished. Found " << paths.size() << " candidate paths" << std::endl;

    // Evaluate paths
    if(progress) std::cout << "Evaluating paths ..." << std::endl;
    auto res = evaluatePaths(paths, aspect, ph, configuration);
    if(!res.has_value()) {
        if(progress) std::cout << "... finished without an result!" << std::endl;
    } else {
        if(progress) std::cout << "... finished" << std::endl;
    }

    return res;
}

namespace {
    double segLength(const KSegment& seg) {
        return sqrt(CGAL::squared_distance(seg.source(), seg.target()));
    }

    std::vector<KPoint> supsampleSegment(const KSegment& seg, double precision) {
        auto s = seg.source();
        auto t = seg.target();

        double dist = segLength(seg);
        size_t target = (size_t) ceil(dist / precision) - 1; // nr of points to insert

        double stepX = (t.x() - s.x()) / (target + 1);
        double stepY = (t.y() - s.y()) / (target + 1);

        std::vector<KPoint> res;
        res.emplace_back(s);
        for (size_t i = 1; i <= target; ++i) {
            res.emplace_back(s.x() + i*stepX, s.y() + i*stepY);
        }
        res.emplace_back(t);

        return res;
    }

    KPolygon supsampleSimplePolygon(const KPolygon& poly, double precision) {
        std::vector<KPoint> res;
        for(auto eit = poly.edges_begin(), end = poly.edges_end(); eit != end; ++eit) {
            auto sups = supsampleSegment(*eit, precision);
            sups.pop_back();
            res.insert(res.end(), sups.begin(), sups.end());
        }

        return KPolygon(res.begin(), res.end());
    }

    double polyLength(const KPolygon& poly) {
        double dist = 0;

        for(auto eit = poly.edges_begin(), end = poly.edges_end(); eit != end; ++eit) {
            dist += segLength(*eit);
        }

        return dist;
    }

    KPolyWithHoles supsamplePolygon(const KPolyWithHoles& poly, size_t target_size) {
        double precision = polyLength(poly.outer_boundary()) / target_size;

        KPolygon supsOuter = supsampleSimplePolygon(poly.outer_boundary(), precision);
        std::vector<KPolygon> supsHoles;
        for(auto hit = poly.holes_begin(), end = poly.holes_end(); hit != end; ++hit) {
            supsHoles.push_back(supsampleSimplePolygon(*hit, precision));
        }

        return KPolyWithHoles(supsOuter, supsHoles.begin(), supsHoles.end());
    }

    KPolyWithHoles constructPolygon(const liblabel::Polygon& poly) {
        std::vector<KPoint> tempPts;
        std::transform(poly.outer.points.begin(), poly.outer.points.end(),
            std::back_inserter(tempPts),
            [](liblabel::Point p) -> KPoint { return {p.x, p.y}; });
        KPolygon outer(tempPts.begin(), tempPts.end());
        std::vector<KPolygon> holes;
        for(liblabel::Polyline hole : poly.holes){
            tempPts.clear();
            std::transform(hole.points.begin(), hole.points.end(),
                std::back_inserter(tempPts),
                [](liblabel::Point p) -> KPoint { return {p.x, p.y}; });
            holes.emplace_back(tempPts.begin(), tempPts.end());
        }

        return supsamplePolygon(
            KPolyWithHoles(outer, holes.begin(), holes.end()),
            TARGET_POLY_SIZE
        );
    }

    std::optional<std::vector<AugmentedSkeletonEdge>> constructSkeleton(const KPolyWithHoles& ph) {
        if(ph.outer_boundary().size() == 0) {
            return {};
        }

        std::vector<KSegment> cgal_segs;
        std::copy(ph.outer_boundary().edges_begin(),
            ph.outer_boundary().edges_end(),
            std::back_inserter(cgal_segs));
        for(auto hit = ph.holes_begin(), end = ph.holes_end(); hit != end; ++hit) {
            std::copy(hit->edges_begin(), hit->edges_end(),
                std::back_inserter(cgal_segs));
        }

        CDT cdt(cgal_segs.begin(), cgal_segs.end());
        if(!cdt.is_valid()) {
            return {};
        }
        auto skeleton_edges = compute_skeleton_edges(cdt);

        std::vector<AugmentedSkeletonEdge> res;
        std::transform(skeleton_edges.begin(), skeleton_edges.end(),
            std::back_inserter(res),
            [](SkeletonEdge e) -> AugmentedSkeletonEdge { return {{e.p.x(), e.p.y()}, {e.q.x(), e.q.y()}, CGAL::squared_distance(e.p, e.q), e.d};});
        return res;
    }

    std::vector<Path> findLongestPathsBarrault( const std::vector<AugmentedSkeletonEdge>& augSkelEdges, size_t k = 50) {
        std::vector<longest_paths::Segment> segs;
        std::transform(augSkelEdges.begin(), augSkelEdges.end(),
            std::back_inserter(segs),
            [](AugmentedSkeletonEdge e) -> longest_paths::Segment {
                return {{e.src.x, e.src.y}, {e.trgt.x, e.trgt.y}, e.dist, e.clear};}
            );
        auto graph = from_edges(segs);

        auto paths = find_longest_paths(graph, k);
        
        std::vector<Path> res;
        std::transform(paths.begin(), paths.end(),
            std::back_inserter(res),
            [&graph](auto path) -> Path {
                Path res;
                std::transform(path.begin(), path.end(),
                    std::back_inserter(res),
                    [&graph](auto p) -> liblabel::Point {return {graph[p].x, graph[p].y};});
                return res;
            });
        return res;
    }

    std::vector<Path> computeLongestPaths( const std::vector<AugmentedSkeletonEdge>& augSkelEdges,
                                           const liblabel::Aspect aspect,
                                           const liblabel::Config& config) {
        std::vector<longest_paths::Segment> segs;
        std::transform(augSkelEdges.begin(), augSkelEdges.end(),
            std::back_inserter(segs),
            [](AugmentedSkeletonEdge e) -> longest_paths::Segment {
                return {{e.src.x, e.src.y}, {e.trgt.x, e.trgt.y}, e.dist, e.clear};}
            );
        auto graph = from_edges(segs);

        auto paths = find_distinct_paths(graph, aspect, config.stepSize, config.numberOfPaths);
        
        std::vector<Path> res;
        std::transform(paths.begin(), paths.end(),
            std::back_inserter(res),
            [&graph](auto path) -> Path {
                Path res;
                std::transform(path.begin(), path.end(),
                    std::back_inserter(res),
                    [&graph](auto p) -> liblabel::Point {return {graph[p].x, graph[p].y};});
                return res;
            });
        return res;
    }

    double normalizeAngle(double angle) {
        angle = fmod(angle, 2*M_PI);
        if(angle < 0) {
            angle += 2*M_PI;
        }
        return angle;
    }

    liblabel::AreaLabel constructLabel(const circle_apx_nsp::Circle circle, KPoint pos, const liblabel::Aspect aspect) {
        liblabel::Point center{circle.x, circle.y};
        double baseRadius = circle.r;

        double baseAngle = pos.x();
        double angleRange = pos.y();
        double x = (aspect * angleRange);
        double h = x * baseRadius / (1 + x);
        return {
            center, baseRadius - h, baseRadius + h,
            normalizeAngle(baseAngle),
            normalizeAngle(angleRange) 
        };
    }

    double lblValue(liblabel::AreaLabel& l) {
        double height = l.rad_upper - l.rad_lower;
        return height;
    }

    std::optional<KPoint> computeOptPlacement(const circle_apx_nsp::Circle& c, const liblabel::Aspect aspect, std::vector<K::Segment_2>& segments, const KPolyWithHoles& ph) {
        K::Circle_2 circle = {{c.x, c.y}, c.r*c.r};
        auto cups = compute_all_cups(segments, circle, aspect);

        auto high_points_list = high_points(cups);

        std::vector<K::Point_2> valid_high_points;
        std::copy_if(high_points_list.begin(), high_points_list.end(),
                    std::back_inserter(valid_high_points), [&](Point_2 p) {
                        return ph.outer_boundary().has_on_bounded_side(
                            polar_point(circle, p.x()));
                    });

        if (valid_high_points.empty()) {
            std::cerr << "Could not find any valid high points" << std::endl;
            return {};
        }

        return *std::max_element(valid_high_points.begin(), valid_high_points.end(),
                                [](Point_2 p, Point_2 q) { return p.y() < q.y(); });
    }

    std::optional<liblabel::AreaLabel> evaluatePaths(
            const std::vector<Path>& paths,
            const liblabel::Aspect aspect,
            const KPolyWithHoles& ph,
            const liblabel::Config& config) {
        std::vector<K::Segment_2> cgal_segs;
        std::copy(ph.outer_boundary().edges_begin(),
            ph.outer_boundary().edges_end(),
            std::back_inserter(cgal_segs));
        
        for(auto hit = ph.holes_begin(), end = ph.holes_end(); hit != end; ++hit) {
            std::copy(hit->edges_begin(), hit->edges_end(),
                std::back_inserter(cgal_segs));
        }

        std::vector<liblabel::AreaLabel> result;
        for(auto path : paths) {
            std::vector<circle_apx_nsp::Point> points;
            std::transform(path.begin(), path.end(),
                std::back_inserter(points),
                [](liblabel::Point p) -> circle_apx_nsp::Point { return {p.x, p.y};});

            auto circle = apx_circle(points);

            auto placement = computeOptPlacement(circle, aspect, cgal_segs, ph);
            if(placement.has_value()) {
                auto placement_val = placement.value();
                if(placement_val.y() > config.maxAngle ) {
                    // if the placement > 90°, return none
                    //std::cout << "Should have returned none for angle > " << config.maxAngle << std::endl;
                    // continue;
                }
                result.emplace_back(constructLabel(circle, placement.value(), aspect));
            }
        }

        if(result.size() == 0) {
            return {};
        }

        return *std::max_element(result.begin(), result.end(),
            [](auto l1, auto l2) { return lblValue(l1) < lblValue(l2); });
    }
}