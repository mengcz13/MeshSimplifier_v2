#ifndef SIMPLIFIER_H
#define SIMPLIFIER_H
#include <string>
#include <set>
#include <queue>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <utility>
#include <opencv2/opencv.hpp>

struct Vertex;
struct Face;

struct Vertex {
    cv::Vec3d pos;
    cv::Matx44d qmat;
    bool left; // false if v is moved
    size_t finalrank; // finalrank is given after simplification!
    std::vector<size_t> faces;
};

struct Face {
    cv::Matx41d series;
    cv::Matx44d kpmat;
    bool left;
    // int rank; // given after simplification
    std::vector<size_t> ves;
};

struct Pair {
    std::pair<size_t, size_t> v;
    double evalue;
    cv::Vec3d bestv;
};

struct PairComp {
    bool operator () (const Pair& pa, const Pair& pb) {
        return (pa.evalue > pb.evalue);
    }
};

class Simplifier {
public:
    void parser(std::string objfile);
    void init();
    void simplify(double rate);
    void writer(std::string outobjfile);
private:
    std::vector<Vertex> vlist;
    std::vector<Face> flist;
    std::priority_queue<Pair, std::vector<Pair>, PairComp> pairheap;

    void set_face(Face& face);
    void set_vertex(Vertex& vertex);
    void set_pair(Pair& pair);

    bool isatborder(size_t vertex_r); // Judge if the vertex is at border, if it is, each of its neighbor vertexes will appear twice
};

#endif // SIMPLIFIER_H
