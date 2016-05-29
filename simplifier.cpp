#include "simplifier.h"
#include <cstdio>

using namespace std;
using namespace cv;

void Simplifier::parser(string objfile) {
    FILE* inobj = fopen(objfile.c_str(), "r");
    char* buffer = new char[1000];
    while (!feof(inobj)) {
        fgets(buffer, 1000, inobj);
        if (buffer[0] == 'v') {
            Vertex ver;
            sscanf(buffer, "v %lf %lf %lf", &ver.pos[0], &ver.pos[1], &ver.pos[2]);
            ver.left = true;
            vlist.push_back(ver);
        }
        else if (buffer[0] == 'f') {
            Face face;
            size_t fv[3];
            sscanf(buffer, "f %lu %lu %lu", &fv[0], &fv[1], &fv[2]);
            for (int i = 0; i < 3; ++i) {
                fv[i] -= 1; // obj file number start from 1!!!
                face.ves.push_back(fv[i]);
            }
            face.left = true;
            flist.push_back(face);
        }
    }
    fclose(inobj);
    delete []buffer;
}

void Simplifier::init() {
    for (auto i = flist.begin(); i != flist.end(); ++i) {
        Face& face = (*i);
        for (auto fv = face.ves.begin(); fv != face.ves.end(); ++fv) {
            vlist.at(*fv).faces.push_back(i - flist.begin());
        }
    }
    for (auto i = flist.begin(); i != flist.end(); ++i) {
        set_face(*i);
    }
    for (auto i = vlist.begin(); i != vlist.end(); ++i) {
        set_vertex(*i);
    }
    set<pair<size_t, size_t> > pairset;
    for (auto i = flist.begin(); i != flist.end(); ++i) {
        vector<size_t> vl;
        for (auto v = (*i).ves.begin(); v !=(*i).ves.end(); ++v) {
            vl.push_back(*v);
        }
        for (int vr = 0; vr < 3; ++vr) {
            size_t v1 = vl[vr], v2 = vl[(vr + 1) % 3];
            if (v1 > v2)
                swap(v1, v2);
            if (pairset.find(make_pair(v1, v2)) == pairset.end()) {
                pairset.insert(make_pair(v1, v2));
                Pair pair;
                pair.v = make_pair(v1, v2);
                set_pair(pair);
                pairheap.push(pair);
            }
        }
    }
}

void Simplifier::simplify(double rate) {
    size_t former_face_num = flist.size();
    size_t facenum = former_face_num;
    while ((double)facenum / (double)former_face_num > rate) {
        cout << (double)facenum / (double)former_face_num << '\r';
        Pair pshrink = pairheap.top();
        pairheap.pop();
        if (!vlist.at(pshrink.v.first).left || !vlist.at(pshrink.v.second).left)
            continue;
        bool border1 = isatborder(pshrink.v.first);
        bool border2 = isatborder(pshrink.v.second);
        if (border1 ^ border2)
            continue;
        vlist.at(pshrink.v.first).left = false;
        vlist.at(pshrink.v.second).left = false;
        vlist.push_back(Vertex());
        size_t newvr = vlist.size() - 1;
        vlist.at(newvr).pos = pshrink.bestv;
        vlist.at(newvr).left = true;

        set<size_t> neighbor_faces;
        set<size_t> neighbor_vets;
        neighbor_faces.insert(vlist.at(pshrink.v.first).faces.begin(), vlist.at(pshrink.v.first).faces.end());
        neighbor_faces.insert(vlist.at(pshrink.v.second).faces.begin(), vlist.at(pshrink.v.second).faces.end());
        for (auto face = neighbor_faces.begin(); face != neighbor_faces.end(); ++face) {
            neighbor_vets.insert(flist.at(*face).ves.begin(), flist.at(*face).ves.end());
        }
        neighbor_vets.erase(pshrink.v.first);
        neighbor_vets.erase(pshrink.v.second);

        for (auto fr = neighbor_faces.begin(); fr != neighbor_faces.end(); ++fr) {
            for (auto vr = flist.at(*fr).ves.begin(); vr != flist.at(*fr).ves.end(); ++vr) {
                if ((*vr) == pshrink.v.first || (*vr) == pshrink.v.second)
                    (*vr) = newvr;
            }
            set<size_t> fvs;
            fvs.insert(flist.at(*fr).ves.begin(), flist.at(*fr).ves.end());
            if (fvs.size() != 3) {
                flist.at(*fr).left = false;
                for (auto vvr = flist.at(*fr).ves.begin(); vvr != flist.at(*fr).ves.end(); ++vvr) {
                    if ((*vvr) != newvr) {
                        auto target = find(vlist.at(*vvr).faces.begin(), vlist.at(*vvr).faces.end(), *fr);
                        vlist.at(*vvr).faces.erase(target);
                    }
                }
                --facenum;
            }
            else {
                set_face(flist.at(*fr));
                vlist.at(newvr).faces.push_back(*fr);
            }
        }

        set_vertex(vlist.at(newvr));
        vector<size_t> refreshed_v;
        refreshed_v.push_back(newvr);
        for (auto vr = neighbor_vets.begin(); vr != neighbor_vets.end(); ++vr) {
            set_vertex(vlist.at(*vr));
            vlist.at(*vr).left = false;
            vlist.push_back(Vertex());
            size_t nv = vlist.size() - 1;
            vlist.at(nv) = vlist.at(*vr);
            vlist.at(nv).left = true;
            for (auto fr = vlist.at(*vr).faces.begin(); fr != vlist.at(*vr).faces.end(); ++fr) {
                auto target = find(flist.at(*fr).ves.begin(), flist.at(*fr).ves.end(), *vr);
                (*target) = nv;
            }
            refreshed_v.push_back(nv);
        }

        set<pair<size_t, size_t> > refreshed_pair;
        for (auto vr = refreshed_v.begin(); vr != refreshed_v.end(); ++vr) {
            set<size_t> nbv;
            for (auto fr = vlist.at(*vr).faces.begin(); fr != vlist.at(*vr).faces.end(); ++fr) {
                nbv.insert(flist.at(*fr).ves.begin(), flist.at(*fr).ves.end());
            }
            for (auto nbvr = nbv.begin(); nbvr != nbv.end(); ++nbvr) {
                if (*nbvr != *vr) {
                    auto refpair = make_pair(*nbvr, *vr);
                    if (refpair.first > refpair.second)
                        swap(refpair.first, refpair.second);
                    if (refreshed_pair.find(refpair) == refreshed_pair.end()) {
                        Pair rpair;
                        rpair.v = refpair;
                        set_pair(rpair);
                        pairheap.push(rpair);
                        refreshed_pair.insert(refpair);
                    }
                }
            }
        }
    }
}

void Simplifier::writer(string outobjfile) {
    ofstream outobj(outobjfile);
    size_t lvrank = 1;
    for (auto i = vlist.begin(); i != vlist.end(); ++i) {
        if ((*i).left) {
            (*i).finalrank = lvrank;
            ++lvrank;
            outobj << "v " << (*i).pos[0] << ' ' << (*i).pos[1] << ' ' << (*i).pos[2] << endl;
        }
    }
    for (auto i = flist.begin(); i != flist.end(); ++i) {
        if ((*i).left) {
            vector<size_t> vl((*i).ves.begin(), (*i).ves.end());
            outobj << "f " << vlist.at(vl[0]).finalrank << ' ' << vlist.at(vl[1]).finalrank << ' ' << vlist.at(vl[2]).finalrank << endl;
        }
    }
    outobj.close();
}

void Simplifier::set_face(Face &face) {
    vector<Vec3d> tv;
    for (auto i = face.ves.begin(); i != face.ves.end(); ++i) {
        tv.push_back(vlist.at(*i).pos);
    }
    Vec3d abc = normalize((tv.at(1) - tv.at(0)).cross(tv.at(2) - tv.at(0)));
    for (int k = 0; k < 3; ++k)
        face.series(k) = abc[k];
    face.series(3) = - (abc.dot(tv.at(0)));
    face.kpmat = face.series * face.series.t();
}

void Simplifier::set_vertex(Vertex &vertex) {
    vertex.qmat = Matx44d::zeros();
    for (auto i = vertex.faces.begin(); i != vertex.faces.end(); ++i) {
        vertex.qmat += flist.at(*i).kpmat;
    }
}

void Simplifier::set_pair(Pair &pair) {
    Matx44d pqmat = vlist.at(pair.v.first).qmat + vlist.at(pair.v.second).qmat;
    Matx44d smata = Matx44d::zeros();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i > j)
                smata(i, j) = smata(j, i);
            else
                smata(i, j) = pqmat(i, j);
        }
    }
    smata(3, 0) = smata(3, 1) = smata(3, 2) = 0;
    smata(3, 3) = 1;
    Matx41d ans = Matx41d::zeros(); ans(3) = 1;
    Matx41d vm = Matx41d::zeros();
    int judge = solve(smata, ans, vm);
    if (judge == 1) {

    }
    else {
        solve(smata, ans, vm, DECOMP_SVD);
    }
    for(int i = 0; i < 3; ++i)
        pair.bestv[i] = vm(i);
    pair.evalue = (vm.t() * pqmat * vm)(0);
}

bool Simplifier::isatborder(size_t vertex_r) {
    map<size_t, int> times_of_neighborv;
    Vertex& vertex = vlist.at(vertex_r);
    for (auto fr = vertex.faces.begin(); fr != vertex.faces.end(); ++fr) {
        for (auto vr = flist.at(*fr).ves.begin(); vr != flist.at(*fr).ves.end(); ++vr) {
            if (*vr != vertex_r) {
                auto target = times_of_neighborv.find(*vr);
                if (target == times_of_neighborv.end()) {
                    times_of_neighborv.insert(make_pair(*vr, 1));
                }
                else {
                    ++(*target).second;
                }
            }
        }
    }
    for (auto mr = times_of_neighborv.begin(); mr != times_of_neighborv.end(); ++mr) {
        if ((*mr).second == 1)
            return true;
    }
    return false;
}
