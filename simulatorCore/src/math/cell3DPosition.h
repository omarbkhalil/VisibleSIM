/*!
 * \file cell3DPosition.h
 * \brief 3D integer position in a grid
 * \date 29/01/2012
 * \author Benoît Piranda
 */

#ifndef CELL3DPOSITION_H
#define CELL3DPOSITION_H

#include <iostream>
#include <fstream>
#include <cmath>
#include "vector3D.h"

using namespace std;

class Cell3DPosition {
public:
    short pt[3]; //!< (x,y,z) values of the vector

    Cell3DPosition();
    constexpr Cell3DPosition(short x, short y, short z) : pt{x, y, z} { }
    Cell3DPosition(const Vector3D& v);
    constexpr Cell3DPosition(const Cell3DPosition& c) : pt{ c.pt[0], c.pt[1], c.pt[2] } { };

    void set(short x, short y, short z);
    int dist_taxi(const Cell3DPosition& p) const;
    double dist_euclid(const Cell3DPosition& p) const;
    double l2_norm() const;
    string to_string() const;
    string config_print() const;

    Cell3DPosition offsetX(short x) const;
    Cell3DPosition offsetY(short y) const;
    Cell3DPosition offsetZ(short z) const;

    inline const short operator[](const int i) const { return pt[i]; }

    bool isAdjacentTo(const Cell3DPosition &other) const;
    bool operator<(const Cell3DPosition &o) const;
    bool operator==(const Cell3DPosition &o) const {
        return (pt[0] == o.pt[0]) && (pt[1] == o.pt[1]) && (pt[2] == o.pt[2]);
    };

    /**
     * Compares two positions by considering first the Z component, then Y, then X.
     */
    static bool compare_ZYX(const Cell3DPosition& first, const Cell3DPosition& second);

    bool operator!=(const Cell3DPosition &o) const { return !(operator==(o)); }
    const Cell3DPosition& operator+=(const Cell3DPosition&p);
    operator Vector3D() const { return Vector3D(pt[0], pt[1], pt[2], 1.0); };

    friend ostream& operator<<(ostream& f, const Cell3DPosition&p);
    friend const Cell3DPosition operator +(const Cell3DPosition&, const Cell3DPosition&);
    friend const Cell3DPosition operator -(const Cell3DPosition&, const Cell3DPosition&);
    friend const Cell3DPosition operator *(const Cell3DPosition&, const Cell3DPosition&);
    friend const Cell3DPosition operator *(int, const Cell3DPosition&);

    /**
     * Serializes the Cell3DPosition to a binary stream.
     *
     * @param bStream output binary stream
     */
    void serialize(std::ofstream &bStream) const;

    /**
     * Writes a clear-text version of the Cell3DPosition to a stream.
     *
     * @param dbStream output text stream
     */
    void serialize_cleartext(std::ofstream &dbStream) const;

    /**
     * Deserializes a Cell3DPosition from a binary stream.
     *
     * @param bStream input binary stream
     */
    void deserialize(std::ifstream &bStream);
};

inline void Cell3DPosition::serialize(std::ofstream &bStream) const {
    bStream.write(reinterpret_cast<const char*>(this->pt), sizeof(this->pt));
}

inline void Cell3DPosition::serialize_cleartext(std::ofstream &dbStream) const {
    dbStream << this->pt[0] << " " << this->pt[1] << " " << this->pt[2];
}

inline void Cell3DPosition::deserialize(std::ifstream &bStream) {
    bStream.read(reinterpret_cast<char*>(this->pt), sizeof(this->pt));
}

#endif // CELL3DPOSITION_H
