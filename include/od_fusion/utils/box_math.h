#pragma once
#include <algorithm>
#include <cmath>
#include "od_fusion/base/obstacle_constant.h"

namespace perception {
namespace fusion {


struct Point2D {
    float x;
    float y;

    Point2D() : x(0), y(0) {}
    Point2D(float x_, float y_) : x(x_), y(y_) {}
};

// 多边形面积计算（vector 版）
float polygonArea(const vector<Point2D>& poly) {
    float area = 0.0f;
    int n = poly.size();
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += poly[i].x * poly[j].y;
        area -= poly[i].y * poly[j].x;
    }
    return fabs(area) * 0.5f;
}

// 点是否在多边形内部（射线法，vector 版）
bool pointInPolygon(const Point2D& p, const vector<Point2D>& poly) {
    bool inside = false;
    int n = poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
            (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x))
            inside = !inside;
    }
    return inside;
}

// 线段相交求交点
bool lineIntersection(const Point2D& a1, const Point2D& a2,
                      const Point2D& b1, const Point2D& b2, Point2D& out) {
    float den = (a1.x - a2.x) * (b1.y - b2.y) - (a1.y - a2.y) * (b1.x - b2.x);
    if (fabs(den) < 1e-6f) return false;

    float t = ((a1.x - b1.x) * (b1.y - b2.y) - (a1.y - b1.y) * (b1.x - b2.x)) / den;
    out.x = a1.x + t * (a2.x - a1.x);
    out.y = a1.y + t * (a2.y - a1.y);
    return true;
}

// 两个任意多边形 IoU（不再限定4点，通用 vector 版）
float calcIoU(const vector<Point2D>& boxA, const vector<Point2D>& boxB) {
    // 收集所有交点 + 内部点
    vector<Point2D> intersectPoly;
    intersectPoly.reserve(64);  // 预分配空间

    // 1. 加入 A 在 B 内部的点
    for (const auto& p : boxA) {
        if (pointInPolygon(p, boxB)) {
            intersectPoly.push_back(p);
        }
    }

    // 2. 加入 B 在 A 内部的点
    for (const auto& p : boxB) {
        if (pointInPolygon(p, boxA)) {
            intersectPoly.push_back(p);
        }
    }

    // 3. 加入所有边交点
    Point2D pt;
    int aSize = boxA.size();
    int bSize = boxB.size();
    for (int i = 0; i < aSize; ++i) {
        int j = (i + 1) % aSize;
        for (int k = 0; k < bSize; ++k) {
            int l = (k + 1) % bSize;
            if (lineIntersection(boxA[i], boxA[j], boxB[k], boxB[l], pt)) {
                intersectPoly.push_back(pt);
            }
        }
    }

    // 点数量不足3个，无交集
    if (intersectPoly.size() < 3) {
        return 0.0f;
    }

    float areaInter = polygonArea(intersectPoly);
    float areaA = polygonArea(boxA);
    float areaB = polygonArea(boxB);
    float areaUnion = areaA + areaB - areaInter;

    return (areaUnion < 1e-6f) ? 0.0f : (areaInter / areaUnion);
}

vector<Point2D> createBox(FusedObject& fobj) {
    float yaw = fobj.object.yaw;
    float length = fobj.object.length;
    float width = fobj.object.width;
    double cx = fobj.object.x;
    double cy = fobj.object.y;
    std::vector<Point2D> box;

    double c = std::cos(yaw);
    double s = std::sin(yaw);

  // 以中心为原点，局部四角点
  std::vector<std::pair<double, double>> corners = {
      {length / 2, width / 2}, {length / 2, -width / 2}, {-length / 2, -width / 2}, {-length / 2, width / 2}};

  for (const auto& corner : corners) {
    Point2D p;
    p.x = cx + (corner.first * c - corner.second * s);
    p.y = cy + (corner.first * s + corner.second * c);
    p.z = z;
    box.push_back(p);
  }

  return box;

}

}  // namespace fusion
}  // namespace perception