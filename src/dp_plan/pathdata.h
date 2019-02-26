//
// Created by jydragon on 18-8-7.
//

#ifndef LATTICEPLAN_PATHDATA_H
#define LATTICEPLAN_PATHDATA_H

#include <list>
#include <string>
#include "common.h"

class PathData {
public:
    PathData() = default;

    bool SetDiscretizedPath(const common::DiscretizedPath &path);

    bool SetFrenetPath(const common::FrenetFramePath &frenet_path);

    void SetReferenceLine(const common::ReferenceLine *reference_line);

    const common::DiscretizedPath &discretized_path() const;

    const common::FrenetFramePath &frenet_frame_path() const;

    bool GetPathPointWithPathS(const double s,
                               common::PathPoint *const path_point) const;

    std::list<std::pair<common::DiscretizedPath, common::FrenetFramePath>> &path_data_history();

    /*
     * brief: this function will find the path_point in discretized_path whose
     * projection to reference line has s value closest to ref_s.
     */
    bool GetPathPointWithRefS(const double ref_s,
                              common::PathPoint *const path_point) const;

    void Clear();

    bool Empty() const;

    std::string DebugString() const;

private:
    /*
     * convert frenet path to cartesian path by reference line
     *///TODO 这里没完成。。。
    bool SLToXY(const common::FrenetFramePath &frenet_path,
                common::DiscretizedPath *const discretized_path);
    bool XYToSL(const common::DiscretizedPath &discretized_path,
                common::FrenetFramePath *const frenet_path);
    const common::ReferenceLine *reference_line_ = nullptr;
    common::DiscretizedPath discretized_path_;
    common::FrenetFramePath frenet_path_;
    std::list<std::pair<common::DiscretizedPath, common::FrenetFramePath>> path_data_history_;
};


#endif //LATTICEPLAN_PATHDATA_H
