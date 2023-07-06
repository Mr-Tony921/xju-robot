//
// Created by tony on 2022/10/8.
//

#include "frontier_search.h"

namespace xju::explore {
FrontierSearch::FrontierSearch() : map_update_(false) {
    map_sub_ = node_.subscribe("carto_map", 1, &FrontierSearch::map_cb, this);
    map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("explore_map", 1, true);
}

auto FrontierSearch::get_frontier_point(std::optional<geometry_msgs::Pose> const& pose) -> std::optional<Frontier> {
    pub_static_map();
    current_pose_ = pose;
    check_current_frontier();
    find_frontier_point();
    if (open_list_.empty()) {
        return std::nullopt;
    }

    boost::mutex::scoped_lock lock(list_mutex_);
    auto min_sort = std::min_element(open_list_.begin(), open_list_.end(),
                                     [&](Frontier const& a, Frontier const& b) { return a.sort > b.sort; });
    auto frontier = *min_sort;
    open_list_.erase(min_sort);
    close_list_.emplace_back(frontier);
    return std::make_optional(frontier);
}

void FrontierSearch::set_frontier_point(Frontier const& point) {
    boost::mutex::scoped_lock lock(list_mutex_);
    auto idx = std::find_if(open_list_.begin(), open_list_.end(), [&](Frontier const& p) {
        return (std::abs(p.x - point.x) < FRONTIER_RESO && std::abs(p.y - point.y) < FRONTIER_RESO);
    });
    if (idx != open_list_.end()) return;

    idx = std::find_if(close_list_.begin(), close_list_.end(), [&](Frontier const& p) {
        return (std::abs(p.x - point.x) < FRONTIER_RESO && std::abs(p.y - point.y) < FRONTIER_RESO);
    });
    if (idx != close_list_.end()) return;

    open_list_.emplace_back(point);
}

void FrontierSearch::find_frontier_point() {
    auto mat = map_to_img(static_map_);

    cv::Mat mid;
    cv::Canny(mat, mid, 200, 400, 3);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mid, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto iter = contours.begin(); iter < contours.end() - 1;) {
        if (cv::norm((iter + 1)->front() - iter->back()) < CONTOUR_LINE_MIN_GRID) {
            iter->insert(iter->end(), (iter + 1)->begin(), (iter + 1)->end());
            contours.erase(iter + 1);
        } else {
            ++iter;
        }
    }

    for (auto const& contour : contours) {
        auto rect = cv::minAreaRect(contour);
        auto [width, height] = std::minmax(rect.size.width, rect.size.height);
        if (height <= CONTOUR_LINE_MAX_GRID)  // longest edge not satisfy through capacity
            continue;

        if (width <= CONTOUR_LINE_MIN_GRID) {  // line model
            frontier_preprocess(rect.center);
            continue;
        }

        cv::Point2f vertexs[4], center_a, center_b;  // polygon model
        rect.points(vertexs);
        if (cv::norm(vertexs[0] - vertexs[1]) > cv::norm(vertexs[1] - vertexs[2])) {
            center_a.x = 0.5 * (vertexs[0].x + vertexs[1].x);
            center_a.y = 0.5 * (vertexs[0].y + vertexs[1].y);
            center_b.x = 0.5 * (vertexs[2].x + vertexs[3].x);
            center_b.y = 0.5 * (vertexs[2].y + vertexs[3].y);
        } else {
            center_a.x = 0.5 * (vertexs[1].x + vertexs[2].x);
            center_a.y = 0.5 * (vertexs[1].y + vertexs[2].y);
            center_b.x = 0.5 * (vertexs[3].x + vertexs[0].x);
            center_b.y = 0.5 * (vertexs[3].y + vertexs[0].y);
        }

        frontier_preprocess(center_a, true);
        frontier_preprocess(center_b, true);
    }

#ifdef DEBUG_MODE
    static int cnt = 0;
    for (auto const& cr : open_list_) {
        cv::Point2f cv_point;
        cv_point.x = wxgx(static_map_, cr.x);
        cv_point.y = static_map_.info.height - wygy(static_map_, cr.y) - 1;
        cv::circle(mat, cv_point, 5, 255, 1);
    }
    cv::drawContours(mat, contours, -1, cv::Scalar(255), 1);
    std::string addr1 = "/home/tony/course_ws/analysis/data/mat" + std::to_string(cnt) + ".jpg";
    //		std::string addr2 = "/home/tony/course_ws/analysis/data/mid" + std::to_string(cnt) + ".jpg";
    imwrite(addr1, mat);
    //		imwrite(addr2, mid);
    cnt++;
#endif
}

void FrontierSearch::check_current_frontier() {
    boost::mutex::scoped_lock lock(list_mutex_);
    for (auto iter = open_list_.begin(); iter != open_list_.end();) {
        if (!frontier_valid(*iter)) {
            ROS_WARN("Remove useless frontier [%.2f, %.2f, %.2f]", iter->x, iter->y, iter->yaw);
            //				close_list_.emplace_back(*iter);
            iter = open_list_.erase(iter);
        } else {
            frontier_cost(static_map_, *iter);
            ++iter;
        }
    }
}

void FrontierSearch::map_cb(nav_msgs::OccupancyGridConstPtr const& msg) {
    if (msg->info.width < INFLATION_SIZE || msg->info.height < INFLATION_SIZE) {
        ROS_ERROR("invalid map!");
        return;
    }

    boost::mutex::scoped_lock lock(map_mutex_);
    map_ = *msg;
    map_update_ = true;
}

auto FrontierSearch::map_to_img(nav_msgs::OccupancyGrid const& map) -> cv::Mat {
    auto sizex = static_cast<int>(map.info.width);
    auto sizey = static_cast<int>(map.info.height);
    cv::Mat mat(sizey, sizex, CV_8U);
    cv::Mat mat_bk(sizey, sizex, CV_8U);
    for (auto r = 0; r < sizey; ++r) {
        for (auto c = 0; c < sizex; ++c) {
            if (map.data.at(c + (sizey - r - 1) * sizex) == to_under_type(NAV_MAP_COST::OBS)) {
                mat.at<uchar>(r, c) = to_under_type(MAT_MAP_COST::OBS);
            } else if (map.data.at(c + (sizey - r - 1) * sizex) == to_under_type(NAV_MAP_COST::UNKNOWN)) {
                mat.at<uchar>(r, c) = to_under_type(MAT_MAP_COST::UNKNOWN);
            } else {
                mat.at<uchar>(r, c) = to_under_type(MAT_MAP_COST::FREE);
            }

            mat_bk.at<uchar>(r, c) = mat.at<uchar>(r, c);
        }
    }

    for (auto r = INFLATION_SIZE; r < sizey - INFLATION_SIZE; ++r) {
        for (auto c = INFLATION_SIZE; c < sizex - INFLATION_SIZE; ++c) {
            if (mat_bk.at<uchar>(r, c) != to_under_type(MAT_MAP_COST::OBS)) continue;

            for (auto i = -INFLATION_SIZE; i <= INFLATION_SIZE; ++i) {
                for (auto j = -INFLATION_SIZE; j <= INFLATION_SIZE; ++j) {
                    if (i == 0 && j == 0) continue;

                    mat.at<uchar>(r + i, c + j) = to_under_type(MAT_MAP_COST::OBS);
                }
            }
        }
    }

    return mat;
}

auto FrontierSearch::frontier_valid(Frontier const& point, bool opt) -> bool {
    auto obs_cnt = 0;
    auto unknow_cnt = 0;
    auto grid_x = wxgx(static_map_, point.x);
    auto grid_y = wygy(static_map_, point.y);
    if (!opt && static_map_.data.at(map_index(static_map_, grid_x, grid_y)) != to_under_type(NAV_MAP_COST::FREE)) {
        return false;
    }

    for (auto i = -POINT_SEARCH_RANGE; i <= POINT_SEARCH_RANGE; ++i) {
        for (auto j = -POINT_SEARCH_RANGE; j <= POINT_SEARCH_RANGE; ++j) {
            auto ii = grid_x + i;
            auto jj = grid_y + j;
            if (!map_valid(static_map_, ii, jj)) continue;
            if (static_map_.data.at(map_index(static_map_, ii, jj)) == to_under_type(NAV_MAP_COST::UNKNOWN))
                ++unknow_cnt;
            if (static_map_.data.at(map_index(static_map_, ii, jj)) == to_under_type(NAV_MAP_COST::OBS)) ++obs_cnt;
        }
    }

    return (unknow_cnt > POINT_SEARCH_RANGE && obs_cnt < POINT_SEARCH_RANGE / 2);
}

auto FrontierSearch::frontier_optimize(Frontier& point) -> bool {
    auto x_old = point.x;
    auto y_old = point.y;
    auto grid_x = wxgx(static_map_, point.x);
    auto grid_y = wygy(static_map_, point.y);
    auto max = -1;
    for (auto i = -POINT_SEARCH_RANGE; i <= POINT_SEARCH_RANGE; ++i) {
        for (auto j = -POINT_SEARCH_RANGE; j <= POINT_SEARCH_RANGE; ++j) {
            if (i == 0 && j == 0) continue;
            auto ii = grid_x + i;
            auto jj = grid_y + j;
            if (!map_valid(static_map_, ii, jj)) continue;
            if (static_map_.data.at(map_index(static_map_, ii, jj)) != to_under_type(NAV_MAP_COST::FREE)) continue;
            if (max > std::abs(i) + std::abs(j)) continue;
            max = std::abs(i) + std::abs(j);
            point.x = gxwx(static_map_, ii);
            point.y = gywy(static_map_, jj);
            point.yaw = std::atan2(y_old - point.y, x_old - point.x);
        }
    }

    if (max == -1 && static_map_.data.at(map_index(static_map_, grid_x, grid_y)) != to_under_type(NAV_MAP_COST::FREE)) {
        return false;
    }

    frontier_cost(static_map_, point);
    return true;
}

void FrontierSearch::frontier_preprocess(cv::Point2f const& cv_point, bool check) {
    Frontier point{gxwx(static_map_, static_cast<int>(cv_point.x)),
                   gywy(static_map_, static_map_.info.height - static_cast<int>(cv_point.y) - 1), 0, 0};
    if (check && !frontier_valid(point, true)) {
        ROS_ERROR("Current found frontier point invalid, pass!");
        return;
    }

    if (!frontier_optimize(point)) {
        ROS_ERROR("Cannot optimize to a proper point, pass!");
        return;
    }

    set_frontier_point(point);
}

void FrontierSearch::pub_static_map() {
    map_update_ = false;
    while (!map_update_) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    {
        boost::mutex::scoped_lock lock(map_mutex_);
        static_map_ = map_;
    }

    for (auto& cost : static_map_.data) {
        if (cost > to_under_type(ORI_MAP_COST::OBS)) {
            cost = to_under_type(NAV_MAP_COST::OBS);
        } else if (cost == to_under_type(ORI_MAP_COST::UNKNOWN)) {
            cost = to_under_type(NAV_MAP_COST::UNKNOWN);
        } else {
            cost = to_under_type(NAV_MAP_COST::FREE);
        }
    }

    map_pub_.publish(static_map_);
}
}  // namespace xju::explore
