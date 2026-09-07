#include <plan_env/collision_detection.h>
#include <plan_env/sdf_map.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>
#include <limits>

using namespace fast_planner;

namespace cane_planner
{
    CollisionDetection::~CollisionDetection()
    {
    }

    void CollisionDetection::init(ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("Collision/margin", margin_, -1.0);//当机器人与障碍物之间的距离小于 margin_ 时，认为有碰撞风险。
        node_.param("Collision/SliceHeight", slice_height_, -1.0);//它指定了要在哪个高度平面上进行碰撞检测。
        node_.param("Collision/use_static_global_map", use_static_global_map_, false);
        node_.param("Collision/use_static_global_esdf", use_static_global_esdf_, false);
        node_.param<std::string>("Collision/static_map_pcd", static_map_pcd_, "");
        node_.param("Collision/static_map_min_height", static_map_min_height_, 0.6);
        node_.param("Collision/static_map_max_height", static_map_max_height_, 2.0);
        node_.param("Collision/static_map_inflation", static_map_inflation_, 0.25);
        node_.param("Collision/static_map_min_points_per_cell", static_map_min_points_per_cell_, 2);
        node_.param("Collision/static_map_min_component_cells", static_map_min_component_cells_, 6);
        node_.param("Collision/static_require_known_region", static_require_known_region_, true);
        node_.param("Collision/static_known_use_enclosed_region", static_known_use_enclosed_region_, true);
        node_.param("Collision/static_known_min_height", static_known_min_height_, -0.25);
        node_.param("Collision/static_known_max_height", static_known_max_height_, 0.35);
        node_.param("Collision/static_known_inflation", static_known_inflation_, 0.55);
        node_.param("Collision/static_known_boundary_inflation", static_known_boundary_inflation_, 0.35);
        node_.param("Collision/global_esdf_min_height", global_esdf_min_height_, 0.2);
        node_.param("Collision/global_esdf_max_height", global_esdf_max_height_, 2.0);
        node_.param("Collision/global_esdf_query_min_height", global_esdf_query_min_height_, 0.2);
        node_.param("Collision/global_esdf_query_max_height", global_esdf_query_max_height_, 1.8);
        node_.param("Collision/global_esdf_query_step", global_esdf_query_step_, 0.3);
        node_.param("Collision/global_esdf_inflation", global_esdf_inflation_, 0.2);
        node_.param("Collision/global_esdf_safe_distance", global_esdf_safe_distance_, 0.15);
        node_.param("Collision/global_esdf_min_points_per_cell", global_esdf_min_points_per_cell_, 1);
        node_.param("Collision/global_esdf_visualization_height", global_esdf_visualization_height_, 0.2);
        node_.param("Collision/global_esdf_visualization_stride", global_esdf_visualization_stride_, 2);
        node_.param("Collision/publish_static_map_debug", publish_static_map_debug_, true);
        node_.param<std::string>("Collision/static_map_frame", static_map_frame_, "world");
        if (use_static_global_map_ && publish_static_map_debug_)
        {
            static_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/static_global_map", 1, true);
            static_map_occupied_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/static_global_occupied", 1, true);
            static_map_inflated_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/static_global_inflated", 1, true);
            static_map_known_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/static_global_known", 1, true);
            static_map_distance_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/static_global_distance", 1, true);
        }
        if (use_static_global_esdf_ && publish_static_map_debug_)
        {
            global_esdf_occupied_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/global_static_occupied", 1, true);
            global_esdf_slice_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/global_static_esdf_slice", 1, true);
        }
        
        cout << "Collision Detection[18]:margin:" << margin_ << endl;
        cout << "Collision Detection[18]:SliceHeight:" << slice_height_ << endl;
    }
    void CollisionDetection::setMap(shared_ptr<SDFMap> &map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
        static_map_resolution_ = sdf_map_->getResolution();
        static_map_inv_resolution_ = 1.0 / static_map_resolution_;
        if (use_static_global_esdf_)
        {
            loadStaticGlobalESDF();
        }
        else if (use_static_global_map_)
        {
            loadStaticGlobalMap();
        }
    }

    int CollisionDetection::staticToAddress(const Eigen::Vector2i &id) const
    {
        return id(0) * static_size_(1) + id(1);
    }

    bool CollisionDetection::isInStaticMap(const Eigen::Vector2i &id) const
    {
        return id(0) >= 0 && id(1) >= 0 && id(0) < static_size_(0) && id(1) < static_size_(1);
    }

    Eigen::Vector2i CollisionDetection::staticPosToIndex(const Eigen::Vector2d &pos) const
    {
        Eigen::Vector2i id;
        id(0) = floor((pos(0) - static_origin_(0)) * static_map_inv_resolution_);
        id(1) = floor((pos(1) - static_origin_(1)) * static_map_inv_resolution_);
        return id;
    }

    int CollisionDetection::globalToAddress(const Eigen::Vector3i &id) const
    {
        return id(0) * global_esdf_size_(1) * global_esdf_size_(2) + id(1) * global_esdf_size_(2) + id(2);
    }

    bool CollisionDetection::isInGlobalESDF(const Eigen::Vector3i &id) const
    {
        return id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
               id(0) < global_esdf_size_(0) && id(1) < global_esdf_size_(1) && id(2) < global_esdf_size_(2);
    }

    Eigen::Vector3i CollisionDetection::globalPosToIndex(const Eigen::Vector3d &pos) const
    {
        Eigen::Vector3i id;
        id(0) = floor((pos(0) - global_esdf_origin_(0)) * global_esdf_inv_resolution_);
        id(1) = floor((pos(1) - global_esdf_origin_(1)) * global_esdf_inv_resolution_);
        id(2) = floor((pos(2) - global_esdf_origin_(2)) * global_esdf_inv_resolution_);
        return id;
    }

    void CollisionDetection::loadStaticGlobalESDF()
    {
        static_global_esdf_ready_ = false;
        if (static_map_pcd_.empty())
        {
            ROS_WARN("[Collision] Static global ESDF requested but Collision/static_map_pcd is empty.");
            return;
        }

        Eigen::Vector3d map_origin_3d, map_size_3d;
        sdf_map_->getRegion(map_origin_3d, map_size_3d);
        global_esdf_resolution_ = sdf_map_->getResolution();
        global_esdf_inv_resolution_ = 1.0 / global_esdf_resolution_;
        global_esdf_origin_ = map_origin_3d;
        for (int i = 0; i < 3; ++i)
            global_esdf_size_(i) = std::max(1, static_cast<int>(std::ceil(map_size_3d(i) * global_esdf_inv_resolution_)));

        const int buffer_size = global_esdf_size_(0) * global_esdf_size_(1) * global_esdf_size_(2);
        global_esdf_occupied_.assign(buffer_size, 0);
        global_esdf_inflated_.assign(buffer_size, 0);
        global_esdf_distance_.assign(buffer_size, std::numeric_limits<float>::infinity());
        std::vector<uint16_t> hit_count(buffer_size, 0);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(static_map_pcd_, cloud) < 0)
        {
            ROS_ERROR("[Collision] Failed to load static global ESDF map: %s", static_map_pcd_.c_str());
            return;
        }

        int kept_points = 0;
        int occupied_cells = 0;
        for (const auto &pt : cloud.points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            if (pt.z < global_esdf_min_height_ || pt.z > global_esdf_max_height_)
                continue;
            Eigen::Vector3i id = globalPosToIndex(Eigen::Vector3d(pt.x, pt.y, pt.z));
            if (!isInGlobalESDF(id))
                continue;
            const int adr = globalToAddress(id);
            if (hit_count[adr] < std::numeric_limits<uint16_t>::max())
                hit_count[adr]++;
            kept_points++;
        }

        for (int adr = 0; adr < buffer_size; ++adr)
        {
            if (hit_count[adr] >= global_esdf_min_points_per_cell_)
            {
                global_esdf_occupied_[adr] = 1;
                occupied_cells++;
            }
        }

        inflateGlobalESDFOccupancy();
        computeGlobalESDF();
        static_global_esdf_ready_ = true;
        if (occupied_cells < 50)
        {
            ROS_WARN("[Collision] Static global ESDF has very few occupied cells (%d). A* may ignore the map; check height filters or global_esdf_min_points_per_cell.",
                     occupied_cells);
        }
        ROS_WARN("[Collision] Loaded static global ESDF %s: points=%zu, used_points=%d, occ_cells=%d, grid=%dx%dx%d, z=[%.2f, %.2f], inflation=%.2fm, safe=%.2fm, min_hits=%d",
                 static_map_pcd_.c_str(), cloud.size(), kept_points, occupied_cells,
                 global_esdf_size_(0), global_esdf_size_(1), global_esdf_size_(2),
                 global_esdf_min_height_, global_esdf_max_height_, global_esdf_inflation_,
                 global_esdf_safe_distance_, global_esdf_min_points_per_cell_);
        publishStaticGlobalESDF();
    }

    void CollisionDetection::inflateGlobalESDFOccupancy()
    {
        const int step = std::max(0, static_cast<int>(std::ceil(global_esdf_inflation_ * global_esdf_inv_resolution_)));
        for (int ix = 0; ix < global_esdf_size_(0); ++ix)
        {
            for (int iy = 0; iy < global_esdf_size_(1); ++iy)
            {
                for (int iz = 0; iz < global_esdf_size_(2); ++iz)
                {
                    Eigen::Vector3i occ_id(ix, iy, iz);
                    if (!global_esdf_occupied_[globalToAddress(occ_id)])
                        continue;
                    for (int dx = -step; dx <= step; ++dx)
                    {
                        for (int dy = -step; dy <= step; ++dy)
                        {
                            for (int dz = -step; dz <= step; ++dz)
                            {
                                if (std::sqrt(dx * dx + dy * dy + dz * dz) * global_esdf_resolution_ >
                                    global_esdf_inflation_ + 1e-6)
                                    continue;
                                Eigen::Vector3i id(ix + dx, iy + dy, iz + dz);
                                if (!isInGlobalESDF(id))
                                    continue;
                                global_esdf_inflated_[globalToAddress(id)] = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    void CollisionDetection::computeGlobalESDF()
    {
        const int nx = global_esdf_size_(0);
        const int ny = global_esdf_size_(1);
        const int nz = global_esdf_size_(2);
        const float inf = 1e20f;
        std::vector<float> tmp1(global_esdf_distance_.size(), inf);
        std::vector<float> tmp2(global_esdf_distance_.size(), inf);

        auto edt_1d = [](int n, const auto &get_val, const auto &set_val)
        {
            std::vector<int> v(n);
            std::vector<float> z(n + 1);
            int k = 0;
            v[0] = 0;
            z[0] = -std::numeric_limits<float>::infinity();
            z[1] = std::numeric_limits<float>::infinity();
            for (int q = 1; q < n; ++q)
            {
                float s;
                do
                {
                    const int vk = v[k];
                    s = ((get_val(q) + q * q) - (get_val(vk) + vk * vk)) / (2.0f * q - 2.0f * vk);
                    if (s <= z[k])
                        --k;
                } while (k >= 0 && s <= z[k]);
                ++k;
                v[k] = q;
                z[k] = s;
                z[k + 1] = std::numeric_limits<float>::infinity();
            }
            k = 0;
            for (int q = 0; q < n; ++q)
            {
                while (z[k + 1] < q)
                    ++k;
                const float d = q - v[k];
                set_val(q, d * d + get_val(v[k]));
            }
        };

        for (int x = 0; x < nx; ++x)
            for (int y = 0; y < ny; ++y)
                edt_1d(nz,
                       [&](int z)
                       {
                           return global_esdf_inflated_[globalToAddress(Eigen::Vector3i(x, y, z))] ? 0.0f : inf;
                       },
                       [&](int z, float val)
                       {
                           tmp1[globalToAddress(Eigen::Vector3i(x, y, z))] = val;
                       });

        for (int x = 0; x < nx; ++x)
            for (int z = 0; z < nz; ++z)
                edt_1d(ny,
                       [&](int y)
                       {
                           return tmp1[globalToAddress(Eigen::Vector3i(x, y, z))];
                       },
                       [&](int y, float val)
                       {
                           tmp2[globalToAddress(Eigen::Vector3i(x, y, z))] = val;
                       });

        for (int y = 0; y < ny; ++y)
            for (int z = 0; z < nz; ++z)
                edt_1d(nx,
                       [&](int x)
                       {
                           return tmp2[globalToAddress(Eigen::Vector3i(x, y, z))];
                       },
                       [&](int x, float val)
                       {
                           global_esdf_distance_[globalToAddress(Eigen::Vector3i(x, y, z))] =
                               global_esdf_resolution_ * std::sqrt(val);
                       });
    }

    double CollisionDetection::getGlobalESDFDistance3D(const Eigen::Vector3d &pos) const
    {
        if (!static_global_esdf_ready_)
            return std::numeric_limits<double>::infinity();
        Eigen::Vector3i id = globalPosToIndex(pos);
        if (!isInGlobalESDF(id))
            return 0.0;
        return global_esdf_distance_[globalToAddress(id)];
    }

    void CollisionDetection::publishStaticGlobalESDF() const
    {
        if (!static_global_esdf_ready_ || !publish_static_map_debug_)
            return;

        auto make_cloud = [&](sensor_msgs::PointCloud2 &msg, int point_count)
        {
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = static_map_frame_;
            msg.height = 1;
            msg.width = point_count;
            msg.is_bigendian = false;
            msg.is_dense = true;
            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32);
            modifier.resize(point_count);
        };

        if (global_esdf_occupied_pub_)
        {
            int point_count = 0;
            for (const auto occ : global_esdf_inflated_)
                if (occ)
                    point_count++;
            sensor_msgs::PointCloud2 msg;
            make_cloud(msg, point_count);
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
            for (int ix = 0; ix < global_esdf_size_(0); ++ix)
                for (int iy = 0; iy < global_esdf_size_(1); ++iy)
                    for (int iz = 0; iz < global_esdf_size_(2); ++iz)
                    {
                        Eigen::Vector3i id(ix, iy, iz);
                        if (!global_esdf_inflated_[globalToAddress(id)])
                            continue;
                        *iter_x = global_esdf_origin_(0) + (ix + 0.5) * global_esdf_resolution_;
                        *iter_y = global_esdf_origin_(1) + (iy + 0.5) * global_esdf_resolution_;
                        *iter_z = global_esdf_origin_(2) + (iz + 0.5) * global_esdf_resolution_;
                        *iter_i = 100.0f;
                        ++iter_x; ++iter_y; ++iter_z; ++iter_i;
                    }
            global_esdf_occupied_pub_.publish(msg);
        }

        if (global_esdf_slice_pub_)
        {
            const int z_id = globalPosToIndex(Eigen::Vector3d(0, 0, global_esdf_visualization_height_))(2);
            if (z_id < 0 || z_id >= global_esdf_size_(2))
                return;
            const int stride = std::max(1, global_esdf_visualization_stride_);
            const int point_count = ((global_esdf_size_(0) + stride - 1) / stride) *
                                    ((global_esdf_size_(1) + stride - 1) / stride);
            sensor_msgs::PointCloud2 msg;
            make_cloud(msg, point_count);
            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
            for (int ix = 0; ix < global_esdf_size_(0); ix += stride)
                for (int iy = 0; iy < global_esdf_size_(1); iy += stride)
                {
                    Eigen::Vector3i id(ix, iy, z_id);
                    *iter_x = global_esdf_origin_(0) + (ix + 0.5) * global_esdf_resolution_;
                    *iter_y = global_esdf_origin_(1) + (iy + 0.5) * global_esdf_resolution_;
                    *iter_z = global_esdf_visualization_height_;
                    *iter_i = std::min(5.0f, global_esdf_distance_[globalToAddress(id)]);
                    ++iter_x; ++iter_y; ++iter_z; ++iter_i;
                }
            global_esdf_slice_pub_.publish(msg);
        }
    }

    void CollisionDetection::inflateStaticMask(const std::vector<uint8_t> &src, std::vector<uint8_t> &dst, double radius) const
    {
        dst.assign(src.size(), 0);
        const int inflate_step = std::max(0, static_cast<int>(std::ceil(radius * static_map_inv_resolution_)));
        for (int ix = 0; ix < static_size_(0); ++ix)
        {
            for (int iy = 0; iy < static_size_(1); ++iy)
            {
                Eigen::Vector2i src_id(ix, iy);
                if (!src[staticToAddress(src_id)])
                    continue;
                for (int dx = -inflate_step; dx <= inflate_step; ++dx)
                {
                    for (int dy = -inflate_step; dy <= inflate_step; ++dy)
                    {
                        Eigen::Vector2i dst_id(ix + dx, iy + dy);
                        if (!isInStaticMap(dst_id))
                            continue;
                        if (std::hypot(dx, dy) * static_map_resolution_ <= radius + 1e-6)
                            dst[staticToAddress(dst_id)] = 1;
                    }
                }
            }
        }
    }

    void CollisionDetection::removeSmallStaticComponents()
    {
        if (static_map_min_component_cells_ <= 1)
            return;

        std::vector<uint8_t> visited(static_occupied_.size(), 0);
        std::vector<int> component;
        std::queue<Eigen::Vector2i> q;
        static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int ix = 0; ix < static_size_(0); ++ix)
        {
            for (int iy = 0; iy < static_size_(1); ++iy)
            {
                Eigen::Vector2i seed(ix, iy);
                int seed_adr = staticToAddress(seed);
                if (!static_occupied_[seed_adr] || visited[seed_adr])
                    continue;

                component.clear();
                visited[seed_adr] = 1;
                q.push(seed);
                while (!q.empty())
                {
                    Eigen::Vector2i cur = q.front();
                    q.pop();
                    component.push_back(staticToAddress(cur));
                    for (int k = 0; k < 8; ++k)
                    {
                        Eigen::Vector2i nid(cur(0) + dx[k], cur(1) + dy[k]);
                        if (!isInStaticMap(nid))
                            continue;
                        int n_adr = staticToAddress(nid);
                        if (!static_occupied_[n_adr] || visited[n_adr])
                            continue;
                        visited[n_adr] = 1;
                        q.push(nid);
                    }
                }

                if (static_cast<int>(component.size()) < static_map_min_component_cells_)
                {
                    for (const int adr : component)
                        static_occupied_[adr] = 0;
                }
            }
        }
    }

    void CollisionDetection::computeKnownRegionFromBoundary()
    {
        std::vector<uint8_t> boundary;
        inflateStaticMask(static_occupied_, boundary, static_known_boundary_inflation_);

        std::vector<uint8_t> outside(static_occupied_.size(), 0);
        std::queue<Eigen::Vector2i> q;
        auto try_push = [&](const Eigen::Vector2i &id)
        {
            if (!isInStaticMap(id))
                return;
            const int adr = staticToAddress(id);
            if (outside[adr] || boundary[adr])
                return;
            outside[adr] = 1;
            q.push(id);
        };

        for (int ix = 0; ix < static_size_(0); ++ix)
        {
            try_push(Eigen::Vector2i(ix, 0));
            try_push(Eigen::Vector2i(ix, static_size_(1) - 1));
        }
        for (int iy = 0; iy < static_size_(1); ++iy)
        {
            try_push(Eigen::Vector2i(0, iy));
            try_push(Eigen::Vector2i(static_size_(0) - 1, iy));
        }

        static const int dx[4] = {1, -1, 0, 0};
        static const int dy[4] = {0, 0, 1, -1};
        while (!q.empty())
        {
            Eigen::Vector2i cur = q.front();
            q.pop();
            for (int k = 0; k < 4; ++k)
                try_push(Eigen::Vector2i(cur(0) + dx[k], cur(1) + dy[k]));
        }

        for (int i = 0; i < static_cast<int>(static_known_inflated_.size()); ++i)
        {
            if (!outside[i])
                static_known_inflated_[i] = 1;
        }
    }

    void CollisionDetection::loadStaticGlobalMap()
    {
        static_global_map_ready_ = false;
        if (static_map_pcd_.empty())
        {
            ROS_WARN("[Collision] Static global map requested but Collision/static_map_pcd is empty.");
            return;
        }

        Eigen::Vector3d map_origin_3d, map_size_3d;
        sdf_map_->getRegion(map_origin_3d, map_size_3d);
        static_origin_ << map_origin_3d(0), map_origin_3d(1);
        static_size_(0) = ceil(map_size_3d(0) * static_map_inv_resolution_);
        static_size_(1) = ceil(map_size_3d(1) * static_map_inv_resolution_);
        const int buffer_size = static_size_(0) * static_size_(1);
        static_occupied_.assign(buffer_size, 0);
        static_inflated_.assign(buffer_size, 0);
        static_known_.assign(buffer_size, 0);
        static_known_inflated_.assign(buffer_size, 0);
        static_distance_.assign(buffer_size, std::numeric_limits<float>::infinity());
        std::vector<uint16_t> hit_count(buffer_size, 0);
        std::vector<uint16_t> known_hit_count(buffer_size, 0);

        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(static_map_pcd_, cloud) < 0)
        {
            ROS_ERROR("[Collision] Failed to load static global map: %s", static_map_pcd_.c_str());
            return;
        }

        int kept_points = 0;
        int known_points = 0;
        for (const auto &pt : cloud.points)
        {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;
            Eigen::Vector2i id = staticPosToIndex(Eigen::Vector2d(pt.x, pt.y));
            if (!isInStaticMap(id))
                continue;
            const int adr = staticToAddress(id);
            if (pt.z >= static_known_min_height_ && pt.z <= static_known_max_height_)
            {
                if (known_hit_count[adr] < std::numeric_limits<uint16_t>::max())
                    known_hit_count[adr]++;
                known_points++;
            }
            if (pt.z >= static_map_min_height_ && pt.z <= static_map_max_height_)
            {
                if (hit_count[adr] < std::numeric_limits<uint16_t>::max())
                    hit_count[adr]++;
                kept_points++;
            }
        }

        int known_cells = 0;
        for (int i = 0; i < buffer_size; ++i)
        {
            if (known_hit_count[i] > 0)
            {
                static_known_[i] = 1;
                known_cells++;
            }
        }

        int occupied_cells = 0;
        for (int i = 0; i < buffer_size; ++i)
        {
            if (hit_count[i] >= static_map_min_points_per_cell_)
            {
                static_occupied_[i] = 1;
                occupied_cells++;
            }
        }

        removeSmallStaticComponents();
        occupied_cells = 0;
        for (const auto occupied : static_occupied_)
        {
            if (occupied)
                occupied_cells++;
        }
        inflateStaticMask(static_occupied_, static_inflated_, static_map_inflation_);
        inflateStaticMask(static_known_, static_known_inflated_, static_known_inflation_);
        if (static_known_use_enclosed_region_)
            computeKnownRegionFromBoundary();

        computeStaticDistanceField();
        static_global_map_ready_ = true;
        ROS_WARN("[Collision] Loaded projected 2D static ESDF %s: points=%zu, obs_points=%d, obs_cells=%d, known_points=%d, known_cells=%d, grid=%dx%d, obs_z=[%.2f, %.2f], known_z=[%.2f, %.2f], obs_inflation=%.2fm, known_required=%d, known_inflation=%.2fm, boundary_inflation=%.2fm, enclosed_known=%d, min_hits=%d, min_component=%d",
                 static_map_pcd_.c_str(), cloud.size(), kept_points, occupied_cells, known_points, known_cells,
                 static_size_(0), static_size_(1), static_map_min_height_, static_map_max_height_,
                 static_known_min_height_, static_known_max_height_, static_map_inflation_, static_require_known_region_,
                 static_known_inflation_, static_known_boundary_inflation_, static_known_use_enclosed_region_,
                 static_map_min_points_per_cell_, static_map_min_component_cells_);
        publishStaticGlobalMap();
    }

    void CollisionDetection::publishStaticGlobalMap() const
    {
        if (!static_global_map_ready_ || !publish_static_map_debug_ || !static_map_pub_)
            return;

        auto publish_cloud = [&](const ros::Publisher &pub, const std::vector<uint8_t> &mask, double z,
                                 float occupied_intensity, bool inflated_only)
        {
            if (!pub)
                return;

            int point_count = 0;
            for (int i = 0; i < static_cast<int>(mask.size()); ++i)
            {
                if (!mask[i])
                    continue;
                if (inflated_only && static_occupied_[i])
                    continue;
                point_count++;
            }

            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = static_map_frame_;
            msg.height = 1;
            msg.width = point_count;
            msg.is_bigendian = false;
            msg.is_dense = true;

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32);
            modifier.resize(point_count);

            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");

            for (int ix = 0; ix < static_size_(0); ++ix)
            {
                for (int iy = 0; iy < static_size_(1); ++iy)
                {
                    Eigen::Vector2i id(ix, iy);
                    const int adr = staticToAddress(id);
                    if (!mask[adr])
                        continue;
                    if (inflated_only && static_occupied_[adr])
                        continue;

                    *iter_x = static_cast<float>(static_origin_(0) + (ix + 0.5) * static_map_resolution_);
                    *iter_y = static_cast<float>(static_origin_(1) + (iy + 0.5) * static_map_resolution_);
                    *iter_z = static_cast<float>(z);
                    *iter_i = static_occupied_[adr] ? occupied_intensity : 30.0f;
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                    ++iter_i;
                }
            }

            pub.publish(msg);
        };

        publish_cloud(static_map_pub_, static_inflated_, 0.08, 100.0f, false);
        publish_cloud(static_map_known_pub_, static_known_inflated_, 0.04, 20.0f, false);
        publish_cloud(static_map_occupied_pub_, static_occupied_, 0.10, 100.0f, false);
        publish_cloud(static_map_inflated_pub_, static_inflated_, 0.12, 30.0f, true);

        if (static_map_distance_pub_)
        {
            const int stride = 2;
            const float max_visual_dist = 3.0f;
            const int nx = (static_size_(0) + stride - 1) / stride;
            const int ny = (static_size_(1) + stride - 1) / stride;

            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = static_map_frame_;
            msg.height = 1;
            msg.width = nx * ny;
            msg.is_bigendian = false;
            msg.is_dense = true;

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2Fields(
                4,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::PointField::FLOAT32);
            modifier.resize(msg.width);

            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");

            for (int ix = 0; ix < static_size_(0); ix += stride)
            {
                for (int iy = 0; iy < static_size_(1); iy += stride)
                {
                    Eigen::Vector2i id(ix, iy);
                    const int adr = staticToAddress(id);
                    const float dist = std::isfinite(static_distance_[adr])
                                           ? std::min(static_distance_[adr], max_visual_dist)
                                           : max_visual_dist;

                    *iter_x = static_cast<float>(static_origin_(0) + (ix + 0.5) * static_map_resolution_);
                    *iter_y = static_cast<float>(static_origin_(1) + (iy + 0.5) * static_map_resolution_);
                    *iter_z = 0.02f;
                    *iter_i = dist;
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                    ++iter_i;
                }
            }

            static_map_distance_pub_.publish(msg);
        }
    }

    void CollisionDetection::computeStaticDistanceField()
    {
        struct QueueNode
        {
            float dist;
            Eigen::Vector2i id;
            bool operator<(const QueueNode &other) const { return dist > other.dist; }
        };

        std::priority_queue<QueueNode> queue;
        for (int ix = 0; ix < static_size_(0); ++ix)
        {
            for (int iy = 0; iy < static_size_(1); ++iy)
            {
                Eigen::Vector2i id(ix, iy);
                const int adr = staticToAddress(id);
                if (static_occupied_[adr])
                {
                    static_distance_[adr] = 0.0f;
                    queue.push({0.0f, id});
                }
            }
        }

        static const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        static const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        while (!queue.empty())
        {
            QueueNode cur = queue.top();
            queue.pop();
            const int cur_adr = staticToAddress(cur.id);
            if (cur.dist > static_distance_[cur_adr] + 1e-6f)
                continue;

            for (int k = 0; k < 8; ++k)
            {
                Eigen::Vector2i nid(cur.id(0) + dx[k], cur.id(1) + dy[k]);
                if (!isInStaticMap(nid))
                    continue;
                const float step = static_map_resolution_ * ((k < 4) ? 1.0f : 1.41421356f);
                const float nd = cur.dist + step;
                const int n_adr = staticToAddress(nid);
                if (nd < static_distance_[n_adr])
                {
                    static_distance_[n_adr] = nd;
                    queue.push({nd, nid});
                }
            }
        }
    }

    // 修改后的 3D 碰撞检测函数
bool CollisionDetection::isTraversable(double x, double y) {
    const double distance = getCollisionDistance(Eigen::Vector2d(x, y));
    return distance > 0.0 && distance >= margin_;
}

    bool CollisionDetection::isTraversable(Eigen::Vector3d pos)
    {
        double dis = sdf_map_->getDistance(pos);
        if (dis <= margin_)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    double CollisionDetection::getCollisionDistance(Eigen::Vector2d pos)
    {
        const double foot_z = slice_height_ - 0.8;
        const double head_z = slice_height_ + 0.8;
        const double check_step = 0.3;
        double min_dist = std::numeric_limits<double>::infinity();
        for (double z = foot_z; z <= head_z + 1e-6; z += check_step)
        {
            min_dist = std::min(min_dist,
                                sdf_map_->getDistance(Eigen::Vector3d(pos(0), pos(1), z)));
        }
        min_dist = std::min(min_dist,
                            sdf_map_->getDistance(Eigen::Vector3d(pos(0), pos(1), head_z)));
        return min_dist;
    }

    bool CollisionDetection::getStaticCorridorGrid(
        const Eigen::Vector2d& lower, const Eigen::Vector2d& upper,
        Eigen::Vector2d& origin, double& resolution, Eigen::Vector2i& size,
        std::vector<uint8_t>& blocked, std::string& reason)
    {
        blocked.clear();
        reason = "INVALID_GRID_REQUEST";
        if (!lower.allFinite() || !upper.allFinite() ||
            (upper.array() <= lower.array()).any() ||
            lower.cwiseAbs().maxCoeff() > 10000. || upper.cwiseAbs().maxCoeff() > 10000.) return false;
        Eigen::Vector2d base, extent;
        if (static_global_esdf_ready_) {
            resolution = global_esdf_resolution_;
            base = global_esdf_origin_.head<2>();
            extent = resolution * global_esdf_size_.head<2>().cast<double>();
            if ((global_esdf_size_.array() <= 0).any() ||
                global_esdf_distance_.size() != static_cast<size_t>(global_esdf_size_.x()) * global_esdf_size_.y() * global_esdf_size_.z()) {
                reason = "INVALID_ESDF_DATA"; return false;
            }
        } else if (static_global_map_ready_) {
            resolution = static_map_resolution_;
            base = static_origin_;
            extent = resolution * static_size_.cast<double>();
            if ((static_size_.array() <= 0).any() ||
                static_inflated_.size() != static_cast<size_t>(static_size_.x()) * static_size_.y() ||
                (static_require_known_region_ && static_known_inflated_.size() != static_inflated_.size())) {
                reason = "INVALID_STATIC_DATA"; return false;
            }
        } else {
            if (!sdf_map_) { reason = "MAP_UNAVAILABLE"; return false; }
            Eigen::Vector3d o, e; sdf_map_->getRegion(o, e);
            base = o.head<2>(); extent = e.head<2>(); resolution = sdf_map_->getResolution();
        }
        if (!base.allFinite() || !extent.allFinite() || !std::isfinite(resolution) || resolution < .01) return false;
        const Eigen::Vector2d lo = lower.cwiseMax(base), hi = upper.cwiseMin(base + extent);
        if ((hi.array() <= lo.array()).any()) { reason = "OUT_OF_MAP"; return false; }
        const Eigen::Vector2i first = ((lo-base)/resolution).array().floor().cast<int>();
        const Eigen::Vector2i last = ((hi-base)/resolution).array().ceil().cast<int>();
        size = last-first; origin = base+resolution*first.cast<double>();
        if ((size.array() <= 0).any() || resolution*size.maxCoeff()>100. ||
            static_cast<int64_t>(size.x())*size.y()>1000000) { reason = "GRID_BUDGET"; return false; }
        blocked.assign(static_cast<size_t>(size.x())*size.y(), 1);
        for (int y=0; y<size.y(); ++y) for (int x=0; x<size.x(); ++x) {
            const Eigen::Vector2d p = origin+resolution*Eigen::Vector2d(x+.5,y+.5);
            // All selected queries use floor-indexed cells, not interpolation.
            // A center identifies an entire aligned XY bin; blocked bins become
            // full convex obstacles. Preserve each backend's margins, vertical
            // samples and unknown/model classification exactly.
            const bool free = (static_global_esdf_ready_ || static_global_map_ready_)
                ? isStaticTraversable(p.x(), p.y()) : isTraversable(p.x(), p.y());
            blocked[y*size.x()+x] = free ? 0 : 1;
        }
        reason = "OK"; return true;
    }

    bool CollisionDetection::isStaticTraversable(double x, double y) const
    {
        if (static_global_esdf_ready_)
        {
            double min_dist = std::numeric_limits<double>::infinity();
            const double step = std::max(global_esdf_resolution_, global_esdf_query_step_);
            for (double z = global_esdf_query_min_height_; z <= global_esdf_query_max_height_ + 1e-6; z += step)
            {
                Eigen::Vector3d pos(x, y, z);
                Eigen::Vector3i id = globalPosToIndex(pos);
                if (!isInGlobalESDF(id))
                    return false;
                min_dist = std::min(min_dist, static_cast<double>(global_esdf_distance_[globalToAddress(id)]));
            }
            return min_dist > 0.0 && min_dist >= global_esdf_safe_distance_;
        }
        if (!static_global_map_ready_)
            return true;
        Eigen::Vector2i id = staticPosToIndex(Eigen::Vector2d(x, y));
        if (!isInStaticMap(id))
            return false;
        const int adr = staticToAddress(id);
        if (static_require_known_region_ && !static_known_inflated_[adr])
            return false;
        return static_inflated_[adr] == 0;
    }

    double CollisionDetection::getStaticCollisionDistance(Eigen::Vector2d pos)
    {
        if (static_global_esdf_ready_)
        {
            double min_dist = std::numeric_limits<double>::infinity();
            const double step = std::max(global_esdf_resolution_, global_esdf_query_step_);
            for (double z = global_esdf_query_min_height_; z <= global_esdf_query_max_height_ + 1e-6; z += step)
            {
                min_dist = std::min(min_dist, getGlobalESDFDistance3D(Eigen::Vector3d(pos(0), pos(1), z)));
            }
            return min_dist;
        }
        if (!static_global_map_ready_)
            return getCollisionDistance(pos);
        Eigen::Vector2i id = staticPosToIndex(pos);
        if (!isInStaticMap(id))
            return 0.0;
        return static_distance_[staticToAddress(id)];
    }

    int CollisionDetection::countStaticTraversableAround(const Eigen::Vector2d &pos, double radius) const
    {
        if (static_global_esdf_ready_)
        {
            const int step = std::max(1, static_cast<int>(std::ceil(radius * global_esdf_inv_resolution_)));
            int count = 0;
            for (int dx = -step; dx <= step; ++dx)
            {
                for (int dy = -step; dy <= step; ++dy)
                {
                    if (std::hypot(dx, dy) * global_esdf_resolution_ > radius)
                        continue;
                    const double x = pos(0) + dx * global_esdf_resolution_;
                    const double y = pos(1) + dy * global_esdf_resolution_;
                    if (isStaticTraversable(x, y))
                        count++;
                }
            }
            return count;
        }
        if (!static_global_map_ready_)
            return 0;
        Eigen::Vector2i center = staticPosToIndex(pos);
        const int step = std::max(1, static_cast<int>(std::ceil(radius * static_map_inv_resolution_)));
        int count = 0;
        for (int dx = -step; dx <= step; ++dx)
        {
            for (int dy = -step; dy <= step; ++dy)
            {
                Eigen::Vector2i id(center(0) + dx, center(1) + dy);
                if (!isInStaticMap(id))
                    continue;
                if (std::hypot(dx, dy) * static_map_resolution_ > radius)
                    continue;
                const int adr = staticToAddress(id);
                if (static_require_known_region_ && !static_known_inflated_[adr])
                    continue;
                if (static_inflated_[adr] == 0)
                    count++;
            }
        }
        return count;
    }

    void CollisionDetection::getSurroundDistance(Eigen::Vector2d pts[2][2][2], double dists[2][2][2])
    {
        Eigen::Vector3d pts_temp[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    pts_temp[x][y][z](0) = pts[x][y][z](0);
                    pts_temp[x][y][z](1) = pts[x][y][z](1);
                    // TODO:this should changle to slice_height_list_;
                    pts_temp[x][y][z](2) = 0.6;

                    dists[x][y][z] = sdf_map_->getDistance(pts_temp[x][y][z]);
                }
    }

} // namespace cane_planner
