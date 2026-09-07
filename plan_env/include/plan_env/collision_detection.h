#if !defined(_COLLISION_DETECTION_H_)
#define _COLLISION_DETECTION_H_

#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
#include <string>
#include <vector>

#include <plan_env/sdf_map.h>

using namespace fast_planner;
using std::shared_ptr;
using std::unique_ptr;
namespace cane_planner
{
  class CollisionDetection
  {
  private:
    friend class StaticMpcTestAccess;
    /* data */
    ros::NodeHandle node_;
    double resolution_inv_;
    double margin_;
    double slice_height_;
    Eigen::Vector3d slice_height_list_;
    bool use_static_global_map_ = false;
    bool static_global_map_ready_ = false;
    bool use_static_global_esdf_ = false;
    bool static_global_esdf_ready_ = false;
    std::string static_map_pcd_;
    double static_map_min_height_ = 0.6;
    double static_map_max_height_ = 2.0;
    double static_map_inflation_ = 0.25;
    int static_map_min_points_per_cell_ = 2;
    int static_map_min_component_cells_ = 6;
    bool static_require_known_region_ = true;
    bool static_known_use_enclosed_region_ = true;
    double static_known_min_height_ = -0.25;
    double static_known_max_height_ = 0.35;
    double static_known_inflation_ = 0.55;
    double static_known_boundary_inflation_ = 0.35;
    double static_map_resolution_ = 0.1;
    double static_map_inv_resolution_ = 10.0;
    bool publish_static_map_debug_ = true;
    std::string static_map_frame_ = "world";
    ros::Publisher static_map_pub_;
    ros::Publisher static_map_occupied_pub_;
    ros::Publisher static_map_inflated_pub_;
    ros::Publisher static_map_known_pub_;
    ros::Publisher static_map_distance_pub_;
    ros::Publisher global_esdf_occupied_pub_;
    ros::Publisher global_esdf_slice_pub_;
    Eigen::Vector2d static_origin_;
    Eigen::Vector2i static_size_;
    std::vector<uint8_t> static_occupied_;
    std::vector<uint8_t> static_inflated_;
    std::vector<uint8_t> static_known_;
    std::vector<uint8_t> static_known_inflated_;
    std::vector<float> static_distance_;

    double global_esdf_resolution_ = 0.1;
    double global_esdf_inv_resolution_ = 10.0;
    double global_esdf_min_height_ = 0.2;
    double global_esdf_max_height_ = 2.0;
    double global_esdf_query_min_height_ = 0.2;
    double global_esdf_query_max_height_ = 1.8;
    double global_esdf_query_step_ = 0.3;
    double global_esdf_inflation_ = 0.2;
    double global_esdf_safe_distance_ = 0.15;
    int global_esdf_min_points_per_cell_ = 1;
    double global_esdf_visualization_height_ = 0.2;
    int global_esdf_visualization_stride_ = 2;
    Eigen::Vector3d global_esdf_origin_;
    Eigen::Vector3i global_esdf_size_;
    std::vector<uint8_t> global_esdf_occupied_;
    std::vector<uint8_t> global_esdf_inflated_;
    std::vector<float> global_esdf_distance_;

    int staticToAddress(const Eigen::Vector2i &id) const;
    bool isInStaticMap(const Eigen::Vector2i &id) const;
    Eigen::Vector2i staticPosToIndex(const Eigen::Vector2d &pos) const;
    void inflateStaticMask(const std::vector<uint8_t> &src, std::vector<uint8_t> &dst, double radius) const;
    void removeSmallStaticComponents();
    void computeKnownRegionFromBoundary();
    void loadStaticGlobalMap();
    void computeStaticDistanceField();
    void publishStaticGlobalMap() const;
    int globalToAddress(const Eigen::Vector3i &id) const;
    bool isInGlobalESDF(const Eigen::Vector3i &id) const;
    Eigen::Vector3i globalPosToIndex(const Eigen::Vector3d &pos) const;
    void loadStaticGlobalESDF();
    void inflateGlobalESDFOccupancy();
    void computeGlobalESDF();
    void publishStaticGlobalESDF() const;
    double getGlobalESDFDistance3D(const Eigen::Vector3d &pos) const;
    

  public:
    
    shared_ptr<SDFMap> sdf_map_;
    CollisionDetection(){};
    ~CollisionDetection();

    void init(ros::NodeHandle &nh);
    void setMap(shared_ptr<SDFMap> &map);

    // main api
    /*!
        \brief evaluates whether the configuration is safe
        \return true if it is traversable, else false
    */
    bool isTraversable(double x, double y);
    bool isTraversable(Eigen::Vector3d pos);
    bool isTraversable(Eigen::Vector3d state, double times);
    double getCollisionDistance(Eigen::Vector2d pos);
    bool hasStaticGlobalMap() const { return static_global_esdf_ready_ || static_global_map_ready_; }
    // Matches the backend selected by isStaticTraversable, with local SDF fallback.
    double getStaticQueryResolution() const
    {
      if (static_global_esdf_ready_) return global_esdf_resolution_;
      if (static_global_map_ready_) return static_map_resolution_;
      return sdf_map_->getResolution();
    }
    // Conservative aligned 2D cell snapshot for static corridor inflation.
    // Preserves selected backend semantics; not an observed-free certificate.
    bool getStaticCorridorGrid(const Eigen::Vector2d& lower, const Eigen::Vector2d& upper,
        Eigen::Vector2d& origin, double& resolution, Eigen::Vector2i& size,
        std::vector<uint8_t>& blocked, std::string& reason);
    bool isStaticTraversable(double x, double y) const;
    double getStaticCollisionDistance(Eigen::Vector2d pos);
    int countStaticTraversableAround(const Eigen::Vector2d &pos, double radius) const;


    void getSurroundDistance(Eigen::Vector2d pts[2][2][2], double dists[2][2][2]);
    void getMapRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
    {
      sdf_map_->getRegion(ori, size);
    }

    double getSliceHeight() const { return slice_height_; }


    typedef shared_ptr<CollisionDetection> Ptr;
  };

} // namespace cane_planner

#endif // _COLLISION_DETECTION_H_
