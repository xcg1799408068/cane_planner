#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <onboard_detector/dynamicDetector.h>

using namespace std;

namespace cv {
class Mat;
}

class RayCaster;

namespace fast_planner {
struct MapParam;
struct MapData;
class MapROS;

class SDFMap {
public:
  SDFMap();
  ~SDFMap();

  enum OCCUPANCY { UNKNOWN, FREE, OCCUPIED };

  void setDetector(const std::shared_ptr<onboardDetector::dynamicDetector>& detector);
  void initMap(ros::NodeHandle& nh);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& camera_pos);
  void BuildsimulationMap(const pcl::PointCloud<pcl::PointXYZ>& points, const int& point_num,
                       const Eigen::Vector3d& camera_pos);      

  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  void boundIndex(Eigen::Vector3i& id);
  int toAddress(const Eigen::Vector3i& id);
  int toAddress(const int& x, const int& y, const int& z);
  bool isInMap(const Eigen::Vector3d& pos);
  bool isInMap(const Eigen::Vector3i& idx);
  bool isInBox(const Eigen::Vector3i& id);
  bool isInBox(const Eigen::Vector3d& pos);
  void boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up);
  int getOccupancy(const Eigen::Vector3d& pos);
  int getOccupancy(const Eigen::Vector3i& id);
  void setOccupied(const Eigen::Vector3d& pos, const int& occ = 1);
  int getInflateOccupancy(const Eigen::Vector3d& pos);
  int getInflateOccupancy(const Eigen::Vector3i& id);
  double getDistance(const Eigen::Vector3d& pos);
  double getDistance(const Eigen::Vector3i& id);
  double getDistWithGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  void updateESDF3d();
  void resetBuffer();
  void resetBuffer(const Eigen::Vector3d& min, const Eigen::Vector3d& max);

  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  void getBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax);
  void getUpdatedBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax, bool reset = false);
  double getResolution();
  int getVoxelNum();

  double getGroundHeight(); 

  void updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
  void freeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
  void freeHistoryRegions();
  bool isInFreeRegion(const Eigen::Vector3d& pos, const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion);
  bool isInFreeRegions(const Eigen::Vector3d& pos, const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions);
  bool isInHistoryFreeRegions(const Eigen::Vector3d& pos);
  void freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
  void setFree(const Eigen::Vector3i& idx);

  typedef std::shared_ptr<SDFMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend class SdfProducerTestAccess;
  void clearAndInflateLocalMap();
  void inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts);
  void setCacheOccupancy(const int& adr, const int& occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

  unique_ptr<MapParam> mp_;
  unique_ptr<MapData> md_;
  unique_ptr<MapROS> mr_;
  unique_ptr<RayCaster> caster_;
  std::shared_ptr<onboardDetector::dynamicDetector> detector_;
  friend MapROS;

  bool is_simulation_;  
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions_;
  std::deque<std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>> histFreeRegions_;
  bool useFreeRegions_ = false;
};

struct MapParam {
  // ROBOT SIZE
	Eigen::Vector3d robotSize_;
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_;  // logit
  double max_ray_length_;
  double max_build_length_;
  double input_min_height_;
  double input_max_height_;
  double local_bound_inflate_;
  
  int local_map_margin_;
  double unknown_flag_;

  Eigen::Vector3d local_update_range_;
};

struct MapData {
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  // data for updating
  vector<short> count_hit_, count_miss_, count_hit_and_miss_;
  vector<char> flag_rayend_, flag_visited_;
  char raycast_num_;
  queue<int> cache_voxel_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

  inline void SDFMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
  }

  inline void SDFMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
  }

  inline void SDFMap::boundIndex(Eigen::Vector3i& id) {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
    id = id1;
  }

  inline int SDFMap::toAddress(const int& x, const int& y, const int& z) {
    return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
  }

  inline int SDFMap::toAddress(const Eigen::Vector3i& id) {
    return toAddress(id[0], id[1], id[2]);
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3d& pos) {
    if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_->map_min_boundary_(2) + 1e-4)
      return false;
    if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_->map_max_boundary_(2) - 1e-4)
      return false;
    return true;
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3i& idx) {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) return false;
    if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
        idx(2) > mp_->map_voxel_num_(2) - 1)
      return false;
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i) {
      if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
        return false;
      }
    }
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i) {
      if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
        return false;
      }
    }
    return true;
  }

  inline void SDFMap::boundBox(Eigen::Vector3d& low, Eigen::Vector3d& up) {
    for (int i = 0; i < 3; ++i) {
      low[i] = max(low[i], mp_->box_mind_[i]);
      up[i] = min(up[i], mp_->box_maxd_[i]);
    }
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3i& id) {
    if (!isInMap(id)) return -1;
    double occ = md_->occupancy_buffer_[toAddress(id)];
    if (occ < mp_->clamp_min_log_ - 1e-3) return UNKNOWN;
    if (occ > mp_->min_occupancy_log_) return OCCUPIED;
    return FREE;
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id);
  }

  inline void SDFMap::setOccupied(const Eigen::Vector3d& pos, const int& occ) {
    if (!isInMap(pos)) return;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i& id) {
    if (!isInMap(id)) return -1;
    return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id);
  }

  inline double SDFMap::getDistance(const Eigen::Vector3i& id) {
    if (!isInMap(id)) return -1;
    return md_->distance_buffer_[toAddress(id)];
  }

  inline double SDFMap::getDistance(const Eigen::Vector3d& pos) {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getDistance(id);
  }

  inline void SDFMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts) {
    int num = 0;

    /* ---------- + shape inflate ---------- */
    // for (int x = -step; x <= step; ++x)
    // {
    //   if (x == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
    // }
    // for (int y = -step; y <= step; ++y)
    // {
    //   if (y == 0)
    //     continue;
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
    // }
    // for (int z = -1; z <= 1; ++z)
    // {
    //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
    // }

    /* ---------- all inflate ---------- */
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z) {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }

  inline void SDFMap::updateFreeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		this->freeRegions_ = freeRegions;
		if (freeRegions.empty()){
			this->histFreeRegions_.clear();
			this->useFreeRegions_ = false;
			return;
		}
		if (this->histFreeRegions_.size() < 5){
			this->histFreeRegions_.push_back(freeRegions);
		}
		else{
			this->histFreeRegions_.pop_front();
			this->histFreeRegions_.push_back(freeRegions);
		}


		if (this->histFreeRegions_.size() != 0){
			this->useFreeRegions_ = true;
		}
		else{
			this->useFreeRegions_ = false;
		}
	}

  	inline void SDFMap::freeRegions(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		for (std::pair<Eigen::Vector3d, Eigen::Vector3d> freeRegion : freeRegions){
			this->freeRegion(freeRegion.first, freeRegion.second);
		}
	}

	inline void SDFMap::freeHistoryRegions(){
		if (!this->useFreeRegions_) return;
		for (const auto& freeRegions : this->histFreeRegions_){
			this->freeRegions(freeRegions);
		}
	}

	inline bool SDFMap::isInFreeRegion(
			const Eigen::Vector3d& pos,
			const std::pair<Eigen::Vector3d, Eigen::Vector3d>& freeRegion){
		return (pos.array() >= freeRegion.first.array()).all() &&
		       (pos.array() <= freeRegion.second.array()).all();
	}

	inline bool SDFMap::isInFreeRegions(
			const Eigen::Vector3d& pos,
			const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& freeRegions){
		for (const auto& freeRegion : freeRegions){
			if (this->isInFreeRegion(pos, freeRegion)){
				return true;
			}
		}
		return false;
	}

	inline bool SDFMap::isInHistoryFreeRegions(const Eigen::Vector3d& pos){
		if (!this->useFreeRegions_) return false;
		for (const auto& freeRegions : this->histFreeRegions_){
			if (this->isInFreeRegions(pos, freeRegions)){
				return true;
			}
		}
		return false;
	}

  inline void SDFMap::freeRegion(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2){
		Eigen::Vector3i idx1, idx2;
		this->posToIndex(pos1, idx1);
		this->posToIndex(pos2, idx2);
		this->boundIndex(idx1);
		this->boundIndex(idx2);
		Eigen::Vector3i min_idx = idx1.cwiseMin(idx2);
		Eigen::Vector3i max_idx = idx1.cwiseMax(idx2);
		for (int xID=min_idx(0); xID<=max_idx(0); ++xID){
			for (int yID=min_idx(1); yID<=max_idx(1); ++yID){
				for (int zID=min_idx(2); zID<=max_idx(2); ++zID){
					this->setFree(Eigen::Vector3i (xID, yID, zID));
				}	
			}
		}
	}

  inline void SDFMap::setFree(const Eigen::Vector3i& idx){
		if (not this->isInMap(idx)) return;
		int address = this->toAddress(idx);
		md_->occupancy_buffer_[address] = mp_->clamp_min_log_ - mp_->unknown_flag_;

		// also set inflated map to free
		int xInflateSize = ceil(mp_->robotSize_(0)/(2*mp_->resolution_));
		int yInflateSize = ceil(mp_->robotSize_(1)/(2*mp_->resolution_));
		int zInflateSize = ceil(mp_->robotSize_(2)/(2*mp_->resolution_));
		Eigen::Vector3i inflateIndex;
		int inflateAddress;
		const int maxIndex = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
		for (int ix=-xInflateSize; ix<=xInflateSize; ++ix){
			for (int iy=-yInflateSize; iy<=yInflateSize; ++iy){
				for (int iz=-zInflateSize; iz<=zInflateSize; ++iz){
					inflateIndex(0) = idx(0) + ix;
					inflateIndex(1) = idx(1) + iy;
					inflateIndex(2) = idx(2) + iz;
					if (not this->isInMap(inflateIndex)){
						continue;
					}
					inflateAddress = this->toAddress(inflateIndex);
					if ((inflateAddress < 0) or (inflateAddress >= maxIndex)){
						continue; // those points are not in the reserved map
					} 
					md_->occupancy_buffer_inflate_[inflateAddress] = false;
				}
			}
		}
	}

}
#endif
