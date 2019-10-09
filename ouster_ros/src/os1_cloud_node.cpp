/**
 * @file
 * @brief Example node to publish OS-1 point clouds and imu topics
 */

#include <ros/console.h>
#include <ros/ros.h>
#include <ros/service.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <chrono>

#include "ouster/os1_packet.h"
#include "ouster/os1_util.h"
#include "ouster_ros/OS1ConfigSrv.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os1_ros.h"

using PacketMsg = ouster_ros::PacketMsg;
using PointOS1 = ouster_ros::OS1::PointOS1;
using PointOS1base = ouster_ros::OS1::PointOS1base;

namespace OS1 = ouster::OS1;

class OusterCloud{
public:
	OusterCloud(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh):
				nh_(nh),
				private_nh_(private_nh),
				publish_raw_points(false),
				use_xyzi_pcl(false),
				H{0},
				W{0},
				next_m_id{W},
				cur_f_id{-1},
				scan_ts{-1L}
	{}

	bool init();
	void lidar_cb(const PacketMsg& msg);
	void imu_cb(const PacketMsg& msg);
	void prepare_packet(pcl::PointCloud<PointOS1base>::iterator packet_it, std::vector<int> &packet_idx, uint64_t packet_ts);
	void prepare_packet(pcl::PointCloud<PointOS1>::iterator packet_it, std::vector<int> &packet_idx, uint64_t packet_ts);

protected:
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
    bool publish_raw_points;
    bool use_xyzi_pcl;
    std::string tf_prefix;
    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;
    ouster_ros::OS1ConfigSrv cfg;

    uint32_t H;
    uint32_t W;

    ros::Publisher lidar_pub;
    ros::Publisher lidar_raw_pub;
    ros::Publisher imu_pub;

    std::vector<double> xyz_lut;

    ros::Subscriber lidar_packet_sub;
    ros::Subscriber imu_packet_sub;

    int     next_m_id;		/// batch_to_iter_with_packet field
    int32_t cur_f_id;       /// batch_to_iter_with_packet field
    int64_t scan_ts;		/// batch_to_iter_with_packet field
    pcl::PointCloud<PointOS1base> cloud_base;
    pcl::PointCloud<PointOS1> cloud;

    tf2_ros::StaticTransformBroadcaster tf_bcast;

//! templates
protected:
    template<typename BaseMakeType>
    void batch_and_publish(BaseMakeType &&base_make_function,
						   const uint8_t *packet_buf,
						   pcl::PointCloud<PointOS1base>::iterator it,
						   const pcl::PointCloud<PointOS1base>::iterator::value_type& empty)
    {
    	const uint8_t* col_buf = OS1::nth_col(0, packet_buf);
    	const uint64_t packet_ts = OS1::col_timestamp(col_buf);
    	std::vector<int> packet_idx;

    	for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
    		const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
    		const uint16_t m_id = OS1::col_measurement_id(col_buf);
    		const uint16_t f_id = OS1::col_frame_id(col_buf);
    		const uint64_t ts = OS1::col_timestamp(col_buf);
    		const bool valid = OS1::col_valid(col_buf) == 0xffffffff;

    		// drop invalid / out-of-bounds data in case of misconfiguration
    		if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

    		if (f_id != cur_f_id) {
    			// if not initializing with first packet
    			if (scan_ts != -1) {
    				// zero out remaining missing columns
    				std::fill(it + (H * next_m_id), it + (H * W), empty);
    				prepare_msg(cloud_base);
    			}

    			// start new frame
    			scan_ts = ts;
    			next_m_id = 0;
    			cur_f_id = f_id;
    		}

    		// zero out missing columns if we jumped forward
    		if (m_id >= next_m_id) {
    			std::fill(it + (H * next_m_id), it + (H * m_id), empty);
    			next_m_id = m_id + 1;
    		}
    		// index of the first point in current packet
    		const int idx = H * m_id;

    		for (uint8_t ipx = 0; ipx < H; ipx++) {
    			const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
    			uint32_t r = OS1::px_range(px_buf);
    			int ind = 3 * (idx + ipx);

				// x, y, z(m), i
				it[idx + ipx] = base_make_function(r * 0.001f * xyz_lut[ind + 0],
									r * 0.001f * xyz_lut[ind + 1],
									r * 0.001f * xyz_lut[ind + 2],
									OS1::px_signal_photons(px_buf));

    			packet_idx.push_back(idx + ipx);
    		}
    	}
    	if (publish_raw_points){
    		prepare_packet(it, packet_idx, packet_ts);
    	}
    }

    template<typename MakeType>
    void batch_and_publish(MakeType &&make_function,
						   const uint8_t *packet_buf,
						   pcl::PointCloud<PointOS1>::iterator it,
						   const pcl::PointCloud<PointOS1>::iterator::value_type& empty)
    {
    	const uint8_t* col_buf = OS1::nth_col(0, packet_buf);
    	const uint64_t packet_ts = OS1::col_timestamp(col_buf);
    	std::vector<int> packet_idx;

    	for (int icol = 0; icol < OS1::columns_per_buffer; icol++) {
    		const uint8_t* col_buf = OS1::nth_col(icol, packet_buf);
    		const uint16_t m_id = OS1::col_measurement_id(col_buf);
    		const uint16_t f_id = OS1::col_frame_id(col_buf);
    		const uint64_t ts = OS1::col_timestamp(col_buf);
    		const bool valid = OS1::col_valid(col_buf) == 0xffffffff;

    		// drop invalid / out-of-bounds data in case of misconfiguration
    		if (!valid || m_id >= W || f_id + 1 == cur_f_id) continue;

    		if (f_id != cur_f_id) {
    			// if not initializing with first packet
    			if (scan_ts != -1) {
    				// zero out remaining missing columns
    				std::fill(it + (H * next_m_id), it + (H * W), empty);
    				prepare_msg(cloud);
    			}

    			// start new frame
    			scan_ts = ts;
    			next_m_id = 0;
    			cur_f_id = f_id;
    		}

    		// zero out missing columns if we jumped forward
    		if (m_id >= next_m_id) {
    			std::fill(it + (H * next_m_id), it + (H * m_id), empty);
    			next_m_id = m_id + 1;
    		}
    		// index of the first point in current packet
    		const int idx = H * m_id;

    		for (uint8_t ipx = 0; ipx < H; ipx++) {
    			const uint8_t* px_buf = OS1::nth_px(ipx, col_buf);
    			uint32_t r = OS1::px_range(px_buf);
    			int ind = 3 * (idx + ipx);

				// x, y, z(m), i, ts, reflectivity, ring, noise, range (mm)
				it[idx + ipx] = make_function(r * 0.001f * xyz_lut[ind + 0],
									r * 0.001f * xyz_lut[ind + 1],
									r * 0.001f * xyz_lut[ind + 2],
									OS1::px_signal_photons(px_buf), ts - scan_ts,
									OS1::px_reflectivity(px_buf), ipx,
									OS1::px_noise_photons(px_buf), r);

				packet_idx.push_back(idx + ipx);
    		}
    	}
    	if (publish_raw_points){
    		prepare_packet(it, packet_idx, packet_ts);
    	}
    }

    template<typename T>
    void prepare_msg(const pcl::PointCloud<T> &input_cloud){
    	sensor_msgs::PointCloud2 msg;
        msg = ouster_ros::OS1::cloud_to_cloud_msg(input_cloud, std::chrono::nanoseconds{scan_ts}, lidar_frame);
        lidar_pub.publish(msg);
    }
};


void OusterCloud::prepare_packet(pcl::PointCloud<PointOS1base>::iterator packet_it, std::vector<int> &packet_idx, uint64_t packet_ts){
	pcl::PointCloud<PointOS1base> cloud_element;
	for (std::vector<int>::iterator id_it = packet_idx.begin(); id_it < packet_idx.end(); id_it++) {
		cloud_element.points.push_back(packet_it[*id_it]);
	}
	sensor_msgs::PointCloud2 packet_msg;
	packet_msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud_element, std::chrono::nanoseconds{packet_ts}, lidar_frame);
	lidar_raw_pub.publish( packet_msg );
}

void OusterCloud::prepare_packet(pcl::PointCloud<PointOS1>::iterator packet_it, std::vector<int> &packet_idx, uint64_t packet_ts){
	pcl::PointCloud<PointOS1> cloud_element;
	for (std::vector<int>::iterator id_it = packet_idx.begin(); id_it < packet_idx.end(); id_it++) {
		cloud_element.points.push_back(packet_it[*id_it]);
	}
	sensor_msgs::PointCloud2 packet_msg;
	packet_msg = ouster_ros::OS1::cloud_to_cloud_msg(cloud_element, std::chrono::nanoseconds{packet_ts}, lidar_frame);
	lidar_raw_pub.publish( packet_msg );
}


void OusterCloud::lidar_cb(const PacketMsg& pm) {
	if (use_xyzi_pcl) {
		pcl::PointCloud<PointOS1base>::iterator it = cloud_base.begin();
		batch_and_publish(&PointOS1base::make, pm.buf.data(), it, {});
	} else {
		pcl::PointCloud<PointOS1>::iterator it = cloud.begin();
		batch_and_publish(&PointOS1::make, pm.buf.data(), it, {});
	}


}


void OusterCloud::imu_cb(const PacketMsg& p) {
	sensor_msgs::Imu msg;
	msg = ouster_ros::OS1::packet_to_imu_msg(p, imu_frame);
	imu_pub.publish(msg);
}


bool OusterCloud::init() {
    private_nh_.param<bool>("publish_raw_points", publish_raw_points, false);
    private_nh_.param<bool>("use_xyzi_pcl", use_xyzi_pcl, false);
    private_nh_.param<std::string>("tf_prefix", tf_prefix, "");

    if (tf_prefix.empty()) {
    	sensor_frame = "os1_sensor";
    	imu_frame = "os1_imu";
    	lidar_frame = "os1_lidar";
    } else {
		sensor_frame = tf_prefix + "/os1_sensor";
		imu_frame = tf_prefix + "/os1_imu";
		lidar_frame = tf_prefix + "/os1_lidar";
    }

    ros::ServiceClient client = nh_.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return 1;
    }

    H = OS1::pixels_per_column;
    W = OS1::n_cols_of_lidar_mode( OS1::lidar_mode_of_string(cfg.response.lidar_mode) );

    next_m_id = W;

    if (use_xyzi_pcl) {
    	cloud_base = pcl::PointCloud<PointOS1base>(W, H);
    } else {
    	cloud = pcl::PointCloud<PointOS1>(W, H);
    }


    lidar_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("points", 10);
    lidar_raw_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("points_raw", 50);
    imu_pub = private_nh_.advertise<sensor_msgs::Imu>("imu", 100);

    xyz_lut = OS1::make_xyz_lut(W, H, cfg.response.beam_azimuth_angles,
                                     cfg.response.beam_altitude_angles);

    lidar_packet_sub = nh_.subscribe("lidar_packets", 2048, &OusterCloud::lidar_cb, this);
    imu_packet_sub = nh_.subscribe("imu_packets", 100, &OusterCloud::imu_cb, this);

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.imu_to_sensor_transform, sensor_frame, imu_frame));

    tf_bcast.sendTransform(ouster_ros::OS1::transform_to_tf_msg(
        cfg.response.lidar_to_sensor_transform, sensor_frame, lidar_frame));

    return 0;
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "os1_cloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    OusterCloud oc(nh, private_nh);
    oc.init();

    ros::spin();

    return 0;
}
