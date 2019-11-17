#ifndef JACKALEXPLORATION_EXPLORATION_H_
#define JACKALEXPLORATION_EXPLORATION_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>


#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <numeric>

#include "sensor_msgs/PointCloud.h"
#include "laser_geometry/laser_geometry.h"

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>


laser_geometry::LaserProjection projector;

using namespace std;

typedef octomap::point3d point3d;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
const double PI = 3.1415926;
const double octo_reso = 0.05;
const double octo_reso_low = 0.1;
int num_of_samples_eva;
const int num_of_bay = 0; //TODO return to 3
bool normalise;
bool preserve_momentum;
float normalising_factor;
float head_amp;
bool run_once;
bool continuous;
bool simple_weight;
bool wait_for_bounds;
float point_weight;
bool do_A;
bool pioneer;
ros::Publisher Hot_points_pub;
visualization_msgs::Marker hot_points_cubelist;

// Map Stuff
const float step_size = 0.25;
const float map_length = 10;
const float map_width =10;
float grid_space;

// Laser scanner 
const float max_range = 6;
const int scan_points = 1024;

// Costmap
nav_msgs::OccupancyGrid occupancy_grid_in;

ros::Time ros_time;


octomap::OcTree* cur_tree_flat;
octomap::OcTree* cur_tree_low;
octomap::OcTree* tree_info;

octomap_msgs::Octomap msg_octomap_flat;
octomap_msgs::Octomap msg_octomap_low;

tf::TransformListener *tf_listener; 

point3d kinect_orig;
double kinect_head;


ofstream explo_log_file;
//std::string octomap_name_3d;
std::string octomap_name_2d;

//Rviz stuff
visualization_msgs::Marker points, line_strip;
ros::Subscriber point_;
bool waiting_for_center_;
geometry_msgs::PolygonStamped input_;

vector<int> sort_MIs(const vector<double> &v){
    vector<int> idx(v.size());
    iota(idx.begin(), idx.end(),0);

    sort(idx.begin(), idx.end(), 
        [&v](int i1, int i2) {return v[i1] > v[i2];});

    return idx;
}


struct sensorModelScan {
    double points;
    double horizontal_fov;
    double max_range;
    double angle_inc;
    // vector<pair<double, double>> pitch_yaws;
    octomap::Pointcloud SensorRays;
    point3d InitialVector;

    sensorModelScan(double _points, double _horizontal_fov, double _max_range)
            : points(_points), horizontal_fov(_horizontal_fov), max_range(_max_range) {
        angle_inc = horizontal_fov/points;
        for(double j = -points / 2; j < points / 2; ++j){
            InitialVector = point3d(1.0, 0.0, 0.0);
            //InitialVector.rotate_IP(0.0, j * angle_inc, 0);
            //InitialVector.rotate_IP(j * angle_inc,0, 0);
            InitialVector.rotate_IP(0,0,j * angle_inc);
            SensorRays.push_back(InitialVector);
        }
    }
}; 
//sensorModel Kinect_360(128, 96, 2*PI*57/360, 2*PI*43/360, 6);    // Construct sensor model : Kinect
sensorModelScan laserScaner(scan_points, 2*PI, max_range);    // Construct sensor model : 2d Lidar


double countFreeVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        if(!octree->isNodeOccupied(*n))
            volume += pow(n.getSize(), 2); //3); // calc 2d area?
    }
    return volume;
}

double countVolume(const octomap::OcTree *octree) {
    double volume = 0;
    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n) {
        volume += 1; //This make little sense
    }
    return volume;
}

octomap::Pointcloud castSensorRays(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    octomap::Pointcloud hits;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(laserScaner.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, laserScaner.max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * laserScaner.max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

// extract 2d frontier points
vector<vector<point3d>> extractFrontierPoints(const octomap::OcTree *octree) {

    vector<vector<point3d>> frontier_groups;
    vector<point3d> frontier_points;
    octomap::OcTreeNode *n_cur_frontier;
    octomap::OcTreeNode *n_low;
    bool frontier_true;         // whether or not a frontier point
    bool belong_old;            //whether or not belong to old group
    bool octomap_empty = true;  // whetehr octomap is empty
    double distance;
    double R1 = 0.4;            //group size
    double x_cur, y_cur, z_cur;

    for(octomap::OcTree::leaf_iterator n = octree->begin_leafs(octree->getTreeDepth()); n != octree->end_leafs(); ++n)
    {
        frontier_true = false;
        octomap_empty = false;
        unsigned long int num_free = 0; //number of free cube around frontier, for filtering out fake frontier
        
        if(!octree->isNodeOccupied(*n)){
            x_cur = n.getX();
            y_cur = n.getY();
            z_cur = n.getZ();


            
            ///if(octree->search(key))
            //    continue;

            //if there are unknown around the cube, the cube is frontier
            //for (double x_cur_buf = x_cur - octo_reso; x_cur_buf < x_cur + octo_reso; x_cur_buf += octo_reso)
            //    for (double y_cur_buf = y_cur - octo_reso; y_cur_buf < y_cur + octo_reso; y_cur_buf += octo_reso)
            num_free = 0;
            frontier_true = false;
            double mult = pow (2.0, 0.0);
            // TODO adding +2*mult kind of fixes offset problem?
            for (double x_cur_buf = x_cur - mult*octo_reso; x_cur_buf < x_cur + 2*mult*octo_reso; x_cur_buf += mult*octo_reso){
                for (double y_cur_buf = y_cur - mult*octo_reso; y_cur_buf < y_cur + 2*mult*octo_reso; y_cur_buf += mult*octo_reso)
                {
                    n_cur_frontier = octree->search(point3d(x_cur_buf, y_cur_buf, z_cur));
                    //key = octree->coordToKey((x_cur_buf, y_cur_buf, z_cur,octree->getTreeDepth()-2);
                    //n_cur_frontier = octree->search(x_cur_buf, y_cur_buf, z_cur,octree->getTreeDepth()); //octree->search(key);
                    if(n_cur_frontier != NULL)
                        continue;
                    else
                        frontier_true = true;
                        num_free++;
                        
                    //if(!n_cur_frontier)
                    //{
                    //    frontier_true = true;
                    //    continue;            
                    //}

                }
            }
            //n_low = octree->search(x_cur_buf, y_cur_buf, z_cur,octree->getTreeDepth()-3);
            //x_cur = x_cur/0.8*0.8;
            //y_cur = y_cur;
            if(frontier_true)
            {
                // divede frontier points into groups
                if(frontier_groups.size() < 1)
                {
                    frontier_points.resize(1);
                    frontier_points[0] = point3d(x_cur,y_cur,z_cur);
                    frontier_groups.push_back(frontier_points);
                    frontier_points.clear();
                }
                else
                {
                    bool belong_old = false;            

                    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
                        distance = sqrt(pow(frontier_groups[u][0].x()-x_cur, 2)+pow(frontier_groups[u][0].y()-y_cur, 2)) ;
                        if(distance < R1){
                            frontier_groups[u].push_back(point3d(x_cur, y_cur, z_cur));
                            belong_old = true;
                            break;
                        }
                    }
                    if(!belong_old){
                        frontier_points.resize(1);
                        frontier_points[0] = point3d(x_cur, y_cur, z_cur);
                        frontier_groups.push_back(frontier_points);
                        frontier_points.clear();
                    }                              
                }

            } 
        }
        
    }
    // Assume small groups of frontiers are noise i.e. remove groups < 3
    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++){
        if(frontier_groups[u].size()<3){
            frontier_groups.erase(frontier_groups.begin()+u);
            u--; //backtrack as ele deleted
        }
    }
    if(octomap_empty)
        ROS_WARN("Octomap empty ??");
    //ROS_INFO("Last Z height %lf",z_cur);
    return frontier_groups;
}

//generate candidates for moving. Input sensor_orig and initial_yaw, Output candidates
//senor_orig: locationg of sensor.   initial_yaw: yaw direction of sensor
vector<pair<point3d, point3d>> extractCandidateViewPoints(vector<vector<point3d>> frontier_groups, point3d sensor_orig, int segments ) {
    double R2_min = 1.0;        // distance from candidate view point to frontier centers, in meters.
    double R2_max = 5.0;
    double R3 = 0.3;        // to other frontiers

    octomap::OcTreeNode *n_cur_3d;
    vector<pair<point3d, point3d>> candidates;
    double z = sensor_orig.z(); // Suspicious about this z height
    z=0.175;
    double x, y, distance_can;

    //Had issues here deleted the second for loop also changed n to segments as the variable was used twice
    for(vector<vector<point3d>>::size_type u = 0; u < frontier_groups.size(); u++) {
        for(double yaw = 0; yaw < 2*PI; yaw += PI*2/segments ){
            for(double R2 = R2_min; R2<=R2_max; R2+=0.5) { 
                x = frontier_groups[u][0].x() - R2 * cos(yaw);
                y = frontier_groups[u][0].y() - R2 * sin(yaw);

                bool candidate_valid = true;
                n_cur_3d = cur_tree_flat->search(point3d(x, y, z));

                if (!n_cur_3d) {
                    candidate_valid = false;
                    continue;
                }

                if(sqrt(pow(x - sensor_orig.x(),2) + pow(y - sensor_orig.y(),2)) < 0.25){ //0.5
                    candidate_valid = false;// delete candidates close to sensor_orig
                    continue;
                }else{
                    // check candidate to other frontiers;
                    for(vector<vector<point3d>>::size_type n = 0; n < frontier_groups.size(); n++)
                        for(vector<point3d>::size_type m = 0; m < frontier_groups[n].size(); m++){
                            distance_can = sqrt(pow(x - frontier_groups[n][m].x(),2) + pow(y - frontier_groups[n][m].y(),2));
                            if(distance_can < R3){
                                candidate_valid = false;        //delete candidates close to frontier // As frontier is where the points are genrated from
                                continue; // shouldn't this be break break
                            }
                    }
                
                    // volume check
                    for (double x_buf = x - 0.3; x_buf < x + 0.3; x_buf += octo_reso) 
                        for (double y_buf = y - 0.3; y_buf < y + 0.3; y_buf += octo_reso){
                            //for (double z_buf = sensor_orig.z()-0.1; z_buf <sensor_orig.z()+0.3; z_buf += octo_reso)
                            //{
                            double z_buf = sensor_orig.z();
                            n_cur_3d = cur_tree_flat->search(point3d(x_buf, y_buf, z_buf));
                            if(!n_cur_3d) 
                                continue;
                            else if (cur_tree_flat->isNodeOccupied(n_cur_3d)){
                                candidate_valid = false;//delete candidates which have occupied cubes around in 3D area //TODO wtf is this
                                continue; //TODO shouldn't this be break break
                            }  
                        }

                }

                if (candidate_valid)
                {
                    candidates.push_back(make_pair<point3d, point3d>(point3d(x, y, z), point3d(0.0, 0.0, 0.0)));
                }
            }
        }
    }
    ROS_INFO("Frontier groups %lu",frontier_groups.size());
    return candidates;
}


double calc_MI(const octomap::OcTree *octree, const point3d &sensor_orig, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);
    if(hits.size()==0)
        ROS_WARN("Hits empty");
    octree_copy->insertPointCloud(hits, sensor_orig, laserScaner.max_range, true, true);
    double after = countFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}

//based on cast ray
double simple_cal(const octomap::OcTree *octree, const point3d &position,
                                 const point3d &sensor_orientation) {
    double hits = 0;

    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(laserScaner.SensorRays);
    RaysToCast.rotate(sensor_orientation.x(),sensor_orientation.y(),sensor_orientation.z());
    point3d end;
    // Cast Rays to 3d OctoTree and get hit points
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(octree->castRay(position, RaysToCast.getPoint(i), end, true, laserScaner.max_range)) {
            //hits.push_back(end);
        } else {
            //end = RaysToCast.getPoint(i) * laserScaner.max_range;
            //end += position;
            //hits.push_back(end);
            hits++;
        }
    }
    if(hits == 0)
        ROS_WARN("No Hits");
    if(hits == RaysToCast.size())
        ROS_WARN("All Hits");
    return hits;
}

// Update the octomap using the incoming LaserScan
//void laserScanCallbacks(const sensor_msgs::LaserScan) {
// From http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){
    if(!wait_for_bounds){
        sensor_msgs::PointCloud2 cloud2_msg;
        //ROS_INFO("Laser scan callback");
        ros::Time now = ros::Time::now();
        tf_listener->waitForTransform("/base_link", "/base_scan", now, ros::Duration(3.0));   
        // Consider casting multiple times into the map slightly shift the robot in yaw each time? to stop aliasing problems 
        projector.transformLaserScanToPointCloud("/base_scan",*scan_in, cloud2_msg,*tf_listener);
        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL(cloud2_msg, cloud2);
        PointCloud* cloud (new PointCloud);
        PointCloud* cloud_local (new PointCloud);
        pcl::fromPCLPointCloud2(cloud2,*cloud_local);
        octomap::Pointcloud hits;
        ros::Duration(0.07).sleep();
        while(!pcl_ros::transformPointCloud("/map", *cloud_local, *cloud, *tf_listener))
        {
            ros::Duration(0.01).sleep();
        }
        // Insert points into octomap one by one...   
        // TODO insert ray for non returned points??
        //for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->points.being(); it!= cloud->points.end(); it++){
        for(int i = 0; i < cloud->points.size(); i++){
                if(isnan(cloud->points[i].x)){
                    //ROS_INFO("is nana");
                    continue;
                }     
                if(cloud->points[i].z < -1.0){
                    //ROS_INFO("is bellow");
                    ROS_ERROR("Ray bellow floor");
                    continue;         
                }      
                cloud->points[i].z = 0.175; //TODO remove hack
                hits.push_back(point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
        }
        //ROS_INFO("Scaned z %f",cloud->points[1].z);
        cur_tree_flat->insertPointCloud(hits, kinect_orig, 6);
        cur_tree_low->insertPointCloud(hits, kinect_orig, 6);
        //ROS_INFO("Name");
        //cur_tree_flat->write(octomap_name_2d);
        //cur_tree_low->write(octomap_name_2d);
        ROS_INFO("Entropy(2d map) : %f", countFreeVolume(cur_tree_flat));

        delete cloud;
        delete cloud_local;
        //ROS_INFO("End laser scan callback ");
    }
}

double toHSV(double input){
    if(input>1)
        input = input-1;
    if(input<2.0/3.0)
        return abs(input-1.0/3.0)*3;
    return 0;
}

void print_query_info(point3d query, octomap::OcTreeNode* node) {
    if (node != NULL)
        cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
    else 
        cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}

void costmapCb(const nav_msgs::OccupancyGrid::ConstPtr& grid_in){
    ROS_INFO("New map");
    occupancy_grid_in = *grid_in; 
}

void pointCb(const geometry_msgs::PointStamped::ConstPtr& point){
    //double average_distance = polygonPerimeter(input_.polygon) / input_.polygon.points.size();
    ROS_INFO("New mgs");
    if (waiting_for_center_)
    {
        ROS_WARN("Waiting for center");
        // flag is set so this is the last point of boundary polygon, i.e. center

        //if (!pointInPolygon(point->point, input_.polygon))
        //{
        //    ROS_ERROR("Center not inside polygon, restarting");
        //}
        //else
        //{
        //    //Start navigation
        //}
        waiting_for_center_ = false;
        input_.polygon.points.clear();
    }
    else if (input_.polygon.points.empty())
    {
        // first control point, so initialize header of boundary polygon
        ROS_INFO("Boundary Started");
        input_.header = point->header;
        input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
    }
    else if (input_.header.frame_id != point->header.frame_id)
    {
        ROS_ERROR("Frame mismatch, restarting polygon selection");
        input_.polygon.points.clear();
    }
    else if (input_.polygon.points.size() > 1 &&
      sqrt(pow(input_.polygon.points.front().x-point->point.x,2)+pow(input_.polygon.points.front().y-point->point.y,2))<1)
      //pointsNearby(input_.polygon.points.front(), point->point, average_distance*0.1))
    {
        // check if last boundary point, i.e. nearby to first point

        if (input_.polygon.points.size() < 3)
        {
            ROS_ERROR("Not a valid polygon, restarting");
            input_.polygon.points.clear();
        }
        else
        {
            waiting_for_center_ = true;
            ROS_WARN("Please select an initial point for exploration inside the polygon");
        }
    }
    else
    {
        ROS_INFO("New PT");
        // otherwise, must be a regular point inside boundary polygon
        input_.polygon.points.push_back(costmap_2d::toPoint32(point->point));
        input_.header.stamp = ros::Time::now();
        //geometry_msgs::Point q;
        //q.x = .x();
        //q.y = .y();
        //q.z = .z()+octo_reso;
        hot_points_cubelist.points.resize(input_.polygon.points.size());
        hot_points_cubelist.header.frame_id = "map";
        hot_points_cubelist.header.stamp = ros::Time::now();
        hot_points_cubelist.ns = "frontier_points_array";
        hot_points_cubelist.id = 0;
        hot_points_cubelist.type = visualization_msgs::Marker::CUBE_LIST;
        hot_points_cubelist.action = visualization_msgs::Marker::ADD;
        hot_points_cubelist.scale.x = 3*
        octo_reso;
        hot_points_cubelist.scale.y = 3*
        octo_reso;
        hot_points_cubelist.scale.z = 3*
        octo_reso;
        hot_points_cubelist.color.a = 1.0;
        hot_points_cubelist.color.r = 0;
        hot_points_cubelist.color.g = (double)255/255;
        hot_points_cubelist.color.b = 0;
        hot_points_cubelist.lifetime = ros::Duration();
        hot_points_cubelist.points.push_back(point->point); 
        Hot_points_pub.publish(hot_points_cubelist); //publish frontier_points
    }
}



int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}


#endif
