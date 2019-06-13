//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------



#include <motion_planning/utils/Visualization.h>
#include <motion_planning/Trajectory.h>
#include <motion_planning/planners/rrtstar.h>
#include <motion_planning/thirdparty/DouglasPeucker.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int _argc, char** _argv){    
    if(_argc < 9 ){
        std::cout <<" ./wp_calculation [map_file] [rtt_iterations] [rrt_step] [douglas_distance] [wpPublisherTopic] [target_X] [target_Y] [target_Z]" <<std::endl;
        return -1;
    }

    std::string mapFilePath = _argv[1];
    int rrtIterations = atoi(_argv[2]);
    float rrtStep = atof(_argv[3]);
    float douglasDistance = atof(_argv[4]);
    std::string wpPublisherTopic = _argv[5];
    float targetX = atof(_argv[6]);
    float targetY = atof(_argv[7]);
    float targetZ = atof(_argv[8]);
    bool visualizeTree = atoi(_argv[9]);
    int pointSize = 1;
    if(_argc > 10)
        pointSize = atoi(_argv[10]);

    ros::init(_argc, _argv, "wp_calculation");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Publisher wpPublisher = nh.advertise<geometry_msgs::PoseStamped>(wpPublisherTopic, 1);
    geometry_msgs::PoseStamped uavPose;
    ros::Subscriber uavPoseSubscriber = nh.subscribe<geometry_msgs::PoseStamped>("/ual/pose", 10, [&](const geometry_msgs::PoseStamped::ConstPtr &_msg){
        uavPose = *_msg;
    });

    std::cout << "Loading input cloud: ";
    auto t0 = std::chrono::system_clock::now();
    // Load cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud, filtered;
    if(mapFilePath.find(".pcd") != std::string::npos){
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (mapFilePath, cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
    }else if(mapFilePath.find(".ply") != std::string::npos){
        if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (mapFilePath, cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
    }else if(mapFilePath.find(".obj") != std::string::npos){
        if (pcl::io::loadOBJFile<pcl::PointXYZRGB> (mapFilePath, cloud) == -1) {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
    }else {
        std::cout << "Bad file format " << std::endl;
        return -1;
    }

    // Filter cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud.makeShared());
    sor.setLeafSize (0.01f,0.01f,0.01f);
    sor.filter(filtered);

    // Draw cloud
    mp::Visualizer viz;
    auto rawViewer = viz.rawViewer();
    viz.draw<pcl::PointXYZRGB>(filtered); 
    rawViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, "cloud_0"); 
    //rawViewer->setBackgroundColor(0.9,0.9,0.9); 
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D (filtered, minPt, maxPt);
    auto t1 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0 << "s." << std::endl;
    std::cout << "Configuring planner";
    // Config planner
    mp::RRTStar planner(rrtStep);
    planner.initPoint({ uavPose.pose.position.x,
                        uavPose.pose.position.y,
                        uavPose.pose.position.z});
    planner.targetPoint({targetX, targetY, targetZ});

    // planner.enableDebugVisualization(viz.rawViewer());
    planner.iterations(rrtIterations);
    planner.dimensions( minPt.x, minPt.y, minPt.z,
                        maxPt.x, maxPt.y, maxPt.z);

    // Add constraint
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(0.1);
    octree.setInputCloud(filtered.makeShared());
    octree.addPointsFromInputCloud();

    float safeDist = 1.0;
    mp::Constraint c1 = [&](const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dest){
        pcl::PointXYZRGB query;
        query.x = _dest[0];
        query.y = _dest[1];
        query.z = _dest[2];
        std::vector<int> index;
        std::vector< float > dist;
        octree.nearestKSearch(query, 1, index, dist);
        
        return dist[0] > safeDist;
    };
    planner.addConstraint(c1);
    auto t2 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()/1000.0 << "s." << std::endl;
    std::cout << "Computing trajectory: ";
    // Compute traj
    auto traj = planner.compute();
    auto t3 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count()/1000.0 << "s." << std::endl;
    std::cout << "Display results: ";
    std::vector<mp::RRTStar::NodeInfo> nodesInfo;
    pcl::PointCloud<pcl::PointXYZ>::Ptr nodes;
    planner.tree(nodes, nodesInfo);

    if(visualizeTree){
        vtkSmartPointer<vtkPolyData> treeBase;
        viz.drawCustom([&](std::shared_ptr<pcl::visualization::PCLVisualizer> &_viewer){
            // Create new graph
            treeBase = vtkSmartPointer<vtkPolyData>::New();
            treeBase->Allocate();
            vtkSmartPointer<vtkPoints> covisibilityNodes = vtkSmartPointer<vtkPoints>::New();

            // Fill-up with nodes
            for(unsigned i = 0; i <  nodes->size(); i++){
                covisibilityNodes->InsertNextPoint(     nodes->points[i].x, 
                                                        nodes->points[i].y, 
                                                        nodes->points[i].z);
                if(i > 0){
                    vtkIdType connectivity[2];
                    connectivity[0] = nodesInfo[i].id_;
                    connectivity[1] = nodesInfo[i].parent_;
                    treeBase->InsertNextCell(VTK_LINE,2,connectivity);
                }
            }

            treeBase->SetPoints(covisibilityNodes);
            _viewer->addModelFromPolyData(treeBase, "treeRRT");
        });
    }

    // Draw result.
    viz.draw(traj, true, 5);
    auto t4 = std::chrono::system_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t3).count()/1000.0 << "s." << std::endl;
    

    std::list<mp::p3d> trajUnopt; 
    for(auto &p: traj.points()){
        trajUnopt.push_back({p[0], p[1], p[2]});
    }
    mp::DouglasPuecker2D<mp::p3d, mp::p3dAccessor> dp3d(trajUnopt);
    dp3d.simplify(douglasDistance);

    std::list<mp::p3d> result = dp3d.getLine();
    mp::Trajectory trajOpt;
    for(auto &p: result){
        trajOpt.appendPoint({std::get<0>(p), std::get<1>(p), std::get<2>(p)});
    }

    viz.draw(trajOpt, false, 10, 255, 0, 0);

    // Publish wps:
    int idCounter = 0;
    for(auto iter = result.rbegin() ; iter != result.rend(); iter++){
        auto p = *iter;
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = std::to_string(idCounter++);
        pose.pose.position.x = std::get<0>(p);
        pose.pose.position.y = std::get<1>(p);
        pose.pose.position.z = std::get<2>(p);

        wpPublisher.publish(pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "-1";
    wpPublisher.publish(pose);

    while(ros::ok()){
        viz.rawViewer()->spinOnce(30);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}