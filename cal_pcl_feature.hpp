//
// Created by xlju on 8/12/2021.
//

#pragma once

#include <cassert>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transforms.h>
#include <Eigen/Core>

double MINIMUM_RANGE = 5;
int N_SCANS = 64;
const double scanPeriod = 0.1;
typedef pcl::PointXYZI PointType;
float cloudCurvature[400000];
int cloudSortInd[400000];
int cloudNeighborPicked[400000];
int cloudLabel[400000];
bool comp (int i,int j) { return (cloudCurvature[i]<cloudCurvature[j]); }


template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                            pcl::PointCloud<PointT> &cloud_out, float thres) {
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x +
        cloud_in.points[i].y * cloud_in.points[i].y +
        cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}



void split_pointcloud2scans(pcl::PointCloud<pcl::PointXYZI>::Ptr ptr, std::vector<pcl::PointCloud<PointType>>& finalLaserCloudScans, int &cnt) {
    pcl::PointCloud<pcl::PointXYZI> laserCloudIn = *ptr;

    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE);


    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                                  2 * M_PI;

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180.0f / M_PI;
        int scanID = 0;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outliers
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            break;
//            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);

//        point.intensity = scanID + scanPeriod * relTime;
        laserCloudScans[scanID].push_back(point);
    }
    finalLaserCloudScans = std::move(laserCloudScans);
    cnt = count;
}



std::map<std::string, pcl::PointCloud<PointType>::Ptr>
        cal_pointcloud_smoothness(std::vector<pcl::PointCloud<PointType>> &laserCloudScans, int valid_count) {
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);
    int cloudSize = valid_count;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }


    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX =
                laserCloud->points[i - 5].x +
                laserCloud->points[i - 4].x +
                laserCloud->points[i - 3].x +
                laserCloud->points[i - 2].x +
                laserCloud->points[i - 1].x -
                10 * laserCloud->points[i].x +
                laserCloud->points[i + 1].x +
                laserCloud->points[i + 2].x +
                laserCloud->points[i + 3].x +
                laserCloud->points[i + 4].x +
                laserCloud->points[i + 5].x;
        float diffY =
                laserCloud->points[i - 5].y +
                laserCloud->points[i - 4].y +
                laserCloud->points[i - 3].y +
                laserCloud->points[i - 2].y +
                laserCloud->points[i - 1].y -
                10 * laserCloud->points[i].y +
                laserCloud->points[i + 1].y +
                laserCloud->points[i + 2].y +
                laserCloud->points[i + 3].y +
                laserCloud->points[i + 4].y +
                laserCloud->points[i + 5].y;
        float diffZ =
                laserCloud->points[i - 5].z +
                laserCloud->points[i - 4].z +
                laserCloud->points[i - 3].z +
                laserCloud->points[i - 2].z +
                laserCloud->points[i - 1].z -
                10 * laserCloud->points[i].z +
                laserCloud->points[i + 1].z +
                laserCloud->points[i + 2].z +
                laserCloud->points[i + 3].z +
                laserCloud->points[i + 4].z +
                laserCloud->points[i + 5].z;

        cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }

    pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>);
    std::map<std::string, pcl::PointCloud<PointType>::Ptr> results;
    results["cornerPointsSharp"] = cornerPointsSharp;
    results["cornerPointsLessSharp"] = cornerPointsLessSharp;
    results["surfPointsFlat"] = surfPointsFlat;
    results["surfPointsLessFlat"] = surfPointsLessFlat;


//    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

//            TicToc t_tmp;
            // ensure feature extraction in every area
            std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);
//            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp->push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp->push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat->push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);
        *surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    return results;
}


void calculate_pillars(pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud, std::vector<PointType> &key_points,
                        int K, double threshold, std::vector<std::vector<int>>& indices_result) {

    pcl::KdTreeFLANN<pcl::PointXY>::Ptr p_kdtree(new pcl::KdTreeFLANN<pcl::PointXY>());
    // project to bev
    pcl::PointCloud<pcl::PointXY>::Ptr bev_cloud(new pcl::PointCloud<pcl::PointXY>);
    for(int i = 0; i != p_cloud->points.size(); i++) {
        pcl::PointXY p;
        p.x = p_cloud->points[i].x;
        p.y = p_cloud->points[i].y;
        bev_cloud->push_back(p);
    }

    p_kdtree->setInputCloud(bev_cloud);
    std::vector<std::vector<int>> indices_all;
    for(auto const &c: key_points) {
        std::vector<int> indices;
        std::vector<int> filtered_indices;

        std::vector<float> sq_distances;
        pcl::PointXY p;
        p.x = c.x;
        p.y = c.y;
        p_kdtree->nearestKSearch(p, K, indices, sq_distances);
        for(int i = 0; i != sq_distances.size(); i++) {
            if(sq_distances.at(i) < threshold) {
                filtered_indices.push_back(indices.at(i));
            }
        }
        indices_all.emplace_back(filtered_indices);
//        std::cout << "pillar pt num:" << filtered_indices.size() << std::endl;
    }
    indices_result =  std::move(indices_all);
}


std::vector<std::vector<std::vector<float>>> cal_pillar_stacked_feature(std::vector<PointType> const &key_points,
                                 std::vector<std::vector<int>>const & pilliar_indices,
                                 pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud
                                 ) {
    assert(key_points.size() == pilliar_indices.size());
    std::vector<std::vector<std::vector<float>>> pilliar_feature_list;
    for (int i = 0; i != key_points.size(); i++) {
        auto key_point = key_points[i];
        auto pilliar_index = pilliar_indices[i];

        // cal gravity center
        pcl::PointXYZ gravity_center(0.f, 0.f, 0.f);
        for(auto pilliar_ele: pilliar_index) {
            auto pilliar_point = p_cloud->points[pilliar_ele];
            gravity_center.x += pilliar_point.x;
            gravity_center.y += pilliar_point.y;
            gravity_center.z += pilliar_point.z;
        }
        gravity_center.x /= pilliar_index.size();
        gravity_center.y /= pilliar_index.size();
        gravity_center.z /= pilliar_index.size();

        std::vector<std::vector<float>> pilliar_feature;
        for(auto pilliar_ele: pilliar_index) {
            auto pilliar_point = p_cloud->points[pilliar_ele];
            std::vector<float> feat_ele(3 + 1 + 3 + 1 + 3, 0);
            feat_ele[0] = pilliar_point.x;
            feat_ele[1] = pilliar_point.y;
            feat_ele[2] = pilliar_point.z;
            feat_ele[3] = pilliar_point.intensity;
            feat_ele[4] = pilliar_point.x - gravity_center.x;
            feat_ele[5] = pilliar_point.y - gravity_center.y;
            feat_ele[6] = pilliar_point.z - gravity_center.z;
            feat_ele[7] = sqrt(pilliar_point.x * pilliar_point.x +
                    pilliar_point.y * pilliar_point.y + pilliar_point.z * pilliar_point.z);
            feat_ele[8] = pilliar_point.x - key_point.x;
            feat_ele[9] = pilliar_point.y - key_point.y;
            feat_ele[10] = pilliar_point.z - key_point.z;
            pilliar_feature.emplace_back(feat_ele);
        }
        pilliar_feature_list.emplace_back(pilliar_feature);
    }
    return pilliar_feature_list;
}

std::vector<std::vector<std::vector<float>>> get_data_for_sticky_pillar(pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud,
                                                                        std::vector<PointType> &selected_pts_result
) {

    std::vector<pcl::PointCloud<PointType>> finalLaserCloudScans;
    int valid_pt_num{0};
    split_pointcloud2scans(p_cloud, finalLaserCloudScans, valid_pt_num);
    auto res_map = cal_pointcloud_smoothness(finalLaserCloudScans, valid_pt_num);
//    for (auto const&c: res_map) {
//
//        std::cout << c.first << ":" << c.second->points.size() << std::endl;
//    }
    int sample_num = 250;
    int pillar_capacity = 128;
    float pillar_dis_threshold = 0.5f;
    assert(res_map["cornerPointsSharp"]->points.size() > sample_num);
    assert(res_map["surfPointsFlat"]->points.size() > sample_num);
    std::vector<PointType> selected_pts;

    std::vector<bool> selected_cornerPointsSharp(res_map["cornerPointsSharp"]->points.size(), 0);
    while (selected_pts.size() != sample_num) {
        int inx = rand() % res_map["cornerPointsSharp"]->points.size();
        if(selected_cornerPointsSharp[inx]) {
            continue;
        }
        selected_cornerPointsSharp[inx] = true;
        selected_pts.emplace_back(res_map["cornerPointsSharp"]->points[inx]);
    }

    std::vector<bool> selected_surfPointsFlat(res_map["surfPointsFlat"]->points.size(), 0);
    while (selected_pts.size() != sample_num) {
        int inx = rand() % res_map["surfPointsFlat"]->points.size();
        if(selected_surfPointsFlat[inx]) {
            continue;
        }
        selected_surfPointsFlat[inx] = true;
        selected_pts.emplace_back(res_map["surfPointsFlat"]->points[inx]);
    }

    std::vector<std::vector<int>> pillar_pt_indices_result;
    calculate_pillars(p_cloud, selected_pts, pillar_capacity, pillar_dis_threshold, pillar_pt_indices_result);
    auto pilliar_feature_list = cal_pillar_stacked_feature(selected_pts, pillar_pt_indices_result, p_cloud);
    selected_pts_result = std::move(selected_pts);
    return pilliar_feature_list;
}

// relpose to trans pt2 to coordinate of pt1
Eigen::MatrixXf get_pillar_matching_gt(pcl::PointCloud<pcl::PointXYZ>::Ptr  pt_cloud_1,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr  pt_cloud_2,
                            Eigen::Matrix4f rel_pose,
                            float radius) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr  pt_cloud_2_t(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*pt_cloud_2, *pt_cloud_2_t, rel_pose);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr p_kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    p_kdtree->setInputCloud(pt_cloud_1);

    Eigen::MatrixXf matching_matrix;
    matching_matrix.resize(pt_cloud_1->points.size(), pt_cloud_2->points.size());
    matching_matrix.setConstant(std::numeric_limits<float>::infinity());
    for (int index_2 = 0; index_2 != pt_cloud_2_t->points.size(); index_2++) {
        std::vector<int> indices_1;
        std::vector<float> sqr_dis;
        p_kdtree->radiusSearch(pt_cloud_2_t->points[index_2], radius, indices_1, sqr_dis);
        for (int i = 0; i != indices_1.size(); i++) {
            matching_matrix(i, index_2) = sqr_dis[i];
        }
    }
    return matching_matrix;
}