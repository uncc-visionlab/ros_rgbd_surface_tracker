/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   ros_plane_visualizer.hpp
 * Author: john-papadakis
 *
 * Created on July 3, 2017, 6:02 PM
 */

#ifndef ROS_PLANE_VISUALIZER_HPP
#define ROS_PLANE_VISUALIZER_HPP

#ifdef __cplusplus

#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "surface_alignment_optimizer.hpp"

class ros_plane_visualizer {
public:

    ros_plane_visualizer(ros::NodeHandlePtr& nodeptr) {
        triangle_list.action = visualization_msgs::Marker::ADD;
        triangle_list.scale.x = triangle_list.scale.y = triangle_list.scale.z = 1.0;
        triangle_list.pose.orientation.w = 1.0;
        triangle_list.color.a = 1.0;
        triangle_list.color.b = 1.0;
        triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
        triangle_list.ns = "planes";
        this->maxplanes = std::numeric_limits<int>::max();
        this->maxtris = std::numeric_limits<int>::max();
        vis_pub = nodeptr->advertise<visualization_msgs::Marker>("planes", 10);
    }
    
    void setFrameID(std::string frame_id_str) {
        triangle_list.header.frame_id = frame_id_str;
    }
    
    void setMaxPlanes(std::size_t maxplanes) {
        this->maxplanes = maxplanes;
        
    }
    
    void clearMarkerList() {
        this->triangle_list.points.clear();
        this->triangle_list.colors.clear();
    }
    
    void publishPlanes(const PlaneVisualizationData& vis_data, 
            ros::Time timestamp = ros::Time::now()) {
        
        if (vis_pub.getNumSubscribers() > 0) {
            triangle_list.header.stamp = timestamp;
            std::size_t numplanes = vis_data.rect_points.size()/4;

            if ((triangle_list.points.size()/6 + numplanes > maxplanes) 
                    && (numplanes < maxplanes)) {
                triangle_list.points.erase(triangle_list.points.begin(), 
                        triangle_list.points.begin() + 6*numplanes);
                triangle_list.colors.erase(triangle_list.colors.begin(), 
                        triangle_list.colors.begin() + 6*numplanes);
                triangle_list.points.reserve(triangle_list.points.size() + 6*numplanes);
                triangle_list.colors.reserve(triangle_list.points.size() + 6*numplanes);
            } else if (numplanes >= maxplanes) {
                triangle_list.points.clear();
                triangle_list.colors.clear();
                triangle_list.points.reserve(6*maxplanes);
                triangle_list.colors.reserve(6*maxplanes);
            }

            for (std::size_t planeid = 0; planeid != numplanes; planeid++) {

                    std::size_t offset = 4*planeid;
                    std::vector<geometry_msgs::Point> corners;
                    corners.reserve(4);

                    for (std::size_t ptid = 0; ptid != 4; ++ptid) {
                        geometry_msgs::Point ptmsg;
                        ptmsg.x = vis_data.rect_points[ptid + offset](0);
                        ptmsg.y = vis_data.rect_points[ptid + offset](1);
                        ptmsg.z = vis_data.rect_points[ptid + offset](2);
                        corners.push_back(ptmsg);
                    }

                    this->triangle_list.points.push_back(corners[0]);
                    this->triangle_list.points.push_back(corners[1]);
                    this->triangle_list.points.push_back(corners[2]);
                    this->triangle_list.points.push_back(corners[2]);
                    this->triangle_list.points.push_back(corners[3]);
                    this->triangle_list.points.push_back(corners[0]);

            }

            ROS_DEBUG("Publishing %i plane markers = %i vertices.", 
                (int)triangle_list.points.size()/6, (int)triangle_list.points.size());

            vis_pub.publish(triangle_list);
        }
        
    }
    
    void publishTriangleMesh(const PlaneVisualizationData& vis_data, 
            ros::Time timestamp = ros::Time::now()) {
        
        if (vis_pub.getNumSubscribers() > 0) {
            triangle_list.header.stamp = timestamp;
            std::size_t incoming_tris = vis_data.triangles.size();

            if ((this->triangle_list.points.size() + 3*incoming_tris > maxtris) 
                    && (incoming_tris < maxtris)) {
                triangle_list.points.erase(triangle_list.points.begin(), 
                        triangle_list.points.begin() + 3*incoming_tris);
                triangle_list.colors.erase(triangle_list.colors.begin(), 
                        triangle_list.colors.begin() + 3*incoming_tris);
                triangle_list.points.reserve(triangle_list.points.size() + 3*incoming_tris);
                triangle_list.colors.reserve(triangle_list.points.size() + 3*incoming_tris);
            } else if (incoming_tris >= maxtris) {
                triangle_list.points.clear();
                triangle_list.colors.clear();
                triangle_list.points.reserve(3*maxtris);
                triangle_list.colors.reserve(3*maxtris);
            }

            for (auto& tri : vis_data.triangles) {
                
                for (std::size_t pt = 0; pt != 3; ++pt) {
                    
                    geometry_msgs::Point ptmsg;
                    ptmsg.x = tri.vertices[pt](0);
                    ptmsg.y = tri.vertices[pt](1);
                    ptmsg.z = tri.vertices[pt](2);

                    this->triangle_list.points.push_back(ptmsg);
                }
                
            }
        
            ROS_DEBUG("Publishing %i triangle markers = %i vertices.", 
                (int)triangle_list.points.size()/3, (int)triangle_list.points.size());

            vis_pub.publish(triangle_list);
        }
        
    }

private:
    ros::Publisher vis_pub;
    visualization_msgs::Marker triangle_list;
    int maxplanes;
    int maxtris;

};

#endif /* __cplusplus*/

#endif /* ROS_PLANE_VISUALIZER_HPP */

