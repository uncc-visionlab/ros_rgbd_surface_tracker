/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include <ros_rgbd_surface_tracker/opengl_iothread.hpp>

using namespace spsc;

int main(int argc, char* argv[]) {
    using namespace std;
    SingleThreaded_IOProducerConsumer parent;
    parent.start();
}
