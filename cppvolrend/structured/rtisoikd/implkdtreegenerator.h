/**
 * Build implicit kd tree on the GPU
 * Original paper from Wald et. al (2005) builds this recursively, this method should 
 * be more effective
 * 
 * . Based on 2009 paper:
 * . Kd-Jump: a Path-Preserving Stackless Traversal for Faster Isosurface Raytracing on GPUs
 * . David M. Hughes and Ik Soo Lim
 * . IEEE Transactions on Visualization and Computer Graphics 15.6 (2009): 1555-1562.
 * . Link: https://ieeexplore.ieee.org/document/5290773
 * . DOI: 10.1109/TVCG.2009.161
 *
 * @author Aritra Bhakat
 * . aritra@kth.se
**/
#pragma once

#include "../../volrenderbase.h"

class ImplicitKDTreeGenerator
{
public:
    ImplicitKDTreeGenerator();
    virtual ~ImplicitKDTreeGenerator();
    
    gl::BufferObject* Generate(vis::DataManager* ext_data_manager);

    int k;
    glm::vec3 R, V, virtualratio;
    std::vector<unsigned int> offsets;
    // Debug texture
    gl::Texture3D* tex;

private:
    gl::ComputeShader* cp_shader_impl_kd_tree;

    gl::BufferObject* kdBuffer;
    gl::BufferObject* matrixSBuffer;
};

