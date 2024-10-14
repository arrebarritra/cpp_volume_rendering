#include "../../defines.h"
#include "implkdtreegenerator.h"

#include <volvis_utils/datamanager.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <gl_utils/bufferobject.h>

#include <math_utils/utils.h>

#include <chrono>

int GetMaxIndex(glm::uvec3 vec) {
    int maxIndex = 0;
    float max = vec.x;
    if (vec.y > max) {
        max = vec.y;
        maxIndex = 1;
    }
    if (vec.z > max) {
        max = vec.z;
        maxIndex = 2;
    }

    return maxIndex;
}

ImplicitKDTreeGenerator::ImplicitKDTreeGenerator()
    : cp_shader_impl_kd_tree(nullptr), k(0), kdBuffer(nullptr), matrixSBuffer(nullptr) {}

ImplicitKDTreeGenerator::~ImplicitKDTreeGenerator() {
    delete kdBuffer;
    delete matrixSBuffer;
}

unsigned int indexToLoc(unsigned int offset, glm::uvec3 U, glm::vec3 Rl) {
    return (unsigned int)offset + U.x + U.y * Rl.x + U.z * Rl.x * Rl.y;
}

struct Node {
    float isoMin, isoMax;
};

gl::BufferObject* ImplicitKDTreeGenerator::Generate(vis::DataManager* ext_data_manager)
{
    // time kd tree creation
    auto start = std::chrono::system_clock().now();

    ////////////////////////////////////////////
    // Create Rendering Buffers and Shaders

    // - definition of uniform grid and bounding box
    glm::vec3 vol_resolution = /*glm::vec3(3, 3, 3);*/
        glm::vec3(ext_data_manager->GetCurrentStructuredVolume()->GetWidth(),
            ext_data_manager->GetCurrentStructuredVolume()->GetHeight(),
            ext_data_manager->GetCurrentStructuredVolume()->GetDepth());

    glm::vec3 vol_voxelsize = /*glm::vec3(1, 1, 1);*/
        glm::vec3(ext_data_manager->GetCurrentStructuredVolume()->GetScaleX(),
            ext_data_manager->GetCurrentStructuredVolume()->GetScaleY(),
            ext_data_manager->GetCurrentStructuredVolume()->GetScaleZ());

    glm::vec3 vol_aabb = vol_resolution * vol_voxelsize;

    cp_shader_impl_kd_tree = new gl::ComputeShader();
    cp_shader_impl_kd_tree->AddShaderFile(CPPVOLREND_DIR"structured/rtisoikd/kd_tree.comp");
    cp_shader_impl_kd_tree->AddShaderFile(CPPVOLREND_DIR"structured/rtisoikd/generate_impl_kd_tree.comp");
    cp_shader_impl_kd_tree->LoadAndLink();
    cp_shader_impl_kd_tree->Bind();

    if (ext_data_manager->GetCurrentVolumeTexture())
        cp_shader_impl_kd_tree->SetUniformTexture3D("TexVolume", ext_data_manager->GetCurrentVolumeTexture()->GetTextureID(), 1);

    // - let the shader know about the uniform grid
    cp_shader_impl_kd_tree->SetUniform("VolumeGridResolution", vol_resolution);
    cp_shader_impl_kd_tree->SetUniform("VolumeVoxelSize", vol_voxelsize);


    // calculate kd tree data
    R = glm::vec3(vol_resolution.x, vol_resolution.y, vol_resolution.z) - glm::vec3(1, 1, 1);
    int m = glm::ceil(glm::log2(R.x));
    int n = glm::ceil(glm::log2(R.y));
    int p = glm::ceil(glm::log2(R.z));
    k = m + n + p + 1;
    V = glm::exp2(glm::vec3(m, n, p));

    cp_shader_impl_kd_tree->SetUniform("k", k);

    std::vector<glm::uvec4> matrix_S;
    matrix_S.resize(k, glm::uvec4(0u));
    matrix_S[0] = glm::uvec4(0u);

    // kd tree level vars
    glm::uvec3 Vl(1, 1, 1);
    glm::uvec3 Vlhat(V);
    glm::uvec3 Rl(1, 1, 1);
    virtualratio = glm::vec3(R) / glm::vec3(V);

    // calculate tree size, split axes and S matrix
    unsigned int kdTreeNodes = 0;
    std::vector<int> a(k, 0);
    offsets = std::vector<unsigned int>(k, 0u);
    for (int l = 0; l < k; l++) {
        a[l] = GetMaxIndex(Vlhat);
        offsets[l] = kdTreeNodes;

        unsigned int Ml = Rl.x * Rl.y * Rl.z;
        kdTreeNodes += Ml;

        if (l > 0) {
            // fill S matrix
            matrix_S[l] = matrix_S[l - 1];
            matrix_S[l][a[l - 1]]++;
        }

        if (l == k - 1) break;
        Vl[a[l]] = Vl[a[l]] * 2;
        Rl[a[l]] = glm::ceil(Vl[a[l]] * virtualratio[a[l]]);
        Vlhat[a[l]] = Vlhat[a[l]] / 2;
    }

    // allocate memory for kd tree SSBO
    int kdTreeSize = 2 * sizeof(float) * kdTreeNodes;
    kdBuffer = new gl::BufferObject(gl::BufferObject::TYPES::SHADERSTORAGEBUFFEROBJECT);
    kdBuffer->SetBufferData(kdTreeSize, nullptr, GL_DYNAMIC_DRAW);
    kdBuffer->BindBase(3u);
    gl::ExitOnGLError("ImplKDTreeGenerator: After allocating memory for tree");

    // create axis accumulation matrix S on GPU
    int matrixSSize = 4 * sizeof(float) * k;
    matrixSBuffer = new gl::BufferObject(gl::BufferObject::TYPES::SHADERSTORAGEBUFFEROBJECT);
    matrixSBuffer->SetBufferData(matrixSSize, &matrix_S[0][0], GL_STATIC_READ);
    matrixSBuffer->BindBase(4u);
    gl::ExitOnGLError("ImplKDTreeGenerator: After copying S matrix");

    // create offsets array on GPU
    int offsetsSize = sizeof(unsigned int) * k;
    matrixSBuffer = new gl::BufferObject(gl::BufferObject::TYPES::SHADERSTORAGEBUFFEROBJECT);
    matrixSBuffer->SetBufferData(offsetsSize, &offsets[0], GL_STATIC_READ);
    matrixSBuffer->BindBase(5u);
    gl::ExitOnGLError("ImplKDTreeGenerator: After copying offsets");

    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // reset kd tree level vars, this time iteration start from the deepest level
    Vl = V;
    Rl = R;

    for (int l = k - 1; l >= 0; l--) {
        cp_shader_impl_kd_tree->SetUniform("l", l);
        cp_shader_impl_kd_tree->SetUniform("a", a[l]);

        // set dimensions
        cp_shader_impl_kd_tree->SetUniform("Rl", Rl);
        cp_shader_impl_kd_tree->BindUniforms();
        cp_shader_impl_kd_tree->RecomputeNumberOfGroups(Rl.x, Rl.y, Rl.z, 4, 4, 4);
        cp_shader_impl_kd_tree->Dispatch();

        if (l == 0) break;
        // calculate dimensions for next iter
        cp_shader_impl_kd_tree->SetUniform("Rlp1", Rl);
        Vl[a[l - 1]] /= 2;
        Rl[a[l - 1]] = glm::ceil(Vl[a[l - 1]] * virtualratio[a[l - 1]]);
    }

    auto time_elapsed = (int)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock().now() - start).count();
    printf("KD-Tree generated: %d levels, %d bytes on GPU generated in %d us\n", k, kdTreeSize, time_elapsed);

    return kdBuffer;
}
