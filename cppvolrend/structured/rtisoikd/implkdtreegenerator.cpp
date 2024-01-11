#include "../../defines.h"
#include "implkdtreegenerator.h"

#include <volvis_utils/datamanager.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <gl_utils/bufferobject.h>

#include <math_utils/utils.h>

#include <chrono>

int GetMaxIndex(glm::vec3 vec) {
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
	kdBuffer->~BufferObject();
	matrixSBuffer->~BufferObject();
}

gl::BufferObject* ImplicitKDTreeGenerator::Generate(vis::DataManager* ext_data_manager)
{
	////////////////////////////////////////////
	// Create Rendering Buffers and Shaders

	// - definition of uniform grid and bounding box
	glm::vec3 vol_resolution = glm::vec3(ext_data_manager->GetCurrentStructuredVolume()->GetWidth(),
		ext_data_manager->GetCurrentStructuredVolume()->GetHeight(),
		ext_data_manager->GetCurrentStructuredVolume()->GetDepth());

	glm::vec3 vol_voxelsize = glm::vec3(ext_data_manager->GetCurrentStructuredVolume()->GetScaleX(),
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
	R = glm::vec3(vol_resolution.x - 1, vol_resolution.y - 1, vol_resolution.z - 1);
	int m = glm::ceil(glm::log2(R.x));
	int n = glm::ceil(glm::log2(R.y));
	int p = glm::ceil(glm::log2(R.z));
	k = m + n + p + 1;

	cp_shader_impl_kd_tree->SetUniform("k", k);

	std::vector<glm::uvec4> matrix_S;
	matrix_S.resize(k, glm::uvec4(0u));
	matrix_S[0] = glm::uvec4(0u);

	V = glm::exp2(glm::vec3(m, n, p));
	cp_shader_impl_kd_tree->SetUniform("VirtualGridDims", V);

	auto start = std::chrono::system_clock().now();

	// kd tree level vars
	glm::vec3 Vl(1, 1, 1);
	glm::vec3 Vlhat(V);
	glm::vec3 Rl(1, 1, 1);
	virtualratio = R / V;

	// calculate tree size, split axes and S matrix
	int kdTreeNodes = 0;
	std::vector<int> a(k, 0);
	std::vector<unsigned int> offsets(k, 0);
	for (int l = 0; l < k; l++) {
		a[l] = GetMaxIndex(Vlhat);
		offsets[l] = kdTreeNodes;

		int Ml = Rl.x * Rl.y * Rl.z;
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
	int offsetsSize = sizeof(int) * k;
	matrixSBuffer = new gl::BufferObject(gl::BufferObject::TYPES::SHADERSTORAGEBUFFEROBJECT);
	matrixSBuffer->SetBufferData(offsetsSize, &offsets[0], GL_STATIC_READ);
	matrixSBuffer->BindBase(5u);
	gl::ExitOnGLError("ImplKDTreeGenerator: After copying offsets");

	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	// reset kd tree level vars, this time iteration start from the deepest level
	Vl = V;
	Vlhat = glm::vec3(1, 1, 1);
	Rl = R;

	for (int l = k - 1; l >= 0; l--) {;
		cp_shader_impl_kd_tree->SetUniform("l", l);
		cp_shader_impl_kd_tree->SetUniform("a", a[l]);

		// calculate amount of nodes and offset in current level
		int Ml = Rl.x * Rl.y * Rl.z;
		cp_shader_impl_kd_tree->SetUniform("Ml", Ml);
		cp_shader_impl_kd_tree->SetUniform("offset", offsets[l]);

		// set dimensions
		cp_shader_impl_kd_tree->SetUniform("Rl", Rl);
		cp_shader_impl_kd_tree->BindUniforms();
		cp_shader_impl_kd_tree->RecomputeNumberOfGroups(Rl.x, Rl.y, Rl.z, 8, 8, 8);
		cp_shader_impl_kd_tree->Dispatch();

		// current parameters become parameters of child in next iter
		cp_shader_impl_kd_tree->SetUniform("Rlp1", Rl);
		cp_shader_impl_kd_tree->SetUniform("child_offset", offsets[l]);

		// calculate dimensions for next iter
		if (l == 0) break;
		Vl[a[l-1]] = Vl[a[l-1]] / 2;
		Rl[a[l-1]] = glm::ceil(Vl[a[l-1]] * virtualratio[a[l-1]]);
		Vlhat[a[l]] *= 2;
	}

	auto time_elapsed = (int) std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock().now() - start).count();
	printf("KD-Tree generated: %d levels, %d bytes on GPU generated in %d us\n", k, kdTreeSize, time_elapsed);

	return kdBuffer;
}
