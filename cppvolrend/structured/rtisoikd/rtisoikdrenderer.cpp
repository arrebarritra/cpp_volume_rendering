#include "../../defines.h"
#include "rtisoikdrenderer.h"

#include <vis_utils/camera.h>
#include <volvis_utils/datamanager.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <math_utils/utils.h>


RaytracingIsoImplKD::RaytracingIsoImplKD()
	:cp_shader_rendering(nullptr)
	, cp_shader_impl_kd_tree(nullptr)
	, m_u_isovalue(0.5f)
	, m_u_color(0.66f, 0.6f, 0.05f, 1.0f)
	, m_apply_gradient_shading(false)
{
}


RaytracingIsoImplKD::~RaytracingIsoImplKD()
{
}


const char* RaytracingIsoImplKD::GetName()
{
	return "Isosurface Ray Tracer - Implicit k-d tree";
}


const char* RaytracingIsoImplKD::GetAbbreviationName()
{
	return "rtisoikd";
}


vis::GRID_VOLUME_DATA_TYPE RaytracingIsoImplKD::GetDataTypeSupport()
{
	return vis::GRID_VOLUME_DATA_TYPE::STRUCTURED;
}


void RaytracingIsoImplKD::Clean()
{
	if (cp_shader_rendering) delete cp_shader_rendering;
	cp_shader_rendering = nullptr;
	kdg.~ImplicitKDTreeGenerator();

	gl::ExitOnGLError("Could not destroy shaders");

	BaseVolumeRenderer::Clean();
}


bool RaytracingIsoImplKD::Init(int swidth, int sheight)
{
	//Clean before we continue
	if (IsBuilt()) Clean();

	//We need data to work on
	if (m_ext_data_manager->GetCurrentVolumeTexture() == nullptr) return false;

	////////////////////////////////////////////
	// Create Implicit KD Tree on GPU
	kdg.Generate(m_ext_data_manager);

	////////////////////////////////////////////
	// Create Rendering Buffers and Shaders

	// - definition of uniform grid and bounding box
	glm::vec3 vol_resolution = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetWidth(),
		m_ext_data_manager->GetCurrentStructuredVolume()->GetHeight(),
		m_ext_data_manager->GetCurrentStructuredVolume()->GetDepth());

	glm::vec3 vol_voxelsize = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleX(),
		m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleY(),
		m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleZ());

	glm::vec3 vol_aabb = vol_resolution * vol_voxelsize;

	// - load shaders
	cp_shader_rendering = new gl::ComputeShader();
	cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/_common_shaders/ray_bbox_intersection.comp");
	cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/rtisoikd/kd_tree.comp");
	cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/rtisoikd/ray_tracing_iso_impl_kd.comp");
	cp_shader_rendering->LoadAndLink();
	cp_shader_rendering->Bind();

	// - data sets to work on: scalar field and its gradient
	if (m_ext_data_manager->GetCurrentVolumeTexture())
		cp_shader_rendering->SetUniformTexture3D("TexVolume", m_ext_data_manager->GetCurrentVolumeTexture()->GetTextureID(), 1);
	if (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture())
		cp_shader_rendering->SetUniformTexture3D("TexVolumeGradient", m_ext_data_manager->GetCurrentGradientTexture()->GetTextureID(), 2);

	// - let the shader know about the uniform grid
	cp_shader_rendering->SetUniform("VolumeGridResolution", vol_resolution);
	cp_shader_rendering->SetUniform("VolumeVoxelSize", vol_voxelsize);
	cp_shader_rendering->SetUniform("VolumeGridSize", vol_aabb);

	// - pass kd tree info to shader
	cp_shader_rendering->SetUniform("k", kdg.k);
	cp_shader_rendering->SetUniform("R", kdg.R);
	cp_shader_rendering->SetUniform("V", kdg.V);
	cp_shader_rendering->SetUniform("virtualratio", kdg.virtualratio);

	cp_shader_rendering->BindUniforms();
	cp_shader_rendering->Unbind();
	gl::ExitOnGLError("RayTracingIsoImplKD: Error on Preparing Rendering Shader");

	/////////////////////////////////
	// Finalization

	//Support for multisampling
	Reshape(swidth, sheight);

	SetBuilt(true);
	SetOutdated();
	return true;
}


void RaytracingIsoImplKD::ReloadShaders()
{
	cp_shader_rendering->Reload();
	m_rdr_frame_to_screen.ClearShaders();
}


bool RaytracingIsoImplKD::Update(vis::Camera* camera)
{
	cp_shader_rendering->Bind();

	/////////////////////////////
	// Multisample
	if (IsPixelMultiScalingSupported() && GetCurrentMultiScalingMode() > 0)
	{
		cp_shader_rendering->RecomputeNumberOfGroups(m_rdr_frame_to_screen.GetWidth(),
			m_rdr_frame_to_screen.GetHeight(), 0);
	}
	else
	{
		cp_shader_rendering->RecomputeNumberOfGroups(m_ext_rendering_parameters->GetScreenWidth(),
			m_ext_rendering_parameters->GetScreenHeight(), 0);
	}

	/////////////////////////////
	// Camera
	cp_shader_rendering->SetUniform("CameraEye", camera->GetEye());
	cp_shader_rendering->SetUniform("u_CameraLookAt", camera->LookAt());
	cp_shader_rendering->SetUniform("ProjectionMatrix", camera->Projection());
	cp_shader_rendering->SetUniform("u_TanCameraFovY", (float)tan(DEGREE_TO_RADIANS(camera->GetFovY()) / 2.0));
	cp_shader_rendering->SetUniform("u_CameraAspectRatio", camera->GetAspectRatio());
	cp_shader_rendering->SetUniform("WorldEyePos", camera->GetEye());

	/////////////////////////////
	// Isosurface aspects
	cp_shader_rendering->SetUniform("Isovalue", m_u_isovalue);
	cp_shader_rendering->SetUniform("Color", m_u_color);

	// Bind all
	cp_shader_rendering->BindUniforms();

	gl::Shader::Unbind();
	gl::ExitOnGLError("RayTracingIsoImplKD: After Update.");
	return true;
}


void RaytracingIsoImplKD::Redraw()
{
	m_rdr_frame_to_screen.ClearTexture();

	cp_shader_rendering->Bind();
	m_rdr_frame_to_screen.BindImageTexture();

	cp_shader_rendering->Dispatch();
	gl::ComputeShader::Unbind();

	m_rdr_frame_to_screen.Draw();
}

void RaytracingIsoImplKD::FillParameterSpace(ParameterSpace& pspace)
{
	pspace.ClearParameterDimensions();
}


void RaytracingIsoImplKD::SetImGuiComponents()
{
	ImGui::Separator();

	ImGui::Text("Isovalue: ");
	if (ImGui::DragFloat("###RayTracingIsoImplKDIsovalue", &m_u_isovalue, 0.01f, 0.01f, 100.0f, "%.2f"))
	{
		m_u_isovalue = std::max(std::min(m_u_isovalue, 100.0f), 0.01f); //When entering with keyboard, ImGui does not take care of the min/max.
		SetOutdated();
	}

	if (ImGui::ColorEdit4("Color", &m_u_color[0]))
	{
		SetOutdated();
	}

	//AddImGuiMultiSampleOptions();
}
