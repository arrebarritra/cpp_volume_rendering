#include "../../defines.h"
#include "rc1pisorenderer.h"

#include <vis_utils/camera.h>
#include <volvis_utils/datamanager.h>

#include <glm/glm.hpp>
#include <glm/ext.hpp>

#include <math_utils/utils.h>


RayCasting1PassIso::RayCasting1PassIso()
  :cp_shader_rendering(nullptr)
  ,m_u_isovalue(0.5f)
  ,m_u_step_size(2.0f)
  ,m_u_color(0.66f, 0.6f, 0.05f, 1.0f)
  ,m_apply_gradient_shading(false)
{
}


RayCasting1PassIso::~RayCasting1PassIso()
{
}


const char* RayCasting1PassIso::GetName()
{
  return "1-Pass - Isosurface Raycaster";
}


const char* RayCasting1PassIso::GetAbbreviationName()
{
  return "iso";
}


vis::GRID_VOLUME_DATA_TYPE RayCasting1PassIso::GetDataTypeSupport()
{
  return vis::GRID_VOLUME_DATA_TYPE::STRUCTURED;
}


void RayCasting1PassIso::Clean()
{
  if (cp_shader_rendering) delete cp_shader_rendering;
  cp_shader_rendering = nullptr;

  gl::ExitOnGLError("Could not destroy shaders");

  BaseVolumeRenderer::Clean();
}


bool RayCasting1PassIso::Init(int swidth, int sheight)
{
  //Clean before we continue
  if (IsBuilt()) Clean();

  //We need data to work on
  if (m_ext_data_manager->GetCurrentVolumeTexture() == nullptr) return false;


  ////////////////////////////////////////////
  // Create Rendering Buffers and Shaders

  // - definition of uniform grid and bounding box
  glm::vec3 vol_resolution = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetWidth() ,
                                       m_ext_data_manager->GetCurrentStructuredVolume()->GetHeight(),
                                       m_ext_data_manager->GetCurrentStructuredVolume()->GetDepth() );

  glm::vec3 vol_voxelsize = glm::vec3(m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleX(),
                                      m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleY(),
                                      m_ext_data_manager->GetCurrentStructuredVolume()->GetScaleZ());

  glm::vec3 vol_aabb = vol_resolution * vol_voxelsize;

  // - load shaders
  cp_shader_rendering = new gl::ComputeShader();
  cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/_common_shaders/ray_bbox_intersection.comp");
  cp_shader_rendering->AddShaderFile(CPPVOLREND_DIR"structured/rc1piso/ray_marching_1p_iso.comp");
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

  cp_shader_rendering->BindUniforms();
  cp_shader_rendering->Unbind();
  gl::ExitOnGLError("RayCasting1PassIso: Error on Preparing Models and Shaders");


  /////////////////////////////////
  // Finalization

  //Support for multisampling
  Reshape(swidth, sheight);

  SetBuilt(true);
  SetOutdated();
  return true;
}


void RayCasting1PassIso::ReloadShaders()
{
  cp_shader_rendering->Reload();
  m_rdr_frame_to_screen.ClearShaders();
}


bool RayCasting1PassIso::Update(vis::Camera* camera)
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
  camera->SetRadius(cam_radius);
  camera->UpdatePositionAndRotations();

  cp_shader_rendering->SetUniform("CameraEye", camera->GetEye());
  cp_shader_rendering->BindUniform("CameraEye");

  cp_shader_rendering->SetUniform("u_CameraLookAt", camera->LookAt());
  cp_shader_rendering->BindUniform("u_CameraLookAt");

  cp_shader_rendering->SetUniform("ProjectionMatrix", camera->Projection());
  cp_shader_rendering->BindUniform("ProjectionMatrix");

  cp_shader_rendering->SetUniform("u_TanCameraFovY", (float)tan(DEGREE_TO_RADIANS(camera->GetFovY()) / 2.0));
  cp_shader_rendering->BindUniform("u_TanCameraFovY");

  cp_shader_rendering->SetUniform("u_CameraAspectRatio", camera->GetAspectRatio());
  cp_shader_rendering->BindUniform("u_CameraAspectRatio");

  cp_shader_rendering->SetUniform("WorldEyePos", camera->GetEye());
  cp_shader_rendering->BindUniform("WorldEyePos");

  /////////////////////////////
  // Isosurface aspects
  cp_shader_rendering->SetUniform("Isovalue", m_u_isovalue);
  cp_shader_rendering->SetUniform("StepSize", m_u_step_size);
  cp_shader_rendering->SetUniform("Color", m_u_color);

  cp_shader_rendering->SetUniform("ApplyGradientPhongShading", (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture()) ? 1 : 0);
  cp_shader_rendering->BindUniform("ApplyGradientPhongShading");

  cp_shader_rendering->SetUniform("BlinnPhongKa", m_ext_rendering_parameters->GetBlinnPhongKambient());
  cp_shader_rendering->BindUniform("BlinnPhongKa");
  cp_shader_rendering->SetUniform("BlinnPhongKd", m_ext_rendering_parameters->GetBlinnPhongKdiffuse());
  cp_shader_rendering->BindUniform("BlinnPhongKd");
  cp_shader_rendering->SetUniform("BlinnPhongKs", m_ext_rendering_parameters->GetBlinnPhongKspecular());
  cp_shader_rendering->BindUniform("BlinnPhongKs");
  cp_shader_rendering->SetUniform("BlinnPhongShininess", m_ext_rendering_parameters->GetBlinnPhongNshininess());
  cp_shader_rendering->BindUniform("BlinnPhongShininess");

  cp_shader_rendering->SetUniform("BlinnPhongIspecular", m_ext_rendering_parameters->GetLightSourceSpecular());
  cp_shader_rendering->BindUniform("BlinnPhongIspecular");

  cp_shader_rendering->SetUniform("LightSourcePosition", m_ext_rendering_parameters->GetBlinnPhongLightingPosition());
  cp_shader_rendering->BindUniform("LightSourcePosition");

  cp_shader_rendering->BindUniforms();

  gl::Shader::Unbind();
  gl::ExitOnGLError("RayCasting1PassIso: After Update.");
  return true;
}


void RayCasting1PassIso::Redraw()
{
  m_rdr_frame_to_screen.ClearTexture();

  cp_shader_rendering->Bind();
  m_rdr_frame_to_screen.BindImageTexture();

  cp_shader_rendering->Dispatch();
  gl::ComputeShader::Unbind();
 
  m_rdr_frame_to_screen.Draw();
}

void RayCasting1PassIso::FillParameterSpace(ParameterSpace& pspace)
{
    pspace.ClearParameterDimensions();
    pspace.AddParameterDimension(new ParameterBool("ApplyGradientShading", &m_apply_gradient_shading));
    pspace.AddParameterDimension(new ParameterRangeFloat("CamRadius", &cam_radius, 50.0f, 650.0f, 100.0f));
    pspace.AddParameterDimension(new ParameterRangeFloat("Alpha", &m_u_color.a, 0.1f, 1.01f, 0.1f));
    pspace.AddParameterDimension(new ParameterRangeFloat("StepSize", &m_u_step_size, 0.2f, 2.01f, 0.2f));
}


void RayCasting1PassIso::SetImGuiComponents()
{
  ImGui::Separator();
  
  ImGui::Text("Isovalue: ");
  if (ImGui::DragFloat("###RayCasting1PassIsoUIIsovalue", &m_u_isovalue, 0.01f, 0.01f, 100.0f, "%.2f"))
  {
    m_u_isovalue = std::max(std::min(m_u_isovalue, 100.0f), 0.01f); //When entering with keyboard, ImGui does not take care of the min/max.
    SetOutdated();
  }
  
  if (ImGui::ColorEdit4("Color", &m_u_color[0]))
  {
      SetOutdated();
  }

  ImGui::Text("Step size: ");
  if (ImGui::DragFloat("###RayCasting1PassIsoUIStepSize", &m_u_step_size, 0.001f, 0.05f, 5.0f, "%.3f"))
  {
      m_u_step_size = std::max(std::min(m_u_step_size, 5.0f), 0.001f); //When entering with keyboard, ImGui does not take care of the min/max.
      SetOutdated();
  }

  //AddImGuiMultiSampleOptions();
  
  if (m_ext_data_manager->GetCurrentGradientTexture())
  {
    ImGui::Separator();
    if (ImGui::Checkbox("Apply Gradient Shading", &m_apply_gradient_shading))
    {
      // Delete current uniform
      cp_shader_rendering->ClearUniform("TexVolumeGradient");

      if (m_apply_gradient_shading && m_ext_data_manager->GetCurrentGradientTexture())
      {
        cp_shader_rendering->Bind();
        cp_shader_rendering->SetUniformTexture3D("TexVolumeGradient", m_ext_data_manager->GetCurrentGradientTexture()->GetTextureID(), 2);
        cp_shader_rendering->BindUniform("TexVolumeGradient");
        gl::ComputeShader::Unbind();
      }
      SetOutdated();
    }
    ImGui::Separator();
  }
}
