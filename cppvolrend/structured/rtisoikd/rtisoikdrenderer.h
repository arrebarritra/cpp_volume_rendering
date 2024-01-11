/**
 * Isosurface Raytracer using implicit k-d trees
 * 
 * . Implementation of 2005 paper:
 * . Faster isosurface ray tracing using implicit kd-trees
 * . Ingo Wald, Heiko Friedrich, Gerd Marmitt, Philipp Slusallek and Hans-Peter Seidel
 * . IEEE Transactions on Visualization and Computer Graphics 11.5 (2005): 562-572.
 * . Link: https://ieeexplore.ieee.org/document/1471693
 * . DOI: 10.1109/TVCG.2005.79
 * 
 * @author Aritra Bhakat
 * . aritra@kth.se
**/
#pragma once

#include "../../volrenderbase.h"
#include "implkdtreegenerator.h"

class RaytracingIsoImplKD : public BaseVolumeRenderer
{
public:
	RaytracingIsoImplKD();
  virtual ~RaytracingIsoImplKD();
  
  virtual const char* GetName();
  virtual const char* GetAbbreviationName();
  virtual vis::GRID_VOLUME_DATA_TYPE GetDataTypeSupport();

  virtual void Clean();
  virtual bool Init(int shader_width, int shader_height);
  virtual void ReloadShaders();

  virtual bool Update(vis::Camera* camera);
  virtual void Redraw();

  virtual void FillParameterSpace(ParameterSpace& pspace) override;

  virtual void SetImGuiComponents();

protected:
  float m_u_isovalue;

  glm::vec4 m_u_color; // RGBA, float
  bool m_apply_gradient_shading;

private:
	gl::ComputeShader* cp_shader_impl_kd_tree;
	gl::ComputeShader* cp_shader_rendering;

	ImplicitKDTreeGenerator kdg;
};

