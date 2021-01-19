#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::max;
using std::min;
using std::swap;

namespace CGL {

// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement MirrorBSDF
  reflect(wo, wi);
  *pdf = 1.0;
  return this->reflectance / abs_cos_theta(*wi);
}

// Microfacet BSDF //

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
  return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: proj3-2, part 3
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  //return std::pow(cos_theta(h), 100.0);
  double theta_h = acos(h.z);
  double alpha_squared = pow(alpha, 2.0);
  double num = exp(-pow(tan(theta_h), 2.0) / alpha_squared);
  double denom = PI * alpha_squared * pow(h.z, 4.0);
  return num / denom;
}

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  Spectrum eta_k = (eta * eta) + (k * k);
  Vector3D eta_cos = 2 * eta * wi.z;
  double cos_sq = pow(wi.z, 2.0);
  Spectrum R_s = (eta_k - eta_cos + cos_sq) / (eta_k + eta_cos + cos_sq);
  Spectrum R_p = (eta_k * cos_sq - eta_cos + 1) / (eta_k * cos_sq + eta_cos + 1);
  return (R_s + R_p) / 2.0;

}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 3
  // Implement microfacet model here.
  if (wo.z <= 0 | wi.z <= 0) {
      return Spectrum();
  }
   Vector3D h = wo + wi;
   h.normalize();
   Spectrum num = F(wi) * G(wo, wi) * D(h);
   Vector3D denom = 4 * cos_theta(wo) * cos_theta(wi);
   return num / denom;
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: proj3-2, part 3
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.
  /**
  *wi = cosineHemisphereSampler.get_sample(pdf); //placeholder
  return MicrofacetBSDF::f(wo, *wi); */

  Vector2D sample = this->sampler.get_sample();
  double r1 = sample.x;
  double r2 = sample.y;
  double alpha_sqr = pow(alpha, 2.0);
  double inside_sqr_root = -alpha_sqr * log(1.0 - r1);
  //Don't want to sqr_root a neg num
  if (inside_sqr_root < 0) {
        *pdf = 0;
        return Spectrum();
  }
  double theta_h = atan(sqrt(inside_sqr_root));
  double phi_h = 2 * PI * r2;
  double sin_theta = sin(theta_h);
  //Don't want to divide by 0
  if (sin_theta == 0) {
      *pdf = 0;
      return Spectrum();
  }
  Vector3D h = Vector3D(sin_theta * cos(phi_h), sin_theta * sin(phi_h), cos(theta_h));
  *wi = 2 * dot(h, wo) * h - wo;
  //Check if sampled wi is valid
  if (wi->z <= 0) {
      *pdf = 0;
      return Spectrum();
  }
  double denom_t = alpha_sqr * pow(cos(theta_h), 3.0) * exp(pow(tan(theta_h), 2.0) / alpha_sqr);
  //Don't want to divide by 0
  if (denom_t == 0) {
      *pdf = 0;
      return Spectrum();
  }
  double pdf_theta = (2 * sin_theta) / denom_t;
  double pdf_phi = 1.0 / (2.0 * PI);
  double pdf_h = (pdf_theta * pdf_phi) / sin_theta;
  double denom_w = 4 * dot(*wi, h);
  //Don't want to divide by 0
  if (denom_w == 0) {
      *pdf = 0;
      return Spectrum();
  }
  *pdf = pdf_h / denom_w;
  return MicrofacetBSDF::f(wo, *wi);
}

// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Implement RefractionBSDF
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO:
  // Compute Fresnel coefficient and use it as the probability of reflection
  // - Fundamentals of Computer Graphics page 305
  if (!refract(wo, wi, this->ior)) {
      reflect(wo, wi);
      *pdf = 1.0;
      return this->reflectance / abs_cos_theta(*wi);
  } else {
      float r_o = (1.f - this->ior) / (1.f + this->ior);
      r_o = powf(r_o, 2.f);
      double temp = 1.0 - abs_cos_theta(wo);
      float r = r_o + (1.f - r_o) * pow(temp, 5.0);
      if (coin_flip(r)) {
          reflect(wo, wi);
          *pdf = r;
          return (r * this->reflectance) / abs_cos_theta(*wi);
      } else {
          *pdf = 1 - r;
          float n = 1.f / ior;
          if (wo.z < 0) {
              n = ior;
          }
          return (*pdf * this->transmittance) / abs_cos_theta(*wi) * powf(n, 2.f);
      }
  }
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
  // TODO:
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
    *wi = Vector3D(-wo.x, -wo.y, wo.z);
}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
  // TODO:
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
  // ray entering the surface through vacuum.
    int sign = -1;
    float n = 1.f / ior;
    if (wo.z < 0) {
        n = ior;
        sign = 1;
    }
    double t_int_reflec = 1.f - (powf(n, 2.f) * (1.0 - pow(wo.z, 2.0)));
    if (t_int_reflec < 0) {
        return false;
    }
    *wi = Vector3D(-n * wo.x, -n * wo.y, sign * sqrt(t_int_reflec));
    return true;
}

} // namespace CGL
