#ifndef MFAC_HPP
#define MFAC_HPP

#include "rclcpp/rclcpp.hpp"
#include <yaml-cpp/yaml.h>
#include <Eigen/Eigen>
#include <memory>

struct MFACParam{
   MFACParam(const std::string& MFAC_params_file){
      YAML::Node config = YAML::LoadFile(MFAC_params_file);
      lambda = config["lambda"].as<float>();
      rho = config["rho"].as<float>();
      mu = config["mu"].as<float>();
      eta = config["eta"].as<float>();
      uk_limit = config["uk_limit"].as<float>();
      phi.fill(0);
      phi(0,0) = 10;
      phi(1,1) = 10;
      yk.fill(0);
      yk_1.fill(0);
      yk_e.fill(0);
      yk_d.fill(0);
      uk.fill(0);
      uk_1.fill(0);
      uk_d.fill(0);

   };

   ~MFACParam() = default;

   float lambda ;
   float rho ;
   float mu ;
   float eta ;
   float uk_limit ;

   Eigen::Matrix<float, 2, 2> phi;

   Eigen::Vector2f yk,yk_1,yk_e,yk_d;
   Eigen::Vector2f uk,uk_1,uk_d;

};

class MFAC : public rclcpp::Node{
 public:
    explicit MFAC(std::string name);
    ~MFAC() = default;

    Eigen::Vector2f operator()(const Eigen::Vector2f& y_e_,
                               const Eigen::Vector2f& y_k_,
                               const std::shared_ptr<MFACParam>& MFAC);
    std::shared_ptr<MFACParam> getMFAC() const { return MFAC_Param_ptr; };
    Eigen::Vector2f MFACLimit(const Eigen::Vector2f& MFAC_OUT, float LimitValue);
    
 private:

    void UpdatePhi(const std::shared_ptr<MFACParam>& MFAC);
    void UpdateUk(const std::shared_ptr<MFACParam>& MFAC);
    void UpdateParams(const Eigen::Vector2f& y_e_,
                      const Eigen::Vector2f& y_k_,
                      const std::shared_ptr<MFACParam>& MFAC);

    std::shared_ptr<MFACParam> MFAC_Param_ptr;
};

#endif