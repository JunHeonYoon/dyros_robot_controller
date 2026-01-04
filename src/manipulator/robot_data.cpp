#include "dyros_robot_controller/manipulator/robot_data.h"

namespace drc
{
    namespace Manipulator
    {
        RobotData::RobotData(const std::string& urdf_path, 
                             const std::string& srdf_path, 
                             const std::string& packages_path)
        : urdf_path_(urdf_path), srdf_path_(srdf_path), packages_path_(packages_path)
        {
            if (std::filesystem::exists(urdf_path))
            {
                std::cout << "[dyros_robot_controller] URDF file loaded successfully!: " << urdf_path << std::endl;
            }
            else
            {
                std::cerr << "\033[31m" << "[dyros_robot_controller] URDF file does not exist! : " << "\033[0m" << urdf_path << "\033[0m" << std::endl;
                std::exit(EXIT_FAILURE);
            }

            pinocchio::urdf::buildModel(urdf_path, model_, /*verbose=*/false);
            data_ = pinocchio::Data(model_);

            if (std::filesystem::exists(packages_path) && std::filesystem::is_directory(packages_path))
            {
                std::cout << "[dyros_robot_controller] Package folder loaded successfully!: " << packages_path << std::endl;
                pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, packages_path);
            }
            else
            {
                std::clog << "\033[33m" << "[dyros_robot_controller] Packages folder does not exist! : " << "\033[0m" << packages_path << "\033[0m" << std::endl;
                std::clog << "\033[33m" << "[dyros_robot_controller] Collision model for the robot may not be perfect!" << "\033[0m" << std::endl;
                pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_);
            }

            geom_model_.addAllCollisionPairs();

            
            if (std::filesystem::exists(srdf_path))
            {
                std::cout << "[dyros_robot_controller] SRDF file loaded successfully!: " << srdf_path << std::endl;
                pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_path);
            }
            else
            {
                std::clog << "\033[33m" << "[dyros_robot_controller] SRDF file does not exist! : " << "\033[0m" << srdf_path << "\033[0m" << std::endl;
                std::clog << "\033[33m" << "[dyros_robot_controller] All the collision pairs is activated!" << "\033[0m" << std::endl;
            }

            geom_data_ = pinocchio::GeometryData(geom_model_);
            
            dof_ = model_.joints.size() - 1; // except world joint

            // Initialize joint space state
            q_.setZero(dof_);
            qdot_.setZero(dof_);

            // Set joint state limit 
            q_lb_ = model_.lowerPositionLimit;
            q_ub_ = model_.upperPositionLimit;
            qdot_lb_ = -model_.velocityLimit;
            qdot_ub_ = model_.velocityLimit;

            // Initialize joint space dynamics
            M_.setZero(dof_,dof_);
            M_inv_.setZero(dof_,dof_);
            g_.setZero(dof_);
            c_.setZero(dof_);
            NLE_.setZero(dof_);

            link_frame_names_.clear();
            joint_frame_names_.clear();
            link_frame_set_.clear();
            joint_frame_set_.clear();
            root_link_name_.clear();

            // Collect frames
            for (pinocchio::FrameIndex i = 0; i < model_.frames.size(); ++i)
            {
                const auto& f = model_.frames[i];

                // LINK frames (URDF <link>)
                if (f.type == pinocchio::FrameType::BODY && f.name != "universe")
                {
                    link_frame_names_.push_back(f.name);
                    link_frame_set_.insert(f.name);

                    // Root link: BODY directly attached to universe joint
                    if (f.parentJoint == 0 && root_link_name_.empty())
                        root_link_name_ = f.name;
                }

                // JOINT frames (URDF <joint>)
                else if (f.type == pinocchio::FrameType::JOINT)
                {
                    joint_frame_names_.push_back(f.name);
                    joint_frame_set_.insert(f.name);
                }
            }

            if (link_frame_names_.empty()) throw std::runtime_error("RobotData: no BODY frames found in model.");

            // Fallback root link
            if (root_link_name_.empty()) root_link_name_ = link_frame_names_.front();
        }

        std::string RobotData::getVerbose() const
        {
            std::ostringstream oss;
            oss << "Total nq = " << model_.nq << '\n'
                << "Total nv = " << model_.nv << "\n\n";
            oss << " id | name                 | nq | nv | idx_q | idx_v\n";
            oss << "----+----------------------+----+----+-------+------\n";
            for (pinocchio::JointIndex id = 1; id < model_.joints.size(); ++id)
            {
                oss << std::setw(3)  << id << " | "
                    << std::setw(20) << model_.names[id] << " | "
                    << std::setw(2)  << model_.nqs[id]   << " | "
                    << std::setw(2)  << model_.nvs[id]   << " | "
                    << std::setw(5)  << model_.idx_qs[id]<< " | "
                    << std::setw(4)  << model_.idx_vs[id]<< '\n';
            }
            return oss.str();
        }

        bool RobotData::updateState(const VectorXd& q, const VectorXd& qdot)
        {
            q_ = q;
            qdot_ = qdot;
        
            if(!updateKinematics(q_, qdot_)) return false;
            if(!updateDynamics(q_, qdot_)) return false;
            return true;
        }

        bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
        {
            pinocchio::computeJointJacobians(model_, data_, q);
            pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        
            return true;
        }
        
        bool RobotData::updateDynamics(const VectorXd& q, const VectorXd& qdot)
        {
            pinocchio::crba(model_, data_, q);
            pinocchio::computeGeneralizedGravity(model_, data_, q);
            pinocchio::nonLinearEffects(model_, data_, q, qdot);
        
            // update joint space dynamics
            M_ = data_.M;
            M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
            M_inv_ = DyrosMath::PinvCOD(M_);
            g_ = data_.g;
            NLE_ = data_.nle;
            c_ = NLE_ - g_;
        
            return true;
        }

        // ================================ Compute Functions ================================
        // Joint space  
        MatrixXd RobotData::computeMassMatrix(const VectorXd& q)
        {      
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::crba(model_, data, q);
            data.M = data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
        
            return data.M;
        }
        
        VectorXd RobotData::computeGravity(const VectorXd& q)
        {      
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::computeGeneralizedGravity(model_, data, q);
            
            return data.g;
        }
        
        VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
        { 
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::computeCoriolisMatrix(model_, data, q, qdot);
        
            return data.C * qdot;
        }
        
        VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
        {
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::nonLinearEffects(model_, data, q, qdot);
        
            return data.nle;
        }
        
        // Task space
        Affine3d RobotData::computePose(const VectorXd& q, const std::string& link_name)
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return Affine3d::Identity();
            }
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::framesForwardKinematics(model_, data, q);
            Affine3d link_pose;
            link_pose.matrix() = data.oMf[link_index].toHomogeneousMatrix();
        
            return link_pose;
        }
        
        MatrixXd RobotData::computeJacobian(const VectorXd& q, const std::string& link_name)
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return MatrixXd::Zero(6,dof_);
            }
            MatrixXd J;
            J.setZero(6,dof_);
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::computeJointJacobians(model_, data, q);
            pinocchio::computeFrameJacobian(model_, data, q, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
        
            return J;
        }
        
        MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return MatrixXd::Zero(6,dof_);
            }
            MatrixXd Jdot;
            Jdot.setZero(6,dof_);
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
            pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
        
            return Jdot;
        }
        
        VectorXd RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
        {
            MatrixXd J = computeJacobian(q, link_name);
            
            return J * qdot;
        }
        
        MinDistResult RobotData::computeMinDistance(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const bool verbose)
        {
            MinDistResult result;
            result.setZero(q.size());
        
            pinocchio::Data data = pinocchio::Data(model_);
            pinocchio::GeometryData geom_data = pinocchio::GeometryData(geom_model_);
        
            pinocchio::computeDistances(model_, data, geom_model_, geom_data, q);
        
            double minDistance = std::numeric_limits<double>::max();
            int    minPairIdx  = -1;
        
            for (std::size_t idx = 0; idx < geom_data.distanceResults.size(); ++idx)
            {
                const auto &res = geom_data.distanceResults[idx];
                if (res.min_distance < minDistance)
                {
                    minDistance = res.min_distance;
                    minPairIdx  = static_cast<int>(idx);
                }
            }
            result.distance = minDistance;
        
            if (minPairIdx >= 0 && verbose)
            {
                const auto &pair   = geom_model_.collisionPairs[minPairIdx];
                const std::string &link1 =
                    geom_model_.geometryObjects[pair.first].name;
                const std::string &link2 =
                    geom_model_.geometryObjects[pair.second].name;
        
                std::cout << "[RobotDataBase] Closest links: " << link1
                        << "  <->  " << link2
                        << "   |  distance = " << minDistance << " [m]\n";
            }
        
            if(with_grad || with_graddot)
            {
                pinocchio::computeJointJacobians(model_, data, q);
                pinocchio::updateGeometryPlacements(model_, data, geom_model_, geom_data, q);
        
                const auto &pair  = geom_model_.collisionPairs[minPairIdx];
                const int geomA = pair.first,  geomB = pair.second;
                const int jointA = geom_model_.geometryObjects[geomA].parentJoint;
                const int jointB = geom_model_.geometryObjects[geomB].parentJoint;
        
                // Witness points & normal (world frame)
                const auto &res = geom_data.distanceResults[minPairIdx];
                const Vector3d pA = res.nearest_points[0];
                const Vector3d pB = res.nearest_points[1];
                const Vector3d n  = (pB - pA).normalized();
        
                // Joint-space 6 × total_dof Jacobians for the two parent joints
                MatrixXd J_jointA = MatrixXd::Zero(6, q.size());
                MatrixXd J_jointB = MatrixXd::Zero(6, q.size());
                pinocchio::getJointJacobian(model_, data, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA);
                pinocchio::getJointJacobian(model_, data, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB);
        
                // r = point - joint-origin (world)
                const Vector3d rA = pA - data.oMi[jointA].translation();
                const Vector3d rB = pB - data.oMi[jointB].translation();
        
                // Linear part of point Jacobian: Jp = Jv + ω×r
                auto skew = [](const Vector3d &v)->Matrix3d{
                            return (Matrix3d() <<   0, -v.z(),  v.y(),
                                                v.z(),      0, -v.x(),
                                            -v.y(),  v.x(),     0).finished(); };
        
                const MatrixXd JA = J_jointA.topRows<3>() - skew(rA) * J_jointA.bottomRows<3>();
                const MatrixXd JB = J_jointB.topRows<3>() - skew(rB) * J_jointB.bottomRows<3>();
        
                // d/dq (‖pB - pA‖) = nᵀ (J_B - J_A)
                result.grad =( n.transpose() * (JB - JA)).transpose();   // total_dof × 1
                if(minDistance < 0) result.grad *= -1.;
        
                if(with_graddot)
                {
                    pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        
                    MatrixXd J_jointA_dot = MatrixXd::Zero(6, q.size());
                    MatrixXd J_jointB_dot = MatrixXd::Zero(6, q.size());
                    pinocchio::getJointJacobianTimeVariation(model_, data, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA_dot);
                    pinocchio::getJointJacobianTimeVariation(model_, data, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB_dot);
        
                    const Vector3d pA_dot = JA * qdot;
                    const Vector3d pB_dot = JB * qdot;
        
                    const Vector3d rA_dot = pA_dot - J_jointA.topRows<3>() * qdot;
                    const Vector3d rB_dot = pB_dot - J_jointB.topRows<3>() * qdot;
        
                    const MatrixXd JA_dot = J_jointA_dot.topRows<3>() - (skew(rA_dot) * J_jointA.bottomRows<3>() + skew(rA) * J_jointA_dot.bottomRows<3>());
                    const MatrixXd JB_dot = J_jointB_dot.topRows<3>() - (skew(rB_dot) * J_jointB.bottomRows<3>() + skew(rB) * J_jointB_dot.bottomRows<3>());
        
                    result.grad_dot = (n.transpose() * (JB_dot - JA_dot)).transpose(); // neglect n_dot term
                }
            }
        
            return result;  
        }
        
        ManipulabilityResult RobotData::computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const std::string& link_name)
        {
            ManipulabilityResult result;
            result.setZero(q.size());
        
            MatrixXd J = computeJacobian(q, link_name);
            result.manipulability = sqrt((J*J.transpose()).determinant());
        
            if(with_grad || with_graddot)
            {
                pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
                if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
                {
                    std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                    return result;
                }
                
                MatrixXd JJt = J*J.transpose();
                MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
                std::vector<MatrixXd> dJ_dq;
                dJ_dq.resize(dof_);
                pinocchio::Data data = pinocchio::Data(model_);
                
                for(size_t i=0; i<dof_; ++i)
                {
                    VectorXd qdot_i = VectorXd::Zero(dof_);
                    qdot_i[i] = 1.0;
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot_i);
                    dJ_dq[i].setZero(6,dof_);
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
        
                    result.grad(i) = result.manipulability * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                }
        
                if(with_graddot)
                {
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
                    MatrixXd Jdot;
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
                    double mani_dot = result.manipulability * (Jdot * J.transpose() * JJt_inv).trace();
        
                    MatrixXd JJt_dot = 2 * Jdot* J.transpose();
                    MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);
        
                    for(size_t i=0; i<dof_; ++i)
                    {
                        result.grad_dot[i] = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                        result.grad_dot[i] += result.manipulability * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                            dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
                    }
                }
            }
        
            return result;
        }

        // ================================ Get Functions ================================
        bool RobotData::hasLinkFrame(const std::string& name) const
        {
            return link_frame_set_.count(name) > 0;
        }

        bool RobotData::hasJointFrame(const std::string& name) const
        {
            return joint_frame_set_.count(name) > 0;
        }

        // Task space  
        Affine3d RobotData::getPose(const std::string& link_name) const
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return Affine3d::Identity();
            }
            Affine3d link_pose;
            link_pose.matrix() = data_.oMf[link_index].toHomogeneousMatrix();
        
            return link_pose;
        }
        
        MatrixXd RobotData::getJacobian(const std::string& link_name)
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return MatrixXd::Zero(6,dof_);
            }
        
            return pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
        }
        
        MatrixXd RobotData::getJacobianTimeVariation(const std::string& link_name)
        {
            pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
            if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
            {
                std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                return MatrixXd::Zero(6,dof_);
            }
            MatrixXd Jdot;
            Jdot.setZero(6,dof_);
            pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
        
            return Jdot;
        }
        
        VectorXd RobotData::getVelocity(const std::string& link_name)
        {
            return getJacobian(link_name) * qdot_;
        }

        MinDistResult RobotData::getMinDistance(const bool& with_grad, const bool& with_graddot, const bool verbose)
        {
            MinDistResult result;
            result.setZero(q_.size());
        
            pinocchio::computeDistances(model_, data_, geom_model_, geom_data_, q_);
        
            double minDistance = std::numeric_limits<double>::max();
            int    minPairIdx  = -1;
        
            for (std::size_t idx = 0; idx < geom_data_.distanceResults.size(); ++idx)
            {
                const auto &res = geom_data_.distanceResults[idx];
                if (res.min_distance < minDistance)
                {
                    minDistance = res.min_distance;
                    minPairIdx  = static_cast<int>(idx);
                }
            }
            result.distance = minDistance;
        
            if (minPairIdx >= 0 && verbose)
            {
                const auto &pair   = geom_model_.collisionPairs[minPairIdx];
                const std::string &link1 =
                    geom_model_.geometryObjects[pair.first].name;
                const std::string &link2 =
                    geom_model_.geometryObjects[pair.second].name;
        
                std::cout << "[RobotDataBase] Closest links: " << link1
                        << "  <->  " << link2
                        << "   |  distance = " << minDistance << " [m]\n";
            }
        
            if(with_grad || with_graddot)
            {
                pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_, q_);
        
                const auto &pair  = geom_model_.collisionPairs[minPairIdx];
                const int   geomA = pair.first,  geomB = pair.second;
                const int   jointA = geom_model_.geometryObjects[geomA].parentJoint;
                const int   jointB = geom_model_.geometryObjects[geomB].parentJoint;
        
                // Witness points & normal (world frame)
                const auto &res = geom_data_.distanceResults[minPairIdx];
                const Vector3d pA = res.nearest_points[0];
                const Vector3d pB = res.nearest_points[1];
                const Vector3d n  = (pB - pA).normalized();
        
                // Joint-space 6 × total_dof Jacobians for the two parent joints
                MatrixXd J_jointA = MatrixXd::Zero(6, q_.size());
                MatrixXd J_jointB = MatrixXd::Zero(6, q_.size());
                pinocchio::getJointJacobian(model_, data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA);
                pinocchio::getJointJacobian(model_, data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB);
        
                // r = point - joint-origin (world)
                const Vector3d rA = pA - data_.oMi[jointA].translation();
                const Vector3d rB = pB - data_.oMi[jointB].translation();
        
                // Linear part of point Jacobian: Jp = Jv + ω×r
                auto skew = [](const Vector3d &v)->Matrix3d{
                            return (Matrix3d() <<   0, -v.z(),  v.y(),
                                                v.z(),      0, -v.x(),
                                            -v.y(),  v.x(),     0).finished(); };
        
                const MatrixXd JA = J_jointA.topRows<3>() - skew(rA) * J_jointA.bottomRows<3>();
                const MatrixXd JB = J_jointB.topRows<3>() - skew(rB) * J_jointB.bottomRows<3>();
        
                // d/dq (‖pB - pA‖) = nᵀ (J_B - J_A)
                result.grad =( n.transpose() * (JB - JA)).transpose();   // total_dof × 1
                if(minDistance < 0) result.grad *= -1.;
        
                if(with_graddot)
                {
                    MatrixXd J_jointA_dot = MatrixXd::Zero(6, q_.size());
                    MatrixXd J_jointB_dot = MatrixXd::Zero(6, q_.size());
                    pinocchio::getJointJacobianTimeVariation(model_, data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA_dot);
                    pinocchio::getJointJacobianTimeVariation(model_, data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB_dot);
        
                    const Vector3d pA_dot = JA * qdot_;
                    const Vector3d pB_dot = JB * qdot_;
        
                    const Vector3d rA_dot = pA_dot - J_jointA.topRows<3>() * qdot_;
                    const Vector3d rB_dot = pB_dot - J_jointB.topRows<3>() * qdot_;
        
                    const MatrixXd JA_dot = J_jointA_dot.topRows<3>() - (skew(rA_dot) * J_jointA.bottomRows<3>() + skew(rA) * J_jointA_dot.bottomRows<3>());
                    const MatrixXd JB_dot = J_jointB_dot.topRows<3>() - (skew(rB_dot) * J_jointB.bottomRows<3>() + skew(rB) * J_jointB_dot.bottomRows<3>());
        
                    result.grad_dot = (n.transpose() * (JB_dot - JA_dot)).transpose(); // neglect n_dot term
                }
            }
        
            return result;     
        }
    
        ManipulabilityResult RobotData::getManipulability(const bool& with_grad, const bool& with_graddot, const std::string& link_name)
        {
            ManipulabilityResult result;
            result.setZero(dof_);
        
            MatrixXd J = getJacobian(link_name);
            MatrixXd JJt = J*J.transpose();
            result.manipulability = sqrt((JJt).determinant());
        
        
            if(with_grad || with_graddot)
            {
                pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
                if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
                {
                    std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
                    return result;
                }
        
                
                MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
                std::vector<MatrixXd> dJ_dq;
                dJ_dq.resize(dof_);
                pinocchio::Data data = pinocchio::Data(model_);
                
                for(size_t i=0; i<dof_; ++i)
                {
                    VectorXd qdot_i = VectorXd::Zero(dof_);
                    qdot_i[i] = 1.0;
                    pinocchio::computeJointJacobiansTimeVariation(model_, data, q_, qdot_i);
                    dJ_dq[i].setZero(6,dof_);
                    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
                    
                    result.grad(i) = result.manipulability * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                }
        
        
                if(with_graddot)
                {
                    MatrixXd Jdot = getJacobianTimeVariation(link_name);
                    double mani_dot = result.manipulability * (Jdot * J.transpose() * JJt_inv).trace();
        
                    MatrixXd JJt_dot = 2 * Jdot* J.transpose();
                    MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);
        
                    for(size_t i=0; i<dof_; ++i)
                    {
                        result.grad_dot[i] = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                        result.grad_dot[i] += result.manipulability * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                            dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
                    }
                }
            }
            return result;
        }
    } // namespace Manipulator
} // namespace drc