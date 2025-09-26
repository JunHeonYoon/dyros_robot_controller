#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>
#include <Eigen/Dense>
#include <numpy/ndarrayobject.h>
#include <numpy/arrayobject.h>

#include "dyros_robot_controller/type_define.h"
#include "dyros_robot_controller/mobile/robot_data.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/mobile_manipulator/robot_data.h"

#include "dyros_robot_controller/mobile/robot_controller.h"
#include "dyros_robot_controller/manipulator/robot_controller.h"
#include "dyros_robot_controller/mobile_manipulator/robot_controller.h"

namespace bp = boost::python;
using namespace drc;

typedef Mobile::RobotData                  MO_RD;
typedef Manipulator::RobotData             MN_RD;
typedef MobileManipulator::RobotData       MM_RD;
typedef Mobile::RobotController            MO_RC;
typedef Manipulator::RobotController       MN_RC;
typedef MobileManipulator::RobotController MM_RC;

namespace
{
    bp::tuple MM_RC_QPIK_tuple(MM_RC& self, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIK(xdot_target, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }
    
    bp::tuple MM_RC_QPIKStep_tuple(MM_RC& self, const Affine3d& x_target, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIKStep(x_target, xdot_target, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }
    
    bp::tuple MM_RC_QPIKCubic_tuple(MM_RC& self, 
                                    const Affine3d& x_target,
                                    const VectorXd& xdot_target,
                                    const Affine3d& x_init,
                                    const VectorXd& xdot_init,
                                    const double& current_time,
                                    const double& init_time,
                                    const double& duration,
                                    const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIKCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }

    bp::tuple MM_RC_QPID_tuple(MM_RC& self, const VectorXd& xddot_target, const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPID(xddot_target, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
    
    bp::tuple MM_RC_QPIDStep_tuple(MM_RC& self, const Affine3d& x_target, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPIDStep(x_target, xdot_target, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
    
    bp::tuple MM_RC_QPIDCubic_tuple(MM_RC& self, 
                                    const Affine3d& x_target,
                                    const VectorXd& xdot_target,
                                    const Affine3d& x_init,
                                    const VectorXd& xdot_init,
                                    const double& current_time,
                                    const double& init_time,
                                    const double& duration,
                                    const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPIDCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
}

struct PairVectorXdToPython
{
    static PyObject* convert(const std::pair<VectorXd, VectorXd>& p)
    {
        bp::object first_obj(p.first);
        bp::object second_obj(p.second);
        bp::tuple t = bp::make_tuple(first_obj, second_obj);
        return bp::incref(t.ptr());
    }
};

struct Affine3dToPython
{
  static PyObject* convert(const Affine3d& T)
  {
    const Matrix4d& M = T.matrix();
    bp::object mat(M);
    return bp::incref(mat.ptr());
  }
};

struct VecDoubleToPython
{
    static PyObject *convert(const std::vector<double> &v)
    {
        bp::list l;
        for (double d : v) l.append(d);
        return bp::incref(l.ptr());
    }
};

struct VecDoubleFromPython
{
    VecDoubleFromPython()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::vector<double>>());
    }

    static void *convertible(PyObject *obj_ptr)
    {
        return PySequence_Check(obj_ptr) ? obj_ptr : nullptr;
    }

    static void construct(PyObject *obj_ptr, bp::converter::rvalue_from_python_stage1_data *data)
    {
        void *storage = ((bp::converter::rvalue_from_python_storage<std::vector<double>> *)data)->storage.bytes;
        new (storage) std::vector<double>();

        auto *vec = static_cast<std::vector<double> *>(storage);
        const Py_ssize_t len = PySequence_Size(obj_ptr);
        vec->reserve(len);

        for (Py_ssize_t i = 0; i < len; ++i)
        {
            bp::object item(bp::handle<>(PySequence_GetItem(obj_ptr, i)));
            vec->push_back(bp::extract<double>(item));
        }
        data->convertible = storage;
    }
};

struct Vec2dToPython
{
    static PyObject *convert(const std::vector<Vector2d> &v)
    {
        bp::list l;
        for (const auto &e : v) l.append(bp::object(e));
        return bp::incref(l.ptr());
    }
};

struct Vec2dFromPython
{
    Vec2dFromPython()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::vector<Vector2d>>());
    }

    static void *convertible(PyObject *obj_ptr)
    { 
        return PySequence_Check(obj_ptr) ? obj_ptr : nullptr; 
    }

    static void construct(PyObject *obj_ptr, bp::converter::rvalue_from_python_stage1_data *data)
    {
        void *storage = ((bp::converter::rvalue_from_python_storage<std::vector<Vector2d>> *)data)->storage.bytes;
        new (storage) std::vector<Vector2d>();
        auto *vec = static_cast<std::vector<Vector2d> *>(storage);
        const Py_ssize_t len = PySequence_Size(obj_ptr);
        vec->reserve(len);
        for (Py_ssize_t i = 0; i < len; ++i)
        {
            bp::object item(bp::handle<>(PySequence_GetItem(obj_ptr, i)));
            vec->emplace_back(bp::extract<Vector2d>(item));
        }
        data->convertible = storage;
    }
};

struct Affine3dFromNumpy
{
    Affine3dFromNumpy()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<Affine3d>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PyArray_Check(obj_ptr))                            return nullptr;
        auto* arr = reinterpret_cast<PyArrayObject*>(obj_ptr);
        if (PyArray_NDIM(arr) != 2)                             return nullptr;
        if (PyArray_DIM(arr,0) != 4 || PyArray_DIM(arr,1) != 4) return nullptr;
        if (PyArray_TYPE(arr) != NPY_DOUBLE)                    return nullptr;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, bp::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((bp::converter::rvalue_from_python_storage<Affine3d>*)data)->storage.bytes;
        double* buf = reinterpret_cast<double*>(PyArray_DATA((PyArrayObject*)obj_ptr));
        Map<Matrix<double,4,4,RowMajor>> M(buf);
        new (storage) Affine3d(M);
        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(dyros_robot_controller_cpp_wrapper)
{
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<Matrix<double, Dynamic, Dynamic>>();
    eigenpy::enableEigenPySpecific<Matrix<double, Dynamic, 1>>();
    eigenpy::enableEigenPySpecific<Matrix4d>();

    bp::to_python_converter<std::pair<VectorXd, VectorXd>, PairVectorXdToPython>();
    bp::to_python_converter<Affine3d, Affine3dToPython>();
    bp::to_python_converter<std::vector<double>, VecDoubleToPython>();
    bp::to_python_converter<std::vector<Vector2d>, Vec2dToPython>();
    
    static VecDoubleFromPython _reg_vecdouble_from_python;
    static Vec2dFromPython     _reg_vec2d_from_python;
    static Affine3dFromNumpy   _reg_affine3d_from_numpy;

    bp::enum_<Mobile::DriveType>("DriveType")
        .value("Differential", Mobile::DriveType::Differential)
        .value("Mecanum",      Mobile::DriveType::Mecanum)
        .value("Caster",       Mobile::DriveType::Caster)
        .export_values();

    bp::class_<Mobile::KinematicParam>("KinematicParam")
        .def(bp::init<>())
        .def_readwrite("type",                  &Mobile::KinematicParam::type)
        .def_readwrite("wheel_radius",          &Mobile::KinematicParam::wheel_radius)
        .def_readwrite("max_lin_speed",         &Mobile::KinematicParam::max_lin_speed)
        .def_readwrite("max_ang_speed",         &Mobile::KinematicParam::max_ang_speed)
        .def_readwrite("max_lin_acc",           &Mobile::KinematicParam::max_lin_acc)
        .def_readwrite("max_ang_acc",           &Mobile::KinematicParam::max_ang_acc)
        .def_readwrite("base_width",            &Mobile::KinematicParam::base_width)
        .def_readwrite("roller_angles",         &Mobile::KinematicParam::roller_angles)
        .def_readwrite("base2wheel_positions",  &Mobile::KinematicParam::base2wheel_positions)
        .def_readwrite("base2wheel_angles",     &Mobile::KinematicParam::base2wheel_angles)
        .def_readwrite("wheel_offset",          &Mobile::KinematicParam::wheel_offset);

    bp::class_<Manipulator::MinDistResult>("MinDistResult")
        .def(bp::init<>())
        .def_readwrite("distance", &Manipulator::MinDistResult::distance)
        .def_readwrite("grad",     &Manipulator::MinDistResult::grad)
        .def_readwrite("grad_dot", &Manipulator::MinDistResult::grad_dot)
        .def("setZero",            &Manipulator::MinDistResult::setZero);

    bp::class_<Manipulator::ManipulabilityResult>("ManipulabilityResult")
        .def(bp::init<>())
        .def_readwrite("manipulability", &Manipulator::ManipulabilityResult::manipulability)
        .def_readwrite("grad",           &Manipulator::ManipulabilityResult::grad)
        .def_readwrite("grad_dot",       &Manipulator::ManipulabilityResult::grad_dot)
        .def("setZero",                  &Manipulator::ManipulabilityResult::setZero);

    bp::class_<MobileManipulator::JointIndex>("JointIndex")
        .def(bp::init<>())
        .def_readwrite("virtual_start", &MobileManipulator::JointIndex::virtual_start)
        .def_readwrite("mani_start",    &MobileManipulator::JointIndex::mani_start)
        .def_readwrite("mobi_start",    &MobileManipulator::JointIndex::mobi_start);

    bp::class_<MobileManipulator::ActuatorIndex>("ActuatorIndex")
        .def(bp::init<>())
        .def_readwrite("mani_start", &MobileManipulator::ActuatorIndex::mani_start)
        .def_readwrite("mobi_start", &MobileManipulator::ActuatorIndex::mobi_start);

    bp::class_<MO_RD, boost::noncopyable>("MobileRobotData", bp::init<const Mobile::KinematicParam&>())
        .def("getVerbose",        &MO_RD::getVerbose)
        .def("updateState",       &MO_RD::updateState)
        .def("computeBaseVel",    &MO_RD::computeBaseVel)
        .def("computeFKJacobian", &MO_RD::computeFKJacobian)
        .def("getWheelNum",       &MO_RD::getWheelNum, bp::return_value_policy<bp::return_by_value>())
        .def("getKineParam",      &MO_RD::getKineParam,     bp::return_internal_reference<>())
        .def("getWheelPosition",  &MO_RD::getWheelPosition, bp::return_internal_reference<>())
        .def("getWheelVelocity",  &MO_RD::getWheelVelocity, bp::return_internal_reference<>())
        .def("getBaseVel",        &MO_RD::getBaseVel,       bp::return_internal_reference<>())
        .def("getFKJacobian",     &MO_RD::getFKJacobian,    bp::return_internal_reference<>())
        ;

    bp::class_<MN_RD, boost::noncopyable>("ManipulatorRobotData", bp::init<const std::string&, const std::string&, const std::string&>())
        .def("getVerbose",                   &MN_RD::getVerbose)
        .def("updateState",                  &MN_RD::updateState)
        .def("computeMassMatrix",            &MN_RD::computeMassMatrix)
        .def("computeGravity",               &MN_RD::computeGravity)
        .def("computeCoriolis",              &MN_RD::computeCoriolis)
        .def("computeNonlinearEffects",      &MN_RD::computeNonlinearEffects)
        .def("computePose",                  &MN_RD::computePose)
        .def("computeJacobian",              &MN_RD::computeJacobian)
        .def("computeJacobianTimeVariation", &MN_RD::computeJacobianTimeVariation)
        .def("computeVelocity",              &MN_RD::computeVelocity)
        .def("computeMinDistance",           &MN_RD::computeMinDistance)
        .def("computeManipulability",        &MN_RD::computeManipulability)
        .def("getDof",                       &MN_RD::getDof)
        .def("getJointPosition",             &MN_RD::getJointPosition)
        .def("getJointVelocity",             &MN_RD::getJointVelocity)
        .def("getJointPositionLimit",        &MN_RD::getJointPositionLimit)
        .def("getJointVelocityLimit",        &MN_RD::getJointVelocityLimit)
        .def("getMassMatrix",                &MN_RD::getMassMatrix)
        .def("getMassMatrixInv",             &MN_RD::getMassMatrixInv)
        .def("getCoriolis",                  &MN_RD::getCoriolis)
        .def("getGravity",                   &MN_RD::getGravity)
        .def("getNonlinearEffects",          &MN_RD::getNonlinearEffects)
        .def("getPose",                      &MN_RD::getPose)
        .def("getJacobian",                  &MN_RD::getJacobian)
        .def("getJacobianTimeVariation",     &MN_RD::getJacobianTimeVariation)
        .def("getVelocity",                  &MN_RD::getVelocity)
        .def("getMinDistance",               &MN_RD::getMinDistance)
        .def("getManipulability",            &MN_RD::getManipulability);

    typedef bool (MM_RD::*Upd6)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&);
    typedef MatrixXd (MM_RD::*Mat3)(const VectorXd&, const VectorXd&, const VectorXd&);
    typedef VectorXd (MM_RD::*Vec3)(const VectorXd&, const VectorXd&, const VectorXd&);
    typedef VectorXd (MM_RD::*Vec6)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&);
    typedef Affine3d (MM_RD::*Aff4)(const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef MatrixXd (MM_RD::*Mat4)(const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef MatrixXd (MM_RD::*Mat7)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (MM_RD::*Vec7)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef Manipulator::MinDistResult (MM_RD::*Min9)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const bool&, const bool&, const bool);
    typedef Manipulator::ManipulabilityResult (MM_RD::*Man5)(const VectorXd&, const VectorXd&, const bool&, const bool&, const std::string&);

    bp::class_<MM_RD, bp::bases<MN_RD, MO_RD>, boost::noncopyable>("MobileManipulatorRobotData", bp::init<const Mobile::KinematicParam&, const std::string&, const std::string&, const std::string&, const MobileManipulator::JointIndex&, const MobileManipulator::ActuatorIndex&>())
        .def("getVerbose",                                     &MM_RD::getVerbose)
        .def("updateState",                  static_cast<Upd6>(&MM_RD::updateState))
        .def("computeMassMatrix",            static_cast<Mat3>(&MM_RD::computeMassMatrix))
        .def("computeGravity",               static_cast<Vec3>(&MM_RD::computeGravity))
        .def("computeCoriolis",              static_cast<Vec6>(&MM_RD::computeCoriolis))
        .def("computeNonlinearEffects",      static_cast<Vec6>(&MM_RD::computeNonlinearEffects))
        .def("computeMassMatrixActuated",                      &MM_RD::computeMassMatrixActuated)
        .def("computeGravityActuated",                         &MM_RD::computeGravityActuated)
        .def("computeCoriolisActuated",                        &MM_RD::computeCoriolisActuated)
        .def("computeNonlinearEffectsActuated",                &MM_RD::computeNonlinearEffectsActuated)
        .def("computePose",                  static_cast<Aff4>(&MM_RD::computePose))
        .def("computeJacobian",              static_cast<Mat4>(&MM_RD::computeJacobian))
        .def("computeJacobianTimeVariation", static_cast<Mat7>(&MM_RD::computeJacobianTimeVariation))
        .def("computeVelocity",              static_cast<Vec7>(&MM_RD::computeVelocity))
        .def("computeMinDistance",                        Min9(&MM_RD::computeMinDistance))
        .def("computeSelectionMatrix",                         &MM_RD::computeSelectionMatrix)
        .def("computeJacobianActuated",                        &MM_RD::computeJacobianActuated)
        .def("computeJacobianTimeVariationActuated",           &MM_RD::computeJacobianTimeVariationActuated)
        .def("computeManipulability",                     Man5(&MM_RD::computeManipulability))
        .def("computeMobileFKJacobian",                        &MM_RD::computeMobileFKJacobian)
        .def("computeMobileBaseVel",                           &MM_RD::computeMobileBaseVel)
        .def("getActuatordDof",                                &MM_RD::getActuatordDof)
        .def("getManipulatorDof",                              &MM_RD::getManipulatorDof)
        .def("getMobileDof",                                   &MM_RD::getMobileDof)
        .def("getJointIndex",                                  &MM_RD::getJointIndex)
        .def("getActuatorIndex",                               &MM_RD::getActuatorIndex)
        .def("getMobileJointPosition",                         &MM_RD::getMobileJointPosition)
        .def("getVirtualJointPosition",                        &MM_RD::getVirtualJointPosition)
        .def("getManiJointPosition",                           &MM_RD::getManiJointPosition)
        .def("getJointVelocityActuated",                       &MM_RD::getJointVelocityActuated)
        .def("getMobileJointVelocity",                         &MM_RD::getMobileJointVelocity)
        .def("getVirtualJointVelocity",                        &MM_RD::getVirtualJointVelocity)
        .def("getManiJointVelocity",                           &MM_RD::getManiJointVelocity)
        .def("getJointPositionActuated",                       &MM_RD::getJointPositionActuated)
        .def("getMassMatrixActuated",                          &MM_RD::getMassMatrixActuated)
        .def("getMassMatrixActuatedInv",                       &MM_RD::getMassMatrixActuatedInv)
        .def("getGravityActuated",                             &MM_RD::getGravityActuated)
        .def("getCoriolisActuated",                            &MM_RD::getCoriolisActuated)
        .def("getNonlinearEffectsActuated",                    &MM_RD::getNonlinearEffectsActuated)
        .def("getJacobianActuated",                            &MM_RD::getJacobianActuated)
        .def("getJacobianActuatedTimeVariation",               &MM_RD::getJacobianActuatedTimeVariation)
        .def("getSelectionMatrix",                             &MM_RD::getSelectionMatrix)
        .def("getManipulability",                              &MM_RD::getManipulability)
        .def("getMobileFKJacobian",                            &MM_RD::getMobileFKJacobian)
        .def("getMobileBaseVel",                               &MM_RD::getMobileBaseVel)
        ;

    bp::class_<MO_RC, boost::noncopyable >("MobileRobotController", bp::init<const double&, std::shared_ptr< MO_RD > >())
        .def("computeWheelVel",   &MO_RC::computeWheelVel)
        .def("computeIKJacobian", &MO_RC::computeIKJacobian)
        .def("VelocityCommand",   &MO_RC::VelocityCommand);
    
    typedef VectorXd (MN_RC::*CLIKStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*CLIKStep2)(const Affine3d&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*CLIKCub1)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*CLIKCub2)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const std::string&);
    typedef VectorXd (MN_RC::*OSF1)(const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*OSF2)(const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*OSFStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*OSFStep2)(const Affine3d&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*OSFCub1)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const VectorXd&, const std::string&);
    typedef VectorXd (MN_RC::*OSFCub2)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const std::string&);

    bp::class_<MN_RC, boost::noncopyable >("ManipulatorRobotController", bp::init<const double&, std::shared_ptr<MN_RD>>())
        .def("setJointGain",                                                                               &MN_RC::setJointGain)
        .def("setTaskGain",                                                                                &MN_RC::setTaskGain)
        .def("moveJointPositionCubic",                                                                     &MN_RC::moveJointPositionCubic)
        .def("moveJointVelocityCubic",                                                                     &MN_RC::moveJointVelocityCubic)
        .def("moveJointTorqueStep",                      static_cast<VectorXd (MN_RC::*)(const VectorXd&)>(&MN_RC::moveJointTorqueStep))
        .def("moveJointTorqueStep",     static_cast<VectorXd (MN_RC::*)(const VectorXd&, const VectorXd&)>(&MN_RC::moveJointTorqueStep))
        .def("moveJointTorqueCubic",                                                                       &MN_RC::moveJointTorqueCubic)
        .def("CLIKStep",                                                            static_cast<CLIKStep1>(&MN_RC::CLIKStep))
        .def("CLIKStep",                                                            static_cast<CLIKStep2>(&MN_RC::CLIKStep))
        .def("CLIKCubic",                                                            static_cast<CLIKCub1>(&MN_RC::CLIKCubic))
        .def("CLIKCubic",                                                            static_cast<CLIKCub2>(&MN_RC::CLIKCubic))
        .def("OSF",                                                                      static_cast<OSF1>(&MN_RC::OSF))
        .def("OSF",                                                                      static_cast<OSF2>(&MN_RC::OSF))
        .def("OSFStep",                                                              static_cast<OSFStep1>(&MN_RC::OSFStep))
        .def("OSFStep",                                                              static_cast<OSFStep2>(&MN_RC::OSFStep))
        .def("OSFCubic",                                                              static_cast<OSFCub1>(&MN_RC::OSFCubic))
        .def("OSFCubic",                                                              static_cast<OSFCub2>(&MN_RC::OSFCubic))
        .def("QPIK",                                                                                       &MN_RC::QPIK)
        .def("QPIKStep",                                                                                   &MN_RC::QPIKStep)
        .def("QPIKCubic",                                                                                  &MN_RC::QPIKCubic)
        .def("QPID",                                                                                       &MN_RC::QPID)
        .def("QPIDStep",                                                                                   &MN_RC::QPIDStep)
        .def("QPIDCubic",                                                                                  &MN_RC::QPIDCubic)
        ;

    typedef VectorXd (MN_RC::*CLIKStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);

    bp::class_<MM_RC, boost::noncopyable>("MobileManipulatorRobotController", bp::init<const double&, std::shared_ptr<MM_RD>>())
        .def("setManipulatorJointGain",                                                                                &MM_RC::setManipulatorJointGain)
        .def("setTaskGain",                                                                                            &MM_RC::setTaskGain)
        .def("moveManipulatorJointPositionCubic",                                                                      &MM_RC::moveManipulatorJointPositionCubic)
        .def("moveManipulatorJointTorqueStep",                        static_cast<VectorXd(MM_RC::*)(const VectorXd&)>(&MM_RC::moveManipulatorJointTorqueStep))
        .def("moveManipulatorJointTorqueStep",       static_cast<VectorXd(MM_RC::*)(const VectorXd&, const VectorXd&)>(&MM_RC::moveManipulatorJointTorqueStep))
        .def("moveManipulatorJointTorqueCubic",                                                                        &MM_RC::moveManipulatorJointTorqueCubic)
        .def("QPIK",                                                                                                   &MM_RC_QPIK_tuple)
        .def("QPIKStep",                                                                                               &MM_RC_QPIKStep_tuple)
        .def("QPIKCubic",                                                                                              &MM_RC_QPIKCubic_tuple)
        .def("QPID",                                                                                                   &MM_RC_QPID_tuple)
        .def("QPIDStep",                                                                                               &MM_RC_QPIDStep_tuple)
        .def("QPIDCubic",                                                                                              &MM_RC_QPIDCubic_tuple)
        ;
}