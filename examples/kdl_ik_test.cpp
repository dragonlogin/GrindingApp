/**
 * @file kdl_ik_test.cpp
 * @brief KDL Forward/Inverse Kinematics Test - ABB IRB2400
 */

#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;

void PrintJointArray(const KDL::JntArray& q, const std::string& label = "Joint Angles") {
    std::cout << "\n=== " << label << " ===" << std::endl;
    for (unsigned int i = 0; i < q.rows(); ++i) {
        std::cout << "  J" << (i + 1) << ": " << q(i) * kRadToDeg << " deg" << std::endl;
    }
}

void PrintFrame(const KDL::Frame& frame, const std::string& label) {
    std::cout << "\n=== " << label << " ===" << std::endl;
    std::cout << "Position (mm): " 
              << frame.p.x() * 1000 << ", " 
              << frame.p.y() * 1000 << ", " 
              << frame.p.z() * 1000 << std::endl;
    double r, p, y;
    frame.M.GetRPY(r, p, y);
    std::cout << "Orientation (RPY, deg): " 
              << "Roll=" << r * kRadToDeg << ", "
              << "Pitch=" << p * kRadToDeg << ", "
              << "Yaw=" << y * kRadToDeg << std::endl;
}
} // namespace

KDL::Chain BuildIRB2400Chain() {
    KDL::Chain chain;
    chain.addSegment(KDL::Segment("base_to_j1", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0))));
    chain.addSegment(KDL::Segment("joint_1",
        KDL::Joint("joint_1", KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 1), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("j1_to_j2", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.1, 0, 0.615))));
    chain.addSegment(KDL::Segment("joint_2",
        KDL::Joint("joint_2", KDL::Vector(0, 0, 0), KDL::Vector(0, 1, 0), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("j2_to_j3", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0, 0, 0.705))));
    chain.addSegment(KDL::Segment("joint_3",
        KDL::Joint("joint_3", KDL::Vector(0, 0, 0), KDL::Vector(0, 1, 0), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("j3_to_j4", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.258, 0, 0.135))));
    chain.addSegment(KDL::Segment("joint_4",
        KDL::Joint("joint_4", KDL::Vector(0, 0, 0), KDL::Vector(1, 0, 0), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("j4_to_j5", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.497, 0, 0))));
    chain.addSegment(KDL::Segment("joint_5",
        KDL::Joint("joint_5", KDL::Vector(0, 0, 0), KDL::Vector(0, 1, 0), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("j5_to_j6", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.085, 0, 0))));
    chain.addSegment(KDL::Segment("joint_6",
        KDL::Joint("joint_6", KDL::Vector(0, 0, 0), KDL::Vector(1, 0, 0), KDL::Joint::RotAxis),
        KDL::Frame::Identity()));
    chain.addSegment(KDL::Segment("tool0", KDL::Joint(KDL::Joint::Fixed),
        KDL::Frame(KDL::Rotation::RPY(0, M_PI/2, 0), KDL::Vector(0, 0, 0))));
    return chain;
}

int main() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "ABB IRB2400 KDL FK/IK Test" << std::endl;
    std::cout << "========================================" << std::endl;

    KDL::Chain chain = BuildIRB2400Chain();
    std::cout << "\nChain Info:" << std::endl;
    std::cout << "  Joints: " << chain.getNrOfJoints() << std::endl;
    std::cout << "  Segments: " << chain.getNrOfSegments() << std::endl;

    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);
    KDL::ChainIkSolverPos_NR_JL ik_solver(chain, fk_solver, ik_vel_solver, 100, 1e-6);

    KDL::JntArray q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints());
    q_min(0) = -3.1416; q_max(0) = 3.1416;
    q_min(1) = -1.7453; q_max(1) = 1.9199;
    q_min(2) = -1.0472; q_max(2) = 1.1345;
    q_min(3) = -3.49;   q_max(3) = 3.49;
    q_min(4) = -2.0944; q_max(4) = 2.0944;
    q_min(5) = -6.9813; q_max(5) = 6.9813;

    // Test 1: Zero Position
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "Test 1: Zero Position (all joints 0 deg)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    KDL::JntArray q_zero(chain.getNrOfJoints());
    for (unsigned int i = 0; i < q_zero.rows(); ++i) q_zero(i) = 0.0;

    KDL::Frame fk_result;
    fk_solver.JntToCart(q_zero, fk_result);
    PrintJointArray(q_zero, "Joint Angles");
    PrintFrame(fk_result, "FK Result");

    KDL::JntArray q_ik(chain.getNrOfJoints());
    int ik_result = ik_solver.CartToJnt(q_zero, fk_result, q_ik);
    if (ik_result >= 0) {
        PrintJointArray(q_ik, "IK Result");
        KDL::Frame verify;
        fk_solver.JntToCart(q_ik, verify);
        double pos_error = std::sqrt(
            pow(verify.p.x() - fk_result.p.x(), 2) +
            pow(verify.p.y() - fk_result.p.y(), 2) +
            pow(verify.p.z() - fk_result.p.z(), 2)) * 1000;
        std::cout << "\nVerify - Position Error (mm): " << pos_error << std::endl;
    } else {
        std::cerr << "IK Failed!" << std::endl;
    }

    // Test 2: Non-zero Position
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "Test 2: Non-zero Position" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    KDL::JntArray q_test(chain.getNrOfJoints());
    q_test(0) = 30.0 * kDegToRad;
    q_test(1) = -45.0 * kDegToRad;
    q_test(2) = 30.0 * kDegToRad;
    q_test(3) = 0.0;
    q_test(4) = 45.0 * kDegToRad;
    q_test(5) = 0.0;

    PrintJointArray(q_test, "Joint Angles");
    fk_solver.JntToCart(q_test, fk_result);
    PrintFrame(fk_result, "FK Result");

    KDL::JntArray q_init(chain.getNrOfJoints());
    for (unsigned int i = 0; i < q_init.rows(); ++i) q_init(i) = 0.0;

    ik_result = ik_solver.CartToJnt(q_init, fk_result, q_ik);
    if (ik_result >= 0) {
        PrintJointArray(q_ik, "IK Result");
        std::cout << "\nJoint Diff (deg):" << std::endl;
        for (unsigned int i = 0; i < q_test.rows(); ++i) {
            double diff = (q_ik(i) - q_test(i)) * kRadToDeg;
            std::cout << "  J" << (i+1) << ": " << diff << " deg" << std::endl;
        }
        KDL::Frame verify;
        fk_solver.JntToCart(q_ik, verify);
        double pos_error = std::sqrt(
            pow(verify.p.x() - fk_result.p.x(), 2) +
            pow(verify.p.y() - fk_result.p.y(), 2) +
            pow(verify.p.z() - fk_result.p.z(), 2)) * 1000;
        std::cout << "\nVerify - Position Error (mm): " << pos_error << std::endl;
    } else {
        std::cerr << "IK Failed!" << std::endl;
    }

    // Test 3: Multiple Poses
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "Test 3: Multiple Poses Test" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    std::vector<std::vector<double>> test_angles = {
        {0.0, -30.0, -30.0, 0.0, 30.0, 0.0},
        {45.0, -60.0, 45.0, 0.0, -45.0, 0.0},
        {-30.0, -45.0, 30.0, 45.0, 30.0, -30.0},
        {90.0, -30.0, -60.0, 0.0, 60.0, 0.0},
    };

    int success_count = 0;
    for (const auto& angles : test_angles) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (int i = 0; i < 6; ++i) q(i) = angles[i] * kDegToRad;

        KDL::Frame target;
        fk_solver.JntToCart(q, target);

        KDL::JntArray q_out(chain.getNrOfJoints());
        int result = ik_solver.CartToJnt(q_init, target, q_out);

        if (result >= 0) {
            KDL::Frame verify;
            fk_solver.JntToCart(q_out, verify);
            double error = std::sqrt(
                pow(verify.p.x() - target.p.x(), 2) +
                pow(verify.p.y() - target.p.y(), 2) +
                pow(verify.p.z() - target.p.z(), 2)) * 1000;
            std::cout << "  [" << angles[0] << ", " << angles[1] << ", " 
                      << angles[2] << ", " << angles[3] << ", " 
                      << angles[4] << ", " << angles[5] << "] -> "
                      << "Error: " << error << " mm" << std::endl;
            ++success_count;
        } else {
            std::cout << "  [" << angles[0] << ", " << angles[1] << ", " 
                      << angles[2] << ", " << angles[3] << ", " 
                      << angles[4] << ", " << angles[5] << "] -> Failed" << std::endl;
        }
    }

    std::cout << "\nSuccess Rate: " << success_count << "/" << test_angles.size() << std::endl;
    std::cout << "\n=== Done ===" << std::endl;
    return 0;
}
