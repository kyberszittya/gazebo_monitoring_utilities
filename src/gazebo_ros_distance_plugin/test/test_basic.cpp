#include <gtest/gtest.h>

#include <chrono>

#include "../src/plugin_collision.hpp"

const int OBJECT_CNT = 1000;
const double UPDATE_DX = 0.01;

TEST(BasicTestDistance, TestBox2Box)
{
    gazebo::DistanceCalculation calc_plugin;
    // This is the dynamic object
    std::string name1("box_1");
    const Eigen::Vector3d size1(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr1(1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori1(0.0, 0.0, 0.0, 1.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addBoxCollisionMeshDynamic(name1, size1, tr1, ori1, center);
    // This is the target definition
    std::string name2("box_2");
    const Eigen::Vector3d size2(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr2(-1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori2(0.0, 0.0, 0.0, 1.0);
    calc_plugin.addBoxCollisionMeshTarget(name2, size2, tr2, ori2, center);
    // Before usage, let's setup
    calc_plugin.setupCollision();
    // Get simple distances
    auto start = std::chrono::high_resolution_clock::now();
    ASSERT_DOUBLE_EQ(1.0, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_1(-0.5, 1.0, 0.0);
    calc_plugin.updateObjectPose(name2, tr2_1, ori2);
    ASSERT_DOUBLE_EQ(0.5, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_2(0.0, 1.0, 0.0);
    calc_plugin.updateObjectPose(name2, tr2_2, ori2);
    ASSERT_DOUBLE_EQ(0.0, calc_plugin.getMinTargetDistance());
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ASSERT_LE(elapsed.count(), 1e-4);
}


TEST(BasicTestDistance, TestBox2Box1000)
{
    gazebo::DistanceCalculation calc_plugin;
    // Set target
    std::string name1("box_target");
    const Eigen::Vector3d size1(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr1(0.0, -1.0, 0.0);
    const Eigen::Quaterniond ori1(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addBoxCollisionMeshTarget(name1, size1, tr1, ori1, center);
    // Initialize 1000 boxes
    for (int i = 0; i < OBJECT_CNT; i++)
    {
        std::string name2("box_"+std::to_string(i));
        const Eigen::Vector3d size2(1.0, 1.0, 1.0);
        const Eigen::Vector3d tr2(i-(OBJECT_CNT/2), 1.0, 0.0);
        const Eigen::Quaterniond ori2(1.0, 0.0, 0.0, 0.0);
        calc_plugin.addBoxCollisionMeshDynamic(name2, size2, tr2, ori2, center);

    }
    // Setup plugin
    calc_plugin.setupCollision();
    auto start = std::chrono::high_resolution_clock::now();
    double distance = calc_plugin.getMinTargetDistance();
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ASSERT_LE(elapsed.count(), 1e-3);
    ASSERT_DOUBLE_EQ(1.0, distance);
}

TEST(BasicTestDistance, TestBox2BoxStatic500)
{
    gazebo::DistanceCalculation calc_plugin;
    // Set target
    std::string name1("box_target");
    const Eigen::Vector3d size1(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr1(0.0, -1.0, 0.0);
    const Eigen::Quaterniond ori1(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addBoxCollisionMeshTarget(name1, size1, tr1, ori1, center);
    // Initialize 1000 boxes
    for (int i = 0; i < OBJECT_CNT/2; i++)
    {
        std::string name2("box_"+std::to_string(i));
        const Eigen::Vector3d size2(1.0, 1.0, 1.0);
        const Eigen::Vector3d tr2(i-(OBJECT_CNT/4), 2.0, 0.0);
        const Eigen::Quaterniond ori2(1.0, 0.0, 0.0, 0.0);
        calc_plugin.addBoxCollisionMeshDynamic(name2, size2, tr2, ori2, center);
        // Static object
        std::string name_static("box_static_"+std::to_string(i));
        const Eigen::Vector3d tr_static(i-(OBJECT_CNT/4), 1.0, 0.0);
        const Eigen::Quaterniond ori_static(0.0, 0.0, 0.0, 1.0);
        calc_plugin.addBoxCollisionMeshStatic(name_static, size2, tr_static, ori_static, center);

    }
    // Setup plugin
    calc_plugin.setupCollision();
    auto start = std::chrono::high_resolution_clock::now();
    double distance = calc_plugin.getMinTargetDistance();
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ASSERT_LE(elapsed.count(), 1e-3);
    ASSERT_DOUBLE_EQ(1.0, distance);
}

TEST(BasicTestDistance, TestBox2Sphere)
{
    gazebo::DistanceCalculation calc_plugin;
    // This is the dynamic object
    std::string name1("sphere_1");
    const Eigen::Vector3d tr1(1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori1(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addSphereCollisionMeshDynamic(name1, 0.5, tr1, ori1, center);
    // This is the target definition
    std::string name2("box_1");
    const Eigen::Vector3d size2(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr2(-1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori2(1.0, 0.0, 0.0, 0.0);
    calc_plugin.addBoxCollisionMeshTarget(name2, size2, tr2, ori2, center);
    // Before usage, let's setup
    calc_plugin.setupCollision();
    // Get simple distances
    auto start = std::chrono::high_resolution_clock::now();
    ASSERT_DOUBLE_EQ(1.0, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_1(0.5, 1.0, 0.0);
    calc_plugin.updateObjectPose(name1, tr2_1, ori2);
    ASSERT_DOUBLE_EQ(0.5, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_2(0.0, 1.0, 0.0);
    calc_plugin.updateObjectPose(name2, tr2_2, ori2);
    ASSERT_DOUBLE_EQ(0.0, calc_plugin.getMinTargetDistance());
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ASSERT_LE(elapsed.count(), 1e-4);
}

TEST(BasicTestDistance, TestBox2Cylinder)
{
    gazebo::DistanceCalculation calc_plugin;
    // This is the dynamic object
    std::string name1("cylinder_1");
    const Eigen::Vector2d size(0.5, 1.0);
    const Eigen::Vector3d tr1(1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori1(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addCylinderCollisionMeshDynamic(name1, size, tr1, ori1, center);
    // This is the target definition
    std::string name2("box_1");
    const Eigen::Vector3d size2(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr2(-1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori2(1.0, 0.0, 0.0, 0.0);
    calc_plugin.addBoxCollisionMeshTarget(name2, size2, tr2, ori2, center);
    // Before usage, let's setup
    calc_plugin.setupCollision();
    // Get simple distances
    auto start = std::chrono::high_resolution_clock::now();
    ASSERT_DOUBLE_EQ(1.0, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_1(0.5, 1.0, 0.0);
    calc_plugin.updateObjectPose(name1, tr2_1, ori2);
    ASSERT_DOUBLE_EQ(0.5, calc_plugin.getMinTargetDistance());
    const Eigen::Vector3d tr2_2(0.0, 1.0, 0.0);
    calc_plugin.updateObjectPose(name2, tr2_2, ori2);
    ASSERT_DOUBLE_EQ(0.0, calc_plugin.getMinTargetDistance());
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    ASSERT_LE(elapsed.count(), 1e-4);
}

TEST(BasicTestDistance, TestBasicUpdate)
{
    gazebo::DistanceCalculation calc_plugin;
    // This is the dynamic object
    std::string name1("box_target");
    const Eigen::Vector3d size1(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr1(1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori1(1.0, 0.0, 0.0, 0.0);
    const Eigen::Vector3d center(0.0, 0.0, 0.5);
    calc_plugin.addBoxCollisionMeshTarget(name1, size1, tr1, ori1, center);
    // This is the target definition
    std::string name2("box_2");
    const Eigen::Vector3d size2(1.0, 1.0, 1.0);
    const Eigen::Vector3d tr2(-1.0, 1.0, 0.0);
    const Eigen::Quaterniond ori2(1.0, 0.0, 0.0, 0.0);
    calc_plugin.addBoxCollisionMeshDynamic(name2, size2, tr2, ori2, center);
    // Before usage, let's setup
    calc_plugin.setupCollision();
    // Update for a short period
    for (int i = 0; i < 1000; i++)
    {
        const Eigen::Vector3d tr_u(1.0+i*UPDATE_DX, 1.0, 0.0);
        calc_plugin.updateObjectPose(name1, tr_u, ori1);
        ASSERT_NEAR(1.0+i*UPDATE_DX, calc_plugin.getMinTargetDistance(), 1e-6);
    }
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}