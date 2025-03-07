// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

/* Things to add
 1) Make apple heart-shaped
 2) Collisiion objects for branches/tree trunk
 3) stem-branch and apple-stem angle errors/warning
 4) Too strong pull-back force
 5) Restoring force for angle (stem)
 6) Starting gripper at another angle
 7) Different apple sizes
 8) Parameter storage
 9) Running static with random selection of parameters to generate data
 
 Things to check
 1) Joint angles on stem-branch connection
 2) Starting gripper at another angle
 */
class Donut
{
	enum
	{
        n_body_ids = 5,
        n_gripper_ids = 2,
        n_joint_ids = 7,
        n_sensor_ids = 2,
        
        tree_id = 0,
        stiff_branch_id = 1,
        bendy_branch_id = 2,
        stem_id = 3,
        apple_id = 4,
        
        // Gripper
        palm_id = 0,
        arm_id = 1,
        
        // Joints
        tree_stiff_branch_id = 0,
        stiff_branch_bendy_branch_id = 1,
        bendy_branch_stem_id = 2,
        stem_apple_id = 3,
        arm_palm_left_id = 4,
        arm_palm_right_id = 5,
        apple_palm_id = 6,
        
        // Sensors
        apple_collision_id = 0,
        arm_collision_id = 1
	};

    enum CollisionBits
    {
        BRANCH_TREE = 0x00000001,
        GRIPPER = 0x00000002,
        STEM_APPLE = 0x00000004,
        SENSOR = 0x00000008,

        ALL_BITS = ( ~0u )
    };

    enum
    {
        stabalize_controller = 1,
        pull_down_controller = 2,
        twist_controller = 4,
        keyboard_controller = 6
    };

    int m_controller;
    
    b2BodyId m_bodyIds[n_body_ids], m_gripperIds[n_gripper_ids];
    b2JointId m_jointIds[n_joint_ids];
    b2ShapeId m_sensorIds[n_sensor_ids];
    b2ShapeProxy m_appleProxy, m_armProxy, m_treeProxies[3];

    struct {
        float k_tree_stiff_branch = 5.0f;
        float k_stiff_bendy_branch = 3.0f;
        float perc_stiff_branch = 0.8f;
        float perc_bendy_branc = 0.9;
        float stem_length = 0.2;
        float apple_radius = 0.2;
        float gripper_orientation = 0.0;
        float k_arm_palm = 10.0f;
        float wrist_gap = 0.02;
    } m_settings;
    // Derived values
    float m_palm_length, m_finger_palm_width, m_finger_length;

    int m_n_time_steps_store, m_current_time_step;

    typedef struct TreeInfo {
        float apple_stem_connection;  // Force on the apple-stem connection
        float stem_branch_connection;  // Force on the apple-stem connection
        float arm_collide;            // Arm/gripper about to collide with branch(es) or tree
        float apple_collide;          // Apple is about to collide with the branch
        b2Vec2 stem_start, stem_end;
        float stem_branch_angle, stem_apple_angle;
    } TreeInfo;
    void SetSensorCollisions( TreeInfo &) const;

    
    TreeInfo *m_tree_info;
    void SetTreeInfo(TreeInfo &) const;
    void WriteTreeInfoHeader() const;
    void WriteTreeInfo(const TreeInfo &) const;
    
    typedef struct SpringInfo {
        b2Vec2 vec_force;
        float sp_length;
    } SpringInfo;
    
    typedef struct ArmInfo {
        b2Vec2 pos_arm;
        float rot_arm;
        SpringInfo left_spring, right_spring;
    } ArmInfo;
        
    ArmInfo *m_arm_info;
    void SetArmInfo(ArmInfo &);
    void WriteArmInfoHeader() const;
    void WriteArmInfo(const ArmInfo &) const;

    bool m_isSpawned;
    
    void SpawnGripper( b2WorldId worldId, int groupIndex, void* userData );
    void SpawnSensors( b2WorldId worldId, int groupIndex, void* userData );
    void StabilizeController();
    void PullDownController(bool b_dir);
    void TwistController(bool b_dir, float radius, float perc_spiral);

    void WriteFile() const;

public:
	Donut();

    // All values are between 0 and 1 - 0 is no errro, 1 is the apple will be applesauce
    void MoveGripper( b2Vec2 newOrigin );
    void CollectStats();
    void KeyCommand(int key);
    void RunController();

	void Spawn( b2WorldId worldId, b2Vec2 position, float scale, int groupIndex, void* userData );
	void Despawn();

};
