// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "donut.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

static std::ofstream outputFile("data.csv");
Donut::Donut()
{
    for ( int i = 0; i < n_body_ids; ++i )
    {
        m_bodyIds[i] = b2_nullBodyId;
    }

    for ( int i = 0; i < n_gripper_ids; ++i )
    {
        m_gripperIds[i] = b2_nullBodyId;
    }

    for ( int i = 0; i < n_joint_ids; ++i )
    {
        m_jointIds[i] = b2_nullJointId;
    }

    m_n_time_steps_store = 2000;
    m_tree_info = new TreeInfo[m_n_time_steps_store];
    m_arm_info = new ArmInfo[m_n_time_steps_store];

    m_current_time_step = 0;
	m_isSpawned = false;
    
    if (outputFile.is_open())
    {
        WriteTreeInfoHeader();
        WriteArmInfoHeader();
    }
    m_controller = -1;
}

void Donut::SetTreeInfo(TreeInfo &ti) const
{
    b2Vec2 vec_apple_stem = b2Joint_GetConstraintForce(m_jointIds[stem_apple_id]);
    ti.apple_stem_connection = pow(vec_apple_stem.x * vec_apple_stem.x + vec_apple_stem.y * vec_apple_stem.y, 2);
    
    b2Vec2 vec_branch_stem = b2Joint_GetConstraintForce(m_jointIds[bendy_branch_stem_id]);
    ti.stem_branch_angle = pow(vec_branch_stem.x * vec_branch_stem.x + vec_branch_stem.y * vec_branch_stem.y, 2);
    
    //ti.arm_collide = b2Shape_GetContactData(,
    //ti.apple_collide =
    b2Vec2 stem_start = b2Joint_GetLocalAnchorB(m_jointIds[bendy_branch_id]);
    b2Vec2 stem_end = b2Joint_GetLocalAnchorA(m_jointIds[stem_apple_id]);
    ti.stem_start = b2Body_GetWorldPoint(m_bodyIds[stem_id], stem_start);
    ti.stem_end = b2Body_GetWorldPoint(m_bodyIds[stem_id], stem_end);
    
    ti.stem_branch_angle = b2RevoluteJoint_GetAngle(m_jointIds[bendy_branch_stem_id]);
    ti.stem_apple_angle = b2RevoluteJoint_GetAngle(m_jointIds[stem_apple_id]);
}

void Donut::WriteTreeInfoHeader() const
{
    outputFile << "Apple_stem_force, Arm_collide, Apple_collide, ";
    outputFile << "Pos_Stem_start_x, Pos_Stem_start_y, ";
    outputFile << "Pos_Stem_end_x, Pos_Stem_end_y, ";
    outputFile << "Stem_branch_angle, Stem_apple_angle, ";
}

void Donut::WriteTreeInfo(const TreeInfo &ti) const
{
    outputFile << ti.apple_stem_connection << ", ";
    outputFile << ti.arm_collide << ", ";
    outputFile << ti.apple_collide << ", ";
    outputFile << ti.stem_start.x << ", " << ti.stem_start.y << ", ";
    outputFile << ti.stem_end.x << ", " << ti.stem_end.y << ", ";
    outputFile << ti.stem_branch_angle << ", " << ti.stem_apple_angle << ", ";
}

void Donut::SetArmInfo(ArmInfo &ai)
{
    ai.pos_arm = b2Body_GetPosition(m_gripperIds[arm_id]);
    b2Rot rot = b2Body_GetRotation(m_gripperIds[arm_id]);
    ai.rot_arm = atan2(rot.s, rot.c);
    ai.left_spring.vec_force = b2Joint_GetConstraintForce(m_jointIds[arm_palm_left_id]);
    ai.right_spring.vec_force = b2Joint_GetConstraintForce(m_jointIds[arm_palm_right_id]);
    ai.left_spring.sp_length = b2DistanceJoint_GetCurrentLength(m_jointIds[arm_palm_left_id]);
    ai.right_spring.sp_length = b2DistanceJoint_GetCurrentLength(m_jointIds[arm_palm_right_id]);
}

void Donut::WriteArmInfoHeader() const
{
    outputFile << "Pos_arm_x, Pos_arm_y, Rot_arm, ";
    outputFile << "Left_spring_force_x, Left_spring_force_y, Left_spring_len, ";
    outputFile << "Right_spring_force_x, Right_spring_force_y, Right_spring_len\n";
}

void Donut::WriteArmInfo(const ArmInfo &ai) const
{
    outputFile << ai.pos_arm.x << ", " << ai.pos_arm.y << ", " << ai.rot_arm << ", ";
    outputFile << ai.left_spring.vec_force.x << ", " << ai.left_spring.vec_force.y << ", ";
    outputFile << ai.left_spring.sp_length << ", ";
    outputFile << ai.right_spring.vec_force.x << ", " << ai.right_spring.vec_force.y << ", ";
    outputFile << ai.right_spring.sp_length << "\n";
}

void Donut::WriteFile() const
{
    if (not outputFile.is_open())
    {
        return;
    }
    
    for (int i = 0; i < m_current_time_step; ++i)
    {
        WriteTreeInfo(m_tree_info[i]);
        WriteArmInfo(m_arm_info[i]);
    }
}

void Donut::RunController()
{
    if (m_controller == stabalize_controller)
    {
        StabilizeController();
    }
    else if (m_controller == pull_down_controller || m_controller == pull_down_controller+1)
    {
        if ( m_controller == pull_down_controller) {
            PullDownController(true);
        }
        else
        {
            PullDownController(false);
        }
    }
    else if (m_controller == twist_controller || m_controller == twist_controller + 1)
    {
        if ( m_controller == twist_controller) {
            TwistController(true, 2.0 * m_settings.apple_radius + m_settings.stem_length, 1.5);
        }
        else
        {
            TwistController(false, 2.0 * m_settings.apple_radius + m_settings.stem_length, 0.8);
        }
    } else if ( m_controller != keyboard_controller)
    {
        float cur_angular_velocity = b2Body_GetAngularVelocity(m_gripperIds[arm_id]);
        b2Vec2 cur_linear_velocity = b2Body_GetLinearVelocity(m_gripperIds[arm_id]);
        float ang_rotate = 0.0;
        b2Vec2 vec_slow = {0.0f, 0.0f};
        b2Body_SetAngularVelocity( m_gripperIds[arm_id], 0.8f * cur_angular_velocity + 0.2f * ang_rotate );
        b2Body_SetLinearVelocity( m_gripperIds[arm_id], 0.8f * cur_linear_velocity + 0.2f * vec_slow);
    }
}

void Donut::SetSensorCollisions( TreeInfo &ti) const
{
    static b2SimplexCache s_cache = b2_emptySimplexCache;
    static bool s_use_cache = true;
    static const int s_n_simplices = n_gripper_ids + n_body_ids + n_sensor_ids + 10;
    static b2Simplex s_simplexes[s_n_simplices];

    ti.apple_collide = 1.0;
    ti.arm_collide = 1.0;
    float *collide[2];
    collide[0] = &ti.apple_collide;
    collide[1] = &ti.arm_collide;
    for (int which_sensor = 0; which_sensor < 2; ++which_sensor)
    {
        const int sensor_id = apple_collision_id + which_sensor;

        // Determine the necessary capacity
        int capacity = b2Shape_GetSensorCapacity( m_sensorIds[sensor_id] );
        std::vector<b2ShapeId> overlaps, shapes_obj_collide(1);
        overlaps.resize( capacity );
        int count = b2Shape_GetSensorOverlaps( m_sensorIds[sensor_id], overlaps.data(), capacity );

        for ( int i = 0; i < count; ++i )
        {
            b2ShapeId objId = overlaps[i];
            const b2BodyId whichBodyId = b2Shape_GetBody(objId);
            int which_proxy = -1;
            for (int j = 0; j < 3; ++j)
            {
                if (whichBodyId.index1 == m_bodyIds[tree_id + j].index1)
                {
                    which_proxy = j;
                }
            }
            if ( which_proxy == -1 )
                continue;
            
            b2DistanceInput input;
            input.proxyA = m_appleProxy;
            input.proxyB = m_treeProxies[which_proxy];
            input.transformA = b2Body_GetTransform(m_bodyIds[apple_id]);
            input.transformB = b2Body_GetTransform(whichBodyId);
            input.useRadii = false;
            
            b2DistanceOutput output = b2ShapeDistance( &s_cache, &input, s_simplexes, s_n_simplices );
            *collide[which_sensor] = fmin(ti.apple_collide, output.distance / 1.5f * m_settings.apple_radius);

            printf("Branch-stem collision which %d obj %d %0.2f\n", which_sensor, whichBodyId.index1, output.distance);
            

        }
    }
}

void Donut::CollectStats( )
{
    if (m_current_time_step == m_n_time_steps_store && m_n_time_steps_store > 0) {
        // Dump out the data
        WriteFile();
        
        m_current_time_step = 0;
    }
    
    if (m_current_time_step < m_n_time_steps_store && m_current_time_step >= 0)
    {
        SetTreeInfo(m_tree_info[m_current_time_step]);
        SetArmInfo(m_arm_info[m_current_time_step]);
        SetSensorCollisions(m_tree_info[m_current_time_step]);
                        
        m_current_time_step += 1;
    }
    
    if ( m_current_time_step == 1 )
    {
        printf("Force left %0.2f %0.f", m_arm_info[0].left_spring.vec_force.x, m_arm_info[0].left_spring.vec_force.y);
        printf("Force right %0.2f %0.f", m_arm_info[0].right_spring.vec_force.x, m_arm_info[0].right_spring.vec_force.y);
    }
}

void Donut::StabilizeController()
{
    b2Rot rot_palm = b2Body_GetRotation(m_gripperIds[arm_id]);
    
    // angle of spring should be perpendicular to palm
    b2Vec2 palm_left = b2Body_GetWorldPoint(m_gripperIds[palm_id], b2Joint_GetLocalAnchorA(m_jointIds[arm_palm_left_id]));
    b2Vec2 palm_right = b2Body_GetWorldPoint(m_gripperIds[palm_id], b2Joint_GetLocalAnchorA(m_jointIds[arm_palm_right_id]));
    b2Vec2 arm_left = b2Body_GetWorldPoint(m_gripperIds[arm_id], b2Joint_GetLocalAnchorB(m_jointIds[arm_palm_left_id]));
    b2Vec2 arm_right = b2Body_GetWorldPoint(m_gripperIds[arm_id], b2Joint_GetLocalAnchorB(m_jointIds[arm_palm_right_id]));

    b2Vec2 vec_left_spring = arm_left - palm_left;
    b2Vec2 vec_right_spring = arm_right - palm_right;
    float left_len_spring = b2Length(vec_left_spring);
    float right_len_spring = b2Length(vec_right_spring);

    // If spring lengths are not equal, rotate in the direction to make them equal
    float arm_palm_gap = b2DistanceJoint_GetLength(m_jointIds[arm_palm_left_id]);
    float arm_palm_gap2 = b2DistanceJoint_GetLength(m_jointIds[arm_palm_right_id]);
    float ang_rotate = 0.1 * (left_len_spring - right_len_spring) / arm_palm_gap;
    
    b2Vec2 f_left_norm = b2Normalize(vec_left_spring);
    b2Vec2 f_right_norm = b2Normalize(vec_right_spring);
    b2Vec2 palm_vec = {rot_palm.c, rot_palm.s};
    
    float dot_left = b2Dot(f_left_norm, palm_vec);
    float dot_right = b2Dot(f_right_norm, palm_vec);
    
    if ( fabs(dot_left + dot_right) > 0.1 )
    {
        // don't try to rotate if the ends are not aligned
        ang_rotate = 0.0;
    }
    
    b2Vec2 slide_sideways = palm_vec * -0.05 * (dot_left + dot_right);
    b2Vec2 palm_up_down = {-palm_vec.y, palm_vec.x};
    float amount_to_move = ((left_len_spring - arm_palm_gap) * 0.5f + (right_len_spring - arm_palm_gap) * 0.5f);
    b2Vec2 slide_up = amount_to_move * palm_up_down;

    float cur_angular_velocity = b2Body_GetAngularVelocity(m_gripperIds[arm_id]);
    b2Vec2 cur_linear_velocity = b2Body_GetLinearVelocity(m_gripperIds[arm_id]);
    b2Body_SetAngularVelocity( m_gripperIds[arm_id], 0.8f * cur_angular_velocity + 0.2f * ang_rotate );
    b2Body_SetLinearVelocity( m_gripperIds[arm_id], 0.8f * cur_linear_velocity + 0.2f * (slide_up + slide_sideways));
}

void Donut::PullDownController(bool pull_down)
{
    b2Rot rot_palm = b2Body_GetRotation(m_gripperIds[arm_id]);
    
    float f_pull = -0.05;
    if (pull_down == false)
    {
        f_pull *= -1.0;
    }
    b2Vec2 palm_up_down = {-f_pull * rot_palm.s, f_pull * rot_palm.c};

    float cur_angular_velocity = b2Body_GetAngularVelocity(m_gripperIds[arm_id]);
    b2Vec2 cur_linear_velocity = b2Body_GetLinearVelocity(m_gripperIds[arm_id]);
    
    // kill angular velocity
    if (cur_angular_velocity < 0.05)
    {
        cur_angular_velocity = 0.0;
    } else {
        cur_angular_velocity *= 0.8f;
    }
    b2Body_SetAngularVelocity( m_gripperIds[arm_id], cur_angular_velocity );
    b2Body_SetLinearVelocity( m_gripperIds[arm_id], 0.8f * cur_linear_velocity + 0.2f * palm_up_down);
}

void Donut::TwistController(bool clockwise, float radius, float perc_spiral)
{
    float vel_magnitude = 0.05;
    float ang_dir = perc_spiral * vel_magnitude / radius;
    
    if (clockwise) {
        ang_dir *= -1;
        vel_magnitude *= -1;
    }

    b2Rot rot_palm = b2Body_GetRotation(m_gripperIds[arm_id]);

    if ( abs(rot_palm.s) > abs(rot_palm.c) )
    {
        // Probably should scale down with amount over
        float f_slow_down = 1.0f - (abs(rot_palm.s) - abs(rot_palm.c)) / (m_settings.wrist_gap);
        if (f_slow_down < 0.0f)
        {
            f_slow_down = 0.0;
        }
        if ( rot_palm.s > 0 && !clockwise ) {
            ang_dir *= f_slow_down;
            vel_magnitude *= f_slow_down;
        }
        if ( rot_palm.s < 0 && clockwise ) {
            ang_dir *= f_slow_down;
            vel_magnitude *= f_slow_down;
        }
    }

    b2Vec2 palm_up_down = {vel_magnitude * rot_palm.c, vel_magnitude * rot_palm.s};

    float cur_angular_velocity = b2Body_GetAngularVelocity(m_gripperIds[arm_id]);
    b2Vec2 cur_linear_velocity = b2Body_GetLinearVelocity(m_gripperIds[arm_id]);

    b2Body_SetAngularVelocity( m_gripperIds[arm_id], 0.8f * cur_angular_velocity + 0.2f * ang_dir );
    b2Body_SetLinearVelocity( m_gripperIds[arm_id], 0.8f * cur_linear_velocity + 0.2f * palm_up_down);
}

void Donut::KeyCommand(int key)
{
    m_controller = key-48;
    if (m_controller == stabalize_controller)
    {
        printf("Switching to stabilize controller\n");
    }
    else if (m_controller == pull_down_controller || m_controller == pull_down_controller )
    {
        printf("Switching to pull down controller, up\n");
    }
    else if (m_controller == pull_down_controller +1 )
    {
        printf("Switching to pull down controller, down\n");
    }
    else if (m_controller == keyboard_controller)
    {
        printf("Keyboard controller\n");
    }
    else if (m_controller == twist_controller || m_controller == twist_controller+1)
    {
        printf("Twist controller {m_controller)\n");
    }
    else
    {
        printf("No controller\n");

    }

}


void Donut::MoveGripper( b2Vec2 shiftDir )
{
    static b2Vec2 lastDir = {0.0f, 0.0f};
    static float last_angular = 0.0f;

    m_controller = keyboard_controller;
    printf("Keyboard controller: ");
    if (shiftDir.x != shiftDir.y)
    {
        printf("Linear: ");
        b2Vec2 new_shiftDir = lastDir + shiftDir;
        
        b2Body_SetAngularVelocity( m_gripperIds[arm_id], 0.0f );
        b2Body_SetLinearVelocity( m_gripperIds[arm_id], new_shiftDir * 0.1 );

        lastDir = new_shiftDir;
    } else {
        printf("Angular: ");
        if ( shiftDir.x < 0.0 )
        {
            last_angular -= 0.1;
        } else {
            last_angular += 0.1;
        }
        b2Body_SetAngularVelocity( m_gripperIds[arm_id], last_angular );
        b2Body_SetLinearVelocity( m_gripperIds[arm_id], {0.0f, 0.0f} );
    }
    printf("Shift dir %0.2f, %0.2f,  angular %0.4f\n", lastDir.x, lastDir.y, last_angular);
    
}

void Donut::SpawnSensors( b2WorldId worldId, int groupIndex, void* userData )
{
    for ( int i = 0; i < n_sensor_ids; ++i )
    {
        assert( B2_IS_NULL(m_sensorIds[i]));
    }
    
    // Make a large circle and attach it to the apple
    
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.friction = 0.3f;
    shapeDef.isSensor = true;
    shapeDef.filter.categoryBits = SENSOR;
    shapeDef.filter.maskBits = BRANCH_TREE;

    b2Circle apple_collision = { { 0.0f, 0.0f }, 0.5f * m_settings.stem_length + m_settings.apple_radius };
    m_sensorIds[apple_collision_id] = b2CreateCircleShape( m_bodyIds[apple_id], &shapeDef, &apple_collision );
    
    float f_scale = 1.1f;
    b2Vec2 tri_vs[4];
    tri_vs[0].x = -f_scale * m_palm_length;
    tri_vs[0].y = -f_scale * m_finger_palm_width;
    tri_vs[1].x =  -1.0f * tri_vs[0].x;
    tri_vs[1].y = tri_vs[0].y;
    tri_vs[2].x = tri_vs[1].x - f_scale * 2.0f * m_finger_length * cos(B2_PI * 0.25f);
    tri_vs[2].y = tri_vs[1].y + f_scale * 2.0f * m_finger_length * sin(B2_PI * 0.25f);
    tri_vs[3].x = -tri_vs[2].x;
    tri_vs[3].y = tri_vs[2].y;
    b2Hull hull = b2ComputeHull( tri_vs, 4 );
    b2Polygon tri_poly = b2MakePolygon( &hull, 0.0f );

    m_sensorIds[arm_collision_id] = b2CreatePolygonShape(m_gripperIds[palm_id], &shapeDef, &tri_poly);
}

void Donut::SpawnGripper( b2WorldId worldId, int groupIndex, void* userData )
{
    for ( int i = 0; i < n_gripper_ids; ++i )
    {
        assert( B2_IS_NULL( m_gripperIds[i] ) );
    }
    b2Vec2 apple_position = b2Body_GetWorldPoint(m_bodyIds[apple_id], {0.0f, 0.0f});
    float x_apple_center = apple_position.x;
    float y_apple_bottom = apple_position.y - m_settings.apple_radius;
    m_finger_palm_width = m_settings.apple_radius * 0.1;
    
    float contact_angle = B2_PI/4.0;
    float x_loc = m_settings.apple_radius * cos(contact_angle);
    float y_loc = m_settings.apple_radius * sin(contact_angle);
    // y - mx = b,  m = -1,
    float m_slope = -y_loc / x_loc;
    float b_intercept = y_loc - (m_slope * x_loc);
    // y = mx + b  0 = m x + b  x = - b / m
    float x_width = - b_intercept / m_slope;
    m_palm_length = m_settings.apple_radius + x_width;
    b2Polygon palm_shape = b2MakeBox(m_palm_length, m_finger_palm_width);
    // distance from the contact point to the palm, plus a bit, cut in half
    m_finger_length = 0.5f * (1.2f * sqrt( pow(x_loc - m_palm_length, 2) + pow(y_loc, 2) ));
    b2Polygon finger_shape = b2MakeBox(m_finger_length, m_finger_palm_width);
    
    // Start of right finger
    b2BodyDef fingerDef_right = b2DefaultBodyDef();
    //fingerDef_right.type = b2_kinematicBody;
    //fingerDef_right.userData = userData;
    
    float x_palm_pos = x_apple_center;
    float y_palm_pos = y_apple_bottom;
    
    float x_pos_f_left = m_palm_length - m_finger_length * cos(contact_angle);
    float x_pos_f_right = m_palm_length + m_finger_length * cos(B2_PI - contact_angle);
    float y_pos_f1 = m_finger_length * sin(contact_angle);
    fingerDef_right.position = {x_palm_pos + x_pos_f_right, y_palm_pos + y_pos_f1};
    fingerDef_right.rotation = b2MakeRot( B2_PI - contact_angle );
    
    // Just make the finger geometry
    b2Transform xform_left = { {-m_palm_length + m_finger_length * cos(contact_angle), y_pos_f1}, b2MakeRot( contact_angle )};
    b2Transform xform_right = { {m_palm_length + m_finger_length * cos(B2_PI - contact_angle), y_pos_f1}, b2MakeRot( B2_PI - contact_angle )};
    b2Polygon right_finger = b2TransformPolygon( xform_right, &finger_shape);
    b2Polygon left_finger = b2TransformPolygon( xform_left, &finger_shape);
    
    //b2BodyDef fingerDef_left = b2DefaultBodyDef();
    //fingerDef_left.type = b2_kinematicBody;
    //fingerDef_left.userData = userData;
    
    //fingerDef_left.position = {x_palm_pos - x_pos_f_left, y_palm_pos + y_pos_f1};
    //fingerDef_left.rotation = b2MakeRot( contact_angle );
    
    b2BodyDef palmDef = b2DefaultBodyDef();
    palmDef.type = b2_dynamicBody;
    palmDef.userData = userData;
    palmDef.position = {x_palm_pos, y_palm_pos};
    palmDef.rotation = b2MakeRot( 0.0 );
    
    b2BodyDef armDef = b2DefaultBodyDef();
    armDef.type = b2_kinematicBody;
    armDef.userData = userData;
    armDef.position = {x_palm_pos, y_palm_pos - m_settings.wrist_gap};
    armDef.rotation = b2MakeRot( 0.0 );
    
    b2Polygon arm_shape = b2MakeBox(m_palm_length, m_finger_palm_width);

    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = 1.0f;
    shapeDef.filter.groupIndex = -groupIndex;
    shapeDef.friction = 0.5f;
    shapeDef.filter.categoryBits = GRIPPER;

    m_gripperIds[0] = b2CreateBody( worldId, &palmDef );
    m_gripperIds[1] = b2CreateBody( worldId, &armDef );
    //m_gripperIds[2] = b2CreateBody( worldId, &fingerDef_left );
    b2CreatePolygonShape( m_gripperIds[0], &shapeDef, &palm_shape );
    b2CreatePolygonShape( m_gripperIds[0], &shapeDef, &right_finger );
    b2CreatePolygonShape( m_gripperIds[0], &shapeDef, &left_finger );

    b2CreatePolygonShape( m_gripperIds[1], &shapeDef, &arm_shape );

    b2Vec2 tri_vs[4];
    tri_vs[0].x = -m_palm_length;
    tri_vs[0].y = -m_finger_palm_width;
    tri_vs[1].x =  -1.0f * tri_vs[0].x;
    tri_vs[1].y = tri_vs[0].y;
    tri_vs[2].x = tri_vs[1].x - 2.0f * m_finger_length * cos(B2_PI * 0.25f);
    tri_vs[2].y = tri_vs[1].y + 2.0f * m_finger_length * sin(B2_PI * 0.25f);
    tri_vs[3].x = -tri_vs[2].x;
    tri_vs[3].y = tri_vs[2].y;
    m_armProxy = b2MakeProxy(tri_vs, 4, m_settings.apple_radius + m_finger_palm_width);
    
    /* Dynamic fingers
     m_gripperIds[0] = b2CreateBody( worldId, &palmDef );
     m_gripperIds[1] = b2CreateBody( worldId, &fingerDef_right );
     m_gripperIds[2] = b2CreateBody( worldId, &fingerDef_left );
     b2CreatePolygonShape( m_gripperIds[0], &shapeDef, &palm_shape );
     b2CreatePolygonShape( m_gripperIds[1], &shapeDef, &finger_shape );
     b2CreatePolygonShape( m_gripperIds[2], &shapeDef, &finger_shape );
     
     float k_tb = 20.0; //AngularHertz for the joint tree-branch
     
     // Make joint for finger 1
     b2WeldJointDef weldDef1 = b2DefaultWeldJointDef();
     weldDef1.angularHertz = k_tb;
     weldDef1.angularDampingRatio = 0.0f;
     weldDef1.localAnchorA = { m_palm_length, 0.0f };
     weldDef1.localAnchorB = { -finger_length, 0.0f };
     
     // right finger
     weldDef1.bodyIdA = m_gripperIds[0];;
     weldDef1.bodyIdB = m_gripperIds[1];
     weldDef1.referenceAngle =  B2_PI - contact_angle;
     m_jointIds[3] = b2CreateWeldJoint( worldId, &weldDef1 );
     
     b2WeldJointDef weldDef2 = b2DefaultWeldJointDef();
     weldDef2.angularHertz = k_tb;
     weldDef2.angularDampingRatio = 0.0f;
     weldDef2.localAnchorA = { -m_palm_length, 0.0f };
     weldDef2.localAnchorB = { -finger_length, 0.0f };
     
     // left finger
     weldDef2.bodyIdA = m_gripperIds[0];;
     weldDef2.bodyIdB = m_gripperIds[2];
     weldDef2.referenceAngle =  contact_angle;
     m_jointIds[4] = b2CreateWeldJoint( worldId, &weldDef2 );
     */
    
    b2DistanceJointDef lhsLift = b2DefaultDistanceJointDef();
    lhsLift.length = m_settings.wrist_gap;
    //lhsLift.dampingRatio = 0.1f;
    lhsLift.maxLength = 10.0 * m_settings.wrist_gap;
    lhsLift.enableSpring = true;
    lhsLift.collideConnected = true;
    lhsLift.enableLimit = true;
    lhsLift.bodyIdA = m_gripperIds[palm_id];
    lhsLift.bodyIdB = m_gripperIds[arm_id];
    lhsLift.localAnchorA = { -m_palm_length, 0.0f };
    lhsLift.localAnchorB = { -m_palm_length, 0.0f };
    lhsLift.hertz = 20.0f;
    m_jointIds[arm_palm_left_id] = b2CreateDistanceJoint( worldId, &lhsLift );
    
    lhsLift.localAnchorA = { m_palm_length, 0.0f };
    lhsLift.localAnchorB = { m_palm_length, 0.0f };
    m_jointIds[arm_palm_right_id] = b2CreateDistanceJoint( worldId, &lhsLift );

}

void Donut::Spawn( b2WorldId worldId, b2Vec2 position, float scale, int groupIndex, void* userData )
{
    assert( m_isSpawned == false );
    for ( int i = 0; i < n_body_ids; ++i)
    {
        assert( B2_IS_NULL( m_bodyIds[i] ) );
    }
    float length_tree = 5.0f;
    float length_stiff_branch = 3.0f;
    float length_bendy_branch = 1.0f;
    
    // Percentage along half of branch
    float perc_stiff_branch = 0.8;
    float perc_bendy_branch = 0.9;
    
    float angle_bendy_branch = -0.2*B2_PI;
    float ref_angle_bendy_branch = -B2_PI/2.0f + angle_bendy_branch;
    
    float k_tb = 8.0; // AngularHertz for the joint tree-stiff branch
    float k_bs = 4.0; // stiff branch-bendy branch
    
    float density = 1.0f; //change the weight of object, same for all objects for now
    
    // Tree - base on the ground, pointed up 0,-y  to 0,+y
    b2Capsule capsule_tree = { { 0.0f, -0.5f * length_tree }, { 0.0f, 0.5f * length_tree }, 0.15f * length_tree };
    b2Polygon poly_tree = b2MakeRoundedBox(0.5f * length_tree, 0.15f * length_tree, 0.5f * length_tree);
    m_treeProxies[0] = b2MakeProxy(poly_tree.vertices, poly_tree.count, 0.5f * length_tree);
    
    b2BodyDef treeDef = b2DefaultBodyDef();
    treeDef.type = b2_staticBody;
    treeDef.userData = userData;
    
    treeDef.position = {0.0, 0.5f*length_tree};
    treeDef.rotation = b2MakeRot( 0.0 );
    
    b2ShapeDef shapeDef = b2DefaultShapeDef();
    shapeDef.density = density;
    shapeDef.filter.groupIndex = -groupIndex;
    shapeDef.friction = 0.3f;
    shapeDef.filter.categoryBits = BRANCH_TREE;
    
    m_bodyIds[tree_id] = b2CreateBody( worldId, &treeDef );
    b2CreateCapsuleShape( m_bodyIds[tree_id], &shapeDef, &capsule_tree );

    // Stiff branch - points horizontally
    b2BodyDef stiffBranchDef = b2DefaultBodyDef();
    stiffBranchDef.type = b2_dynamicBody;
    stiffBranchDef.userData = userData;
    
    // +- y
    b2Capsule capsule_branch = { { 0.0f, -0.5f * length_stiff_branch},
        { 0.0f,  0.5f * length_stiff_branch }, 0.1f * length_stiff_branch };
    
    // Rotate to the left 90 degrees and move up
    stiffBranchDef.position = {-0.5f*length_stiff_branch, 0.5f * length_tree + perc_stiff_branch * 0.5f * length_tree };
    stiffBranchDef.rotation = b2MakeRot( B2_PI/2.0f );
    
    m_bodyIds[stiff_branch_id] = b2CreateBody( worldId, &stiffBranchDef );
    b2CreateCapsuleShape( m_bodyIds[stiff_branch_id], &shapeDef, &capsule_branch );
    b2Polygon poly_branch = b2MakeRoundedBox(0.5f * length_stiff_branch, 0.1f * length_stiff_branch, 0.5f * length_stiff_branch);
    m_treeProxies[1] = b2MakeProxy(poly_branch.vertices, poly_branch.count, 0.5f * length_stiff_branch);

    // Tree-stiff-branch joint
    b2WeldJointDef weldDef = b2DefaultWeldJointDef();
    weldDef.angularHertz = k_tb;
    weldDef.angularDampingRatio = 0.0f;
    weldDef.localAnchorA = { 0.0f, perc_stiff_branch * 0.5f * length_tree };
    weldDef.localAnchorB = { 0.0f, -0.5f * length_stiff_branch };
    
    weldDef.bodyIdA = m_bodyIds[tree_id];
    weldDef.bodyIdB = m_bodyIds[stiff_branch_id];
    weldDef.referenceAngle =  B2_PI/2.0f;
    m_jointIds[tree_stiff_branch_id] = b2CreateWeldJoint( worldId, &weldDef );
    
    // Bendy branch
    b2BodyDef bendy_branchDef = b2DefaultBodyDef();
    bendy_branchDef.type = b2_dynamicBody;
    bendy_branchDef.userData = userData;
    
    b2Vec2 pos_end_stiff = b2Body_GetWorldPoint(m_bodyIds[stiff_branch_id], { 0.0f, perc_stiff_branch * 0.5f * length_stiff_branch });
    
    bendy_branchDef.position = {pos_end_stiff.x - sin(-angle_bendy_branch)*length_bendy_branch*0.5f,
        pos_end_stiff.y - cos(-angle_bendy_branch)*length_bendy_branch*0.5f};
    bendy_branchDef.rotation = b2MakeRot(angle_bendy_branch );
    
    b2Capsule capsule_bendy_branch = { { 0.0f, -0.5f * length_bendy_branch}, { 0.0f, 0.5f * length_bendy_branch }, 0.05f * length_bendy_branch };
    poly_branch = b2MakeRoundedBox(0.5f * length_bendy_branch, 0.05f * length_bendy_branch, 0.5f * length_bendy_branch);
    m_treeProxies[2] = b2MakeProxy(poly_branch.vertices, poly_branch.count, 0.5f * length_bendy_branch);

    shapeDef.density = 0.8f;
    
    m_bodyIds[bendy_branch_id] = b2CreateBody( worldId, &bendy_branchDef );
    b2CreateCapsuleShape( m_bodyIds[2], &shapeDef, &capsule_bendy_branch );
    
    // Bendy-branch stiff branch joint
    b2WeldJointDef weldBranchesDef = b2DefaultWeldJointDef();
    weldBranchesDef.angularHertz = k_bs;
    weldBranchesDef.angularDampingRatio = 0.0f;
    weldBranchesDef.localAnchorA = { 0.0f, 0.5f * perc_stiff_branch * length_stiff_branch };
    weldBranchesDef.localAnchorB = { 0.0f, 0.5f * length_bendy_branch };
    
    weldBranchesDef.bodyIdA = m_bodyIds[stiff_branch_id];
    weldBranchesDef.bodyIdB = m_bodyIds[bendy_branch_id];
    weldBranchesDef.referenceAngle =  ref_angle_bendy_branch;
    m_jointIds[stiff_branch_bendy_branch_id] = b2CreateWeldJoint( worldId, &weldBranchesDef );
    
    // Stem
    b2BodyDef stemDef = b2DefaultBodyDef();
    stemDef.type = b2_dynamicBody;
    stemDef.userData = userData;
    
    b2Vec2 pos_end_bendy = b2Body_GetWorldPoint(m_bodyIds[bendy_branch_id], { 0.0f, -0.5f * perc_bendy_branch * length_bendy_branch });
    stemDef.position = { pos_end_bendy.x, pos_end_bendy.y - 0.5f * m_settings.stem_length };
    stemDef.rotation = b2MakeRot(0.0f);
    
    b2Capsule capsule_stem = { { 0.0f, -0.5f * m_settings.stem_length}, { 0.0f, 0.5f * m_settings.stem_length }, 0.08f * m_settings.stem_length };
    
    shapeDef.density = 0.8f;
    shapeDef.filter.categoryBits = STEM_APPLE;
    
    m_bodyIds[stem_id] = b2CreateBody( worldId, &stemDef );
    b2CreateCapsuleShape( m_bodyIds[stem_id], &shapeDef, &capsule_stem );
    
    // bendy-branch to stem joint - revolute joint
    b2RevoluteJointDef rev_JointDef = b2DefaultRevoluteJointDef();
    
    b2Rot rot_bendy_branch = b2Body_GetRotation(m_bodyIds[bendy_branch_id]);
    b2Rot rot_stem = b2Body_GetRotation(m_bodyIds[stem_id]);
    float angle_cur = b2RelativeAngle( rot_stem, rot_bendy_branch );
    angle_cur = b2UnwindAngle( angle_cur );
    
    rev_JointDef.referenceAngle = angle_cur;
    rev_JointDef.lowerAngle = -(B2_PI / 2.0f - abs(angle_bendy_branch));
    rev_JointDef.upperAngle =  B2_PI - rev_JointDef.lowerAngle;
    rev_JointDef.enableLimit = true;
    rev_JointDef.localAnchorA = { 0.0f,  -0.5f * perc_bendy_branch * length_bendy_branch };
    rev_JointDef.localAnchorB = { 0.0f, 0.5f * m_settings.stem_length };
    
    rev_JointDef.bodyIdA = m_bodyIds[bendy_branch_id];
    rev_JointDef.bodyIdB = m_bodyIds[stem_id];
    
    m_jointIds[bendy_branch_stem_id] = b2CreateRevoluteJoint( worldId, &rev_JointDef );
    
    // Apple
    b2BodyDef appleDef = b2DefaultBodyDef();
    appleDef.type = b2_dynamicBody;
    appleDef.userData = userData;
    
    b2Vec2 pos_end_stem = b2Body_GetWorldPoint(m_bodyIds[stem_id], { 0.0f, -0.5f * m_settings.stem_length });
    
    appleDef.position = {pos_end_stem.x, pos_end_stem.y - m_settings.apple_radius};
    appleDef.rotation = b2MakeRot(0);
    
    b2Circle apple = { { 0.0f, 0.0f }, m_settings.apple_radius };
    const int n_vertices = 32;
    const float f_ang = (2.0f * B2_PI) / (float) n_vertices;
    b2Vec2 vertices[n_vertices];
    for (int i = 0; i < n_vertices; i++)
    {
        vertices[i].x = m_settings.apple_radius * cos( f_ang * i);
        vertices[i].y = m_settings.apple_radius * sin( f_ang * i);
    }
    m_appleProxy = b2MakeProxy(vertices, n_vertices, m_settings.apple_radius);
	
	m_bodyIds[apple_id] = b2CreateBody( worldId, &appleDef );
    b2CreateCircleShape( m_bodyIds[apple_id], &shapeDef, &apple );

    // Apple-stem joint
    b2RevoluteJointDef stem_apple_jointDef = b2DefaultRevoluteJointDef();

    b2Rot rot_apple = b2Body_GetRotation(m_bodyIds[apple_id]);
    angle_cur = b2RelativeAngle( rot_apple, rot_stem );
    angle_cur = b2UnwindAngle( angle_cur );

    stem_apple_jointDef.referenceAngle = angle_cur;
    stem_apple_jointDef.lowerAngle = -3.0 * B2_PI / 4.0; //stem_apple_jointDef.referenceAngle - 3.0 * B2_PI / 4.0f;
    stem_apple_jointDef.upperAngle =  3.0 * B2_PI / 4.0; //stem_apple_jointDef.referenceAngle + 3.0 * B2_PI / 4.0f;
    stem_apple_jointDef.enableLimit = true;
    stem_apple_jointDef.localAnchorA = { 0.0f,  - 0.5f * m_settings.stem_length };
    stem_apple_jointDef.localAnchorB = { 0.0f, m_settings.apple_radius };

    stem_apple_jointDef.bodyIdA = m_bodyIds[stem_id];
    stem_apple_jointDef.bodyIdB = m_bodyIds[apple_id];
    stem_apple_jointDef.referenceAngle = 0.0f;

    m_jointIds[stem_apple_id] = b2CreateRevoluteJoint( worldId, &stem_apple_jointDef );

    SpawnGripper(worldId, groupIndex,userData);
    
    b2DistanceJointDef appleGripperJointDef = b2DefaultDistanceJointDef();
    float finger_palm_width = m_settings.apple_radius * 0.1;
    appleGripperJointDef.length = finger_palm_width;
    //lhsLift.dampingRatio = 0.1f;
    appleGripperJointDef.maxLength = 10.0;
    appleGripperJointDef.enableSpring = true;
    appleGripperJointDef.collideConnected = true;
    appleGripperJointDef.bodyIdA = m_gripperIds[0];
    appleGripperJointDef.bodyIdB = m_bodyIds[apple_id];
    appleGripperJointDef.localAnchorA = { 0.0, 0.0 };
    appleGripperJointDef.localAnchorB = { 0.0, -m_settings.apple_radius };
    appleGripperJointDef.hertz = 20.0f;
    m_jointIds[apple_palm_id] = b2CreateDistanceJoint( worldId, &appleGripperJointDef );

    SpawnSensors(worldId, groupIndex, userData);
    m_isSpawned = true;

}

void Donut::Despawn()
{
    assert( m_isSpawned == true );
    
    for ( int i = 0; i < n_body_ids; ++i )
    {
        b2DestroyBody( m_bodyIds[i] );
        m_bodyIds[i] = b2_nullBodyId;
    }
    for ( int i = 0; i < n_joint_ids; ++i )
    {
        b2DestroyJoint( m_jointIds[i] );
    }
    for ( int i = 0; i < n_gripper_ids; ++i )
    {
        b2DestroyBody(m_gripperIds[i]);
        m_gripperIds[i] = b2_nullBodyId;
    }
    for ( int i = 0; i < n_sensor_ids; ++i )
    {
        //b2DestroyShape(m_sensorIds[i]);
        //m_sensorIds[i] = b2_nullShapeId;
    }
    
    WriteFile();
    
    m_n_time_steps_store = 0;
    m_current_time_step = -1;
    delete m_tree_info;
    m_tree_info = nullptr;
    delete m_arm_info;
    m_arm_info = nullptr;

    m_isSpawned = false;
}
